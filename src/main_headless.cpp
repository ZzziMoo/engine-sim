// engine-sim headless entry point
// Targets ARM64 Linux (Raspberry Pi 4) / EV-Tesla sound project.
// Windows MSVC is the build-test platform; Pi 4 ARM64 Linux is production.
//
// Build:  cmake -B build -DHEADLESS=ON -DDISCORD_ENABLED=OFF
// Usage:  engine-sim-headless --script assets/engines/.../engine.mr
//             [--port 9999] [--throttle 0.5] [--benchmark]
//             [--sample-rate 44100]

// Prevent windows.h min/max macros from shadowing std::min/std::max
#define NOMINMAX

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#include "../include/engine.h"
#include "../include/piston_engine_simulator.h"
#include "../include/synthesizer.h"
#include "../include/vehicle.h"
#include "../include/transmission.h"
#include "../include/impulse_response.h"
#include "../include/units.h"

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
#include "../scripting/include/compiler.h"
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#ifdef __unix__
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifdef __linux__
#include <time.h>
static double processCpuSeconds() {
    struct timespec ts;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}
#else
static double processCpuSeconds() { return -1.0; }
#endif

// ---------------------------------------------------------------------------
// Control mode
// ---------------------------------------------------------------------------
enum class ControlMode : int {
    Throttle = 0,   // legacy: throttle <0-1> passes directly to setSpeedControl
    Rpm      = 1,   // setrpm <value> drives target RPM
    Hybrid   = 2    // pedal controls throttle; speed maps to fake RPM
};

// ---------------------------------------------------------------------------
// SPSC audio ring buffer -- decouples synthesis (60 Hz main loop) from the
// audio callback (~86 Hz).  Producer: main loop.  Consumer: audio callback.
// Mirrors the GUI's two-stage model: synthesis -> ring -> hardware callback.
// ---------------------------------------------------------------------------
struct AudioRing {
    static constexpr int kSize = 16384; // power-of-2, ~372 ms at 44100 Hz
    static constexpr int kMask = kSize - 1;

    int16_t data[kSize] {};
    std::atomic<int> wIdx { 0 };
    std::atomic<int> rIdx { 0 };

    int available() const {
        return (wIdx.load(std::memory_order_acquire) -
                rIdx.load(std::memory_order_relaxed) + kSize) & kMask;
    }
    int freeSlots() const { return kSize - 1 - available(); }

    // Producer only
    int write(const int16_t *src, int n) {
        const int free = freeSlots();
        if (n > free) n = free;
        const int w = wIdx.load(std::memory_order_relaxed);
        for (int i = 0; i < n; ++i)
            data[(w + i) & kMask] = src[i];
        wIdx.store((w + n) & kMask, std::memory_order_release);
        return n;
    }

    // Consumer only
    int read(int16_t *dst, int n) {
        const int avail = available();
        if (n > avail) n = avail;
        const int r = rIdx.load(std::memory_order_relaxed);
        for (int i = 0; i < n; ++i)
            dst[i] = data[(r + i) & kMask];
        rIdx.store((r + n) & kMask, std::memory_order_release);
        return n;
    }
};

// ---------------------------------------------------------------------------
// Backfire: cross-thread config + lock-free trigger mailbox
// ---------------------------------------------------------------------------
struct BackfireCfg {
    std::atomic<bool>  enabled   { false }; // off by default; enable with "backfire on"
    std::atomic<float> chance    { 0.35f };
    std::atomic<float> volume    { 0.25f }; // conservative default
    std::atomic<int>   rpmMin    { 3000  };
    std::atomic<int>   count     { 0     };

    // Mailbox: main/control threads write, audio callback reads-and-clears.
    // Amplitude stored before samples count so the callback never sees an
    // inconsistent (samples>0 but stale amplitude) state.
    std::atomic<float> trigAmplitude { 0.0f };
    std::atomic<int>   trigSamples   { 0    };
};

// Audio-callback-exclusive state -- no atomics needed, only one thread touches it.
struct BackfireCbState {
    int      active   = 0;
    int      total    = 0;
    float    curAmp   = 0.0f;        // decays each sample via decayMul
    float    decayMul = 1.0f;        // exp(-k/total), computed once at trigger
    float    lpState  = 0.0f;        // one-pole lowpass state (~2 kHz) softens noise
    uint32_t rng      = 0xCAFEBABEu;
};

// ---------------------------------------------------------------------------
// Shared application state
// ---------------------------------------------------------------------------
struct HeadlessState {
    Simulator          *sim        = nullptr;
    Engine             *engine     = nullptr;
    int                 sampleRate = 44100;

    std::atomic<int>    underruns  { 0 };
    std::atomic<bool>   running    { true };

    // --- throttle mode ---
    // DirectThrottleLinkage convention: s=0 -> full throttle, s=1 -> idle.
    std::atomic<double> throttle   { 0.5 };

    // --- extended EV/CAN inputs ---
    std::atomic<int>    mode       { 0   }; // cast of ControlMode
    std::atomic<double> pedal      { 0.0 }; // 0-100
    std::atomic<double> brake      { 0.0 }; // 0-1
    std::atomic<double> speedMph   { 0.0 };
    std::atomic<double> torque     { 0.0 };
    std::atomic<double> targetRpm  { 900.0 };

    // --- RPM mapping (hybrid/rpm modes) ---
    std::atomic<double> idleRpm    { 900.0  };
    std::atomic<double> rpmPerMph  { 85.0   };
    std::atomic<double> maxRpm     { 8000.0 };

    // --- backfire ---
    BackfireCfg     backfire;
    BackfireCbState backfireCb;         // audio callback exclusive
    std::atomic<int> backfireEpochCount { 0 }; // reset each benchmark interval

    // --- RPM PI controller (rpm mode) ---
    // DirectThrottleLinkage is NOT a direct RPM setter. A PI controller
    // adjusts setSpeedControl each frame so actual RPM tracks targetRpm.
    std::atomic<double> rpmKp { 0.3 };  // proportional gain (normalized units)
    std::atomic<double> rpmKi { 0.1 };  // integral gain (/s)
    std::atomic<double> rpmKd { 0.0 };  // derivative gain (future use)

    // --- sweep_rpm: linear RPM ramp for audible testing ---
    std::atomic<double> sweepFromRpm { 0.0 };
    std::atomic<double> sweepToRpm   { 0.0 };
    std::atomic<double> sweepSeconds { 0.0 };
    std::atomic<bool>   sweepTrigger { false }; // set by command, cleared by main loop

    // --- startup / gear ---
    std::atomic<bool> autostart      { true };

    // Pending one-shot commands dispatched to main thread each frame.
    std::atomic<int>  pendingIgnition { -1 }; // 0=off, 1=on
    std::atomic<int>  pendingStarter  { -1 }; // 0=off, 1=on
    std::atomic<int>  pendingGear     { -2 }; // -1=neutral, 0..N=gear (0-indexed); -2=none

    // --- audio ring (producer: main loop, consumer: audio callback) ---
    // The ring decouples 60 Hz synthesis bursts from the ~86 Hz callback.
    AudioRing audioRing;

    // Samples-read counter updated by the audio callback (atomic for cross-thread read).
    std::atomic<int> cbSamplesRead { 0 };

    // --- audio diagnostics -- all written by main loop, read by status/debug ---
    // No atomics needed here; status reads the snapshots which are plain values.
    // (The main loop is the single writer; status runs from a command thread but
    //  reads advisory data that can be slightly stale -- that is fine.)
    float  snapshotRms       = 0.0f;
    float  snapshotPeak      = 0.0f;
    int    synthBufFill      = 0;     // synthesizer input-latency buffer samples
    int    samplesGenPerSec  = 0;     // samples drained from synthesizer last second
    int    samplesReadPerSec = 0;     // samples consumed by audio callback last second
    uint32_t audioHash       = 0;     // rolling hash of generated audio (detects loops)
    float  zeroCrossRate     = 0.0f;  // zero crossings/second (approximates 2x fundamental)

    std::atomic<bool> audioDebug { false };
};

// ---------------------------------------------------------------------------
// Fire a synthetic backfire pop: write to the audio callback's mailbox.
// Safe to call from any non-audio thread.
// ---------------------------------------------------------------------------
static void triggerBackfire(HeadlessState &state) {
    const int ms      = 30 + std::rand() % 61; // 30-90 ms
    const int samples = ms * state.sampleRate / 1000;
    const float amp   = state.backfire.volume.load(std::memory_order_relaxed);
    // Amplitude first so the callback never reads samples>0 with a stale amp.
    state.backfire.trigAmplitude.store(amp,     std::memory_order_relaxed);
    state.backfire.trigSamples.store  (samples, std::memory_order_release);
    state.backfire.count.fetch_add(1, std::memory_order_relaxed);
    state.backfireEpochCount.fetch_add(1, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// miniaudio callback -- reads from the SPSC ring then mixes backfire.
// The ring is filled by the main loop after each physics frame, exactly
// mirroring the GUI's two-stage model (synthesis -> ring -> hardware).
// No mutex, no synthesizer access from this thread.
// ---------------------------------------------------------------------------
static void audioCallback(
        ma_device *device, void *output, const void *, ma_uint32 frameCount)
{
    HeadlessState *s   = static_cast<HeadlessState *>(device->pUserData);
    int16_t       *out = static_cast<int16_t *>(output);

    const int got = s->audioRing.read(out, static_cast<int>(frameCount));
    if (got < static_cast<int>(frameCount)) {
        std::memset(out + got, 0, (frameCount - got) * sizeof(int16_t));
        s->underruns.fetch_add(1, std::memory_order_relaxed);
    }
    s->cbSamplesRead.fetch_add(got, std::memory_order_relaxed);

    // Backfire pop mix (unchanged -- no synthesizer access)
    if (!s->backfire.enabled.load(std::memory_order_relaxed))
        return;

    BackfireCbState &cb = s->backfireCb;

    if (cb.active == 0) {
        const int pending =
            s->backfire.trigSamples.exchange(0, std::memory_order_acquire);
        if (pending > 0) {
            cb.total    = pending;
            cb.active   = pending;
            cb.curAmp   = s->backfire.trigAmplitude.load(std::memory_order_relaxed);
            // k=10: amplitude is ~0.005% of peak at end -- sharp transient
            cb.decayMul = expf(-10.0f / static_cast<float>(pending));
            cb.lpState  = 0.0f;
        }
    }

    if (cb.active > 0) {
        const ma_uint32 toPlay = (static_cast<ma_uint32>(cb.active) < frameCount)
                                    ? static_cast<ma_uint32>(cb.active)
                                    : frameCount;
        for (ma_uint32 i = 0; i < toPlay; ++i) {
            cb.rng ^= cb.rng << 13u;
            cb.rng ^= cb.rng >> 17u;
            cb.rng ^= cb.rng <<  5u;
            const float raw = static_cast<float>(cb.rng) *
                               (1.0f / 4294967296.0f) * 2.0f - 1.0f;
            // One-pole lowpass at ~2 kHz: warms the noise into a softer crack
            cb.lpState = 0.25f * raw + 0.75f * cb.lpState;
            cb.curAmp *= cb.decayMul;
            const float pop  = cb.lpState * cb.curAmp;
            const float eng  = out[i] * (1.0f / 32767.0f);
            float mixed = eng + pop;
            if (mixed >  1.0f) mixed =  1.0f;
            if (mixed < -1.0f) mixed = -1.0f;
            out[i] = static_cast<int16_t>(mixed * 32767.0f);
        }
        cb.active -= static_cast<int>(toPlay);
    }
}

// ---------------------------------------------------------------------------
// Command parser -- shared by stdin and UDP threads
// ---------------------------------------------------------------------------
static void applyCommand(const std::string &line, HeadlessState &state) {
    std::istringstream ss(line);
    std::string cmd;
    if (!(ss >> cmd)) return;

    auto clamp01  = [](double v) { return v < 0.0 ? 0.0 : v > 1.0   ? 1.0   : v; };
    auto clamp100 = [](double v) { return v < 0.0 ? 0.0 : v > 100.0 ? 100.0 : v; };
    auto clampPos = [](double v) { return v < 0.0 ? 0.0 : v; };

    if (cmd == "throttle" || cmd == "speedcontrol") {
        double v = 0.5;
        if (ss >> v)
            state.throttle.store(clamp01(v), std::memory_order_relaxed);

    } else if (cmd == "pedal") {
        double v = 0.0;
        if (ss >> v)
            state.pedal.store(clamp100(v), std::memory_order_relaxed);

    } else if (cmd == "brake") {
        double v = 0.0;
        if (ss >> v)
            state.brake.store(clamp01(v), std::memory_order_relaxed);

    } else if (cmd == "speed") {
        double v = 0.0;
        if (ss >> v)
            state.speedMph.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "torque") {
        double v = 0.0;
        if (ss >> v)
            state.torque.store(v, std::memory_order_relaxed);

    } else if (cmd == "setrpm") {
        double v = 900.0;
        if (ss >> v)
            state.targetRpm.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "mode") {
        std::string sub;
        if (!(ss >> sub)) return;
        if      (sub == "throttle")
            state.mode.store((int)ControlMode::Throttle, std::memory_order_relaxed);
        else if (sub == "rpm") {
            state.mode.store((int)ControlMode::Rpm, std::memory_order_relaxed);
            // Neutral removes vehicle inertia so PI controller can reach
            // any target RPM within a few seconds.
            state.pendingGear.store(-1, std::memory_order_relaxed);
        }
        else if (sub == "hybrid") {
            state.mode.store((int)ControlMode::Hybrid, std::memory_order_relaxed);
            // Neutral removes drivetrain resistance; hybrid uses direct RPM
            // control so the transmission must not fight the crankshaft.
            state.pendingGear.store(-1, std::memory_order_relaxed);
        }

    } else if (cmd == "rpm_per_mph") {
        double v = 85.0;
        if (ss >> v) state.rpmPerMph.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "idle_rpm") {
        double v = 900.0;
        if (ss >> v) state.idleRpm.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "max_rpm") {
        double v = 8000.0;
        if (ss >> v) state.maxRpm.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "rpm_kp") {
        double v = 0.5;
        if (ss >> v) state.rpmKp.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "rpm_ki") {
        double v = 0.2;
        if (ss >> v) state.rpmKi.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "rpm_kd") {
        double v = 0.0;
        if (ss >> v) state.rpmKd.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "sweep_rpm") {
        double lo = 1000.0, hi = 6000.0, secs = 8.0;
        if (ss >> lo >> hi >> secs) {
            state.sweepFromRpm.store(std::max(0.0, lo),   std::memory_order_relaxed);
            state.sweepToRpm.store  (std::max(0.0, hi),   std::memory_order_relaxed);
            state.sweepSeconds.store(std::max(1.0, secs), std::memory_order_relaxed);
            state.sweepTrigger.store(true, std::memory_order_release);
        }

    } else if (cmd == "backfire") {
        std::string sub;
        if (!(ss >> sub)) return;
        if      (sub == "on")
            state.backfire.enabled.store(true,  std::memory_order_relaxed);
        else if (sub == "off")
            state.backfire.enabled.store(false, std::memory_order_relaxed);
        else if (sub == "chance") {
            double v = 0.35;
            if (ss >> v)
                state.backfire.chance.store(
                    static_cast<float>(clamp01(v)), std::memory_order_relaxed);
        } else if (sub == "volume") {
            double v = 0.7;
            if (ss >> v)
                state.backfire.volume.store(
                    static_cast<float>(clamp01(v)), std::memory_order_relaxed);
        } else if (sub == "rpm_min") {
            int v = 3000;
            if (ss >> v)
                state.backfire.rpmMin.store(v < 0 ? 0 : v, std::memory_order_relaxed);
        } else if (sub == "test") {
            triggerBackfire(state);
            std::cout << "[backfire] test pop fired\n" << std::flush;
        }

    } else if (cmd == "ignition") {
        std::string sub;
        if (ss >> sub)
            state.pendingIgnition.store(sub == "on" ? 1 : 0, std::memory_order_relaxed);

    } else if (cmd == "starter") {
        std::string sub;
        if (ss >> sub)
            state.pendingStarter.store(sub == "on" ? 1 : 0, std::memory_order_relaxed);

    } else if (cmd == "gear") {
        int n = 1;
        if (ss >> n) {
            // User-visible: 0=neutral, 1=1st, 2=2nd... -> API: -1=neutral, 0=1st...
            const int apiGear = (n <= 0) ? -1 : n - 1;
            state.pendingGear.store(apiGear, std::memory_order_relaxed);
        }

    } else if (cmd == "autostart") {
        std::string sub;
        if (ss >> sub)
            state.autostart.store(sub == "on", std::memory_order_relaxed);

    } else if (cmd == "audio_debug") {
        std::string sub;
        if (ss >> sub)
            state.audioDebug.store(sub == "on", std::memory_order_relaxed);

    } else if (cmd == "quit") {
        state.running.store(false, std::memory_order_relaxed);

    } else if (cmd == "status") {
        const char *modeStr = "throttle";
        switch (static_cast<ControlMode>(state.mode.load())) {
            case ControlMode::Rpm:    modeStr = "rpm";    break;
            case ControlMode::Hybrid: modeStr = "hybrid"; break;
            default: break;
        }
        const double rpm        = state.engine ? state.engine->getRpm() : 0.0;
        const double crankRpm   = state.engine && state.engine->getCrankshaftCount() > 0
            ? std::abs(units::toRpm(state.engine->getCrankshaft(0)->m_body.v_theta))
            : 0.0;
        const double throttlePos = state.engine
            ? state.engine->getThrottle() : 0.0;
        const bool   ignition   = state.engine
            ? state.engine->getIgnitionModule()->m_enabled : false;
        const bool   starter    = state.sim
            ? state.sim->m_starterMotor.m_enabled : false;
        const int    gearApi    = state.sim
            ? state.sim->getTransmission()->getGear() : -1;
        const std::string gearStr = (gearApi == -1)
            ? "N" : std::to_string(gearApi + 1);

        std::cout
            << "mode="              << modeStr                                   << "\n"
            << "ignition="          << (ignition ? "on" : "off")                 << "\n"
            << "starter="           << (starter  ? "on" : "off")                 << "\n"
            << "gear="              << gearStr                                    << "\n"
            << "throttle_sc="       << state.throttle.load()                     << "\n"
            << "throttle_position=" << throttlePos                                << "\n"
            << "pedal="             << state.pedal.load()                        << "\n"
            << "brake="             << state.brake.load()                        << "\n"
            << "speed_mph="         << state.speedMph.load()                     << "\n"
            << "engine_rpm="        << rpm                                        << "\n"
            << "crankshaft_rpm="    << crankRpm                                   << "\n"
            << "target_rpm="        << state.targetRpm.load()                    << "\n"
            << "rpm_kp="            << state.rpmKp.load()                        << "\n"
            << "rpm_ki="            << state.rpmKi.load()                        << "\n"
            << "idle_rpm="          << state.idleRpm.load()                      << "\n"
            << "rpm_per_mph="       << state.rpmPerMph.load()                    << "\n"
            << "max_rpm="           << state.maxRpm.load()                       << "\n"
            << "audio_rms="              << state.snapshotRms                          << "\n"
            << "audio_peak="             << state.snapshotPeak                         << "\n"
            << "audio_hash="             << state.audioHash                             << "\n"
            << "audio_zcr="              << state.zeroCrossRate                         << "\n"
            << "audio_leveler_gain="     << state.sim->synthesizer().getLevelerGain()   << "\n"
            << "samples_gen_per_sec="    << state.samplesGenPerSec                      << "\n"
            << "samples_read_per_sec="   << state.samplesReadPerSec                     << "\n"
            << "ring_fill="              << state.audioRing.available()                 << "\n"
            << "synth_input_buf="        << state.synthBufFill                          << "\n"
            << "backfire="               << (state.backfire.enabled.load()?"on":"off")  << "\n"
            << "backfire_chance="        << state.backfire.chance.load()                << "\n"
            << "backfire_volume="        << state.backfire.volume.load()                << "\n"
            << "backfire_count="         << state.backfire.count.load()                 << "\n"
            << "underruns="              << state.underruns.load()                      << "\n"
            << std::flush;
    }
}

// ---------------------------------------------------------------------------
// stdin control thread
// ---------------------------------------------------------------------------
static void stdinThread(HeadlessState &state) {
    std::string line;
    while (state.running.load() && std::getline(std::cin, line))
        applyCommand(line, state);
}

// ---------------------------------------------------------------------------
// UDP control thread (POSIX only)
// ---------------------------------------------------------------------------
#ifdef __unix__
static void udpThread(HeadlessState &state, int port) {
    const int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "[headless] UDP socket failed, UDP control disabled\n";
        return;
    }

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[headless] UDP bind failed on port " << port << "\n";
        close(sock);
        return;
    }

    struct timeval tv{};
    tv.tv_sec  = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    std::cerr << "[headless] UDP control listening on port " << port << "\n";

    char buf[256];
    while (state.running.load()) {
        const ssize_t n = recv(sock, buf, sizeof(buf) - 1, 0);
        if (n > 0) {
            buf[n] = '\0';
            applyCommand(std::string(buf), state);
        }
    }
    close(sock);
}
#else
static void udpThread(HeadlessState &, int) {}
#endif

// ---------------------------------------------------------------------------
// Load WAV impulse response via miniaudio decoder
// ---------------------------------------------------------------------------
static void loadImpulseResponse(
        Simulator *sim, int index, const std::string &path, float volume)
{
    if (path.empty()) return;

    ma_decoder_config cfg = ma_decoder_config_init(ma_format_s16, 1, 44100);
    ma_uint64 frameCount  = 0;
    void     *pData       = nullptr;

    if (ma_decode_file(path.c_str(), &cfg, &frameCount, &pData) != MA_SUCCESS) {
        std::cerr << "[headless] WARNING: cannot load impulse response: " << path << "\n";
        return;
    }

    sim->synthesizer().initializeImpulseResponse(
        static_cast<const int16_t *>(pData),
        static_cast<unsigned int>(frameCount),
        volume, index);

    ma_free(pData, nullptr);
}

// ---------------------------------------------------------------------------
// CLI helpers
// ---------------------------------------------------------------------------
static std::string argValue(int argc, char **argv,
                            const char *flag, const char *def) {
    for (int i = 1; i < argc - 1; ++i)
        if (std::string(argv[i]) == flag) return argv[i + 1];
    return def;
}

static bool argFlag(int argc, char **argv, const char *flag) {
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == flag) return true;
    return false;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    const std::string scriptPath   = argValue(argc, argv, "--script",      "../assets/main.mr");
    const int         udpPort      = std::stoi(argValue(argc, argv, "--port",        "9999"));
    const double      initThrottle = std::stod(argValue(argc, argv, "--throttle",    "0.5"));
    const bool        benchmark    = argFlag(argc, argv, "--benchmark");
    const int         sampleRate   = std::stoi(argValue(argc, argv, "--sample-rate", "44100"));

    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // -----------------------------------------------------------------------
    // 1. Load engine script
    // -----------------------------------------------------------------------
    Engine       *engine       = nullptr;
    Vehicle      *vehicle      = nullptr;
    Transmission *transmission = nullptr;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    // Engine scripts define `public node main` but never instantiate it.
    // Generate a temporary wrapper (placed beside the script) that imports
    // the engine file and calls main() -- mirrors what assets/main.mr does.
    std::string compileTarget = scriptPath;
    std::string wrapperPath;

    auto hasSuffix = [](const std::string &s, const std::string &sfx) {
        return s.size() >= sfx.size() &&
               s.compare(s.size() - sfx.size(), sfx.size(), sfx) == 0;
    };
    if (!hasSuffix(scriptPath, "main.mr")) {
        const auto sep = scriptPath.find_last_of("/\\");
        const std::string dir  = (sep == std::string::npos) ? "." : scriptPath.substr(0, sep);
        const std::string base = (sep == std::string::npos) ? scriptPath : scriptPath.substr(sep + 1);

        wrapperPath = dir + "/__headless_main.mr";
        std::ofstream wf(wrapperPath);
        wf << "import \"engine_sim.mr\"\n";
        wf << "import \"" << base << "\"\n";
        wf << "main()\n";
        wf.close();
        compileTarget = wrapperPath;
    }

    es_script::Compiler compiler;
    compiler.initialize();
    std::cerr << "[headless] Compiling: " << scriptPath << "\n";
    if (compiler.compile(compileTarget.c_str())) {
        const es_script::Compiler::Output output = compiler.execute();
        engine       = output.engine;
        vehicle      = output.vehicle;
        transmission = output.transmission;
    } else {
        std::cerr << "[headless] ERROR: script compilation failed\n";
        if (!wrapperPath.empty()) std::remove(wrapperPath.c_str());
        compiler.destroy();
        return 1;
    }
    if (!wrapperPath.empty()) std::remove(wrapperPath.c_str());
    compiler.destroy();
#else
    std::cerr << "[headless] ERROR: built without Piranha scripting\n";
    return 1;
#endif

    if (!engine) {
        std::cerr << "[headless] ERROR: script produced no engine\n";
        return 1;
    }

    // Fallback vehicle
    if (!vehicle) {
        Vehicle::Parameters vp;
        vp.mass              = units::mass(1597, units::kg);
        vp.diffRatio         = 3.42;
        vp.tireRadius        = units::distance(10, units::inch);
        vp.dragCoefficient   = 0.25;
        vp.crossSectionArea  =
            units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
        vp.rollingResistance = 2000.0;
        vehicle = new Vehicle;
        vehicle->initialize(vp);
    }

    // Fallback transmission
    if (!transmission) {
        static const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
        Transmission::Parameters tp;
        tp.GearCount       = 6;
        tp.GearRatios      = gearRatios;
        tp.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        transmission = new Transmission;
        transmission->initialize(tp);
    }

    // -----------------------------------------------------------------------
    // 2. Create and configure simulator
    // -----------------------------------------------------------------------
    Simulator *sim = engine->createSimulator(vehicle, transmission);
    engine->calculateDisplacement();
    sim->setSimulationFrequency(static_cast<int>(engine->getSimulationFrequency()));

    Synthesizer::AudioParameters audioParams = sim->synthesizer().getAudioParameters();
    audioParams.inputSampleNoise = static_cast<float>(engine->getInitialJitter());
    audioParams.airNoise         = static_cast<float>(engine->getInitialNoise());
    audioParams.dF_F_mix         = static_cast<float>(engine->getInitialHighFrequencyGain());
    sim->synthesizer().setAudioParameters(audioParams);

    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse *ir = engine->getExhaustSystem(i)->getImpulseResponse();
        if (ir)
            loadImpulseResponse(sim, i, ir->getFilename(),
                                static_cast<float>(ir->getVolume()));
    }

    // -----------------------------------------------------------------------
    // 3. Start synthesizer rendering thread
    // -----------------------------------------------------------------------
    sim->startAudioRenderingThread();

    // -----------------------------------------------------------------------
    // 4. Open audio device
    // -----------------------------------------------------------------------
    HeadlessState state;
    state.sim        = sim;
    state.engine     = engine;
    state.sampleRate = sampleRate;
    state.throttle.store(initThrottle);

    ma_device_config maCfg   = ma_device_config_init(ma_device_type_playback);
    maCfg.playback.format    = ma_format_s16;
    maCfg.playback.channels  = 1;
    maCfg.sampleRate         = static_cast<ma_uint32>(sampleRate);
    maCfg.dataCallback       = audioCallback;
    maCfg.pUserData          = &state;
    maCfg.periodSizeInFrames = 512; // ~11 ms at 44100 Hz

    ma_device maDevice;
    if (ma_device_init(nullptr, &maCfg, &maDevice) != MA_SUCCESS) {
        std::cerr << "[headless] ERROR: failed to open audio device\n";
        sim->endAudioRenderingThread();
        return 1;
    }
    if (ma_device_start(&maDevice) != MA_SUCCESS) {
        std::cerr << "[headless] ERROR: failed to start audio device\n";
        ma_device_uninit(&maDevice);
        sim->endAudioRenderingThread();
        return 1;
    }

    // -----------------------------------------------------------------------
    // Default operating profile for EV use case
    // -----------------------------------------------------------------------
    state.mode.store((int)ControlMode::Hybrid, std::memory_order_relaxed);
    // Hybrid uses direct crankshaft RPM control; neutral removes drivetrain load.
    // The autostart sequence selects gear 1 after 2 s, but we immediately
    // override that once the gear-selection window passes -- so set neutral now
    // and also schedule it again via pendingGear after the loop starts.
    sim->getTransmission()->changeGear(-1);

    if (state.autostart.load()) {
        // Ignition ON immediately
        engine->getIgnitionModule()->m_enabled = true;
        // Starter ON -- held for 2 s then released in the main loop
        sim->m_starterMotor.m_enabled = true;
        std::cerr << "[headless] Autostart: ignition ON, starter ON (2 s)\n";
    }

    std::cerr << "[headless] Engine: "  << engine->getName() << "\n";
    std::cerr << "[headless] Audio: "   << sampleRate << " Hz, period=512 frames\n";
    std::cerr << "[headless] Control: stdin or UDP port " << udpPort << "\n";
    std::cerr << "[headless] Commands: throttle|pedal|brake|speed|setrpm|torque"
                 " mode[throttle|rpm|hybrid] ignition|starter|gear|autostart status quit\n";
    std::cerr << "[headless] Backfire: backfire on|off|chance|volume|rpm_min|test\n";

    // -----------------------------------------------------------------------
    // 5. Control threads
    // -----------------------------------------------------------------------
    std::thread stdinThr(stdinThread, std::ref(state));
    std::thread udpThr(udpThread, std::ref(state), udpPort);

    // -----------------------------------------------------------------------
    // 6. Main simulation loop
    // -----------------------------------------------------------------------
    using Clock   = std::chrono::steady_clock;
    using Seconds = std::chrono::duration<double>;

    constexpr double TARGET_FPS = 60.0;
    constexpr double FRAME_DT   = 1.0 / TARGET_FPS;
    constexpr double DT_MIN     = 1.0 / 200.0;
    constexpr double DT_MAX     = 1.0 / 30.0;

    auto loopStart  = Clock::now();
    auto prevFrame  = loopStart;
    auto benchEpoch = loopStart;

    double cpuPrev        = processCpuSeconds();
    long long stepCount   = 0;
    int underrunsPrev     = 0;

    // Backfire trigger state (main-thread only -- no atomics needed)
    double prevPedal  = 0.0;
    double bfCooldown = 0.0;

    // Autostart state
    bool starterReleased = !state.autostart.load();

    // RPM PI controller state (main-thread only)
    double rpmIntegral  = 0.0;
    double rpmPrevError = 0.0;

    // sweep_rpm state (main-thread only)
    bool   sweepActive = false;
    double sweepFrom   = 0.0, sweepTo = 0.0, sweepDur = 0.0;
    Clock::time_point sweepStart = loopStart;

    // Audio diagnostic snapshot -- updated every 1 s
    auto audioSnapEpoch = loopStart;

    // Per-epoch accumulators (main-thread only)
    int      epochGenSamples = 0;
    float    epochPeak       = 0.0f;
    float    epochSumSq      = 0.0f;
    int      epochZeroCross  = 0;
    uint32_t epochHash       = 0;
    int16_t  diagPrevSample  = 0;

    engine->setSpeedControl(initThrottle);

    while (state.running.load()) {
        const auto   frameStart = Clock::now();
        const double wallDt     = std::max(DT_MIN,
            std::min(DT_MAX, Seconds(frameStart - prevFrame).count()));
        prevFrame = frameStart;

        // ------------------------------------------------------------------
        // Dispatch pending one-shot commands (safe: main thread owns engine objects)
        // ------------------------------------------------------------------
        {
            const int pi = state.pendingIgnition.exchange(-1, std::memory_order_relaxed);
            if (pi != -1) engine->getIgnitionModule()->m_enabled = (pi == 1);

            const int ps = state.pendingStarter.exchange(-1, std::memory_order_relaxed);
            if (ps != -1) sim->m_starterMotor.m_enabled = (ps == 1);

            const int pg = state.pendingGear.exchange(-2, std::memory_order_relaxed);
            if (pg != -2) sim->getTransmission()->changeGear(pg);
        }

        // ------------------------------------------------------------------
        // Autostart timer: release starter and select gear 1 after 2 s
        // ------------------------------------------------------------------
        if (!starterReleased) {
            const double elapsed = Seconds(frameStart - loopStart).count();
            if (elapsed >= 2.0) {
                sim->m_starterMotor.m_enabled = false;
                // In hybrid/rpm modes keep neutral so the drivetrain doesn't
                // resist direct RPM control; throttle mode uses gear 1.
                const ControlMode curMode = static_cast<ControlMode>(
                    state.mode.load(std::memory_order_relaxed));
                if (curMode == ControlMode::Throttle) {
                    sim->getTransmission()->changeGear(0);
                    std::cerr << "[headless] Autostart: starter OFF, gear 1 selected\n";
                } else {
                    sim->getTransmission()->changeGear(-1);
                    std::cerr << "[headless] Autostart: starter OFF, gear neutral (hybrid/rpm mode)\n";
                }
                starterReleased = true;
            }
        }

        // ------------------------------------------------------------------
        // sweep_rpm: linearly ramp targetRpm, auto-switches to rpm mode
        // ------------------------------------------------------------------
        if (state.sweepTrigger.exchange(false, std::memory_order_acquire)) {
            sweepFrom  = state.sweepFromRpm.load(std::memory_order_relaxed);
            sweepTo    = state.sweepToRpm.load(std::memory_order_relaxed);
            sweepDur   = state.sweepSeconds.load(std::memory_order_relaxed);
            sweepStart = frameStart;
            sweepActive  = true;
            rpmIntegral  = 0.0;
            rpmPrevError = 0.0;
            state.mode.store((int)ControlMode::Rpm, std::memory_order_relaxed);
            // Neutral removes vehicle inertia: engine reaches target RPM in ~3 s
            sim->getTransmission()->changeGear(-1);
            std::cerr << "[headless] Sweep: " << sweepFrom << " -> "
                      << sweepTo << " RPM over " << sweepDur << "s (gear: neutral)\n";
        }
        if (sweepActive) {
            const double t = Seconds(frameStart - sweepStart).count();
            if (t >= sweepDur) {
                state.targetRpm.store(sweepTo, std::memory_order_relaxed);
                sweepActive = false;
                std::cerr << "[headless] Sweep complete\n";
            } else {
                const double frac = t / sweepDur;
                state.targetRpm.store(
                    sweepFrom + (sweepTo - sweepFrom) * frac,
                    std::memory_order_relaxed);
            }
        }

        // ------------------------------------------------------------------
        // Compute setSpeedControl from active mode.
        //
        // DirectThrottleLinkage: s=0 -> full throttle, s=1 -> idle.
        // rpm mode uses a PI controller so the engine actually reaches the
        // target RPM instead of just mapping it through a static formula.
        // ------------------------------------------------------------------
        const ControlMode mode     = static_cast<ControlMode>(
                                        state.mode.load(std::memory_order_relaxed));
        const double pedal         = state.pedal.load(std::memory_order_relaxed);
        const double brake         = state.brake.load(std::memory_order_relaxed);
        const double speedMph      = state.speedMph.load(std::memory_order_relaxed);
        const double idleRpm       = state.idleRpm.load(std::memory_order_relaxed);
        const double rpmPerMph     = state.rpmPerMph.load(std::memory_order_relaxed);
        const double maxRpm        = state.maxRpm.load(std::memory_order_relaxed);
        const double rpmSpan       = maxRpm - idleRpm;

        // Reset PI integrator whenever we enter rpm mode from a different mode.
        {
            static ControlMode prevMode = ControlMode::Hybrid;
            if (mode == ControlMode::Rpm && prevMode != ControlMode::Rpm) {
                rpmIntegral  = 0.0;
                rpmPrevError = 0.0;
            }
            prevMode = mode;
        }

        double sc;
        switch (mode) {
            case ControlMode::Rpm: {
                const double tRpm    = state.targetRpm.load(std::memory_order_relaxed);
                const double aRpm    = engine->getRpm();
                const double kp      = state.rpmKp.load(std::memory_order_relaxed);
                const double ki      = state.rpmKi.load(std::memory_order_relaxed);
                const double kd      = state.rpmKd.load(std::memory_order_relaxed);
                // Error normalised to [−1..+1] over the full RPM span.
                const double errNorm = (rpmSpan > 0.0)
                    ? (tRpm - aRpm) / rpmSpan : 0.0;
                // PI(D): positive error -> need more RPM -> decrease sc (open throttle)
                rpmIntegral  = std::max(-0.5, std::min(0.5,
                                   rpmIntegral + ki * errNorm * wallDt));
                const double dTerm   = kd * (errNorm - rpmPrevError) / wallDt;
                rpmPrevError = errNorm;
                // Feedforward from target + closed-loop correction
                const double sc_ff   = (rpmSpan > 0.0)
                    ? std::max(0.0, std::min(1.0, 1.0 - (tRpm - idleRpm) / rpmSpan))
                    : 0.5;
                sc = std::max(0.0, std::min(1.0,
                         sc_ff - kp * errNorm - rpmIntegral - dTerm));
                break;
            }
            case ControlMode::Hybrid: {
                // EV sound mapping layer -- no drivetrain physics.
                // Speed maps deterministically to crankshaft RPM (set below via
                // v_theta, same path as rpm mode). Pedal controls throttle opening
                // so combustion runs louder/harder at higher pedal. Brake reduces
                // throttle for an engine-braking feel.
                const double hybridRpm = std::max(idleRpm,
                    std::min(maxRpm, idleRpm + speedMph * rpmPerMph));
                // Store the computed target so status can display it.
                state.targetRpm.store(hybridRpm, std::memory_order_relaxed);
                // Throttle: pedal 0 = idle (s=1), pedal 100 = full open (s=0).
                // Brake adds engine-braking by reducing throttle further.
                const double s_pedal = std::max(0.0, std::min(1.0, 1.0 - pedal / 100.0));
                sc = std::min(1.0, s_pedal + brake * 0.3);
                break;
            }
            default: // ControlMode::Throttle -- legacy direct pass-through
                sc = state.throttle.load(std::memory_order_relaxed);
                break;
        }
        engine->setSpeedControl(sc);

        // ------------------------------------------------------------------
        // rpm mode: directly enforce crankshaft angular velocity so that
        // audio pitch tracks targetRpm immediately.
        //
        // Engine-sim's RotationFrictionConstraint resists angular changes but
        // does NOT actively decelerate the engine -- a coasting engine with
        // closed throttle maintains RPM indefinitely.  Directly setting v_theta
        // before each physics step forces combustion to run at the correct
        // frequency, producing genuinely synthesized audio at that RPM.
        // The PI controller still drives throttle for realistic combustion.
        // ------------------------------------------------------------------
        // Both rpm and hybrid modes directly enforce crankshaft angular velocity.
        // targetRpm is already set (rpm mode: from setrpm; hybrid mode: updated
        // inside the switch above from the speed mapping).
        if ((mode == ControlMode::Rpm || mode == ControlMode::Hybrid)
                && engine->getCrankshaftCount() > 0) {
            constexpr double kRpmToRad = 2.0 * 3.14159265358979323846 / 60.0;
            const double tRpm   = state.targetRpm.load(std::memory_order_relaxed);
            const double tOmega = std::max(0.0, tRpm) * kRpmToRad;
            Crankshaft   *cs    = engine->getCrankshaft(0);
            const double sign   = (cs->m_body.v_theta <= 0.0) ? -1.0 : 1.0;
            cs->m_body.v_theta  = sign * tOmega;
        }

        // ------------------------------------------------------------------
        // Physics step
        // ------------------------------------------------------------------
        sim->startFrame(wallDt);
        while (sim->simulateStep()) {}
        sim->endFrame();
        stepCount += sim->getFrameIterationCount();

        // ------------------------------------------------------------------
        // Drain synthesizer output into the SPSC ring buffer.
        // This is the GUI-equivalent step: readAudioOutput() on the main
        // thread, not the audio callback.  The ring provides headroom so the
        // callback always has samples even when this runs slower than 86 Hz.
        // ------------------------------------------------------------------
        {
            static int16_t drainBuf[2048];
            // Pull everything available (up to 2048 per call; loop in case more)
            int drained;
            do {
                drained = sim->readAudioOutput(2048, drainBuf);
                if (drained > 0) {
                    state.audioRing.write(drainBuf, drained);

                    // Accumulate diagnostics (main-thread only, no atomics needed)
                    epochGenSamples += drained;
                    float pk = 0.0f;
                    for (int i = 0; i < drained; ++i) {
                        const float v  = drainBuf[i] * (1.0f / 32767.0f);
                        const float av = v < 0.0f ? -v : v;
                        if (av > pk) pk = av;
                        if (av > epochPeak) epochPeak = av;
                        epochSumSq += v * v;
                        if ((drainBuf[i] >= 0) != (diagPrevSample >= 0)) ++epochZeroCross;
                        diagPrevSample = drainBuf[i];
                        // Rolling hash -- detects if same audio is repeating
                        epochHash = (epochHash << 5u) ^ (epochHash >> 27u)
                                    ^ static_cast<uint32_t>(static_cast<uint16_t>(drainBuf[i]));
                    }
                    (void)pk;
                }
            } while (drained == 2048); // keep draining if the buffer was full
        }

        // ------------------------------------------------------------------
        // Backfire trigger detection (uses pedal regardless of mode)
        // ------------------------------------------------------------------
        if (state.backfire.enabled.load(std::memory_order_relaxed)) {
            bfCooldown -= wallDt;

            const double currentRpm = engine->getRpm();
            const bool   pedalDrop  = (prevPedal > 65.0 && pedal < 15.0);
            const bool   rapidDrop  = (prevPedal - pedal > 30.0);
            const bool   rpmOk      = (currentRpm >
                static_cast<double>(state.backfire.rpmMin.load(std::memory_order_relaxed)));
            const bool   brakeOk    = (brake > 0.2);

            if (pedalDrop && rpmOk && (brakeOk || rapidDrop) && bfCooldown <= 0.0) {
                const double r = static_cast<double>(std::rand()) /
                                 (static_cast<double>(RAND_MAX) + 1.0);
                if (r < static_cast<double>(
                        state.backfire.chance.load(std::memory_order_relaxed))) {
                    triggerBackfire(state);
                    bfCooldown = 0.120; // 120 ms minimum gap between backfires
                }
            }
            prevPedal = pedal;
        }

        // ------------------------------------------------------------------
        // Benchmark report every 5 s
        // ------------------------------------------------------------------
        if (benchmark) {
            const auto   now          = Clock::now();
            const double benchElapsed = Seconds(now - benchEpoch).count();
            if (benchElapsed >= 5.0) {
                const double simFreq  = static_cast<double>(sim->getSimulationFrequency());
                const double simulated = stepCount / simFreq;
                const double rtf       = benchElapsed > 0.0 ? simulated / benchElapsed : 0.0;

                const double cpuNow   = processCpuSeconds();
                const double cpuPct   = benchElapsed > 0.0
                    ? (cpuNow - cpuPrev) / benchElapsed * 100.0 : 0.0;
                cpuPrev = cpuNow;

                const double latencyMs  = sim->getSynthesizerInputLatency() * 1000.0;
                const double latTarget  = sim->getSynthesizerInputLatencyTarget();
                const double bufFillPct = (latTarget > 0.0)
                    ? std::min(1.0, sim->getSynthesizerInputLatency() / latTarget) * 100.0
                    : 0.0;

                const int undrNow   = state.underruns.load();
                const int undrEpoch = undrNow - underrunsPrev;
                underrunsPrev = undrNow;

                const int bfEpoch = state.backfireEpochCount.exchange(
                    0, std::memory_order_relaxed);

                std::fprintf(stderr,
                    "[BENCH] rtf=%.3f underruns=%d buf=%.0f%% latency=%.1fms"
                    " cpu=%.1f%% steps/s=%lld backfires=%d\n",
                    rtf, undrEpoch, bufFillPct, latencyMs, cpuPct,
                    static_cast<long long>(benchElapsed > 0.0
                        ? stepCount / benchElapsed : 0),
                    bfEpoch);

                benchEpoch = now;
                stepCount  = 0;
            }
        }

        // ------------------------------------------------------------------
        // Audio diagnostic snapshot -- once per second
        // ------------------------------------------------------------------
        {
            const auto   now         = Clock::now();
            const double snapElapsed = Seconds(now - audioSnapEpoch).count();
            if (snapElapsed >= 1.0) {
                const int cbRead = state.cbSamplesRead.exchange(0, std::memory_order_relaxed);

                state.snapshotRms       = (epochGenSamples > 0)
                    ? std::sqrt(epochSumSq / static_cast<float>(epochGenSamples)) : 0.0f;
                state.snapshotPeak      = epochPeak;
                state.audioHash         = epochHash;
                state.zeroCrossRate     = (snapElapsed > 0.0)
                    ? static_cast<float>(epochZeroCross / snapElapsed) : 0.0f;
                state.samplesGenPerSec  = static_cast<int>(epochGenSamples / snapElapsed);
                state.samplesReadPerSec = static_cast<int>(cbRead / snapElapsed);
                state.synthBufFill      = static_cast<int>(
                    sim->getSynthesizerInputLatency() * static_cast<double>(sampleRate));

                if (state.audioDebug.load(std::memory_order_relaxed)) {
                    std::fprintf(stderr,
                        "[AUDIO] rpm=%.0f throttle=%.3f"
                        " rms=%.4f peak=%.4f leveler=%.5f"
                        " gen/s=%d read/s=%d ring=%d zcr=%.0f hash=%08X underruns=%d\n",
                        engine->getRpm(),
                        engine->getThrottle(),
                        state.snapshotRms,
                        state.snapshotPeak,
                        sim->synthesizer().getLevelerGain(),
                        state.samplesGenPerSec,
                        state.samplesReadPerSec,
                        state.audioRing.available(),
                        static_cast<double>(state.zeroCrossRate),
                        state.audioHash,
                        state.underruns.load());
                }

                // Reset epoch accumulators
                epochGenSamples = 0;
                epochPeak       = 0.0f;
                epochSumSq      = 0.0f;
                epochZeroCross  = 0;
                epochHash       = 0;
                audioSnapEpoch  = now;
            }
        }

        // Pace at ~60 Hz
        const double frameTook = Seconds(Clock::now() - frameStart).count();
        const double sleepSec  = FRAME_DT - frameTook;
        if (sleepSec > 5e-4)
            std::this_thread::sleep_for(Seconds(sleepSec - 5e-4));
    }

    // -----------------------------------------------------------------------
    // 7. Cleanup
    // -----------------------------------------------------------------------
    std::cerr << "[headless] Shutting down...\n";

    ma_device_stop(&maDevice);
    ma_device_uninit(&maDevice);

    sim->endAudioRenderingThread();

    stdinThr.detach(); // may be blocked on getline
    udpThr.join();

    sim->releaseSimulation();
    delete sim;
    engine->destroy();
    delete engine;
    delete vehicle;
    delete transmission;

    std::cerr << "[headless] Done.\n";
    return 0;
}
