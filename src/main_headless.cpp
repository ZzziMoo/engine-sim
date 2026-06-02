// engine-sim headless entry point
// Targets ARM64 Linux (Raspberry Pi 4) / EV-Tesla sound project.
// Windows MSVC is the build-test platform; Pi 4 ARM64 Linux is production.
//
// Build:  cmake -B build -DHEADLESS=ON -DDISCORD_ENABLED=OFF
// Usage:  engine-sim-headless --script assets/engines/.../engine.mr
//             [--port 9999] [--throttle 0.5] [--benchmark]
//             [--sample-rate 44100] [--period 512]
//             [--write-wav out.wav] [--wav-seconds 30]
//             [--no-audio-device]
//
// Diagnostic flags:
//   --write-wav out.wav      capture synthesis output to a WAV file (pre-ring)
//   --write-live-wav out.wav capture hardware output to a WAV file (post-ring)
//   --wav-seconds 30         WAV duration (default 30 s), then exit
//   --no-audio-device        render to WAV only, no speaker output
//   --period 512|1024|2048   miniaudio period size (default 512)
//
// Comparison test:
//   If synth.wav is clean but live.wav crackles -> ring/callback path corrupting audio
//
// Commands:  tone <Hz>|off   -- replace engine audio with a pure sine wave

#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
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
#include <cstdio>
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
    Throttle = 0,
    Rpm      = 1,
    Hybrid   = 2
};

// ---------------------------------------------------------------------------
// Virtual gear table -- Hybrid mode RPM calculator (no v_theta forcing)
// ---------------------------------------------------------------------------
struct VGear { double minMph, maxMph, minRpm, maxRpm; };
static const VGear kVGears[5] = {
    {  0.0,  30.0,  900.0, 4500.0 },
    { 20.0,  50.0, 1800.0, 5000.0 },
    { 40.0,  75.0, 2200.0, 5200.0 },
    { 65.0, 105.0, 2500.0, 4800.0 },
    { 90.0, 130.0, 2500.0, 4200.0 },
};
static const double kUpshiftOffsetMph[5] = { -8.0, -5.0, -5.0, -5.0, -5.0 };

static double vGearRpm(int gear, double speedMph) {
    const VGear &g = kVGears[gear - 1];
    const double span = g.maxMph - g.minMph;
    if (span <= 0.0) return g.minRpm;
    const double t = (speedMph - g.minMph) / span;
    const double tc = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    return g.minRpm + tc * (g.maxRpm - g.minRpm);
}

// ---------------------------------------------------------------------------
// WAV writer -- captures PCM int16 mono to a file for offline analysis.
// open() writes a placeholder header; close() patches it with final sizes.
// ---------------------------------------------------------------------------
struct WavWriter {
    FILE    *f              = nullptr;
    int      sampleRate     = 44100;
    uint32_t samplesWritten = 0;

    bool open(const char *path, int sr) {
        sampleRate = sr;
        f = std::fopen(path, "wb");
        if (!f) return false;
        // 44-byte placeholder; patched by close()
        uint8_t hdr[44] = {};
        std::fwrite(hdr, 1, 44, f);
        samplesWritten = 0;
        return true;
    }

    void write(const int16_t *data, int n) {
        if (!f || n <= 0) return;
        std::fwrite(data, sizeof(int16_t), static_cast<size_t>(n), f);
        samplesWritten += static_cast<uint32_t>(n);
    }

    void close() {
        if (!f) return;
        const uint32_t dataBytes = samplesWritten * 2;
        const uint32_t riffSize  = 36 + dataBytes;
        const uint32_t byteRate  = static_cast<uint32_t>(sampleRate) * 2;

        uint8_t hdr[44];
        auto wr2 = [&](int off, uint16_t v) {
            hdr[off] = v & 0xFF; hdr[off+1] = (v >> 8) & 0xFF;
        };
        auto wr4 = [&](int off, uint32_t v) {
            hdr[off]=v&0xFF; hdr[off+1]=(v>>8)&0xFF;
            hdr[off+2]=(v>>16)&0xFF; hdr[off+3]=(v>>24)&0xFF;
        };
        hdr[0]='R'; hdr[1]='I'; hdr[2]='F'; hdr[3]='F';
        wr4(4,  riffSize);
        hdr[8]='W'; hdr[9]='A'; hdr[10]='V'; hdr[11]='E';
        hdr[12]='f'; hdr[13]='m'; hdr[14]='t'; hdr[15]=' ';
        wr4(16, 16);          // fmt chunk size
        wr2(20, 1);           // PCM
        wr2(22, 1);           // mono
        wr4(24, static_cast<uint32_t>(sampleRate));
        wr4(28, byteRate);
        wr2(32, 2);           // block align
        wr2(34, 16);          // bits per sample
        hdr[36]='d'; hdr[37]='a'; hdr[38]='t'; hdr[39]='a';
        wr4(40, dataBytes);

        std::rewind(f);
        std::fwrite(hdr, 1, 44, f);
        std::fclose(f);
        f = nullptr;
    }

    bool isOpen() const { return f != nullptr; }
    ~WavWriter() { close(); }
};

// ---------------------------------------------------------------------------
// SPSC audio ring -- decouples 60 Hz synthesis from ~86 Hz audio callback.
// Producer: main loop. Consumer: audio callback (and captureRing drain).
// ---------------------------------------------------------------------------
struct AudioRing {
    static constexpr int kSize = 16384;
    static constexpr int kMask = kSize - 1;

    int16_t data[kSize] {};
    std::atomic<int> wIdx     { 0 };
    std::atomic<int> rIdx     { 0 };
    std::atomic<int> overruns { 0 }; // samples dropped due to ring full

    int available() const {
        return (wIdx.load(std::memory_order_acquire) -
                rIdx.load(std::memory_order_relaxed) + kSize) & kMask;
    }
    int freeSlots() const { return kSize - 1 - available(); }

    // Returns how many samples were actually written (< n if ring was full).
    // Counts truncated samples in overruns.
    int write(const int16_t *src, int n) {
        const int free = freeSlots();
        if (n > free) {
            overruns.fetch_add(n - free, std::memory_order_relaxed);
            n = free;
        }
        const int w = wIdx.load(std::memory_order_relaxed);
        for (int i = 0; i < n; ++i)
            data[(w + i) & kMask] = src[i];
        wIdx.store((w + n) & kMask, std::memory_order_release);
        return n;
    }

    int read(int16_t *dst, int n) {
        const int avail = available();
        if (n > avail) n = avail;
        const int r = rIdx.load(std::memory_order_relaxed);
        for (int i = 0; i < n; ++i)
            dst[i] = data[(r + i) & kMask];
        rIdx.store((r + n) & kMask, std::memory_order_release);
        return n;
    }

    int getAndClearOverruns() {
        return overruns.exchange(0, std::memory_order_relaxed);
    }
};

// ---------------------------------------------------------------------------
// Shared application state
// ---------------------------------------------------------------------------
struct HeadlessState {
    Simulator *sim    = nullptr;
    Engine    *engine = nullptr;
    int sampleRate    = 44100;

    std::atomic<int>  underruns { 0 };
    std::atomic<bool> running   { true };

    std::atomic<double> throttle { 0.5 };

    std::atomic<int>    mode     { (int)ControlMode::Throttle };
    std::atomic<double> pedal    { 0.0 };
    std::atomic<double> brake    { 0.0 };
    std::atomic<double> speedMph { 0.0 };
    std::atomic<double> targetRpm{ 900.0 };

    std::atomic<double> idleRpm  { 900.0  };
    std::atomic<double> maxRpm   { 8000.0 };

    std::atomic<int>    vGearCurrent { 1  };
    std::atomic<int>    vGearManual  { -1 };

    std::atomic<bool> autostart       { true };
    std::atomic<int>  pendingIgnition { -1 };
    std::atomic<int>  pendingStarter  { -1 };
    std::atomic<int>  pendingGear     { -2 };

    std::atomic<bool>  backfireEnabled { false };
    std::atomic<float> backfireChance  { 0.35f };
    std::atomic<float> backfireVolume  { 0.7f  };
    std::atomic<int>   backfireRpmMin  { 3000  };

    // Tone generator (replaces engine audio when active)
    std::atomic<bool>   toneActive { false };
    std::atomic<double> toneFreq   { 440.0 };

    AudioRing audioRing;
    std::atomic<int> cbSamplesRead { 0 };

    // Live WAV capture: callback writes here; main loop drains to WAV file.
    AudioRing         captureRing;
    std::atomic<bool> captureActive { false };

    // Diagnostic snapshots (written by main loop, read by status/audio_debug)
    float    snapshotRms        = 0.0f;
    float    snapshotPeak       = 0.0f;
    int      synthBufFill       = 0;
    int      samplesGenPerSec   = 0;   // samples drained from synth
    int      samplesReadPerSec  = 0;   // samples consumed by callback
    uint32_t audioHash          = 0;
    float    zeroCrossRate      = 0.0f;
    int      synthClipsPerSec   = 0;   // synth samples at INT16 ceiling/floor
    int      ringOverrunsPerSec = 0;   // samples dropped by audioRing.write() truncation

    std::atomic<bool> audioDebug { false };
};

// ---------------------------------------------------------------------------
// miniaudio callback -- reads from SPSC ring only, no synthesizer access
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

    // Capture entire hardware output buffer (including underrun zeros) for live WAV.
    if (s->captureActive.load(std::memory_order_relaxed))
        s->captureRing.write(out, static_cast<int>(frameCount));
}

// ---------------------------------------------------------------------------
// Command parser
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
        if (ss >> v) state.throttle.store(clamp01(v), std::memory_order_relaxed);

    } else if (cmd == "pedal") {
        double v = 0.0;
        if (ss >> v) state.pedal.store(clamp100(v), std::memory_order_relaxed);

    } else if (cmd == "brake") {
        double v = 0.0;
        if (ss >> v) state.brake.store(clamp01(v), std::memory_order_relaxed);

    } else if (cmd == "speed") {
        double v = 0.0;
        if (ss >> v) state.speedMph.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "setrpm") {
        double v = 900.0;
        if (ss >> v) state.targetRpm.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "mode") {
        std::string sub;
        if (!(ss >> sub)) return;
        if      (sub == "throttle")
            state.mode.store((int)ControlMode::Throttle, std::memory_order_relaxed);
        else if (sub == "rpm")
            state.mode.store((int)ControlMode::Rpm,      std::memory_order_relaxed);
        else if (sub == "hybrid")
            state.mode.store((int)ControlMode::Hybrid,   std::memory_order_relaxed);

    } else if (cmd == "idle_rpm") {
        double v = 900.0;
        if (ss >> v) state.idleRpm.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "max_rpm") {
        double v = 8000.0;
        if (ss >> v) state.maxRpm.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "gear") {
        int n = 1;
        if (ss >> n) {
            const int apiGear = (n <= 0) ? -1 : n - 1;
            state.pendingGear.store(apiGear, std::memory_order_relaxed);
        }

    } else if (cmd == "vgear") {
        int n = 0;
        if (ss >> n) state.vGearManual.store(n, std::memory_order_relaxed);

    } else if (cmd == "ignition") {
        std::string sub;
        if (ss >> sub)
            state.pendingIgnition.store(sub == "on" ? 1 : 0, std::memory_order_relaxed);

    } else if (cmd == "starter") {
        std::string sub;
        if (ss >> sub)
            state.pendingStarter.store(sub == "on" ? 1 : 0, std::memory_order_relaxed);

    } else if (cmd == "autostart") {
        std::string sub;
        if (ss >> sub)
            state.autostart.store(sub == "on", std::memory_order_relaxed);

    } else if (cmd == "tone") {
        // "tone 440" -- replace engine audio with pure sine at given Hz
        // "tone off" -- restore engine audio
        std::string sub;
        if (!(ss >> sub)) return;
        if (sub == "off") {
            state.toneActive.store(false, std::memory_order_relaxed);
            std::cerr << "[headless] Tone: off (engine audio restored)\n";
        } else {
            double freq = 440.0;
            try { freq = std::stod(sub); } catch (...) {}
            if (freq > 0.0) {
                state.toneFreq.store(freq, std::memory_order_relaxed);
                state.toneActive.store(true, std::memory_order_relaxed);
                std::cerr << "[headless] Tone: " << freq << " Hz\n";
            }
        }

    } else if (cmd == "backfire") {
        std::string sub;
        if (!(ss >> sub)) return;
        if      (sub == "on")  state.backfireEnabled.store(true,  std::memory_order_relaxed);
        else if (sub == "off") state.backfireEnabled.store(false, std::memory_order_relaxed);
        else if (sub == "chance") {
            double v = 0.35;
            if (ss >> v)
                state.backfireChance.store(
                    static_cast<float>(clamp01(v)), std::memory_order_relaxed);
        } else if (sub == "volume") {
            double v = 0.7;
            if (ss >> v)
                state.backfireVolume.store(
                    static_cast<float>(clamp01(v)), std::memory_order_relaxed);
        } else if (sub == "rpm_min") {
            int v = 3000;
            if (ss >> v) state.backfireRpmMin.store(v < 0 ? 0 : v, std::memory_order_relaxed);
        }

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
        const double rpm       = state.engine ? state.engine->getRpm() : 0.0;
        const double crankRpm  = (state.engine && state.engine->getCrankshaftCount() > 0)
            ? std::abs(units::toRpm(state.engine->getCrankshaft(0)->m_body.v_theta))
            : 0.0;
        const double throttlePos = state.engine ? state.engine->getThrottle() : 0.0;
        const bool   ignition  = state.engine
            ? state.engine->getIgnitionModule()->m_enabled : false;
        const bool   starter   = state.sim ? state.sim->m_starterMotor.m_enabled : false;
        const int    gearApi   = state.sim ? state.sim->getTransmission()->getGear() : -1;
        const std::string gearStr = (gearApi == -1) ? "N" : std::to_string(gearApi + 1);

        std::cout
            << "mode="               << modeStr                                     << "\n"
            << "ignition="           << (ignition ? "on" : "off")                   << "\n"
            << "starter="            << (starter  ? "on" : "off")                   << "\n"
            << "gear="               << gearStr                                      << "\n"
            << "throttle_sc="        << state.throttle.load()                       << "\n"
            << "throttle_position="  << throttlePos                                  << "\n"
            << "pedal="              << state.pedal.load()                          << "\n"
            << "brake="              << state.brake.load()                          << "\n"
            << "speed_mph="          << state.speedMph.load()                       << "\n"
            << "engine_rpm="         << rpm                                          << "\n"
            << "crankshaft_rpm="     << crankRpm                                     << "\n"
            << "target_rpm="         << state.targetRpm.load()                      << "\n"
            << "idle_rpm="           << state.idleRpm.load()                        << "\n"
            << "max_rpm="            << state.maxRpm.load()                         << "\n"
            << "vgear="              << state.vGearCurrent.load()                   << "\n"
            << "tone="               << (state.toneActive.load() ? "on" : "off")    << "\n"
            << "tone_freq="          << state.toneFreq.load()                       << "\n"
            << "audio_rms="          << state.snapshotRms                            << "\n"
            << "audio_peak="         << state.snapshotPeak                           << "\n"
            << "audio_hash="         << state.audioHash                              << "\n"
            << "audio_zcr="          << state.zeroCrossRate                          << "\n"
            << "audio_leveler_gain=" << state.sim->synthesizer().getLevelerGain()   << "\n"
            << "samples_gen_per_sec="   << state.samplesGenPerSec                   << "\n"
            << "samples_read_per_sec="  << state.samplesReadPerSec                  << "\n"
            << "ring_fill="          << state.audioRing.available()                 << "\n"
            << "synth_input_buf="    << state.synthBufFill                          << "\n"
            << "synth_clips_per_sec="    << state.synthClipsPerSec                   << "\n"
            << "ring_overruns_per_sec=" << state.ringOverrunsPerSec                 << "\n"
            << "backfire="           << (state.backfireEnabled.load() ? "on":"off") << "\n"
            << "underruns="          << state.underruns.load()                      << "\n"
            << std::flush;
    }
}

// ---------------------------------------------------------------------------
// stdin / UDP threads
// ---------------------------------------------------------------------------
static void stdinThread(HeadlessState &state) {
    std::string line;
    while (state.running.load() && std::getline(std::cin, line))
        applyCommand(line, state);
}

#ifdef __unix__
static void udpThread(HeadlessState &state, int port) {
    const int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "[headless] UDP socket failed\n";
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
    struct timeval tv{}; tv.tv_sec = 1;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    std::cerr << "[headless] UDP control on port " << port << "\n";
    char buf[256];
    while (state.running.load()) {
        const ssize_t n = recv(sock, buf, sizeof(buf) - 1, 0);
        if (n > 0) { buf[n] = '\0'; applyCommand(std::string(buf), state); }
    }
    close(sock);
}
#else
static void udpThread(HeadlessState &, int) {}
#endif

// ---------------------------------------------------------------------------
// Load impulse response
// ---------------------------------------------------------------------------
static void loadImpulseResponse(
        Simulator *sim, int index, const std::string &path, float volume)
{
    if (path.empty()) return;
    ma_decoder_config cfg = ma_decoder_config_init(ma_format_s16, 1, 44100);
    ma_uint64 frameCount  = 0;
    void     *pData       = nullptr;
    if (ma_decode_file(path.c_str(), &cfg, &frameCount, &pData) != MA_SUCCESS) {
        std::cerr << "[headless] WARNING: cannot load IR: " << path << "\n";
        return;
    }
    sim->synthesizer().initializeImpulseResponse(
        static_cast<const int16_t *>(pData),
        static_cast<unsigned int>(frameCount), volume, index);
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
    const int         periodFrames = std::stoi(argValue(argc, argv, "--period",      "512"));
    const std::string wavPath      = argValue(argc, argv, "--write-wav",      "");
    const std::string liveWavPath  = argValue(argc, argv, "--write-live-wav", "");
    const int         wavSeconds   = std::stoi(argValue(argc, argv, "--wav-seconds", "30"));
    const bool        noDevice     = argFlag(argc, argv, "--no-audio-device");

    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // -----------------------------------------------------------------------
    // 1. Load engine script
    // -----------------------------------------------------------------------
    Engine       *engine       = nullptr;
    Vehicle      *vehicle      = nullptr;
    Transmission *transmission = nullptr;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    std::string compileTarget = scriptPath;
    std::string wrapperPath;

    auto hasSuffix = [](const std::string &s, const std::string &sfx) {
        return s.size() >= sfx.size() &&
               s.compare(s.size() - sfx.size(), sfx.size(), sfx) == 0;
    };
    if (!hasSuffix(scriptPath, "main.mr")) {
        const auto sep = scriptPath.find_last_of("/\\");
        const std::string dir  = (sep == std::string::npos) ? "." : scriptPath.substr(0, sep);
        const std::string base = (sep == std::string::npos) ? scriptPath
                                                             : scriptPath.substr(sep + 1);
        wrapperPath   = dir + "/__headless_main.mr";
        compileTarget = wrapperPath;
        std::ofstream wf(wrapperPath);
        wf << "import \"engine_sim.mr\"\n"
           << "import \"" << base << "\"\n"
           << "main()\n";
        wf.close();
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

    if (!engine) { std::cerr << "[headless] ERROR: no engine\n"; return 1; }

    if (!vehicle) {
        Vehicle::Parameters vp;
        vp.mass = units::mass(1597, units::kg);
        vp.diffRatio = 3.42;
        vp.tireRadius = units::distance(10, units::inch);
        vp.dragCoefficient = 0.25;
        vp.crossSectionArea =
            units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
        vp.rollingResistance = 2000.0;
        vehicle = new Vehicle; vehicle->initialize(vp);
    }
    if (!transmission) {
        static const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
        Transmission::Parameters tp;
        tp.GearCount = 6; tp.GearRatios = gearRatios;
        tp.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        transmission = new Transmission; transmission->initialize(tp);
    }

    // -----------------------------------------------------------------------
    // 2. Configure simulator
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

    sim->startAudioRenderingThread();

    // -----------------------------------------------------------------------
    // 3. Open audio device (optional)
    // -----------------------------------------------------------------------
    HeadlessState state;
    state.sim        = sim;
    state.engine     = engine;
    state.sampleRate = sampleRate;
    state.throttle.store(initThrottle);

    ma_device maDevice;
    bool deviceOpen = false;

    if (!noDevice) {
        ma_device_config maCfg   = ma_device_config_init(ma_device_type_playback);
        maCfg.playback.format    = ma_format_s16;
        maCfg.playback.channels  = 1;
        maCfg.sampleRate         = static_cast<ma_uint32>(sampleRate);
        maCfg.dataCallback       = audioCallback;
        maCfg.pUserData          = &state;
        maCfg.periodSizeInFrames = static_cast<ma_uint32>(periodFrames);

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
        deviceOpen = true;

        // Print device info for diagnostics
        const char *backendName = ma_get_backend_name(maDevice.pContext->backend);
        const char *fmtName     = ma_get_format_name(maDevice.playback.format);
        std::fprintf(stderr,
            "[DEVICE] backend=%s name=\"%s\" rate=%u ch=%u fmt=%s"
            " period=%u periods=%u\n",
            backendName,
            maDevice.playback.name,
            maDevice.sampleRate,
            maDevice.playback.channels,
            fmtName,
            maDevice.playback.internalPeriodSizeInFrames,
            maDevice.playback.internalPeriods);
    } else {
        std::cerr << "[headless] No audio device (WAV-only mode)\n";
    }

    // -----------------------------------------------------------------------
    // 4. Open WAV file if requested
    // -----------------------------------------------------------------------
    WavWriter wavWriter;
    WavWriter liveWavWriter;
    const uint32_t wavSamplesTarget =
        static_cast<uint32_t>(wavSeconds) * static_cast<uint32_t>(sampleRate);
    bool wavDone     = false;
    bool liveWavDone = false;

    if (!wavPath.empty()) {
        if (!wavWriter.open(wavPath.c_str(), sampleRate)) {
            std::cerr << "[headless] ERROR: cannot open WAV: " << wavPath << "\n";
        } else {
            std::fprintf(stderr, "[WAV] Recording %d s to %s\n",
                         wavSeconds, wavPath.c_str());
        }
    }
    if (!liveWavPath.empty()) {
        if (!liveWavWriter.open(liveWavPath.c_str(), sampleRate)) {
            std::cerr << "[headless] ERROR: cannot open live WAV: " << liveWavPath << "\n";
        } else {
            state.captureActive.store(true, std::memory_order_relaxed);
            std::fprintf(stderr, "[LIVE-WAV] Capturing %d s (post-ring) to %s\n",
                         wavSeconds, liveWavPath.c_str());
        }
    }

    // -----------------------------------------------------------------------
    // 5. Autostart
    // -----------------------------------------------------------------------
    if (state.autostart.load()) {
        engine->getIgnitionModule()->m_enabled = true;
        sim->m_starterMotor.m_enabled          = true;
        std::cerr << "[headless] Autostart: ignition ON, starter ON (2 s)\n";
    }

    std::fprintf(stderr,
        "[headless] Engine: %s | audio: %d Hz period=%d | %s\n",
        engine->getName().c_str(), sampleRate, periodFrames,
        noDevice ? "no-device (WAV only)" : "device open");
    std::cerr << "[headless] Commands: throttle|pedal|brake|speed|setrpm"
                 " mode ignition|starter|gear|vgear autostart"
                 " tone backfire audio_debug status quit\n";

    // -----------------------------------------------------------------------
    // 6. Control threads
    // -----------------------------------------------------------------------
    std::thread stdinThr(stdinThread, std::ref(state));
    std::thread udpThr(udpThread, std::ref(state), udpPort);

    // -----------------------------------------------------------------------
    // 7. Main simulation loop
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

    double cpuPrev      = processCpuSeconds();
    long long stepCount = 0;
    int underrunsPrev   = 0;

    bool starterReleased = !state.autostart.load();

    int    vGear        = 1;
    double shiftDropRem = 0.0;
    double shiftDropAmt = 0.0;

    // Tone generator phase (main-thread only)
    double tonePhase = 0.0;

    auto     audioSnapEpoch  = loopStart;
    int      epochGenSamples = 0;
    float    epochPeak       = 0.0f;
    float    epochSumSq      = 0.0f;
    int      epochZeroCross  = 0;
    uint32_t epochHash       = 0;
    int16_t  diagPrevSample  = 0;
    int      epochSynthClips = 0;

    engine->setSpeedControl(initThrottle);

    while (state.running.load()) {
        const auto   frameStart = Clock::now();
        const double wallDt     = std::max(DT_MIN,
            std::min(DT_MAX, Seconds(frameStart - prevFrame).count()));
        prevFrame = frameStart;

        // ------------------------------------------------------------------
        // Pending one-shot commands
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
        // Autostart timer
        // ------------------------------------------------------------------
        if (!starterReleased) {
            if (Seconds(frameStart - loopStart).count() >= 2.0) {
                sim->m_starterMotor.m_enabled = false;
                sim->getTransmission()->changeGear(0);
                starterReleased = true;
                std::cerr << "[headless] Autostart: starter OFF, gear 1\n";
            }
        }

        // ------------------------------------------------------------------
        // Control mode -> setSpeedControl (no v_theta forcing)
        // ------------------------------------------------------------------
        const ControlMode mode  = static_cast<ControlMode>(
                                      state.mode.load(std::memory_order_relaxed));
        const double pedal      = state.pedal.load(std::memory_order_relaxed);
        const double brake      = state.brake.load(std::memory_order_relaxed);
        const double speedMph   = state.speedMph.load(std::memory_order_relaxed);
        const double idleRpm    = state.idleRpm.load(std::memory_order_relaxed);
        const double maxRpm     = state.maxRpm.load(std::memory_order_relaxed);
        const double rpmSpan    = maxRpm - idleRpm;

        double sc;
        switch (mode) {
        case ControlMode::Rpm: {
            const double tRpm = state.targetRpm.load(std::memory_order_relaxed);
            sc = (rpmSpan > 0.0)
                ? std::max(0.0, std::min(1.0, 1.0 - (tRpm - idleRpm) / rpmSpan)) : 0.5;
            break;
        }
        case ControlMode::Hybrid: {
            const int manualGear = state.vGearManual.load(std::memory_order_relaxed);
            if (manualGear >= 1 && manualGear <= 5) {
                vGear = manualGear;
            } else {
                const double upshiftBonus = (pedal > 75.0) ? 8.0 : (pedal < 30.0) ? -3.0 : 0.0;
                const double upshiftMph   = kVGears[vGear-1].maxMph
                                            + kUpshiftOffsetMph[vGear-1] + upshiftBonus;
                if (vGear < 5 && speedMph > upshiftMph) {
                    ++vGear;
                    shiftDropAmt = 800.0 + static_cast<double>(std::rand() % 401);
                    shiftDropRem = 0.150 + static_cast<double>(std::rand() % 101) * 0.001;
                } else if (vGear > 1 && speedMph < kVGears[vGear-1].minMph - 2.0) {
                    --vGear; shiftDropRem = 0.0;
                }
            }
            state.vGearCurrent.store(vGear, std::memory_order_relaxed);
            double tRpm = vGearRpm(vGear, speedMph);
            if (shiftDropRem > 0.0) {
                const double frac = shiftDropRem / 0.200;
                tRpm -= shiftDropAmt * (frac < 1.0 ? frac : 1.0);
                shiftDropRem -= wallDt;
                if (shiftDropRem < 0.0) shiftDropRem = 0.0;
            }
            tRpm = std::max(idleRpm, std::min(maxRpm, tRpm));
            state.targetRpm.store(tRpm, std::memory_order_relaxed);
            const double s_speed = (rpmSpan > 0.0)
                ? std::max(0.0, std::min(1.0, 1.0 - (tRpm - idleRpm) / rpmSpan)) : 0.5;
            const double s_pedal = std::max(0.0, std::min(1.0, 1.0 - pedal / 100.0));
            sc = std::min(s_speed, s_pedal);
            sc = std::min(1.0, sc + brake * 0.5);
            break;
        }
        default:
            sc = state.throttle.load(std::memory_order_relaxed);
            break;
        }
        engine->setSpeedControl(sc);

        // ------------------------------------------------------------------
        // Physics step
        // ------------------------------------------------------------------
        sim->startFrame(wallDt);
        while (sim->simulateStep()) {}
        sim->endFrame();
        stepCount += sim->getFrameIterationCount();

        // ------------------------------------------------------------------
        // Audio: either engine synthesis or tone generator
        // ------------------------------------------------------------------
        if (state.toneActive.load(std::memory_order_relaxed)) {
            // Tone mode: generate sine wave directly to ring and/or WAV.
            // Keep ring at ~50% capacity; let synth buffer stay full (it pauses).
            const int target  = AudioRing::kSize / 2;
            const int ringFill = state.audioRing.available();
            int toGen = target - ringFill;
            if (toGen <= 0) toGen = 0;
            // Also generate for WAV even if ring is full
            const int toGenWav = (!wavDone && wavWriter.isOpen())
                ? static_cast<int>(wallDt * sampleRate) + 64 : 0;
            const int total = std::max(toGen, toGenWav);

            if (total > 0) {
                static int16_t toneBuf[AudioRing::kSize];
                const int n = total < AudioRing::kSize ? total : AudioRing::kSize - 1;
                const double freq     = state.toneFreq.load(std::memory_order_relaxed);
                const double phaseInc = 2.0 * M_PI * freq / sampleRate;
                for (int i = 0; i < n; ++i) {
                    toneBuf[i] = static_cast<int16_t>(
                        std::sin(tonePhase) * 0.707 * 32767.0);
                    tonePhase += phaseInc;
                    if (tonePhase > 2.0 * M_PI) tonePhase -= 2.0 * M_PI;
                }
                if (!noDevice && toGen > 0)
                    state.audioRing.write(toneBuf, toGen < n ? toGen : n);
                // Write to WAV
                if (!wavDone && wavWriter.isOpen()) {
                    const uint32_t remaining = wavSamplesTarget - wavWriter.samplesWritten;
                    const int toWav = static_cast<int>(remaining) < n
                                      ? static_cast<int>(remaining) : n;
                    if (toWav > 0) wavWriter.write(toneBuf, toWav);
                    if (wavWriter.samplesWritten >= wavSamplesTarget) {
                        wavWriter.close();
                        wavDone = true;
                        std::fprintf(stderr, "[WAV] Done: %d s (%u samples) -- tone\n",
                                     wavSeconds, wavSamplesTarget);
                        if (noDevice) state.running.store(false, std::memory_order_relaxed);
                    }
                }
                epochGenSamples += n;
            }
        } else {
            // Adaptive drain -- keep ring at ~50% fill.
            //
            // Root cause of crackling: ring fills to ~91% (15000/16384) because main
            // loop drains the synth aggressively. With only 1384 free slots, any brief
            // burst causes AudioRing.write() to truncate, creating a discontinuity.
            //
            // Fix: read at most drainMax samples from the synth per frame. Excess stays
            // in the synth output buffer (up to 2000 samples). The synth rendering thread
            // stalls when its output buffer is full -- no samples are discarded, so the
            // audio stream has no gaps. The ring converges to ~50% fill in ~10 frames.
            //
            // P-controller: drainMax = consumed + (target - fill) / 8
            //   consumed  = what the callback will read before next main loop iteration
            //   (target-fill)/8 = gentle correction toward 50% fill (gain = 1/8)
            const int kRingTarget = AudioRing::kSize / 2;          // 8192 samples = ~186 ms
            const int ringFill    = state.audioRing.available();
            const int consumed    = static_cast<int>(wallDt * sampleRate) + 32; // ~735 + margin
            const int correction  = (kRingTarget - ringFill) / 8;
            // drainMax >= 32 so synth thread is always ticking, even when ring is full.
            const int drainMax    = std::max(32, consumed + correction);

            static int16_t drainBuf[2048];
            int drained = 0, drainedTotal = 0, lastToRead = 0;
            do {
                const int toRead = std::min(2048, drainMax - drainedTotal);
                lastToRead = toRead;
                if (toRead <= 0) break;
                drained = sim->readAudioOutput(toRead, drainBuf);
                if (drained > 0) {
                    drainedTotal += drained;

                    // Write to synth WAV (pre-ring; always full capture)
                    if (!wavDone && wavWriter.isOpen()) {
                        const uint32_t remaining = wavSamplesTarget - wavWriter.samplesWritten;
                        const int toWav = static_cast<int>(remaining) < drained
                                          ? static_cast<int>(remaining) : drained;
                        if (toWav > 0) wavWriter.write(drainBuf, toWav);
                        if (wavWriter.samplesWritten >= wavSamplesTarget) {
                            wavWriter.close();
                            wavDone = true;
                            std::fprintf(stderr,
                                "[WAV] Done: %d s (%u samples) -- engine audio\n",
                                wavSeconds, wavSamplesTarget);
                            if (noDevice)
                                state.running.store(false, std::memory_order_relaxed);
                        }
                    }

                    // Write to ring
                    if (!noDevice)
                        state.audioRing.write(drainBuf, drained);

                    // Diagnostics
                    epochGenSamples += drained;
                    for (int i = 0; i < drained; ++i) {
                        const int16_t s = drainBuf[i];
                        const float   v  = s * (1.0f / 32767.0f);
                        const float   av = v < 0.0f ? -v : v;
                        if (av > epochPeak) epochPeak = av;
                        epochSumSq += v * v;
                        if ((s >= 0) != (diagPrevSample >= 0)) ++epochZeroCross;
                        diagPrevSample = s;
                        epochHash = (epochHash << 5u) ^ (epochHash >> 27u)
                                    ^ static_cast<uint32_t>(static_cast<uint16_t>(s));
                        if (s >= 32767 || s <= -32767) ++epochSynthClips;
                    }
                }
            } while (drained == lastToRead && drainedTotal < drainMax);
        }

        // ------------------------------------------------------------------
        // Drain capture ring to live WAV (post-ring, exactly what callback sent)
        // ------------------------------------------------------------------
        if (!liveWavDone && liveWavWriter.isOpen()) {
            static int16_t capBuf[2048];
            int capDrained;
            do {
                capDrained = state.captureRing.read(capBuf, 2048);
                if (capDrained > 0) {
                    const uint32_t rem = wavSamplesTarget - liveWavWriter.samplesWritten;
                    const int toW = static_cast<int>(rem) < capDrained
                                   ? static_cast<int>(rem) : capDrained;
                    if (toW > 0) liveWavWriter.write(capBuf, toW);
                    if (liveWavWriter.samplesWritten >= wavSamplesTarget) {
                        liveWavWriter.close();
                        liveWavDone = true;
                        state.captureActive.store(false, std::memory_order_relaxed);
                        std::fprintf(stderr, "[LIVE-WAV] Done: %d s\n", wavSeconds);
                    }
                }
            } while (capDrained == 2048);
        }

        // ------------------------------------------------------------------
        // Audio diagnostic snapshot -- once per second
        // ------------------------------------------------------------------
        {
            const auto   now         = Clock::now();
            const double snapElapsed = Seconds(now - audioSnapEpoch).count();
            if (snapElapsed >= 1.0) {
                const int cbRead = state.cbSamplesRead.exchange(0, std::memory_order_relaxed);

                state.snapshotRms      = (epochGenSamples > 0)
                    ? std::sqrt(epochSumSq / static_cast<float>(epochGenSamples)) : 0.0f;
                state.snapshotPeak     = epochPeak;
                state.audioHash        = epochHash;
                state.zeroCrossRate    = (snapElapsed > 0.0)
                    ? static_cast<float>(epochZeroCross / snapElapsed) : 0.0f;
                state.samplesGenPerSec  = static_cast<int>(epochGenSamples / snapElapsed);
                state.samplesReadPerSec = static_cast<int>(cbRead / snapElapsed);
                state.synthBufFill      = static_cast<int>(
                    sim->getSynthesizerInputLatency() * static_cast<double>(sampleRate));
                state.synthClipsPerSec   = epochSynthClips;
                state.ringOverrunsPerSec = state.audioRing.getAndClearOverruns();

                if (state.audioDebug.load(std::memory_order_relaxed)) {
                    std::fprintf(stderr,
                        "[AUDIO] rpm=%.0f sc=%.3f"
                        " rms=%.4f peak=%.4f leveler=%.5f"
                        " gen/s=%d read/s=%d ring=%d zcr=%.0f"
                        " hash=%08X synth_clips=%d overruns=%d underruns=%d%s\n",
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
                        state.synthClipsPerSec,
                        state.ringOverrunsPerSec,
                        state.underruns.load(),
                        state.toneActive.load() ? " [TONE]" : "");
                }

                epochGenSamples = 0;
                epochPeak       = 0.0f;
                epochSumSq      = 0.0f;
                epochZeroCross  = 0;
                epochHash       = 0;
                epochSynthClips = 0;
                audioSnapEpoch  = now;
            }
        }

        // ------------------------------------------------------------------
        // Benchmark report every 5 s
        // ------------------------------------------------------------------
        if (benchmark) {
            const auto   now          = Clock::now();
            const double benchElapsed = Seconds(now - benchEpoch).count();
            if (benchElapsed >= 5.0) {
                const double simFreq = static_cast<double>(sim->getSimulationFrequency());
                const double rtf     = (benchElapsed > 0.0)
                    ? (stepCount / simFreq) / benchElapsed : 0.0;
                const double cpuNow  = processCpuSeconds();
                const double cpuPct  = (benchElapsed > 0.0)
                    ? (cpuNow - cpuPrev) / benchElapsed * 100.0 : 0.0;
                cpuPrev = cpuNow;
                const int undrEpoch = state.underruns.load() - underrunsPrev;
                underrunsPrev = state.underruns.load();
                std::fprintf(stderr,
                    "[BENCH] rtf=%.3f underruns=%d ring=%d cpu=%.1f%% steps/s=%lld\n",
                    rtf, undrEpoch, state.audioRing.available(), cpuPct,
                    static_cast<long long>(benchElapsed > 0.0
                        ? stepCount / benchElapsed : 0));
                benchEpoch = now;
                stepCount  = 0;
            }
        }

        // Pace at ~60 Hz
        const double frameTook = Seconds(Clock::now() - frameStart).count();
        const double sleepSec  = FRAME_DT - frameTook;
        if (sleepSec > 5e-4)
            std::this_thread::sleep_for(Seconds(sleepSec - 5e-4));
    }

    // -----------------------------------------------------------------------
    // 8. Cleanup
    // -----------------------------------------------------------------------
    std::cerr << "[headless] Shutting down...\n";

    if (deviceOpen) {
        ma_device_stop(&maDevice);
        ma_device_uninit(&maDevice);
    }

    // Finalize WAV files if still open (e.g. quit before wavSeconds elapsed)
    if (wavWriter.isOpen()) {
        std::fprintf(stderr, "[WAV] Finalizing %u samples (%.1f s)\n",
                     wavWriter.samplesWritten,
                     static_cast<double>(wavWriter.samplesWritten) / sampleRate);
        wavWriter.close();
    }
    if (liveWavWriter.isOpen()) {
        std::fprintf(stderr, "[LIVE-WAV] Finalizing %u samples (%.1f s)\n",
                     liveWavWriter.samplesWritten,
                     static_cast<double>(liveWavWriter.samplesWritten) / sampleRate);
        liveWavWriter.close();
    }

    sim->endAudioRenderingThread();
    stdinThr.detach();
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
