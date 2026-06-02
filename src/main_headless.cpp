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
// Backfire: cross-thread config + lock-free trigger mailbox
// ---------------------------------------------------------------------------
struct BackfireCfg {
    std::atomic<bool>  enabled   { true  };
    std::atomic<float> chance    { 0.35f };
    std::atomic<float> volume    { 0.7f  };
    std::atomic<int>   rpmMin    { 3000  };
    std::atomic<int>   count     { 0     };

    // Mailbox: main/control threads write, audio callback reads-and-clears.
    // Amplitude stored before samples count so the callback never sees an
    // inconsistent (samples>0 but stale amplitude) state.
    std::atomic<float> trigAmplitude { 0.0f };
    std::atomic<int>   trigSamples   { 0    };
};

// Audio-callback-exclusive state — no atomics needed, only one thread touches it.
struct BackfireCbState {
    int      active   = 0;           // remaining samples in current pop
    int      total    = 0;
    float    curAmp   = 0.0f;        // decays each sample
    float    decayMul = 1.0f;        // per-sample multiplier: exp(-k/total)
    uint32_t rng      = 0xCAFEBABEu; // xorshift32 for white noise
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
    // DirectThrottleLinkage convention: s=0 → full throttle, s=1 → idle.
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
// miniaudio callback — pulls engine audio then mixes synthetic backfire pop
// ---------------------------------------------------------------------------
static void audioCallback(
        ma_device *device, void *output, const void *, ma_uint32 frameCount)
{
    HeadlessState *s   = static_cast<HeadlessState *>(device->pUserData);
    int16_t       *out = static_cast<int16_t *>(output);

    const int got = s->sim->readAudioOutput(static_cast<int>(frameCount), out);
    if (got < static_cast<int>(frameCount)) {
        std::memset(out + got, 0, (frameCount - got) * sizeof(int16_t));
        s->underruns.fetch_add(1, std::memory_order_relaxed);
    }

    if (!s->backfire.enabled.load(std::memory_order_relaxed))
        return;

    BackfireCbState &cb = s->backfireCb;

    // Pick up a pending trigger only when the previous pop has finished.
    if (cb.active == 0) {
        const int pending =
            s->backfire.trigSamples.exchange(0, std::memory_order_acquire);
        if (pending > 0) {
            cb.total    = pending;
            cb.active   = pending;
            cb.curAmp   = s->backfire.trigAmplitude.load(std::memory_order_relaxed);
            // exp(-8/N): amplitude falls to ~0.03% of peak over the full pop duration.
            cb.decayMul = expf(-8.0f / static_cast<float>(pending));
        }
    }

    if (cb.active > 0) {
        const ma_uint32 toPlay = (static_cast<ma_uint32>(cb.active) < frameCount)
                                    ? static_cast<ma_uint32>(cb.active)
                                    : frameCount;
        for (ma_uint32 i = 0; i < toPlay; ++i) {
            // xorshift32 white noise — no heap, no branches
            cb.rng ^= cb.rng << 13u;
            cb.rng ^= cb.rng >> 17u;
            cb.rng ^= cb.rng <<  5u;
            const float noise = static_cast<float>(cb.rng) *
                                 (1.0f / 4294967296.0f) * 2.0f - 1.0f;

            cb.curAmp *= cb.decayMul;
            const float pop  = noise * cb.curAmp;
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
// Command parser — shared by stdin and UDP threads
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
        else if (sub == "rpm")
            state.mode.store((int)ControlMode::Rpm,      std::memory_order_relaxed);
        else if (sub == "hybrid")
            state.mode.store((int)ControlMode::Hybrid,   std::memory_order_relaxed);

    } else if (cmd == "rpm_per_mph") {
        double v = 85.0;
        if (ss >> v) state.rpmPerMph.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "idle_rpm") {
        double v = 900.0;
        if (ss >> v) state.idleRpm.store(clampPos(v), std::memory_order_relaxed);

    } else if (cmd == "max_rpm") {
        double v = 8000.0;
        if (ss >> v) state.maxRpm.store(clampPos(v), std::memory_order_relaxed);

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

    } else if (cmd == "quit") {
        state.running.store(false, std::memory_order_relaxed);

    } else if (cmd == "status") {
        const char *modeStr = "throttle";
        switch (static_cast<ControlMode>(state.mode.load())) {
            case ControlMode::Rpm:    modeStr = "rpm";    break;
            case ControlMode::Hybrid: modeStr = "hybrid"; break;
            default: break;
        }
        const double rpm = state.engine ? state.engine->getRpm() : 0.0;
        std::cout
            << "mode="           << modeStr                                  << "\n"
            << "throttle="       << state.throttle.load()                    << "\n"
            << "pedal="          << state.pedal.load()                       << "\n"
            << "brake="          << state.brake.load()                       << "\n"
            << "speed_mph="      << state.speedMph.load()                    << "\n"
            << "rpm="            << rpm                                       << "\n"
            << "target_rpm="     << state.targetRpm.load()                   << "\n"
            << "idle_rpm="       << state.idleRpm.load()                     << "\n"
            << "rpm_per_mph="    << state.rpmPerMph.load()                   << "\n"
            << "max_rpm="        << state.maxRpm.load()                      << "\n"
            << "backfire="       << (state.backfire.enabled.load()?"on":"off")<< "\n"
            << "backfire_chance="<< state.backfire.chance.load()             << "\n"
            << "backfire_volume="<< state.backfire.volume.load()             << "\n"
            << "backfire_count=" << state.backfire.count.load()              << "\n"
            << "underruns="      << state.underruns.load()                   << "\n"
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
    // the engine file and calls main() — mirrors what assets/main.mr does.
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

    std::cerr << "[headless] Engine: "  << engine->getName() << "\n";
    std::cerr << "[headless] Audio: "   << sampleRate << " Hz, period=512 frames\n";
    std::cerr << "[headless] Control: stdin or UDP port " << udpPort << "\n";
    std::cerr << "[headless] Commands: throttle|pedal|brake|speed|setrpm|torque"
                 " mode[throttle|rpm|hybrid] status quit\n";
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

    // Backfire trigger state (main-thread only — no atomics needed)
    double prevPedal  = 0.0;
    double bfCooldown = 0.0; // seconds until next backfire is eligible

    engine->setSpeedControl(initThrottle);

    while (state.running.load()) {
        const auto   frameStart = Clock::now();
        const double wallDt     = std::max(DT_MIN,
            std::min(DT_MAX, Seconds(frameStart - prevFrame).count()));
        prevFrame = frameStart;

        // ------------------------------------------------------------------
        // Compute setSpeedControl value from active mode.
        //
        // DirectThrottleLinkage maps setSpeedControl(s) as:
        //   s=0 → throttle_position=1 → full throttle
        //   s=1 → throttle_position=0 → idle
        // So pedal 0-100 maps to s = 1 - pedal/100.
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

        double sc;
        switch (mode) {
            case ControlMode::Rpm: {
                const double tRpm = state.targetRpm.load(std::memory_order_relaxed);
                sc = (rpmSpan > 0.0)
                    ? std::max(0.0, std::min(1.0, 1.0 - (tRpm - idleRpm) / rpmSpan))
                    : 0.5;
                break;
            }
            case ControlMode::Hybrid: {
                // Speed sets baseline RPM; pedal adds extra throttle on top.
                const double speedRpm = std::max(idleRpm,
                    std::min(maxRpm, idleRpm + speedMph * rpmPerMph));
                const double s_speed  = (rpmSpan > 0.0)
                    ? std::max(0.0, std::min(1.0, 1.0 - (speedRpm - idleRpm) / rpmSpan))
                    : 0.5;
                const double s_pedal  = std::max(0.0, std::min(1.0, 1.0 - pedal / 100.0));
                // Lower s = more throttle; take the more aggressive of the two.
                sc = std::min(s_speed, s_pedal);
                // Brake input increases s = engine braking / deceleration feel.
                sc = std::min(1.0, sc + brake * 0.5);
                break;
            }
            default: // ControlMode::Throttle — legacy direct pass-through
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
