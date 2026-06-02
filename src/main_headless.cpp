// engine-sim headless entry point
// Runs physics + audio simulation without any GUI or delta-studio dependency.
// Targets ARM64 Linux (Raspberry Pi 4) but is POSIX-portable.
//
// Build: cmake -B build -DHEADLESS=ON -DDISCORD_ENABLED=OFF
// Usage: engine-sim-headless --script assets/engines/.../engine.mr [--port 9999]
//            [--throttle 0.5] [--benchmark] [--sample-rate 44100] [--buffer-size 44100]
// Control (stdin or UDP): throttle <0-1>  |  speedcontrol <0-1>  |  status  |  quit

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
#include <cstring>
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
// Shared application state (between main loop, audio callback, control threads)
// ---------------------------------------------------------------------------
struct HeadlessState {
    Simulator          *sim          = nullptr;
    Engine             *engine       = nullptr;
    std::atomic<int>    underruns    { 0 };
    std::atomic<bool>   running      { true };
    std::atomic<double> throttle     { 0.5 };
};

// ---------------------------------------------------------------------------
// miniaudio callback — pulls from the synthesizer's ring buffer
// ---------------------------------------------------------------------------
static void audioCallback(
        ma_device *device, void *output, const void *, ma_uint32 frameCount)
{
    HeadlessState *s = static_cast<HeadlessState *>(device->pUserData);
    int16_t *out = static_cast<int16_t *>(output);
    const int got = s->sim->readAudioOutput(static_cast<int>(frameCount), out);
    if (got < static_cast<int>(frameCount)) {
        std::memset(out + got, 0, (frameCount - got) * sizeof(int16_t));
        s->underruns.fetch_add(1, std::memory_order_relaxed);
    }
}

// ---------------------------------------------------------------------------
// Command parser — shared by stdin and UDP threads
// ---------------------------------------------------------------------------
static void applyCommand(const std::string &line, HeadlessState &state) {
    std::istringstream ss(line);
    std::string cmd;
    if (!(ss >> cmd)) return;

    if (cmd == "throttle" || cmd == "speedcontrol") {
        double v = 0.5;
        if (ss >> v) {
            v = std::max(0.0, std::min(1.0, v));
            state.throttle.store(v, std::memory_order_relaxed);
        }
    } else if (cmd == "quit") {
        state.running.store(false, std::memory_order_relaxed);
    } else if (cmd == "status") {
        Engine *eng = state.engine;
        if (eng) {
            std::cout << "RPM=" << eng->getRpm()
                      << " throttle=" << state.throttle.load()
                      << "\n" << std::flush;
        }
    }
}

// ---------------------------------------------------------------------------
// stdin control thread
// ---------------------------------------------------------------------------
static void stdinThread(HeadlessState &state) {
    std::string line;
    while (state.running.load() && std::getline(std::cin, line)) {
        applyCommand(line, state);
    }
}

// ---------------------------------------------------------------------------
// UDP control thread (Linux/POSIX only)
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
        volume,
        index);

    ma_free(pData, nullptr);
}

// ---------------------------------------------------------------------------
// CLI helpers
// ---------------------------------------------------------------------------
static std::string argValue(int argc, char **argv, const char *flag, const char *def) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == flag) return argv[i + 1];
    }
    return def;
}

static bool argFlag(int argc, char **argv, const char *flag) {
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == flag) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    const std::string scriptPath   = argValue(argc, argv, "--script",      "../assets/main.mr");
    const int         udpPort      = std::stoi(argValue(argc, argv, "--port",         "9999"));
    const double      initThrottle = std::stod(argValue(argc, argv, "--throttle",     "0.5"));
    const bool        benchmark    = argFlag(argc, argv, "--benchmark");
    const int         sampleRate   = std::stoi(argValue(argc, argv, "--sample-rate",  "44100"));

    // -----------------------------------------------------------------------
    // 1. Load engine script
    // -----------------------------------------------------------------------
    Engine       *engine       = nullptr;
    Vehicle      *vehicle      = nullptr;
    Transmission *transmission = nullptr;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    es_script::Compiler compiler;
    compiler.initialize();
    std::cerr << "[headless] Compiling: " << scriptPath << "\n";
    if (compiler.compile(scriptPath.c_str())) {
        const es_script::Compiler::Output output = compiler.execute();
        engine       = output.engine;
        vehicle      = output.vehicle;
        transmission = output.transmission;
    } else {
        std::cerr << "[headless] ERROR: script compilation failed\n";
        compiler.destroy();
        return 1;
    }
    compiler.destroy();
#else
    std::cerr << "[headless] ERROR: built without Piranha scripting\n";
    return 1;
#endif

    if (!engine) {
        std::cerr << "[headless] ERROR: script produced no engine\n";
        return 1;
    }

    // Fallback vehicle if script didn't define one
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

    // Fallback transmission if script didn't define one
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

    // Load per-exhaust impulse responses
    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse *ir = engine->getExhaustSystem(i)->getImpulseResponse();
        if (ir) {
            loadImpulseResponse(
                sim, i, ir->getFilename(), static_cast<float>(ir->getVolume()));
        }
    }

    // -----------------------------------------------------------------------
    // 3. Start synthesizer audio rendering thread
    // -----------------------------------------------------------------------
    sim->startAudioRenderingThread();

    // -----------------------------------------------------------------------
    // 4. Open miniaudio device (ALSA / PulseAudio / PipeWire — auto-detected)
    // -----------------------------------------------------------------------
    HeadlessState state;
    state.sim    = sim;
    state.engine = engine;
    state.throttle.store(initThrottle);

    ma_device_config maCfg     = ma_device_config_init(ma_device_type_playback);
    maCfg.playback.format      = ma_format_s16;
    maCfg.playback.channels    = 1;
    maCfg.sampleRate           = static_cast<ma_uint32>(sampleRate);
    maCfg.dataCallback         = audioCallback;
    maCfg.pUserData            = &state;
    maCfg.periodSizeInFrames   = 512;   // ~11 ms at 44100 Hz — low latency for Pi 4

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

    std::cerr << "[headless] Engine: " << engine->getName() << "\n";
    std::cerr << "[headless] Audio: " << sampleRate << " Hz, period=512 frames\n";
    std::cerr << "[headless] Control: stdin or UDP port " << udpPort << "\n";
    std::cerr << "[headless] Commands: throttle <0-1>  speedcontrol <0-1>  status  quit\n";

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
    // Clamp frame dt to avoid absurd step counts if the system stalls
    constexpr double DT_MIN = 1.0 / 200.0;
    constexpr double DT_MAX = 1.0 / 30.0;

    auto loopStart = Clock::now();
    auto prevFrame = loopStart;

    // Benchmark state
    auto   benchEpoch      = loopStart;
    double cpuPrev         = processCpuSeconds();  // bootstrap CPU measurement
    long long stepCount    = 0;
    int   underrunsPrev    = 0;

    engine->setSpeedControl(initThrottle);

    while (state.running.load()) {
        const auto frameStart = Clock::now();

        // Apply throttle from control interface
        engine->setSpeedControl(state.throttle.load(std::memory_order_relaxed));

        // Step physics
        const double wallDt = std::max(DT_MIN,
            std::min(DT_MAX, Seconds(frameStart - prevFrame).count()));
        prevFrame = frameStart;

        sim->startFrame(wallDt);
        while (sim->simulateStep()) { /* runs physics steps at sim frequency */ }
        sim->endFrame();
        stepCount += sim->getFrameIterationCount();

        // Benchmark report every 5 s
        if (benchmark) {
            const auto   now          = Clock::now();
            const double benchElapsed = Seconds(now - benchEpoch).count();
            if (benchElapsed >= 5.0) {
                // Real-time factor: simulated seconds / wall seconds (for this epoch)
                const double simFreq  = static_cast<double>(sim->getSimulationFrequency());
                const double simulated = benchElapsed > 0.0 ? stepCount / simFreq : 0.0;
                const double rtfEpoch = benchElapsed > 0.0 ? simulated / benchElapsed : 0.0;

                const double cpuNow    = processCpuSeconds();
                const double cpuDelta  = cpuNow - cpuPrev;
                const double cpuPct    = benchElapsed > 0.0
                    ? cpuDelta / benchElapsed * 100.0 : 0.0;
                cpuPrev = cpuNow;

                const double latencyMs  = sim->getSynthesizerInputLatency() * 1000.0;
                const double latTarget  = sim->getSynthesizerInputLatencyTarget();
                const double bufFillPct = latTarget > 0.0
                    ? std::min(1.0, sim->getSynthesizerInputLatency() / latTarget) * 100.0
                    : 0.0;

                const int undrNow   = state.underruns.load();
                const int undrEpoch = undrNow - underrunsPrev;
                underrunsPrev = undrNow;

                std::fprintf(stderr,
                    "[BENCH] rtf=%.3f underruns=%d buf=%.0f%% latency=%.1fms"
                    " cpu=%.1f%% steps/s=%lld\n",
                    rtfEpoch,
                    undrEpoch,
                    bufFillPct,
                    latencyMs,
                    cpuPct,
                    static_cast<long long>(stepCount / benchElapsed));

                benchEpoch = now;
                stepCount  = 0;
            }
        }

        // Sleep remainder of frame to pace at ~60 Hz
        const double frameTook = Seconds(Clock::now() - frameStart).count();
        const double sleepSec  = FRAME_DT - frameTook;
        if (sleepSec > 5e-4) {
            std::this_thread::sleep_for(Seconds(sleepSec - 5e-4));
        }
    }

    // -----------------------------------------------------------------------
    // 7. Cleanup
    // -----------------------------------------------------------------------
    std::cerr << "[headless] Shutting down...\n";

    // Stop audio device first so the callback no longer reads from sim
    ma_device_stop(&maDevice);
    ma_device_uninit(&maDevice);

    // Stop synthesizer thread
    sim->endAudioRenderingThread();

    // Detach stdin thread (may be blocked on getline); join UDP thread
    stdinThr.detach();
    udpThr.join();

    // Free simulation
    sim->releaseSimulation();
    delete sim;

    engine->destroy();
    delete engine;
    delete vehicle;
    delete transmission;

    std::cerr << "[headless] Done.\n";
    return 0;
}
