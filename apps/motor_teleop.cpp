/*
 * motor_teleop — PC ↔ Teensy ↔ CAN (AK MIT): A = bus0 node0, B = bus0 node1.
 * Modes: drive-only (B follows A, A free) or bilateral (A feels B).
 */

#include "spine_board.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

namespace cfg {
constexpr const char *kInterface = "enp8s0";
constexpr const char *kTeensyIp = "192.168.0.101";
constexpr int kPort = 8003;
constexpr int kNodesPerBus = 2;
constexpr int kNumBuses = 1;
constexpr int kWireBuses = 2;
constexpr int kWireSlotsPerBus = 3;
constexpr ActuatorType kMotor = ActuatorType::AK_60_6;
constexpr float kKpB = 1.0f;
constexpr float kKdB = 0.01f;
constexpr float kMaxSlewRadPerTick = 0.035f;kKdB
constexpr float kVffLpfAlpha = 0.2f;
constexpr float kVffMax = 10.0f;

constexpr float kTorqueReflect = 1.0f;
constexpr float kReflectLpfAlpha = 0.25f;
constexpr float kPoseReflect = 2.0f;
constexpr float kMasterDamping = 0.12f;
}  // namespace cfg

enum class RunMode
{
    DriveOnly = 0,
    Bilateral = 1,
};

static void print_usage(const char *prog)
{
    std::cerr << "Usage: " << prog << " <mode>\n"
              << "  0 | drive       — A drive only (A free, B follows A)\n"
              << "  1 | bilateral   — B follows A and A feels B (τ reflection + lag)\n";
}

static RunMode parse_mode(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "No mode provided; defaulting to mode 0 (drive-only).\n";
        std::cout << "Tip: run `" << argv[0] << " 1` for bilateral.\n";
        return RunMode::DriveOnly;
    }
    const char *s = argv[1];
    if (s[0] == '0' && s[1] == '\0')
        return RunMode::DriveOnly;
    if (s[0] == '1' && s[1] == '\0')
        return RunMode::Bilateral;
    if (std::strcmp(s, "drive") == 0)
        return RunMode::DriveOnly;
    if (std::strcmp(s, "bilateral") == 0)
        return RunMode::Bilateral;
    std::cerr << "Unknown mode: " << s << "\n";
    print_usage(argv[0]);
    std::exit(1);
}

static float shortest_angle_diff(float a_wrapped, float b_wrapped)
{
    float d = b_wrapped - a_wrapped;
    const float half = WRAP_RANGE * 0.5f;
    if (d > half)
        d -= WRAP_RANGE;
    else if (d < -half)
        d += WRAP_RANGE;
    return d;
}

int main(int argc, char **argv)
{
    static_assert(cfg::kNumBuses >= 1 && cfg::kNumBuses <= cfg::kWireBuses, "buses");
    static_assert(cfg::kNodesPerBus >= 1 && cfg::kNodesPerBus <= cfg::kWireSlotsPerBus, "nodes");

    const RunMode mode = parse_mode(argc, argv);

    std::vector<std::vector<ActuatorParams>> params(cfg::kNumBuses);
    for (int b = 0; b < cfg::kNumBuses; ++b) {
        params[b].resize(cfg::kNodesPerBus, getActuatorParams(cfg::kMotor));
    }

    SpineBoard board(cfg::kTeensyIp, cfg::kInterface, cfg::kPort, cfg::kNodesPerBus, cfg::kNumBuses, "teensy");
    board.setActuatorParams(params);
    board.start();

    while (!board.boardInitialized)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    board.setWaitForFeedbackAfterSend(false);
    board.setAllowCommandSend(true);

    std::cout << "motor_teleop: exit motor mode + re-enter (enable)...\n";
    board.exitAndEnableMotorMode();

    if (mode == RunMode::DriveOnly)
        std::cout << "motor_teleop: mode 0 — A drive only, B follows A (Ctrl+C to stop)\n";
    else
        std::cout << "motor_teleop: mode 1 — bilateral, A feels B (Ctrl+C to stop)\n";

    constexpr int kBus = 0;
    constexpr int kA = 0;
    constexpr int kB = 1;
    bool have_offset = false;
    float offset_B_minus_A = 0.0f;
    float cmd_p_B = 0.0f;
    float v_ff_B = 0.0f;
    float tau_reflect_lpf = 0.0f;

    while (true) {
        board.withBusList([&](std::vector<bus> &buses) {
            bus &bus0 = buses[kBus];

            const float qA = bus0.state.j[kA].p_orig;
            const float qB = bus0.state.j[kB].p_orig;
            const float vA = bus0.state.j[kA].v;
            const float tB = bus0.state.j[kB].t;

            if (!have_offset) {
                offset_B_minus_A = shortest_angle_diff(qA, qB);
                have_offset = true;
                cmd_p_B = wrap_angle(qA + offset_B_minus_A);
                v_ff_B = vA;
                tau_reflect_lpf = 0.0f;
            }

            const float p_target = wrap_angle(qA + offset_B_minus_A);
            float delta = shortest_angle_diff(cmd_p_B, p_target);
            delta = sb_fminf(sb_fmaxf(delta, -cfg::kMaxSlewRadPerTick), cfg::kMaxSlewRadPerTick);
            cmd_p_B = wrap_angle(cmd_p_B + delta);

            v_ff_B += cfg::kVffLpfAlpha * (vA - v_ff_B);
            const float v_cmd_B = sb_fminf(sb_fmaxf(-cfg::kVffMax, v_ff_B), cfg::kVffMax);

            bus0.command.j[kB].p_des = cmd_p_B;
            bus0.command.j[kB].v_des = v_cmd_B;
            bus0.command.j[kB].kp = cfg::kKpB;
            bus0.command.j[kB].kd = cfg::kKdB;
            bus0.command.j[kB].t_ff = 0.0f;

            bus0.command.j[kA].p_des = 0.0f;
            bus0.command.j[kA].v_des = 0.0f;
            bus0.command.j[kA].kp = 0.0f;
            bus0.command.j[kA].kd = 0.0f;

            if (mode == RunMode::DriveOnly) {
                bus0.command.j[kA].t_ff = 0.0f;
            } else {
                const float tau_from_B = -cfg::kTorqueReflect * tB;
                tau_reflect_lpf += cfg::kReflectLpfAlpha * (tau_from_B - tau_reflect_lpf);
                const float e_lag = shortest_angle_diff(qB, cmd_p_B);
                const float tau_from_lag = -cfg::kPoseReflect * e_lag;
                float tau_A = tau_reflect_lpf + tau_from_lag - cfg::kMasterDamping * vA;
                tau_A = sb_fminf(sb_fmaxf(bus0.params[kA].t_min, tau_A), bus0.params[kA].t_max);
                bus0.command.j[kA].t_ff = tau_A;
            }
        });
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
}
