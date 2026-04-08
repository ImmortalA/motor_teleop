/*
 * motor_teleop — PC: B follows A over UDP ↔ Teensy ↔ CAN (AK MIT).
 * A = bus0 node0 (backdrivable), B = bus0 node1 tracks A using wrapped positions (p_orig).
 */

#include "spine_board.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

namespace cfg {
constexpr const char *kInterface = "enp8s0";
constexpr const char *kTeensyIp = "192.168.0.101";
constexpr int kPort = 8003;
constexpr int kNodesPerBus = 2;
constexpr int kNumBuses = 1;
/* Wire padding: must match spine_board.cpp kTeensy* and teensy_mt.ino TEENSY_*. */
constexpr int kWireBuses = 2;
constexpr int kWireSlotsPerBus = 3;
constexpr ActuatorType kMotor = ActuatorType::AK_60_6;
/* Softer gains: AK60-6 MIT torque is ~ kp*Δθ + kd*Δω (±15 Nm cap); stiff B faults when A moves fast. */
constexpr float kKpB = 16.0f;
constexpr float kKdB = 1.6f;
/* Limit how fast B’s position setpoint can move (rad per 1 kHz tick). Reduces torque spikes when A is flicked. */
constexpr float kMaxSlewRadPerTick = 0.035f;
/* Low-pass + clamp on velocity feedforward (raw v_A is noisy and spikes when hand-driving A). */
constexpr float kVffLpfAlpha = 0.2f;
constexpr float kVffMax = 10.0f;
}  // namespace cfg

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

int main()
{
    static_assert(cfg::kNumBuses >= 1 && cfg::kNumBuses <= cfg::kWireBuses, "buses");
    static_assert(cfg::kNodesPerBus >= 1 && cfg::kNodesPerBus <= cfg::kWireSlotsPerBus, "nodes");

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

    /* Second exit → MIT zero → enter → MIT zero so teleop always starts from a clean enabled state. */
    std::cout << "motor_teleop: exit motor mode + re-enter (enable)...\n";
    board.exitAndEnableMotorMode();

    std::cout << "motor_teleop: B follows A (Ctrl+C to stop)\n";

    constexpr int kBus = 0;
    constexpr int kA = 0;
    constexpr int kB = 1;
    bool have_offset = false;
    float offset_B_minus_A = 0.0f;
    float cmd_p_B = 0.0f;
    float v_ff_B = 0.0f;

    while (true) {
        board.withBusList([&](std::vector<bus> &buses) {
            bus &bus0 = buses[kBus];

            const float qA = bus0.state.j[kA].p_orig;
            const float qB = bus0.state.j[kB].p_orig;
            const float vA = bus0.state.j[kA].v;

            if (!have_offset) {
                offset_B_minus_A = shortest_angle_diff(qA, qB);
                have_offset = true;
                cmd_p_B = wrap_angle(qA + offset_B_minus_A);
                v_ff_B = vA;
            }

            bus0.command.j[kA].p_des = 0.0f;
            bus0.command.j[kA].v_des = 0.0f;
            bus0.command.j[kA].kp = 0.0f;
            bus0.command.j[kA].kd = 0.0f;
            bus0.command.j[kA].t_ff = 0.0f;

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
        });
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
}
