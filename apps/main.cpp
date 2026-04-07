/*
 * test_spine — PC-side demo for T-Motor AK MIT over UDP ↔ Teensy ↔ CAN.
 *
 * Flow: SpineBoard runs two threads (UDP receive from Teensy, UDP send with optional wait-for-feedback).
 * This file fills joint commands from a simple mode menu; topology (boards / buses / nodes) lives in host_cfg.
 * Keep host_cfg consistent with teensy/teensy.ino CONFIG and with spine_board.cpp wire constants.
 */

#include "spine_board.h"
#include <memory>
#include <cmath>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>

/* One entry per actuator: which SpineBoard instance and which (bus, node) slot in its bus_list. */
struct ActuatorInfo
{
    int board; /* index into _spine_boards */
    int bus;   /* 0 .. num_buses-1 (logical CAN bus in UDP layout) */
    int node;  /* 0 .. num_nodes-1 along that bus */
};

namespace host_cfg {
constexpr int kNumTeensyBoards = 1;
/* Logical CAN buses used per board (SpineBoard second ctor arg). ≤ kWireNumLogicalBuses. */
constexpr int kNumCanBuses = 1;
/* Nodes per bus (first ctor arg). Must match firmware NUM_ACTIVE_NODES_BUS* totals. */
constexpr int kNodesPerBus = 2;
constexpr ActuatorType kDefaultActuator = ActuatorType::AK_60_6;
/* Protocol capacity: 2 buses × 3 nodes × 8 B — same numbers as teensy.ino / spine_board.cpp. */
constexpr int kWireNumLogicalBuses = 2;
constexpr int kWireMaxNodesPerBus = 3;
}  // namespace host_cfg

static_assert(host_cfg::kNumCanBuses >= 1 && host_cfg::kNumCanBuses <= host_cfg::kWireNumLogicalBuses,
              "kNumCanBuses");
static_assert(host_cfg::kNodesPerBus >= 1 && host_cfg::kNodesPerBus <= host_cfg::kWireMaxNodesPerBus,
              "kNodesPerBus");

/* Flatten (board × bus × node) into the list the control loop iterates. */
static std::vector<ActuatorInfo> makeActuatorMap()
{
    std::vector<ActuatorInfo> m;
    for (int b = 0; b < host_cfg::kNumTeensyBoards; ++b)
        for (int bus = 0; bus < host_cfg::kNumCanBuses; ++bus)
            for (int node = 0; node < host_cfg::kNodesPerBus; ++node)
                m.push_back({b, bus, node});
    return m;
}

static std::vector<std::vector<std::vector<ActuatorParams>>> makeActuatorParams()
{
    std::vector<std::vector<std::vector<ActuatorParams>>> out;
    for (int b = 0; b < host_cfg::kNumTeensyBoards; ++b)
    {
        std::vector<std::vector<ActuatorParams>> board;
        for (int bus = 0; bus < host_cfg::kNumCanBuses; ++bus)
        {
            std::vector<ActuatorParams> row;
            for (int n = 0; n < host_cfg::kNodesPerBus; ++n)
                row.push_back(getActuatorParams(host_cfg::kDefaultActuator));
            board.push_back(row);
        }
        out.push_back(board);
    }
    return out;
}

int main()
{
    const std::vector<ActuatorInfo> ACTUATOR_INFO_MAP = makeActuatorMap();
    const std::vector<std::vector<std::vector<ActuatorParams>>> ACTUATOR_PARAMS = makeActuatorParams();

    /* Linux: bind UDP to this interface; must exist (`ip link`). PC IP often 192.168.0.100 on same /24 as Teensy. */
    static std::string BOARD_INTERFACE_NAME = "enp8s0";
    static std::string ACTUATOR_TEENSY_BOARD_IPS[host_cfg::kNumTeensyBoards] = {"192.168.0.101"};
    static int ACTUATOR_TEENSY_BOARD_PORTS[host_cfg::kNumTeensyBoards] = {8003}; /* Teensy udp.begin port */

    std::vector<std::unique_ptr<SpineBoard>> _spine_boards;
    for (size_t i = 0; i < host_cfg::kNumTeensyBoards; i++)
    {
        _spine_boards.push_back(
            std::make_unique<SpineBoard>(ACTUATOR_TEENSY_BOARD_IPS[i],
                                         BOARD_INTERFACE_NAME,
                                         ACTUATOR_TEENSY_BOARD_PORTS[i],
                                         host_cfg::kNodesPerBus,
                                         host_cfg::kNumCanBuses,
                                         "board_" + std::to_string(i)));
    }

    for (size_t id = 0; id < _spine_boards.size(); ++id)
    {
        auto &board = *_spine_boards[id];
        board.setActuatorParams(ACTUATOR_PARAMS[id]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        board.start();
        board.setThreadAffinityAndPriority(static_cast<int>(id));
    }

    /* Trajectory sample period for this thread only; MIT UDP rate is limited by Teensy round-trip when wait-for-feedback is on. */
    int dt_us = 10000;
    float dt = dt_us / 1e6f;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool all_initialized = true;
        for (size_t id = 0; id < _spine_boards.size(); ++id)
            all_initialized = all_initialized && _spine_boards[id]->boardInitialized;
        if (all_initialized)
            break;
    }

    std::cout << "Select test mode (T-Motor AK — MIT impedance control on CAN):\n";
    std::cout << "  0: Enable only (no MIT commands from host)\n";
    std::cout << "  1: Position sine (amp=1 rad, 1 rad/s)\n";
    std::cout << "  2: Motor teleop (A->B position coupling + B->A torque reflection)\n";
    std::cout << "Mode: " << std::flush;

    int mode = 0;
    if (!(std::cin >> mode))
        mode = 0;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (mode == 0)
    {
        std::cout << "Running SpineBoard... Motor enabled. No MIT commands sent." << std::endl;
        std::cout << "Press Enter to exit." << std::flush;
        std::cin.get();
    }
    else if (mode == 1 || mode == 2)
    {
        /*
         * Control pacing:
         * - Default modes (1-4) match firmware gating: one MIT UDP per completed feedback round.
         * - Teleop (mode 5) needs higher command rate; it disables host-side wait. For true 1kHz end-to-end,
         *   also disable gating in `teensy/teensy.ino` (see firmware option).
         */
        for (auto &board : _spine_boards) {
            board->setWaitForFeedbackAfterSend(mode != 2);
            board->setAllowCommandSend(true);
        }

        std::cout << "Running control test (Ctrl+C to stop)...\n";

        const float PRIVATE_SINE_AMP = 1.0f;

        float p_center = 0.0f;
        bool center_initialized = false;

        int iter = 0;
        while (true)
        {
            /* Copy current state, write new commands into bus_list copies, then push back to SpineBoard. */
            std::vector<std::vector<bus>> boards_bus_lists(host_cfg::kNumTeensyBoards);
            for (size_t b = 0; b < host_cfg::kNumTeensyBoards; b++)
                boards_bus_lists[b] = _spine_boards[b]->getBusList();

            for (size_t i = 0; i < ACTUATOR_INFO_MAP.size(); i++)
            {
                auto info = ACTUATOR_INFO_MAP[i];
                auto &bus_list = boards_bus_lists[static_cast<size_t>(info.board)];

                if (!center_initialized && mode == 1)
                {
                    p_center = bus_list[info.bus].state.j[info.node].p;
                    center_initialized = true;
                }

                float p_target = 0.0f;
                float v_target = 0.0f;
                float t_ff = 0.0f;

                if (mode == 2)
                {
                    /* Teleop expects exactly 2 nodes on bus0: node0=A (master), node1=B (follower). */
                    if (host_cfg::kNumCanBuses != 1 || host_cfg::kNodesPerBus < 2) {
                        if (iter % 1000 == 0) {
                            std::cerr << "Teleop mode requires kNumCanBuses=1 and kNodesPerBus>=2\n";
                        }
                        continue;
                    }
                    if (info.bus != 0 || info.node > 1) {
                        /* Only control A and B; leave any other configured nodes at zeros. */
                        bus_list[info.bus].command.j[info.node].p_des = 0.0f;
                        bus_list[info.bus].command.j[info.node].v_des = 0.0f;
                        bus_list[info.bus].command.j[info.node].kp = 0.0f;
                        bus_list[info.bus].command.j[info.node].kd = 0.0f;
                        bus_list[info.bus].command.j[info.node].t_ff = 0.0f;
                        continue;
                    }

                    constexpr int BUS = 0;
                    constexpr int A = 0;
                    constexpr int B = 1;

                    const float qA = bus_list[BUS].state.j[A].p;
                    const float dqA = bus_list[BUS].state.j[A].v;
                    const float tauB = bus_list[BUS].state.j[B].t;
                    const float qB = bus_list[BUS].state.j[B].p;
                    const float dqB = bus_list[BUS].state.j[B].v;

                    /*
                     * IMPORTANT: Soft-start.
                     * If B instantly jumps to qA (or stale qA), AK drives may "click" then fault/disable.
                     * We ramp coupling and gains in over ~1s and preserve initial offset (qB0 - qA0).
                     */
                    static bool teleop_init = false;
                    static float qA0 = 0.0f;
                    static float qB0 = 0.0f;
                    static int teleop_iter0 = 0;
                    if (!teleop_init) {
                        qA0 = qA;
                        qB0 = qB;
                        teleop_iter0 = iter;
                        teleop_init = true;
                    }
                    const float t_ramp = (iter - teleop_iter0) * 0.001f;  // 1kHz loop -> seconds
                    float alpha = t_ramp / 1.0f;                          // 1s ramp
                    if (alpha < 0.0f) alpha = 0.0f;
                    if (alpha > 1.0f) alpha = 1.0f;

                    /* Conservative defaults; tune on hardware (start low; ramp up). */
                    const float KP_B = (1.0f + 9.0f * alpha);  /* 1 -> 10 */
                    const float KD_B = (0.1f + 0.9f * alpha);  /* 0.1 -> 1.0 */
                    /* Master should be freely backdrivable: keep kp=0,kd=0 and only inject torque reflection. */
                    constexpr float KP_A = 0.0f;
                    constexpr float KD_A = 0.0f;
                    constexpr float KF = 0.35f;     /* torque reflection gain: tauA += KF * tauB */
                    constexpr float KC = 0.0f;      /* optional coupling stiffness A<->B */
                    constexpr float BC = 0.0f;      /* optional coupling damping A<->B */

                    /* Clamp reflection to a fraction of actuator capability (from params). */
                    const float tauA_limit = 0.35f * ACTUATOR_PARAMS[info.board][BUS][A].t_max;

                    if (info.node == A) {
                        /* With kp=kd=0, p_des/v_des don't matter; keep them 0 to avoid any vendor-side behavior. */
                        p_target = 0.0f;
                        v_target = 0.0f;
                        t_ff = alpha * (KF * tauB + KC * (qB - qA) + BC * (dqB - dqA));
                        if (t_ff > tauA_limit) t_ff = tauA_limit;
                        if (t_ff < -tauA_limit) t_ff = -tauA_limit;

                        bus_list[BUS].command.j[A].p_des = p_target;
                        bus_list[BUS].command.j[A].v_des = v_target;
                        bus_list[BUS].command.j[A].kp = KP_A;
                        bus_list[BUS].command.j[A].kd = KD_A;
                        bus_list[BUS].command.j[A].t_ff = t_ff;
                    } else { /* B */
                        const float q_des_follow = qA + (qB0 - qA0); /* preserve initial alignment */
                        p_target = (1.0f - alpha) * qB0 + alpha * q_des_follow;
                        v_target = alpha * dqA;
                        t_ff = 0.0f;

                        bus_list[BUS].command.j[B].p_des = p_target;
                        bus_list[BUS].command.j[B].v_des = v_target;
                        bus_list[BUS].command.j[B].kp = KP_B;
                        bus_list[BUS].command.j[B].kd = KD_B;
                        bus_list[BUS].command.j[B].t_ff = t_ff;
                    }

                    if (iter % 200 == 0 && info.node == A) {
                        std::cout << "teleop alpha=" << alpha
                                  << " qA=" << qA << " qB=" << qB
                                  << " tauB=" << tauB << " tauA_cmd=" << bus_list[BUS].command.j[A].t_ff
                                  << std::endl;
                    }
                    continue;
                }
                else if (mode == 1)
                {
                    float t = iter * dt;
                    p_target = p_center + PRIVATE_SINE_AMP * std::sin(t);
                }

                bus_list[info.bus].command.j[info.node].p_des = p_target;
                bus_list[info.bus].command.j[info.node].v_des = v_target;
                bus_list[info.bus].command.j[info.node].kp = 25.0f;
                bus_list[info.bus].command.j[info.node].kd = 2.5f;
                bus_list[info.bus].command.j[info.node].t_ff = t_ff;

                if (iter % 100 == 0)
                {
                    std::cout << "p=" << bus_list[info.bus].state.j[info.node].p
                              << " p_des=" << p_target << " v_des=" << v_target
                              << " t_ff=" << t_ff << std::endl;
                }
            }

            for (size_t b = 0; b < host_cfg::kNumTeensyBoards; b++)
                _spine_boards[b]->setBusList(boards_bus_lists[b]);

            iter++;
            if (mode == 2) {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(dt_us));
            }
        }
    }
    else
    {
        std::cout << "Unsupported mode. Only 0, 1, 2 are available.\n";
        return 1;
    }

    for (size_t id = 0; id < _spine_boards.size(); ++id)
        _spine_boards[id]->end();

    return 0;
}
