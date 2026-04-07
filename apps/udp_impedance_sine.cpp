#include <asio.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ifaddrs.h>
#include <netdb.h>
#include <netinet/in.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "utils.h"

using asio::ip::udp;

namespace {
constexpr int kWireNumLogicalBuses = 2;
constexpr int kWireMaxNodesPerBus = 3;
constexpr int kWirePayloadBytes = kWireNumLogicalBuses * kWireMaxNodesPerBus * 8;  // 48 bytes

static asio::ip::address_v4 resolve_interface_ipv4(const std::string &iface) {
    struct ifaddrs *ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == -1) {
        throw std::runtime_error("getifaddrs failed");
    }

    asio::ip::address_v4 out;
    bool found = false;
    for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr)
            continue;
        if (ifa->ifa_addr->sa_family != AF_INET)
            continue;
        if (iface != ifa->ifa_name)
            continue;

        char host[NI_MAXHOST];
        int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, nullptr, 0,
                            NI_NUMERICHOST);
        if (s != 0) {
            freeifaddrs(ifaddr);
            throw std::runtime_error("getnameinfo failed");
        }
        out = asio::ip::make_address_v4(host);
        found = true;
        break;
    }

    freeifaddrs(ifaddr);
    if (!found || out.is_unspecified()) {
        throw std::runtime_error("Interface IPv4 not found: " + iface);
    }
    return out;
}

static void usage(const char *argv0) {
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " --iface IFACE --teensy-ip IP [--port 8003]\n"
        << "         [--rate-hz 1000] [--amp 1.0] [--omega 1.0]\n"
        << "         [--kp 0] [--kd 0] [--tff 0] [--actuator ak60]\n"
        << "\n"
        << "Sends MIT impedance commands (p_des from sine wave) over UDP to the Teensy.\n"
        << "Defaults are intentionally backdrivable (kp=0).\n";
}

static bool arg_eq(const char *a, const char *b) { return std::string(a) == std::string(b); }

struct MitBus {
    bus b;
    std::vector<joint_state> state;
    std::vector<joint_control> command;
    std::vector<ActuatorParams> params;

    explicit MitBus(int nodes, ActuatorType type) : state(nodes), command(nodes), params(nodes) {
        b.state.j = state.data();
        b.command.j = command.data();
        b.params = params.data();
        b.params_vec = params;
        for (int i = 0; i < nodes; ++i) {
            params[i] = getActuatorParams(type);
            b.params_vec[i] = params[i];
        }
    }
};
}  // namespace

int main(int argc, char **argv) {
    std::string iface;
    std::string teensy_ip;
    int port = 8003;
    int rate_hz = 1000;
    float amp = 1.0f;
    float omega = 1.0f;  // rad/s
    float kp = 0.0f;     // default: do not lock
    float kd = 0.0f;
    float tff = 0.0f;
    ActuatorType actuator = ActuatorType::AK_60_6;

    for (int i = 1; i < argc; ++i) {
        if (arg_eq(argv[i], "--help") || arg_eq(argv[i], "-h")) {
            usage(argv[0]);
            return 0;
        } else if (arg_eq(argv[i], "--iface") && i + 1 < argc) {
            iface = argv[++i];
        } else if (arg_eq(argv[i], "--teensy-ip") && i + 1 < argc) {
            teensy_ip = argv[++i];
        } else if (arg_eq(argv[i], "--port") && i + 1 < argc) {
            port = std::stoi(argv[++i]);
        } else if (arg_eq(argv[i], "--rate-hz") && i + 1 < argc) {
            rate_hz = std::stoi(argv[++i]);
        } else if (arg_eq(argv[i], "--amp") && i + 1 < argc) {
            amp = std::stof(argv[++i]);
        } else if (arg_eq(argv[i], "--omega") && i + 1 < argc) {
            omega = std::stof(argv[++i]);
        } else if (arg_eq(argv[i], "--kp") && i + 1 < argc) {
            kp = std::stof(argv[++i]);
        } else if (arg_eq(argv[i], "--kd") && i + 1 < argc) {
            kd = std::stof(argv[++i]);
        } else if (arg_eq(argv[i], "--tff") && i + 1 < argc) {
            tff = std::stof(argv[++i]);
        } else if (arg_eq(argv[i], "--actuator") && i + 1 < argc) {
            std::string a = argv[++i];
            if (a == "ak60")
                actuator = ActuatorType::AK_60_6;
            else if (a == "ak10")
                actuator = ActuatorType::AK_10_9;
            else {
                std::cerr << "Unknown actuator type: " << a << " (use ak60 or ak10)\n";
                return 2;
            }
        } else {
            std::cerr << "Unknown/invalid argument: " << argv[i] << "\n";
            usage(argv[0]);
            return 2;
        }
    }

    if (iface.empty() || teensy_ip.empty()) {
        usage(argv[0]);
        return 2;
    }
    if (rate_hz <= 0) {
        std::cerr << "rate_hz must be > 0\n";
        return 2;
    }

    try {
        constexpr int kNumNodes = 2;  // bus0 node0,node1
        MitBus bus0(kNumNodes, actuator);

        asio::io_context io;
        udp::socket sock(io);
        sock.open(udp::v4());

        asio::ip::address_v4 bind_addr = resolve_interface_ipv4(iface);
        sock.bind(udp::endpoint(bind_addr, 0));

        udp::endpoint teensy_ep(asio::ip::make_address(teensy_ip), port);
        std::cout << "Sending UDP MIT @ " << rate_hz << " Hz from " << sock.local_endpoint() << " -> "
                  << teensy_ep << "\n";
        std::cout << "Sine: amp=" << amp << " rad, omega=" << omega << " rad/s, kp=" << kp << " kd=" << kd
                  << " tff=" << tff << "\n";

        const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate_hz));
        auto t0 = std::chrono::steady_clock::now();
        auto next = t0 + period;

        std::vector<uint8_t> payload(kWirePayloadBytes, 0);
        std::vector<uint8_t> packet(kWirePayloadBytes + 1, 0);

        uint64_t iter = 0;
        while (true) {
            auto now = std::chrono::steady_clock::now();
            float t = std::chrono::duration_cast<std::chrono::duration<float>>(now - t0).count();

            float p_des = amp * std::sin(omega * t);
            float v_des = amp * omega * std::cos(omega * t);

            // bus0 node0 and node1 into wire payload offsets 0..15
            for (int n = 0; n < kNumNodes; ++n) {
                bus0.b.command.j[n].p_des = p_des;
                bus0.b.command.j[n].v_des = v_des;
                bus0.b.command.j[n].kp = kp;
                bus0.b.command.j[n].kd = kd;
                bus0.b.command.j[n].t_ff = tff;
                pack_cmd(payload.data() + n * 8, bus0.b, n);
            }

            uint8_t crc = calculate_crc8(payload.data(), payload.size());
            std::memcpy(packet.data(), payload.data(), payload.size());
            packet[payload.size()] = crc;

            sock.send_to(asio::buffer(packet), teensy_ep);

            if ((iter++ % static_cast<uint64_t>(rate_hz)) == 0) {
                std::cout << "t=" << t << " p_des=" << p_des << " v_des=" << v_des << " crc=" << int(crc)
                          << "\n";
            }

            std::this_thread::sleep_until(next);
            next += period;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

