#include "spine_board.h"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

namespace {
/*
 * UDP wire layout (bytes on the wire before CRC8):
 *   [ bus0: node0..nodeN-1 ][ bus1: node0.. ]  with N ≤ kTeensyMaxNodesPerBus, 8 bytes per node (MIT frame).
 * Must match TEENSY_* constants at the top of teensy_mt/teensy_mt.ino.
 */
constexpr int kTeensyNumLogicalBuses = 2;
constexpr int kTeensyMaxNodesPerBus = 3;
constexpr int kTeensyPayloadBytes = kTeensyNumLogicalBuses * kTeensyMaxNodesPerBus * 8;
}  // namespace

// -----------------------------------------------------------------------------
// Teensy packet layout and helpers
// -----------------------------------------------------------------------------
/* Map SpineBoard’s linear buffer (num_buses × num_nodes × 8) into the padded UDP payload (zeros in unused slots). */
static void logical_to_teensy_payload(const uint8_t *logical, int num_buses, int num_nodes, uint8_t *out48)
{
    std::memset(out48, 0, kTeensyPayloadBytes);
    const int nb = num_buses < kTeensyNumLogicalBuses ? num_buses : kTeensyNumLogicalBuses;
    const int nn = num_nodes < kTeensyMaxNodesPerBus ? num_nodes : kTeensyMaxNodesPerBus;
    for (int bus = 0; bus < nb; ++bus)
    {
        for (int node = 0; node < nn; ++node)
        {
            std::memcpy(out48 + bus * kTeensyMaxNodesPerBus * 8 + node * 8,
                        logical + bus * num_nodes * 8 + node * 8, 8);
        }
    }
}

// -----------------------------------------------------------------------------
// Constructor: resolve interface, bind UDP sockets, allocate bus_list[joint arrays]
// -----------------------------------------------------------------------------
SpineBoard::SpineBoard(const std::string &ip, const std::string &interface, int port, int nodes, int buses, std::string _board_name)
    : num_nodes(nodes), num_buses(buses), teensy_ip(ip), teensy_port(port),
      first_state_received(false), bus_list(buses),
      sock_send(io_context), server_socket(io_context), board_name(_board_name)
{
    // -------- Resolve bind address: requested interface or first non-loopback --------
    asio::ip::address_v4 bind_address;
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];
    bool found = false;

    if (getifaddrs(&ifaddr) == -1)
    {
        perror("getifaddrs");
        exit(EXIT_FAILURE);
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        if (family == AF_INET && strcmp(ifa->ifa_name, interface.c_str()) == 0)
        {
            s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                            host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0)
            {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                exit(EXIT_FAILURE);
            }
            bind_address = asio::ip::make_address_v4(host);
            found = true;
            break;
        }
    }

    // Fallback: first non-loopback IPv4 interface (e.g. eth0, enp0s3 when eno1 missing)
    if (!found)
    {
        for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == NULL)
                continue;
            family = ifa->ifa_addr->sa_family;
            if (family != AF_INET)
                continue;
            s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                            host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0)
                continue;
            asio::ip::address_v4 addr = asio::ip::make_address_v4(host);
            if (addr.is_loopback())
                continue;
            bind_address = addr;
            found = true;
            std::cout << "Interface '" << interface << "' not found, using " << ifa->ifa_name
                      << " (" << host << ")" << std::endl;
            break;
        }
    }

    freeifaddrs(ifaddr);

    if (!found || bind_address.is_unspecified())
    {
        std::cerr << "Failed to find the " << interface << " interface IP address (no fallback)" << std::endl;
        throw std::runtime_error("Failed to find network interface IP address");
    }

    // -------- Bind UDP sockets --------
    sock_send.open(asio::ip::udp::v4());
    sock_send.bind(asio::ip::udp::endpoint(bind_address, 0));

    server_socket.open(asio::ip::udp::v4());
    server_socket.bind(asio::ip::udp::endpoint(bind_address, teensy_port));

    std::cout << "Server bound to " << server_socket.local_endpoint() << std::endl;

    // -------- Allocate bus_list (state, command, params) --------
    for (int j = 0; j < num_buses; j++)
    {
        bus_list[j].state.j = new joint_state[num_nodes];
        bus_list[j].command.j = new joint_control[num_nodes];
        bus_list[j].params = new ActuatorParams[num_nodes];
        bus_list[j].params_vec = std::vector<ActuatorParams>(num_nodes);

        for (int i = 0; i < num_nodes; i++)
        {
            bus_list[j].params[i] = getActuatorParams(ActuatorType::AK_60_6);
            bus_list[j].params_vec[i] = getActuatorParams(ActuatorType::AK_60_6);
        }
    }
}

// -----------------------------------------------------------------------------
// initBoard — reset, then exit / MIT zero / enter / MIT zero (same as exitAndEnableMotorMode without UDP lock contention)
// -----------------------------------------------------------------------------
void SpineBoard::initBoard()
{
    if (!actuator_params_set)
    {
        std::cerr << "Actuator parameters not set. Exiting..." << std::endl;
        exit(-1);
    }

    std::vector<uint8_t> reset_data(1, 0xFF);
    send_data_to_teensy(reset_data, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));

    mitExitZeroEnterZeroHandshake();
    std::cout << "initBoard: ready\n";
}

void SpineBoard::mitExitZeroEnterZeroHandshake()
{
    std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);

    for (int j = 0; j < num_buses; j++)
    {
        uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;
        for (int i = 0; i < num_nodes; i++)
            pack_exit_motor_mode_cmd(bus_data + i * 8);
    }
    send_payload_unlocked(data_to_send, num_buses * num_nodes * 8);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));

    {
        std::lock_guard<std::mutex> lock(bus_list_mutex);
        for (int j = 0; j < num_buses; j++)
        {
            for (int i = 0; i < num_nodes; i++)
            {
                bus_list[j].command.j[i].v_des = 0.0f;
                bus_list[j].command.j[i].p_des = 0.0f;
                bus_list[j].command.j[i].kp = 0.0f;
                bus_list[j].command.j[i].kd = 0.0f;
                bus_list[j].command.j[i].t_ff = 0.0f;
            }
        }
        for (int j = 0; j < num_buses; j++)
        {
            bus &current_bus = bus_list[j];
            uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;
            for (int i = 0; i < num_nodes; i++)
                pack_cmd(bus_data + i * 8, current_bus, i);
        }
    }
    send_payload_unlocked(data_to_send, num_buses * num_nodes * 8);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));

    for (int j = 0; j < num_buses; j++)
    {
        uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;
        for (int i = 0; i < num_nodes; i++)
            pack_enter_motor_mode_cmd(bus_data + i * 8);
    }
    send_payload_unlocked(data_to_send, num_buses * num_nodes * 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::microseconds(500000));

    {
        std::lock_guard<std::mutex> lock(bus_list_mutex);
        for (int j = 0; j < num_buses; j++)
        {
            for (int i = 0; i < num_nodes; i++)
            {
                bus_list[j].command.j[i].v_des = 0.0f;
                bus_list[j].command.j[i].p_des = 0.0f;
                bus_list[j].command.j[i].kp = 0.0f;
                bus_list[j].command.j[i].kd = 0.0f;
                bus_list[j].command.j[i].t_ff = 0.0f;
            }
        }
        for (int j = 0; j < num_buses; j++)
        {
            bus &current_bus = bus_list[j];
            uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;
            for (int i = 0; i < num_nodes; i++)
                pack_cmd(bus_data + i * 8, current_bus, i);
        }
    }
    send_payload_unlocked(data_to_send, num_buses * num_nodes * 8);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));
}

void SpineBoard::exitAndEnableMotorMode()
{
    std::lock_guard<std::mutex> udp_lock(send_udp_mutex_);
    mitExitZeroEnterZeroHandshake();
}

void SpineBoard::send_payload_unlocked(const std::vector<uint8_t> &data, const int data_size)
{
    std::vector<uint8_t> payload;
    if (data_size == kTeensyPayloadBytes)
    {
        if (static_cast<int>(data.size()) < kTeensyPayloadBytes)
        {
            std::cerr << "send_payload_unlocked: need " << kTeensyPayloadBytes << " bytes, got " << data.size() << std::endl;
            return;
        }
        payload.assign(data.begin(), data.begin() + kTeensyPayloadBytes);
    }
    else if (data_size == num_buses * num_nodes * 8 && data_size <= kTeensyPayloadBytes)
    {
        payload.resize(kTeensyPayloadBytes);
        logical_to_teensy_payload(data.data(), num_buses, num_nodes, payload.data());
    }
    else
    {
        payload.assign(data.begin(), data.end());
        payload.resize(static_cast<size_t>(data_size), 0);
    }

    uint8_t crc_value = calculate_crc8(payload.data(), payload.size());
    std::vector<uint8_t> packet(payload);
    packet.push_back(crc_value);
    sock_send.send_to(asio::buffer(packet), udp::endpoint(asio::ip::make_address(teensy_ip), teensy_port));
}

void SpineBoard::zeroEncoders()
{
    std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);

    bool any_recalibrate = false;
    for (int j = 0; j < num_buses; j++)
    {
        uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;

        for (int i = 0; i < num_nodes; i++)
        {
            if (bus_list[j].params[i].recalibrate)
            {
                pack_zero_encoder(bus_data + i * 8);
                any_recalibrate = true;
                printf("Zeroing encoder for bus %d, node %d\n", j, i);
            }
            else
            {
                // Send zero command for actuators not being recalibrated
                pack_cmd(bus_data + i * 8, bus_list[j], i);
            }
        }
    }

    if (any_recalibrate)
    {
        send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
        std::this_thread::sleep_for(std::chrono::microseconds(1000000));
        printf("Zero encoder sent\n");
    }
}

// -----------------------------------------------------------------------------
// start — receive_thread: blocking recv from Teensy; notify send thread when feedback arrives (if pacing on).
//         send_thread: initBoard once, then MIT command loop.
// -----------------------------------------------------------------------------
void SpineBoard::start()
{
    std::cout << "UDP server listening on " << server_socket.local_endpoint() << std::endl;

    // Incoming feedback from Teensy (same port as we use for outbound; reply source addr).
    receive_thread = std::thread([&]()
                                 {
            while (true) {
                std::vector<uint8_t> recv_buffer(kTeensyPayloadBytes + 16);
                asio::ip::udp::endpoint client_endpoint;
                size_t bytes_received = server_socket.receive_from(asio::buffer(recv_buffer), client_endpoint);
                std::vector<uint8_t> received_data(recv_buffer.begin(), recv_buffer.begin() + bytes_received);
                handle_udp_packet(client_endpoint, received_data);
                if (wait_for_feedback_after_send_.load()) {
                    {
                        std::lock_guard<std::mutex> lk(feedback_sync_mutex_);
                        feedback_received_ = true;
                    }
                    feedback_cv_.notify_one();
                } else {
                    /* Legacy pacing when not waiting on Teensy (avoids tight loop). */
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                }
            } });

    /* Outbound MIT: pack_cmd for each joint, send_data_to_teensy; optionally block on feedback_cv_. */
    send_thread = std::thread([&]()
                              {
            bool first_time = true;
            while (true) {
                if (first_time)
                {

                    initBoard();
                    zeroEncoders();
                    boardInitialized = true;

                    first_time = false;
                    continue;
                }

                // When false, do not send so Teensy keeps last MIT frame (e.g. enable-only mode).
                if (!allow_command_send_) {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    continue;
                }

                update_command();

                std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);

                {
                    /* bus_list is shared with user threads via getBusList()/setBusList() and process_data(). */
                    std::lock_guard<std::mutex> lock(bus_list_mutex);
                    for (int j = 0; j < num_buses; j++) {
                        bus& current_bus = bus_list[j];
                        uint8_t* bus_data = data_to_send.data() + j * num_nodes * 8;
                        for (int i = 0; i < num_nodes; i++) {
                            pack_cmd(bus_data + i * 8, current_bus, i);
                        }
                    }
                }

                if (wait_for_feedback_after_send_.load()) {
                    {
                        std::unique_lock<std::mutex> lk(feedback_sync_mutex_);
                        feedback_received_ = false;
                    }
                }

                send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);

                if (wait_for_feedback_after_send_.load()) {
                    std::unique_lock<std::mutex> lk(feedback_sync_mutex_);
                    feedback_cv_.wait(lk, [&]{ return feedback_received_; });
                } else {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                }
            } });
}

// -----------------------------------------------------------------------------
// UDP send: payload (kTeensyPayloadBytes) + CRC8 — same CRC polynomial as teensy.ino
// -----------------------------------------------------------------------------
void SpineBoard::send_data_to_teensy(const std::vector<uint8_t> &data, const int data_size)
{
    std::lock_guard<std::mutex> lock(send_udp_mutex_);
    send_payload_unlocked(data, data_size);
}

// -----------------------------------------------------------------------------
// process_data — unpack_reply() for each joint; offsets follow bus-major layout.
// -----------------------------------------------------------------------------
void SpineBoard::process_data(const std::vector<uint8_t> &data_list)
{
    std::lock_guard<std::mutex> lock(bus_list_mutex);

    if (data_list.size() >= static_cast<size_t>(kTeensyPayloadBytes)) {
        /* Padded wire: bus j at j * kTeensyMaxNodesPerBus * 8 (must match logical_to_teensy_payload). */
        for (int j = 0; j < num_buses; ++j) {
            const size_t bus_wire_off = static_cast<size_t>(j) * kTeensyMaxNodesPerBus * 8;
            for (int i = 0; i < num_nodes; ++i) {
                const size_t off = bus_wire_off + static_cast<size_t>(i) * 8;
                std::vector<uint8_t> node_data(data_list.begin() + off, data_list.begin() + off + 8);
                unpack_reply(node_data, bus_list[j], i);
            }
        }
        return;
    }

    for (int j = 0; j < num_buses; ++j) {
        const size_t bus_offset = static_cast<size_t>(j) * static_cast<size_t>(num_nodes) * 8;
        if (bus_offset + static_cast<size_t>(num_nodes) * 8 > data_list.size())
            break;
        for (int i = 0; i < num_nodes; ++i) {
            const size_t off = bus_offset + static_cast<size_t>(i) * 8;
            std::vector<uint8_t> node_data(data_list.begin() + off, data_list.begin() + off + 8);
            unpack_reply(node_data, bus_list[j], i);
        }
    }
}

void SpineBoard::update_command()
{
}

// -----------------------------------------------------------------------------
// handle_udp_packet — called from receive_thread after recv_from
// -----------------------------------------------------------------------------
void SpineBoard::handle_udp_packet(const udp::endpoint &client_endpoint, const std::vector<uint8_t> &data)
{
    (void)client_endpoint;
    std::vector<uint8_t> data_list(data.begin(), data.end());

    if (boardInitialized)
        process_data(data_list);
    if (!first_state_received)
        first_state_received = true;
}
