#ifndef SPINE_BOARD_H
#define SPINE_BOARD_H
/*
 * SpineBoard — one Teensy reachable over UDP: pack MIT commands, unpack feedback into bus_list.
 *
 * Constructor: (nodes, buses) define the host-side logical layout; send_data_to_teensy() maps that into
 * the fixed-size UDP payload (kTeensy* in spine_board.cpp) which must match teensy_mt/teensy_mt.ino.
 *
 * Threads: receive_thread blocks on server_socket (feedback from Teensy); send_thread runs initBoard() once
 * then loops packing pack_cmd and sending. Optional wait-for-feedback pairs one send with one receive.
 */
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <vector>
#include <asio.hpp>
#include "utils.h"
#define ZERO_ENCODERS /* zeroEncoders() in start() may send encoder-zero frames where params say so */

using asio::ip::udp;
class SpineBoard
{
private:
    const int num_nodes;  /* joints per logical CAN bus */
    const int num_buses; /* logical buses (rows in UDP wire) */
    std::string teensy_ip;
    int teensy_port; /* same port Teensy passes to udp.begin() */
    bool first_state_received;
    std::vector<bus> bus_list; /* state/command/params per bus — protected by bus_list_mutex */

    asio::io_context io_context;
    asio::ip::udp::socket sock_send;     /* outbound to teensy_ip:teensy_port */
    asio::ip::udp::socket server_socket; /* inbound: bound to interface:teensy_port (Teensy sends feedback here) */

    bool actuator_params_set = false;
    mutable std::mutex bus_list_mutex;
    std::thread receive_thread;
    std::thread send_thread;
    std::string board_name;
    /* After init, when false the send thread sleeps and does not stream MIT (enable-only / idle). */
    std::atomic<bool> allow_command_send_{false};

    /*
     * When true: after each UDP command to Teensy, send thread waits until receive_thread gets one datagram.
     * Matches firmware that accepts the next command only after motor feedback (see teensy_mt.ino).
     */
    std::atomic<bool> wait_for_feedback_after_send_{true};
    std::mutex feedback_sync_mutex_;
    std::condition_variable feedback_cv_;
    bool feedback_received_{false};

    /* Serialize UDP sends so exit/enable handshake does not interleave with the MIT stream. */
    std::mutex send_udp_mutex_;
    void send_payload_unlocked(const std::vector<uint8_t> &data, int data_size);
    /** Exit motor mode, MIT zero, enter motor mode, MIT zero (uses send_payload_unlocked). */
    void mitExitZeroEnterZeroHandshake();

public:
    /** @param nodes  joints per bus  @param buses  number of logical CAN buses  @param port  UDP port (host listens and sends) */
    SpineBoard(const std::string &ip, const std::string &interface, int port, int nodes, int buses, std::string board_name = "board_1");

    void setActuatorParams(const std::vector<std::vector<ActuatorParams>> &params)
    {
        if (static_cast<int>(params.size()) != num_buses)
        {
            std::cout << "Error: Invalid parameters size. Expected " << num_buses << " buses." << std::endl;
            return;
        }
        for (int j = 0; j < num_buses; j++)
        {
            if (static_cast<int>(params[j].size()) != num_nodes)
            {
                std::cout << "Error: Invalid parameters row " << j << ". Expected " << num_nodes << " nodes per bus." << std::endl;
                return;
            }
        }

        for (int j = 0; j < num_buses; j++)
        {
            for (int i = 0; i < num_nodes; i++)
            {
                bus_list[j].params[i] = params[j][i];
            }
            bus_list[j].params_vec = params[j];
        }
        actuator_params_set = true;
    }

    std::vector<bus> getBusList() const
    {
        std::lock_guard<std::mutex> lock(bus_list_mutex);
        return bus_list;
    }

    void setBusList(const std::vector<bus> &new_bus_list)
    {
        std::lock_guard<std::mutex> lock(bus_list_mutex);
        bus_list = new_bus_list;
    }
    void setAllowCommandSend(bool allow) { allow_command_send_ = allow; }
    /** Enable/disable RTT pacing between MIT send and next feedback (default true). */
    void setWaitForFeedbackAfterSend(bool wait) { wait_for_feedback_after_send_ = wait; }
    /** Block until a full exit → MIT zero → enter → MIT zero sequence is sent (thread-safe vs send thread). */
    void exitAndEnableMotorMode();
    ~SpineBoard()
    {
        for (int j = 0; j < num_buses; j++)
        {
            delete[] bus_list[j].state.j;
            delete[] bus_list[j].command.j;
            delete[] bus_list[j].params;
        }
    }
    void process_data(const std::vector<uint8_t> &data_list);
    void send_data_to_teensy(const std::vector<uint8_t> &data, const int data_size);
    void handle_udp_packet(const asio::ip::udp::endpoint &client_endpoint, const std::vector<uint8_t> &data);
    void update_command();
    void initBoard();
    void start();
    void end()
    {
        join();
        closeSockets();
    }
    void closeSockets()
    {
        try
        {
            sock_send.close();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error closing sock_send: " << e.what() << std::endl;
        }

        // Close the server socket
        try
        {
            server_socket.close();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error closing server_socket: " << e.what() << std::endl;
        }
    }
    void join()
    {
        if (receive_thread.joinable())
        {
            receive_thread.join();
        }
        if (send_thread.joinable())
        {
            send_thread.join();
        }
    }
    void zeroEncoders();
    /** Set true at end of send-thread init (after initBoard / zeroEncoders). */
    bool boardInitialized = false;
};
#endif // SPINE_BOARD_H