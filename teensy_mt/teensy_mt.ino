/*
 * Teensy 4.1 — Ethernet UDP to T-Motor AK (MIT mode) on 11-bit CAN.
 *
 * Data flow (one “round”):
 *   1) PC sends UDP: payload bytes + CRC8 (host builds the same layout as spine_board.cpp).
 *   2) We unpack into per-bus command buffers, then transmit MIT frames on CAN (see CONFIG for which port).
 *   3) While waiting for this round’s feedback, we re-send the same MIT at MIT_RESEND_MIN_US (drive expects a stream).
 *   4) After feedback from NUM_DAISY_MOTORS motors (order: bus0 nodes, then bus1 nodes), we reply to PC with UDP.
 *   5) Until (4) completes, further UDP commands are ignored (parsePacket still drains the queue).
 *
 * Wire size: TEENSY_NUM_LOGICAL_BUSES × TEENSY_MAX_NODES_PER_BUS × 8 — must match spine_board.cpp anonymous constants.
 * Physical CAN: logical bus 0 → FlexCAN CAN3; logical bus 1 → CAN2 when NUM_ACTIVE_NODES_BUS1 > 0.
 */

 #include <FlexCAN_T4.h>
 #include <QNEthernet.h>
 #define NUM_TX_MAILBOXES 32
 #define NUM_RX_MAILBOXES 32
 using namespace qindesign::network;
 
 /* Fixed UDP/MIT layout (change together with spine_board.cpp kTeensy* and main.cpp host_cfg::kWire*). */
 constexpr int TEENSY_NUM_LOGICAL_BUSES = 2;
 constexpr int TEENSY_MAX_NODES_PER_BUS = 3;
 constexpr int TEENSY_UDP_PAYLOAD_BYTES = TEENSY_NUM_LOGICAL_BUSES * TEENSY_MAX_NODES_PER_BUS * 8;
 
 // =============================================================================
 // CONFIG — topology and IDs (mirror host_cfg / SpineBoard constructor in apps/main.cpp)
 // =============================================================================
 /* Motors physically on CAN3 (logical bus 0), node indices 0 .. N-1 in the UDP wire’s first bus block. */
 #define NUM_ACTIVE_NODES_BUS0 2
 /* Motors on CAN2 (logical bus 1). Set to 0 to use only CAN3; enable only if wired to CAN2. */
 #define NUM_ACTIVE_NODES_BUS1 0
 /* Base CAN ID on each physical bus: motor k uses MOTOR_ID_BASE + k (11-bit). */
 #define MOTOR_ID_BASE 1
 /*
  * Gate width: how many motors must send feedback before we accept the next UDP command.
  * Typically all active motors; flat order is bus0 nodes first, then bus1 nodes (see flatMotorIndex()).
  */
 #define NUM_DAISY_MOTORS (NUM_ACTIVE_NODES_BUS0 + NUM_ACTIVE_NODES_BUS1)
 
 /*
  * UDP gating:
  * - When 1 (default): accept one UDP command only after feedback from NUM_DAISY_MOTORS motors (safer, matches old host pacing).
  * - When 0: accept commands continuously (required for true ~1kHz command streaming / haptics loops).
  *   Feedback UDP will be sent periodically at `FEEDBACK_UDP_PERIOD_US` using the latest cached CAN frames.
  */
 #define GATE_UDP_ON_DAISY_FEEDBACK 1
 constexpr uint32_t FEEDBACK_UDP_PERIOD_US = 1000;
 
 static_assert(NUM_ACTIVE_NODES_BUS0 >= 1 && NUM_ACTIVE_NODES_BUS0 <= TEENSY_MAX_NODES_PER_BUS,
               "NUM_ACTIVE_NODES_BUS0");
 static_assert(NUM_ACTIVE_NODES_BUS1 >= 0 && NUM_ACTIVE_NODES_BUS1 <= TEENSY_MAX_NODES_PER_BUS,
               "NUM_ACTIVE_NODES_BUS1");
 static_assert(NUM_DAISY_MOTORS >= 1 &&
                   NUM_DAISY_MOTORS <= TEENSY_MAX_NODES_PER_BUS * TEENSY_NUM_LOGICAL_BUSES,
               "NUM_DAISY_MOTORS");
 
 #if NUM_ACTIVE_NODES_BUS1 > 0
 FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CanBus1; /* logical bus 1 */
 #endif
 FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CanBus0; /* logical bus 0 */
 
 void canReceiveBus0(const CAN_message_t &msg);
 #if NUM_ACTIVE_NODES_BUS1 > 0
 void canReceiveBus1(const CAN_message_t &msg);
 #endif
 
 /* Set USE_STATIC_IP 0 to use DHCP (see kDHCPTimeout). */
 #define USE_STATIC_IP 1
 #if USE_STATIC_IP
 IPAddress teensyIP(192, 168, 0, 101);
 IPAddress teensySubnet(255, 255, 255, 0);
 IPAddress teensyGateway(192, 168, 0, 1);
 #endif
 constexpr uint32_t kDHCPTimeout = 15000;
 constexpr uint16_t kPort = 8003; /* UDP port: must match SpineBoard / PC */
 
 constexpr int MAX_SLOTS = TEENSY_MAX_NODES_PER_BUS; /* per-bus slot count in wire (padding unused slots) */
 constexpr int MAX_MOTORS_TOTAL = TEENSY_MAX_NODES_PER_BUS * TEENSY_NUM_LOGICAL_BUSES;
 
 /* Rolling-window stats: print averages every MAX_NUM_SAMPLES feedbacks per motor. */
 constexpr int MAX_NUM_SAMPLES = 5000;
 
 /* Disable to maximize comm/control loop frequency (no time accumulation, no Serial timing prints). */
 #define ENABLE_TIMING_STATS 0
 
 EthernetUDP udp;
 
 /* Latest MIT command from host (per slot); feedback snapshots in can_data_*. */
 uint8_t can_data_b0[MAX_SLOTS][8];
 uint8_t can_command_b0[MAX_SLOTS][8];
 uint8_t can_data_b1[MAX_SLOTS][8];
 uint8_t can_command_b1[MAX_SLOTS][8];
 
 #if ENABLE_TIMING_STATS
 /* Per-motor timing accumulators; index = flatMotorIndex(bus, local_node). */
 unsigned long last_packet_time[MAX_MOTORS_TOTAL] = {0};
 unsigned long total_fb_interval[MAX_MOTORS_TOTAL] = {0};
 unsigned int packet_count[MAX_MOTORS_TOTAL] = {0};
 
 unsigned long last_cmd_time[MAX_MOTORS_TOTAL] = {0};
 unsigned long total_cmd_latency[MAX_MOTORS_TOTAL] = {0};
 unsigned int cmd_count[MAX_MOTORS_TOTAL] = {0};
 
 /* One-shot “first feedback after this UDP batch” for Serial (uses first CAN TX time of batch). */
 bool udp_cmd_waiting_fb[MAX_MOTORS_TOTAL] = {false};
 
 /* Daisy summary line: stash each motor’s avg cmd→fb until all NUM_DAISY_MOTORS have a sample (+1 avoids 0 sentinel). */
 static unsigned long last_printed_cmd_to_fb_daisy[MAX_MOTORS_TOTAL] = {0};
 #endif
 
 /* Round-trip state: waiting_for_daisy_fb true = accepted UDP, not yet received all required feedbacks. */
 static bool waiting_for_daisy_fb = false;
 static bool fb_received[NUM_DAISY_MOTORS] = {false};
 static bool pending_can_send = false;  /* run sendCANCMD on next loop (or after resend interval) */
 static bool pending_udp_send = false; /* send feedback UDP to PC */
 static bool have_valid_command = false;
 
 const uint8_t RESET_COMMAND = 0xFF;
 
 constexpr uint32_t MIT_RESEND_MIN_US = 200; /* min spacing between MIT retransmits during an open round */
 static uint32_t last_mit_tx_us = 0;
 /* Only the first sendCANCMD after a new UDP sets last_cmd_time (latency stats stay meaningful). */
 static bool first_mit_tx_this_batch = false;
 
 /* Maps (bus, local_node) to a single index 0.. for arrays sized MAX_MOTORS_TOTAL. */
 static inline int flatMotorIndex(int bus, int local_node) {
     return (bus == 0) ? local_node : (NUM_ACTIVE_NODES_BUS0 + local_node);
 }
 
 static inline uint16_t motorCanId(int local_node_index) {
     return (uint16_t)(MOTOR_ID_BASE + local_node_index) & 0x7FF;
 }
 
 /* AK startup: bytes 0..6 = 0xFF, byte 7 = magic (exit / zero / enter). */
 static void fillMagicPayload(uint8_t out[8], uint8_t magic) {
     for (int i = 0; i < 7; ++i)
         out[i] = 0xFF;
     out[7] = magic;
 }
 
 /*
  * Identify which local node (0..MAX_SLOTS-1) this 11-bit AK feedback belongs to,
  * using buf[0] motor id or matching standard ID.
  */
 static int akFeedbackLocalNode(const CAN_message_t &msg) {
     if (msg.flags.extended || msg.len < 6)
         return -1;
     int node_id = (int)msg.buf[0] - MOTOR_ID_BASE;
     if (node_id >= 0 && node_id < MAX_SLOTS)
         return node_id;
     uint16_t id11 = (uint16_t)(msg.id & 0x7FF);
     for (int n = 0; n < MAX_SLOTS; n++) {
         if (id11 == motorCanId(n))
             return n;
     }
     return -1;
 }
 
 /* Split host UDP payload into bus0 and bus1 command buffers (fixed layout). */
 static void unpackWire48ToBuses(const uint8_t *data) {
     for (int node = 0; node < MAX_SLOTS; node++)
         memcpy(can_command_b0[node], data + node * 8, 8);
     for (int node = 0; node < MAX_SLOTS; node++)
         memcpy(can_command_b1[node], data + MAX_SLOTS * 8 + node * 8, 8);
 }
 
 /* Per-motor: FD (exit) → FE (zero) → FC (enter) on each active physical bus. */
 static void sendAkEnableSequence() {
     CAN_message_t msg;
     msg.flags.extended = 0;
     msg.len = 8;
 
     Serial.println(F("AK enable FD->FE->FC"));
     for (int n = 0; n < NUM_ACTIVE_NODES_BUS0; n++) {
         uint16_t mid = motorCanId(n);
         fillMagicPayload(msg.buf, 0xFD);
         msg.id = mid;
         CanBus0.write(msg);
         delay(200);
         fillMagicPayload(msg.buf, 0xFE);
         msg.id = mid;
         CanBus0.write(msg);
         delay(200);
         fillMagicPayload(msg.buf, 0xFC);
         msg.id = mid;
         CanBus0.write(msg);
         delay(200);
     }
 #if NUM_ACTIVE_NODES_BUS1 > 0
     for (int n = 0; n < NUM_ACTIVE_NODES_BUS1; n++) {
         uint16_t mid = motorCanId(n);
         fillMagicPayload(msg.buf, 0xFD);
         msg.id = mid;
         CanBus1.write(msg);
         delay(200);
         fillMagicPayload(msg.buf, 0xFE);
         msg.id = mid;
         CanBus1.write(msg);
         delay(200);
         fillMagicPayload(msg.buf, 0xFC);
         msg.id = mid;
         CanBus1.write(msg);
         delay(200);
     }
 #endif
     Serial.println(F("Enable sequence done."));
 }
 
 const uint8_t CRC8_POLYNOMIAL = 0x31; /* Dallas/Maxim — must match spine_board calculate_crc8 */
 uint8_t calculate_crc8(const uint8_t *data, size_t length) {
     uint8_t crc = 0;
     for (size_t i = 0; i < length; ++i) {
         crc ^= data[i];
         for (int j = 0; j < 8; ++j) {
             if (crc & 0x80)
                 crc = (crc << 1) ^ CRC8_POLYNOMIAL;
             else
                 crc <<= 1;
         }
     }
     return crc;
 }
 
 /* Common FlexCAN_T4 bring-up: 1 Mbit/s, accept-all filter, ISR → handler. */
 /*
  * NOTE: Keep this non-templated in a .ino.
  * The Arduino preprocessor may emit prototypes that drop the template<> line,
  * which breaks compilation ("CanT was not declared in this scope").
  */
 static void setupOneCan(decltype(CanBus0) &can, void (*handler)(const CAN_message_t &)) {
     can.begin();
     can.setBaudRate(1000000);
     can.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
     can.setMBFilter(MB1, 0, 0x0);
     can.enhanceFilter(MB1);
     can.distribute();
     can.enableMBInterrupts();
     can.onReceive(handler);
     can.setClock(CLK_60MHz);
 }
 
 #if NUM_ACTIVE_NODES_BUS1 > 0
 static void setupOneCan(decltype(CanBus1) &can, void (*handler)(const CAN_message_t &)) {
     can.begin();
     can.setBaudRate(1000000);
     can.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
     can.setMBFilter(MB1, 0, 0x0);
     can.enhanceFilter(MB1);
     can.distribute();
     can.enableMBInterrupts();
     can.onReceive(handler);
     can.setClock(CLK_60MHz);
 }
 #endif
 
 void setup() {
     Serial.begin(115200);
     while (!Serial && millis() < 4000)
         ;
     Serial.println(F("\r\n=== Teensy AK MIT bridge ==="));
 
     setupEthernet();
     setupOneCan(CanBus0, canReceiveBus0);
     Serial.println(F("CAN (bus0 / logical 0) OK"));
 #if NUM_ACTIVE_NODES_BUS1 > 0
     setupOneCan(CanBus1, canReceiveBus1);
     Serial.println(F("CAN (bus1 / logical 1) OK"));
 #endif
 
     sendAkEnableSequence();
 
     Serial.printf("MOTOR_ID_BASE=%d  bus0 nodes=%d  bus1 nodes=%d  daisy wait=%d  timing avg every %d fb\r\n",
                   MOTOR_ID_BASE, NUM_ACTIVE_NODES_BUS0, NUM_ACTIVE_NODES_BUS1, NUM_DAISY_MOTORS, MAX_NUM_SAMPLES);
     Serial.println(F("Ready — UDP payload+CRC per layout consts; PC: test_spine"));
 
     pinMode(LED_BUILTIN, OUTPUT);
     digitalWrite(LED_BUILTIN, HIGH);
 }
 
 void setupEthernet() {
     uint8_t mac[6];
     Ethernet.macAddress(mac);
     Serial.printf("MAC %02x:%02x:%02x:%02x:%02x:%02x\r\n",
                   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
     Ethernet.onLinkState([](bool state) {
         Serial.printf("[Ethernet] link %s\r\n", state ? "ON" : "OFF");
     });
 
 #if USE_STATIC_IP
     if (!Ethernet.begin(teensyIP, teensySubnet, teensyGateway)) {
         Serial.println(F("Ethernet.begin static failed"));
         return;
     }
 #else
     if (!Ethernet.begin() || !Ethernet.waitForLocalIP(kDHCPTimeout)) {
         Serial.println(F("Ethernet DHCP failed"));
         return;
     }
 #endif
     IPAddress ip = Ethernet.localIP();
     Serial.printf("IP %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
     udp.begin(kPort);
 }
 
 /*
  * Process one feedback frame: update gate flags, rolling stats, one-shot and periodic Serial lines.
  * `bus` is 0 or 1; `local_node` is index along that daisy; `t` is micros() at RX.
  */
 static void handleMotorFeedback(int bus, int local_node, unsigned long t) {
     const int max_on_bus = (bus == 0) ? NUM_ACTIVE_NODES_BUS0 : NUM_ACTIVE_NODES_BUS1;
     if (local_node < 0 || local_node >= max_on_bus)
         return;
 
     const int flat = flatMotorIndex(bus, local_node);
 
     if (flat < NUM_DAISY_MOTORS) {
         fb_received[flat] = true;
         if (waiting_for_daisy_fb) {
             bool all = true;
             for (int i = 0; i < NUM_DAISY_MOTORS; i++)
                 all = all && fb_received[i];
             if (all) {
                 waiting_for_daisy_fb = false;
                 pending_udp_send = true;
             }
         }
     }
 
 #if ENABLE_TIMING_STATS
     unsigned long dt_fb = t - last_packet_time[flat];
     total_fb_interval[flat] += dt_fb;
     packet_count[flat]++;
 
     if (last_cmd_time[flat] != 0) {
         total_cmd_latency[flat] += t - last_cmd_time[flat];
         cmd_count[flat]++;
     }
 
     if (udp_cmd_waiting_fb[flat] && last_cmd_time[flat] != 0) {
         unsigned long us = t - last_cmd_time[flat];
         float hz = (us > 0) ? (1e6f / (float)us) : 0.0f;
         Serial.printf("CAN bus%d node %d: send->response %.2f Hz (%lu us)\r\n", bus, local_node + 1, hz, us);
         udp_cmd_waiting_fb[flat] = false;
     }
 
     if (packet_count[flat] == MAX_NUM_SAMPLES) {
         unsigned long avg_int = total_fb_interval[flat] / MAX_NUM_SAMPLES;
         unsigned long avg_cmd = 0;
         if (cmd_count[flat] > 0)
             avg_cmd = total_cmd_latency[flat] / cmd_count[flat];
 
         float fb_hz = (avg_int > 0) ? (1e6f / (float)avg_int) : 0.0f;
         float cmd_hz = (avg_cmd > 0) ? (1e6f / (float)avg_cmd) : 0.0f;
         Serial.printf(
             "CAN bus%d node %d: avg fb interval %lu us (%.1f Hz)  avg cmd->fb %lu us (%.1f Hz)\r\n",
             bus, local_node + 1, avg_int, fb_hz, avg_cmd, cmd_hz);
 
         if (flat < NUM_DAISY_MOTORS) {
             last_printed_cmd_to_fb_daisy[flat] = avg_cmd + 1;
             bool all_set = true;
             for (int i = 0; i < NUM_DAISY_MOTORS; i++)
                 if (last_printed_cmd_to_fb_daisy[i] == 0) {
                     all_set = false;
                     break;
                 }
             if (all_set) {
                 Serial.print(F("Daisy timing (cmd->fb): "));
                 for (int i = 0; i < NUM_DAISY_MOTORS; i++) {
                     unsigned long us = last_printed_cmd_to_fb_daisy[i] - 1;
                     float h = (us > 0) ? (1e6f / (float)us) : 0.0f;
                     if (i)
                         Serial.print(F("  "));
                     Serial.printf("m%d=%lu us (%.1f Hz)", i, us, h);
                 }
                 Serial.print(F("\r\n"));
                 for (int i = 0; i < NUM_DAISY_MOTORS; i++)
                     last_printed_cmd_to_fb_daisy[i] = 0;
             }
         }
 
         total_fb_interval[flat] = 0;
         packet_count[flat] = 0;
         total_cmd_latency[flat] = 0;
         cmd_count[flat] = 0;
     }
     last_packet_time[flat] = t;
 #else
     (void)t;
 #endif
 }
 
 void canReceiveBus0(const CAN_message_t &msg) { /* ISR context — keep short */
     int local = akFeedbackLocalNode(msg);
     if (local < 0)
         return;
     memcpy(can_data_b0[local], msg.buf, 8);
     handleMotorFeedback(0, local, micros());
 }
 
 #if NUM_ACTIVE_NODES_BUS1 > 0
 void canReceiveBus1(const CAN_message_t &msg) {
     int local = akFeedbackLocalNode(msg);
     if (local < 0)
         return;
     memcpy(can_data_b1[local], msg.buf, 8);
     handleMotorFeedback(1, local, micros());
 }
 #endif
 
 void loop() {
 #if NUM_ACTIVE_NODES_BUS1 > 0
     CanBus1.events();
 #endif
     CanBus0.events();
     receiveUDPPacket();
 
     /* During an open round, (re)transmit MIT for all active nodes at least every MIT_RESEND_MIN_US. */
     uint32_t now_us = micros();
     const bool can_stream = have_valid_command && (!GATE_UDP_ON_DAISY_FEEDBACK || waiting_for_daisy_fb);
     if (can_stream &&
         (pending_can_send || (now_us - last_mit_tx_us) >= MIT_RESEND_MIN_US)) {
         if (pending_can_send)
             pending_can_send = false;
         sendCANCMD();
         last_mit_tx_us = now_us;
     }
 
 #if !GATE_UDP_ON_DAISY_FEEDBACK
     static uint32_t last_fb_udp_us = 0;
     if ((now_us - last_fb_udp_us) >= FEEDBACK_UDP_PERIOD_US) {
         pending_udp_send = true;
         last_fb_udp_us = now_us;
     }
 #endif
 
     if (pending_udp_send) {
         sendUDPPacket();
         pending_udp_send = false;
     }
 }
 
 /* Drain UDP; accept one new command packet only when not waiting for feedback (gated RX). */
 void receiveUDPPacket() {
     int size;
     while ((size = udp.parsePacket()) > 0) {
         const uint8_t *data = udp.data();
 
         if (size == 2 && data[0] == RESET_COMMAND) {
             Serial.println(F("UDP: RESET from PC"));
             reset();
         } else if (
 #if GATE_UDP_ON_DAISY_FEEDBACK
             !waiting_for_daisy_fb &&
 #endif
             size == TEENSY_UDP_PAYLOAD_BYTES + 1) {
             uint8_t crc = calculate_crc8(data, TEENSY_UDP_PAYLOAD_BYTES);
             if (data[TEENSY_UDP_PAYLOAD_BYTES] == crc) {
                 unpackWire48ToBuses(data);
 
                 pending_can_send = true;
                 first_mit_tx_this_batch = true;
                 have_valid_command = true;
 
 #if ENABLE_TIMING_STATS
                 for (int i = 0; i < MAX_MOTORS_TOTAL; i++)
                     udp_cmd_waiting_fb[i] = true;
 #endif
 
 #if GATE_UDP_ON_DAISY_FEEDBACK
                 for (int i = 0; i < NUM_DAISY_MOTORS; i++)
                     fb_received[i] = false;
                 waiting_for_daisy_fb = true;
 #endif
             }
         }
     }
 }
 
 /* Push current MIT command buffers to all active CAN ports (one frame per active node per bus). */
 void sendCANCMD() {
 #if ENABLE_TIMING_STATS
     unsigned long now = micros();
 #endif
 #if ENABLE_TIMING_STATS
     const bool stamp_cmd_time = first_mit_tx_this_batch;
 #else
     const bool stamp_cmd_time = false;
 #endif
     if (first_mit_tx_this_batch)
         first_mit_tx_this_batch = false;
 
     CAN_message_t msg;
     msg.flags.extended = 0;
     msg.len = 8;
 
     for (int i = 0; i < NUM_ACTIVE_NODES_BUS0; i++) {
         msg.id = motorCanId(i);
         memcpy(msg.buf, can_command_b0[i], 8);
         CanBus0.write(msg);
 #if ENABLE_TIMING_STATS
         if (stamp_cmd_time)
             last_cmd_time[flatMotorIndex(0, i)] = now;
 #else
         (void)stamp_cmd_time;
 #endif
     }
 #if NUM_ACTIVE_NODES_BUS1 > 0
     for (int i = 0; i < NUM_ACTIVE_NODES_BUS1; i++) {
         msg.id = motorCanId(i);
         memcpy(msg.buf, can_command_b1[i], 8);
         CanBus1.write(msg);
 #if ENABLE_TIMING_STATS
         if (stamp_cmd_time)
             last_cmd_time[flatMotorIndex(1, i)] = now;
 #endif
     }
 #endif
 }
 
 /* Pack latest feedback into wire layout and send to PC (IP must match host). */
 void sendUDPPacket() {
     uint8_t out[TEENSY_UDP_PAYLOAD_BYTES];
     memset(out, 0, sizeof(out));
     for (int node = 0; node < MAX_SLOTS; node++) {
         memcpy(out + node * 8, can_data_b0[node], 8);
         memcpy(out + MAX_SLOTS * 8 + node * 8, can_data_b1[node], 8);
     }
     udp.send("192.168.0.100", kPort, out, sizeof(out));
 }
 
 /* Host reset datagram: clear commands, state, and timing; reopen UDP acceptance. */
 void reset() {
     memset(can_command_b0, 0, sizeof(can_command_b0));
     memset(can_data_b0, 0, sizeof(can_data_b0));
     memset(can_command_b1, 0, sizeof(can_command_b1));
     memset(can_data_b1, 0, sizeof(can_data_b1));
 #if ENABLE_TIMING_STATS
     for (int i = 0; i < MAX_MOTORS_TOTAL; i++) {
         last_packet_time[i] = 0;
         total_fb_interval[i] = 0;
         packet_count[i] = 0;
         last_cmd_time[i] = 0;
         total_cmd_latency[i] = 0;
         cmd_count[i] = 0;
         udp_cmd_waiting_fb[i] = false;
     }
     for (int i = 0; i < NUM_DAISY_MOTORS; i++)
         last_printed_cmd_to_fb_daisy[i] = 0;
 #endif
     waiting_for_daisy_fb = false;
     pending_can_send = false;
     pending_udp_send = false;
     first_mit_tx_this_batch = false;
     for (int i = 0; i < NUM_DAISY_MOTORS; i++)
         fb_received[i] = false;
 }
 