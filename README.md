## Teensy-Spine-Board

Teensy 4.1 bridges **Ethernet UDP** and **T-Motor AK** drives in **MIT mode** on **11-bit CAN** at 1 Mbit/s. The PC app **`test_spine`** sends packed MIT frames; the Teensy unpacks them, transmits on CAN, and returns feedback over UDP.

**Behavior (firmware):** After each valid host command, the Teensy **waits for feedback from every motor counted in `NUM_DAISY_MOTORS`** before accepting the next UDP packet. While waiting, it **re-sends the same MIT command** on CAN at a minimum interval (`MIT_RESEND_MIN_US`) so drives see a steady command stream. **Serial** (115200) can print timing statistics.

**Configuration is three-way consistent:** edit the **`CONFIG`** block in `teensy/teensy.ino`, **`host_cfg`** in `apps/main.cpp`, and the anonymous **`kTeensy*`** constants in `spine_board.cpp` so logical buses, nodes per bus, and UDP payload size stay aligned. The sketch is a **single `.ino` file** (no separate Teensy headers in-repo).

### Repository layout

| Path | Description |
|------|-------------|
| **`apps/main.cpp`** | Host: `host_cfg` (boards, buses, nodes), network strings, `test_spine` mode menu (0–4), MIT targets via `pack_cmd`. |
| **`teensy/teensy.ino`** | Firmware: `CONFIG` macros, optional second CAN (bus1), UDP + CRC, CAN TX/RX, timing prints. |
| **`spine_board.cpp`**, **`spine_board.h`** | UDP client/server, receive/send threads, optional **wait-for-feedback** after each MIT send, `bus_list` state. |
| **`utils.h`** | AK MIT `pack_cmd` / `unpack_reply`, actuator parameters. |
| **`ETHERNET_SETUP.md`** | PC and Teensy IP/interface setup. |
| **`datasheet.txt`** | AK reference notes. |
| **`CMakeLists.txt`** | Builds `libspine_board.so` and executable **`test_spine`**. |

### Requirements

- CMake ≥ 3.10, C++14, pthreads, **Asio** (`asio.hpp` on the include path).
- **Teensy 4.1** with the sketch uploaded; **QNEthernet** and **FlexCAN_T4** (Arduino/Teensyduino).

### Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build .
```

Run from `build/`:

```bash
./test_spine
```

Set the PC Ethernet interface name and IP in `apps/main.cpp` (`BOARD_INTERFACE_NAME`, `ACTUATOR_TEENSY_BOARD_IPS`, ports). The Teensy uses a matching IP/port in the sketch (`USE_STATIC_IP`, `kPort`). Details: **`ETHERNET_SETUP.md`**.

### Runtime notes

- **Modes:** `test_spine` provides **0 (enable-only)**, **1 (sine demo)**, **2 (motor teleop A↔B)**.
- **`SpineBoard`** defaults to **wait-for-feedback** so the host does not outrun the Teensy’s gated RX; see `setWaitForFeedbackAfterSend` in `spine_board.h`.
- **UDP payload size** is fixed by the wire constants (default **48 bytes** + CRC = 2 logical buses × 3 node slots × 8 bytes). Changing capacity requires coordinated edits in firmware, `spine_board.cpp`, and host limits in `main.cpp`.

### Motor teleop documentation

See `MOTOR_TELEOP.md`.
