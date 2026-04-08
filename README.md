## motor_teleop

PC ↔ **Teensy 4.1** ↔ **T-Motor AK** (MIT mode on 11-bit CAN). The host binary **`motor_teleop`** runs a minimal law: **bus0 node1 (B) follows bus0 node0 (A)** with a fixed offset from the first sample, using wrapped positions (`p_orig`).

### Layout

| Path | Role |
|------|------|
| `apps/motor_teleop.cpp` | Network + `cfg::` (interface, Teensy IP, port, nodes/buses); teleop loop at ~1 kHz. |
| `teensy_mt/teensy_mt.ino` | UDP + CRC, CAN, must match `kTeensy*` in `spine_board.cpp` (48-byte MIT wire: 2×3×8). |
| `spine_board.cpp` / `spine_board.h` | UDP threads, `bus_list`, pack/unpack. |
| `utils.h` | MIT pack/unpack, `WRAP_RANGE`, actuator tables. |
| `ETHERNET_SETUP.md` | IP and interface notes. |

### Build

Requires CMake ≥ 3.10, C++14, **Asio** (`asio.hpp`), Teensyduino libs **QNEthernet** and **FlexCAN_T4**.

```bash
mkdir -p build && cd build && cmake .. && cmake --build .
./motor_teleop
```

Edit `cfg::` in `apps/motor_teleop.cpp` and the `CONFIG` block in `teensy_mt.ino` together (IP, port, node counts, wire padding). See `ETHERNET_SETUP.md`.
