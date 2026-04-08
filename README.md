## motor_teleop

PC ↔ **Teensy 4.1** ↔ **T-Motor AK** (MIT mode on 11-bit CAN).

`motor_teleop` runs a minimal two-motor teleoperation where:
- **A** = `bus0 node0` (master / handle)
- **B** = `bus0 node1` (follower / environment side)

There are **two runtime modes**:
- **Mode 0 (drive-only)**: **B follows A**, **A stays free** (no torque command).
- **Mode 1 (bilateral)**: **B follows A**, and **A feels B** via reflected torque + “lag spring”.

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
./motor_teleop 0    # A drive only — B follows A
./motor_teleop 1    # bilateral — A feels B (same follower law + haptics)
# Aliases: ./motor_teleop drive | bilateral
```

Edit `cfg::` in `apps/motor_teleop.cpp` and the `CONFIG` block in `teensy_mt.ino` together (IP, port, node counts, wire padding). See `ETHERNET_SETUP.md`.

For **low-latency command streaming** (~1 kHz), set `GATE_UDP_ON_DAISY_FEEDBACK` to `0` in `teensy_mt.ino` so the Teensy does not wait for a full daisy feedback round before accepting each UDP packet (must match how you use `setWaitForFeedbackAfterSend` on the host).

### Runtime modes

#### Mode 0 — drive-only

- **B** tracks **A** using wrapped position (`p_orig`) + velocity feedforward:
  - B `p_des` is **slew-limited** (prevents torque spikes when you flick A)
  - B `v_des` is **low-passed and clamped**
  - B uses `cfg::kKpB`, `cfg::kKdB`
- **A** stays backdrivable:
  - A `kp=kd=0`, `t_ff=0`

Run:

```bash
./motor_teleop 0
# or:
./motor_teleop drive
```

#### Mode 1 — bilateral (A feels B)

Everything in mode 0 still applies, plus:
- **A** remains `kp=kd=0` but gets a **torque command** `t_ff` that makes A “feel” B.
- The master torque is:
  - **Reflected measured torque** from B: \( \tau_A \leftarrow -k \cdot \tau_B \) (low-passed)
  - **Lag spring** if B can’t keep up: \( e = \mathrm{shortest}(q_B \rightarrow p_{cmd,B}) \), then \( \tau_A \leftarrow -k_{pose} \cdot e \)
  - **Damping** on A: \( -k_d \cdot \dot{q}_A \) for stability
- `t_ff` is clamped to the actuator’s configured torque limits (AK60-6 table is ±15 N·m in `utils.h`).

Run:

```bash
./motor_teleop 1
# or:
./motor_teleop bilateral
```

### Tuning knobs (in `apps/motor_teleop.cpp`)

Follower B (tracking quality vs stability):
- **`cfg::kKpB`, `cfg::kKdB`**: follower stiffness/damping
- **`cfg::kMaxSlewRadPerTick`**: max change in B `p_des` per 1 kHz tick (smaller = safer/softer)
- **`cfg::kVffLpfAlpha`**: low-pass aggressiveness on velocity feedforward
- **`cfg::kVffMax`**: clamp on velocity feedforward magnitude

Master A (bilateral feel):
- **`cfg::kTorqueReflect`**: scales B measured torque into A
- **`cfg::kReflectLpfAlpha`**: low-pass on reflected torque (smaller = smoother but more lag)
- **`cfg::kPoseReflect`**: “stuck spring” strength (set to `0` to disable)
- **`cfg::kMasterDamping`**: stabilizing damping on A (increase if A oscillates/buzzes)

If the “push-back” direction feels wrong in mode 1, flip the sign by changing **`cfg::kTorqueReflect`** to negative (or invert the `tau_from_B` sign in code).

### Safety / troubleshooting

- If **B makes a harsh noise or drops out of motor mode** when you move A fast, reduce:
  - `cfg::kKpB`, `cfg::kKdB`, and/or `cfg::kMaxSlewRadPerTick`
  - `cfg::kVffMax` (or set it to `0` to remove velocity feedforward)
- For true ~1 kHz streaming, set Teensy `GATE_UDP_ON_DAISY_FEEDBACK` to `0`. If you keep it at `1`, the firmware will intentionally ignore new UDP commands until feedback arrives, which can feel “bursty”.
