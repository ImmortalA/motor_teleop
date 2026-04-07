# Motor teleoperation (A↔B) in this repo

This repo contains a **PC host** (`test_spine`) and a **Teensy 4.1 firmware** (`teensy/teensy.ino`) that bridge:

- **PC ⇄ (Ethernet UDP) ⇄ Teensy ⇄ (CAN) ⇄ T-Motor AK drives (MIT mode)**

The “motor teleop” feature implements **bilateral coupling** between two motors on the same CAN bus:

- **Motor A (master)**: you rotate it by hand.
- **Motor B (slave)**: follows A.
- If **B hits a load / gets stuck**, A produces resistive torque so you can **feel** it.

---

## What you built

### Host program: `build/test_spine`

`apps/main.cpp` provides 3 modes:

- **Mode 0**: enable-only; no streaming MIT commands.
- **Mode 1**: position sine demo.
- **Mode 2**: **motor teleop** (A↔B).

Teleop mode assumes:

- `host_cfg::kNumCanBuses = 1`
- `host_cfg::kNodesPerBus >= 2`
- **A = bus0 node0**, **B = bus0 node1**

### Firmware: `teensy/teensy.ino`

The Teensy:

- verifies CRC8 on incoming UDP command frames
- unpacks the **fixed 48-byte payload** (2 logical buses × 3 slots × 8 bytes)
- streams CAN MIT frames to the AK drives
- caches latest feedback frames and sends them back over UDP

---

## Teleop control law (Mode 2)

The implementation is intentionally conservative and “haptics-friendly”:

### B follows A (position/velocity coupling)

B is commanded to track A **without a step jump**:

- On entry, capture \(q_{A0}\), \(q_{B0}\)
- Preserve initial alignment offset: \(q_{B,\mathrm{des}} = q_A + (q_{B0} - q_{A0})\)
- Ramp in the coupling for ~1 second:
  - \(q_{B,\mathrm{des}} \leftarrow (1-\alpha)q_{B0} + \alpha \, q_{B,\mathrm{des}}\)
  - \(v_{B,\mathrm{des}} \leftarrow \alpha \, v_A\)

This avoids the “click then disable” behavior caused by a large instantaneous position step on the first control cycle.

### A feels B (torque reflection)

A is commanded to be **backdrivable** (no position hold):

- `kp_A = 0`, `kd_A = 0`
- A’s MIT command uses only feedforward torque:

\[
\tau_{A,\mathrm{ff}} = \alpha \cdot k_f \cdot \tau_B
\]

where \(\tau_B\) is the AK-reported torque estimate from B’s feedback.

To limit risk, \(\tau_{A,\mathrm{ff}}\) is **clamped** to a fraction of the actuator torque range (`ActuatorParams::t_max`).

---

## Getting 1 kHz behavior

There are **two different rates** to think about:

- **Host control loop rate**: how often the PC computes and writes new MIT commands
- **End-to-end update rate**: how often Teensy accepts new UDP commands and returns feedback

### Host side (PC)

In teleop (Mode 2), `test_spine` sleeps `1000 µs` per iteration to target **1 kHz**.

### Firmware side (Teensy)

The original firmware behavior was “gated”: accept a new UDP command only after receiving feedback from all configured motors.

For 1 kHz command streaming, disable gating:

- In `teensy/teensy.ino` set:
  - `#define GATE_UDP_ON_DAISY_FEEDBACK 0`
  - `FEEDBACK_UDP_PERIOD_US = 1000` (default) to send feedback at ~1 kHz using latest cached CAN frames

---

## Performance knobs (Teensy)

### Disable Serial timing stats

Serial printing and time-accumulation can reduce throughput. To disable:

- `#define ENABLE_TIMING_STATS 0`

With this off, the firmware removes per-motor timing math and all timing `Serial.printf(...)` lines.

---

## Safety + tuning checklist

- **Start with low gains** on B and low reflection gain on A.
- **Ramp-in** is already implemented for B coupling and A reflection (first ~1 second).
- If motors feel “locked”:
  - check that `kp/kd` are not accidentally set high
  - ensure you are not replaying an old stiff frame (power cycle or send `kp=kd=0`)
- If teleop “clicks” and disables:
  - reduce B gains further
  - reduce reflection gain/clamp
  - confirm the initial offset logic is used (Mode 2 soft-start)

---

## Files to look at

- **Teleop host logic**: `apps/main.cpp` (Mode 2 block)
- **MIT packing/unpacking + actuator limits**: `utils.h`
- **UDP/CAN bridge and pacing**: `teensy/teensy.ino`
- **SpineBoard transport**: `spine_board.cpp`, `spine_board.h`

