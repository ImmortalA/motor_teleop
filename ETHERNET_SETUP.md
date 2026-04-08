# Ethernet setup (PC ↔ Teensy)

Same subnet: **192.168.0.x** / 255.255.255.0.

| Role        | IP             | Interface / note      |
|------------|----------------|------------------------|
| PC (host)  | 192.168.0.100  | enp8s0                 |
| First Teensy (board 0) | 192.168.0.101 | port 8003 (static in sketch) |
| Second Teensy (board 1) | 192.168.0.102 | port 8004 (if used)   |

---

## 1. PC (Linux)

Set static IP on Ethernet:

```bash
sudo ip addr add 192.168.0.100/24 dev enp8s0
```

Check: `ifconfig enp8s0` → `inet 192.168.0.100`.

---

## 2. Host app (`apps/motor_teleop.cpp`)

- **`cfg::kInterface`** = `"enp8s0"` (Ethernet).
- **`cfg::kTeensyIp`** = `"192.168.0.101"` — must match the Teensy static IP in firmware.

---

## 3. Teensy firmware (`teensy_mt/teensy_mt.ino`)

**First Teensy (board 0) = 192.168.0.101, port 8003**

### Static IP on Teensy

1. In **teensy_mt.ino** set:
   - `#define USE_STATIC_IP 1`
   - `IPAddress teensyIP(192, 168, 0, 101);`  — board 0; use 102 for board 1
   - `IPAddress teensySubnet(255, 255, 255, 0);`
   - `IPAddress teensyGateway(192, 168, 0, 1);`  — optional if no router on the link
2. Build and upload the sketch. The Teensy will come up with that IP (no DHCP).
3. For DHCP instead, set `USE_STATIC_IP` to `0`.

Other settings:

- **kPort** = 8003 (board 0). Use 8004 for a second Teensy (board 1).
- **udp.send("192.168.0.100", kPort, ...)** → PC IP; host receives on 8003/8004.
- **NUM_DAISY_MOTORS**: number of motors that must report feedback before the next UDP batch (e.g. **2** for a two-motor daisy on CAN3).

### Send → wait for response → next batch

The firmware accepts a UDP control packet only after it has received **MIT feedback** (standard CAN, mode 2 in ID bits 10–8, or motor ID in `buf[0]`) from the expected motor(s) for the current batch. Set `NUM_DAISY_MOTORS` to match how many motors must report before the next batch.

---

## 4. Serial timing (`teensy_mt.ino`)

Serial (115200) prints per-motor timing (avg fb interval, cmd→fb, one-shot send→response, and a daisy summary line when both motors finish a stats window). The next UDP batch is accepted only after feedback from all `NUM_DAISY_MOTORS` motors.

---

## 5. Checklist

- [ ] PC: enp8s0 = 192.168.0.100
- [ ] apps/motor_teleop.cpp: kInterface = "enp8s0", kTeensyIp = "192.168.0.101"
- [ ] teensy_mt.ino: USE_STATIC_IP 1, teensyIP = 192.168.0.101, kPort 8003, udp.send("192.168.0.100", kPort, ...)
- [ ] Rebuild host (`cmake --build build`), re-upload Teensy, run `./motor_teleop`

---

## 6. Motor not moving (feedback OK, no motion)

If you see `Motor (board 0 CAN 1): p=-12.5` (or other values) but the motor does not move:

1. **Motor CAN ID** – In **teensy_mt.ino** set `MOTOR_ID_BASE` to match the first actuator’s configured CAN ID. Re-upload after changes.
2. **Confirm ID** – Use T-Motor configuration software or your CAN sniffer so each AK module’s ID matches `MOTOR_ID`, `MOTOR_ID+1`, … along the chain.
3. **Debug** – Re-upload teensy_mt.ino, open Serial Monitor (115200). Confirm the CAN ID in transmitted frames matches the motor.
4. **Test position** – The host app `motor_teleop` sends position/velocity commands. If the motor is at -12.5 rad (raw 0), it should try to move toward the commanded position.
