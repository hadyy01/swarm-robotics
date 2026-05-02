# Hardware Reference — Swarm Robot Platform

**Active Matter Lab, IIT Bombay**

---

## Pin Assignments (Arduino Nano)

### TB6612FNG Motor Driver

| Signal | Arduino Pin | Description |
|--------|-------------|-------------|
| PWMA   | D3          | Motor A PWM speed |
| AIN2   | D4          | Motor A direction |
| AIN1   | D5          | Motor A direction |
| STBY   | D1          | Standby (active HIGH) |
| BIN1   | D7          | Motor B direction |
| BIN2   | D8          | Motor B direction |
| PWMB   | D9          | Motor B PWM speed |

### nRF24L01+ Transceiver (SPI)

| Signal | Arduino Pin |
|--------|-------------|
| CE     | D6          |
| CSN    | A0          |
| SCK    | D13 (SPI)   |
| MOSI   | D11 (SPI)   |
| MISO   | D12 (SPI)   |

### CD4051BE 8:1 Multiplexer (IR Sensors)

| Signal | Arduino Pin |
|--------|-------------|
| S0     | A5          |
| S1     | A2          |
| S2     | A1          |
| Z (output) | A4      |
| IR LED enable | D10 |

### IR Sensor Channel Map (TCRT5000 × 8)

| Channel | Direction | MUX input |
|---------|-----------|-----------|
| 0       | South (S) | MUX ch 0  |
| 1       | South-East (SE) | MUX ch 1 |
| 2       | East (E)  | MUX ch 2  |
| 3       | North-East (NE) | MUX ch 3 |
| 4       | North (N) | MUX ch 4  |
| 5       | North-West (NW) | MUX ch 5 |
| 6       | West (W)  | MUX ch 6  |
| 7       | South-West (SW) | MUX ch 7 |

### LDR Light Sensors

| Sensor | Arduino Pin | Purpose |
|--------|-------------|---------|
| LDR Right | A6   | Right trail intensity |
| LDR Left  | A7   | Left trail intensity |
| LDR LED enable | D2 | Projector LED control |

---

## Velocity Calibration

Motor velocity is derived from the ABP noise term `eta`:

```
Vr [cm/s] = (eta * 7.0 + 2 * V0) / 2
Vl [cm/s] = 2 * V0 - Vr

# Convert to analog PWM value:
Vr_analog = (Vr + 1.0015) / 0.1059     # v1 calibration
Vl_analog = (Vl + 1.0015) / 0.1059

# v2 calibration (updated robots):
Vr_analog = (Vr + 0.729) / 0.1114
Vl_analog = (Vl + 0.729) / 0.1114
```

The two calibration sets correspond to different motor batches; use `v2` constants for robots built with the second hardware revision.

---

## IR Detection Thresholds

| Threshold | Value | Meaning |
|-----------|-------|---------|
| `> 500`   | Boundary or object near | Trigger avoidance |
| `> 900`   | Very close object (another robot) | Brake + reorient |

Thresholds may need adjustment based on arena surface reflectivity.

---

## LDR Trail Detection

Calibrate per robot before each experiment:

```
I_L = (Iwhite_L + Itrail_L) / 2   # Left LDR midpoint
I_R = (Iwhite_R + Itrail_R) / 2   # Right LDR midpoint
```

Typical values (blue projector trail on white arena):

| Robot | I_L | I_R |
|-------|-----|-----|
| 5     | 480 | 530 |
| 6     | 500 | 610 |
| 7     | 465 | 555 |
| 10    | 410 | 580 |
