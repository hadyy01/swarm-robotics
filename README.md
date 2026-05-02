# Swarm Robotics — Active Matter Lab, IIT Bombay

> Decentralised multi-robot systems exhibiting emergent collective behaviour inspired by active-matter physics

**Author:** Hady Khan  
**Affiliation:** Active Matter Lab, Department of Physics, IIT Bombay  
**Advisor:** Prof. Dr. Nitin Kumar, Department of Physics, IIT Bombay  

---

## Overview

This repository contains the complete firmware, simulation, and analysis code developed during research at the **Active Matter Lab, IIT Bombay**. The work investigates how a swarm of small, resource-constrained robots can exhibit rich collective behaviours — Brownian motion, run-and-tumble dynamics, active Brownian particle (ABP) motion, and Reynolds flocking — using only local sensor information and short-range wireless communication.

The robots are custom-built differential-drive platforms (75 mm diameter) equipped with eight TCRT5000 IR proximity sensors, a TB6612FNG motor driver, and an nRF24L01+ 2.4 GHz transceiver. Each robot costs under ₹4,000 to manufacture.

---

## Research Questions

- How do local interaction rules give rise to global, ordered collective motion in a swarm?
- Can Reynolds' three flocking rules (separation, alignment, cohesion) be implemented entirely on an embedded microcontroller using IR and RF sensors?
- How does a projector-based light trail trigger a transition from disordered ABP motion to coordinated flocking without explicit communication?

---

## Repository Structure

```
swarm-robotics-active-matter/
│
├── motion_primitives/              # Individual robot motion modes
│   ├── brownian_motion.ino         # Isotropic Brownian (passive particle)
│   ├── active_brownian_particle.ino# ABP: constant speed + rotational noise
│   ├── run_and_tumble.ino          # Bacterial-inspired run-and-tumble
│   └── abp_obstacle_avoidance.ino  # ABP + inter-robot & boundary avoidance
│
├── communication/                  # nRF24L01+ wireless protocols
│   ├── nrf_master_simple.ino       # Point-to-point master
│   ├── nrf_slave_simple.ino        # Point-to-point slave
│   ├── nrf_master_network.ino      # Tree-topology master (v1)
│   ├── nrf_master_network_v2.ino   # Tree-topology master (v2, recalibrated)
│   ├── nrf_slave_network.ino       # Tree-topology slave (v1)
│   ├── nrf_slave_network_v2.ino    # Tree-topology slave (v2)
│   ├── nrf_transmitter.ino         # Multi-pipe transmitter
│   ├── nrf_receiver.ino            # Receiver / slave
│   └── nrf_diagnostic.ino          # Register inspection utility
│
├── flocking/                       # Collective behaviour algorithms
│   ├── flocking_math_model.ino     # Reynolds rules via 2D vector maths
│   ├── flocking_state.ino          # Decentralised flocking state machine
│   ├── trail_following_flock.ino   # LDR-based projector trail following
│   ├── abp_trail_boundary.ino      # ABP + trail detection + boundary avoid
│   └── sheepflock_projector.ino    # Yellow-trail herding (sheepdog model)
│
├── simulation/
│   ├── flocking_simulation.ipynb   # JAX-based flocking simulation (Python)
│   └── matlab/
│       └── robot_trail_projection.m# GigE camera → projector trail system
│
├── lib/
│   └── Vector2D.h                  # 2D vector class for Arduino (header-only)
│
└── docs/
    └── hardware.md                 # Wiring guide and pin reference
```

---

## Motion Primitives

All motion modes are implemented on Arduino-compatible microcontrollers using the SparkFun TB6612 library.

### Brownian Motion
Fully random motion — each motor receives an independent uniform random velocity at each time step. Models a passive colloidal particle in thermal equilibrium.

```
Vr ~ U(-V, V),  Vl ~ U(-V, V)
```

### Active Brownian Particle (ABP)
The robot translates at constant speed V₀ while its heading angle θ undergoes rotational diffusion:

```
dθ/dt = η(t),    η ~ U(-etaMax, etaMax)
Vr = (η·7 + 2·V₀) / 2
Vl = 2·V₀ - Vr
```

### Run-and-Tumble
Inspired by *E. coli* locomotion — straight runs interspersed with random reorientations drawn from U[0°, 360°]. Boundary detection via front IR sensors.

---

## Wireless Communication

All inter-robot communication uses the **nRF24L01+** module at 2.4 GHz. Two topologies are implemented:

| Topology | Files | Use Case |
|----------|-------|----------|
| Point-to-point | `nrf_master_simple`, `nrf_slave_simple` | 2-robot synchronised ABP |
| RF24Network tree | `nrf_master_network`, `nrf_slave_network` | Up to 5 robots, routed packets |

The master broadcasts a shared rotational noise value `eta` so all robots in the swarm execute correlated motion — a simple form of distributed synchronisation.

---

## Flocking

### Mathematical Model
Implements Reynolds' three rules using the custom `Vector2D` header-only library:

- **Separation** — repulsion vector away from neighbours within 100 mm
- **Alignment + Cohesion** — weighted combination steering toward neighbours' heading and position
- Final `flocking_vector` drives differential motor speeds

### Trail-Induced Flocking
A projector illuminates a light path on the arena floor. Robots detect the trail via two LDR sensors and steer to follow it. When multiple robots converge on the same trail, emergent flocking arises without any explicit inter-robot messages.

```
Arena Setup (Active Matter Lab):
  Overhead Camera → MATLAB blob detection → GigE projector → Arena floor
  Robot LDR sensors detect projected trail → Trail-following controller
```

---

## Simulation

`simulation/flocking_simulation.ipynb` — a full Python/JAX implementation of the Reynolds boids model. Used to validate parameter choices (J_align, J_avoid, J_cohesion, D_align, …) before deploying on physical hardware.

`simulation/matlab/robot_trail_projection.m` — MATLAB script that drives the overhead projector system, captures robot centroids from the GigE camera, and renders a persistent rolling trail.

---

## Hardware

| Component | Part | Qty per Robot |
|-----------|------|--------------|
| Microcontroller | Arduino Nano (ATmega328P) | 1 |
| Motor driver | SparkFun TB6612FNG | 1 |
| IR proximity | TCRT5000 | 8 |
| Multiplexer | CD4051BE 8:1 MUX | 1 |
| RF transceiver | nRF24L01+ with PA/LNA | 1 |
| Light sensor | LDR (GL5528) | 2 |
| Motors | N20 DC gear motor (6V, 200 RPM) | 2 |

See `docs/hardware.md` for pin assignments and wiring diagrams.

---

## Dependencies

**Arduino Libraries**
```
SparkFun TB6612 Motor Driver  v1.0.1
RF24                          v1.4.x
RF24Network                   v1.0.x
```

**Python (Simulation)**
```
jax>=0.4.1
jax-md>=0.2.0
matplotlib>=3.7.0
```

**MATLAB (Trail Projection)**
```
Computer Vision Toolbox
Image Acquisition Toolbox (GigE Vision support)
```

---

## Future Work

- Reinforcement learning (RL) for adaptive flocking with obstacle avoidance
- 3D mapping using RPLiDAR mounted on upgraded robot chassis
- Heterogeneous swarms: robots with different motion parameters
- SLAM-based coordination without external projector infrastructure

---

## License

MIT — see [LICENSE](LICENSE) for details.
