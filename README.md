# Ţlaser (tzlaser)

Industrial fiber laser cutting control and CAM software written in Rust, targeting SC2000-compatible motion & capacitive height controller boards over Ethernet/UDP.

## Current Project Status

| Subsystem | Status | Notes |
|---|---|---|
| **Controller Connection** | Working | Auto-connects to XY motion controller (UDP `:502`) and Z/capacitive height controller (UDP `:999`) with live polling and heartbeat telemetry. |
| **Z + XY Homing** | Working | Sequential Z-up homing followed by corner XY homing (e.g. Bottom-Right `Xmax, Y0`) with tuned speeds to prevent servo drive `Err 102` overcurrent faults. |
| **Manual & Remote Jogging** | Working | On-screen arrow D-Pad (`X`, `Y`, `Z`, Step & Continuous modes) and physical wireless remote control (`R1042` bitmask decoding, including Fast mode and Guide laser toggle). |
| **Stop / Emergency Halt** | Working | Both on-screen **Stop** and the wireless remote **Stop** button (`Bit 27` / `Bit 19`) immediately halt XY/Z motion, tear down active FIFO streams, and stop job execution. |
| **XY Dry-Cutting (Curves & Paths)** | Working | Continuous 4-kHz Motion FIFO streaming (`REG_FIFO_DATA`) with S-curve acceleration/deceleration ramps traces circles, arcs, splines, and sharp corners smoothly without step-by-step pauses. |
| **Offline Simulation & Signal Graph** | Working | Real-time head marker preview, rapid travel path rendering, pause/resume checkpoints, and live velocity/acceleration telemetry plots. |
| **Live Laser Cutting & Capacitive Follower** | In Progress | CAM layer parameters, lead-in/lead-out, obstacle avoidance routing, and laser/gas register commands are implemented; live high-power cutting and closed-loop Z follower runs are pending full hardware validation. |

---

## Features & Functions

### CAD / CAM & Geometry (`tzlaser-core`)
- **DXF Importer**: Parses `LINE`, `LWPOLYLINE`, `POLYLINE`, `CIRCLE`, `ARC`, `ELLIPSE`, and `SPLINE` entities with full WCS/OCS arbitrary-axis transforms and adaptive arc/bulge tessellation.
- **Automatic Profile Classification & Grouping**: Classifies geometry into Outer profiles, Interior holes, and Bends; supports compound groups, scaling, rotation, mirroring, and undo/redo history.
- **Cut Sequence & Obstacle Routing**: Greedy nearest-neighbor TSP ordering prioritizing interior holes before outer contours, plus obstacle exclusion polygon offsetting and rapid-travel detour routing.
- **Lead-In / Lead-Out & Material Library**: Configurable lead lines/arcs, pierce times, layer cutting speeds, and material/thickness presets.

### Hardware Protocol (`tzlaser-protocol`)
- **Port `502` (XY/W Motion Controller)**:
  - Custom Modbus-style UDP framing with little-endian CRC-16 (`crc.rs`, `packet.rs`).
  - Live 144-register status polling (`R1000..R1143`) for axis encoder positions (`R1008` X, `R1009` Y), hardware limit masks (`R1002`), FIFO free buffer (`R1006`), and wireless remote key bitmask (`R1042`).
  - Point-to-point / jog commands (`REG_JOG_MOTION = 116`), deceleration ramps (`REG_JOG_RAMP = 120`), homing (`REG_HOME_CMD = 112`), and 4-kHz continuous Motion FIFO streaming (`REG_FIFO_DATA = 100`).
  - Explicit shared axis/direction constants (`machine_directions`, `fifo_flags`) verified against Wireshark captures (`sharks/*.pcapng`).
- **Port `999` (Z-Axis & Capacitive Height Controller)**:
  - Modbus RTU over UDP for Z jogging, Z homing, Z stop, and live Z position + capacitance polling.

### Desktop GUI (`tzlaser-gui`)
- Built with `eframe` / `egui` featuring a dark industrial UI, ribbon toolbar, 2D vector canvas with origin/bed axes, directional arrowheads, cut-order badges, and rapid travel path display.
- **CNC Control Panel**: Connect/disconnect, Home All / Home Z, Step & Continuous jogging, **Start**, **Dry-Cut**, **Pause**, **Continue** (with return-to-checkpoint), **Stop**, **Frame** bounding-box trace, and **Red Pointer** toggle.
- **Signal Graph & In-App Console**: Real-time axis velocity/acceleration plots and a collapsible diagnostic message console.

---

## Prerequisites

- [Rust](https://rustup.rs/) (edition 2021)

## Running

```bash
cargo run -p tzlaser-gui
```

## Testing

```bash
cargo test --workspace
```
