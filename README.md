# Ţlaser (tzlaser)

Industrial laser cutting control and CAM software written in Rust.

## Architecture

- **`tzlaser-core`**: CAD/CAM geometry models, AutoCAD DXF importer (LWPOLYLINE, CIRCLE, ARC, SPLINE, LINE with WCS/OCS support), automatic profile classifier (Outer, Interior holes, Bends), cut sequence optimizer (greedy nearest-neighbor TSP prioritizing interior holes), lead-in / lead-out generator, compound profile grouping, and undo/redo history engine.
- **`tzlaser-protocol`**: Low-level UDP communication protocol (port 502), Modbus CRC-16 packet framing, jog motion control, capacitive height sensor calibration, and hardware commands.
- **`tzlaser-gui`**: Modern desktop application built with `eframe` and `egui`, featuring an industrial ribbon menu, 2D vector canvas with directional arrowheads and cut order sequence badges, floating D-Pad coordinate transform HUD, and CNC control panels.

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
