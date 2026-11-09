# ADR-0006 — Scout runs on the Pi 5 + Arduino UNO R4 only (no ESP32)
- **Status:** Accepted (2026-11-03). Supersedes [ADR-0002](0002-esp32-micro-ros.md); makes [ADR-0005](0005-r4-usb-serial-base.md) permanent instead of interim.
- **Context:** The R4 + Elegoo shield base (ADR-0005) works end to end: motors, MPU-6050, servo, battery and the 300 ms watchdog, driven from the Pi over USB. Keeping a second microcontroller path (classic ESP32 + micro-ROS) doubles the firmware, the toolchain (PlatformIO + micro-ROS agent) and the docs without adding anything the Scout needs today.
- **Decision:** The Scout uses exactly two compute parts:
  - **Raspberry Pi 5** for all logic, as ROS 2 nodes.
  - **Arduino UNO R4 WiFi** as the thin hardware layer, firmware `firmware/r4_base`, talking the serial line protocol ([serial-r4-v1](../specs/interfaces/serial-r4-v1.md), v2 planned) over USB.

  A ROS 2 bridge node on the Pi ([DD-SCOUT-BRIDGE](../design/DD-SCOUT-BRIDGE.md)) owns the ROS side. There is no ESP32 and no micro-ROS on the Scout.
- **Consequences:**
  - Removed: `firmware/esp32_base/` and the IF-MICROROS contract (never implemented).
  - `DD-SCOUT-FW` now designs the R4 firmware: modules, standard libraries, protocol v2 (issue #16).
  - **Encoder motors (ADR-0003, deferred) must be read by the R4 if they are ever added.** The UNO R4 has a limited set of interrupt-capable pins and the Elegoo shield already uses several. Before buying encoder motors, map the free interrupt pins and decide between 2 encoders (one per side) and 4. If the R4 can't do it, reopen this ADR rather than adding a board silently.
  - `cad/scout-v2` (a design study for a future revision) places a classic ESP32 on Pi UART2. In that layout the R4 + shield takes its place (68.6 × 53.4 mm vs. ~52 × 28 mm) on USB. Re-check the fit when the v2 deck is built.
  - The TerraHawk garden keeps its ESP32 sensor nodes; this decision is Scout-only.
