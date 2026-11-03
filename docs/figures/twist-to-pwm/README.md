# Fig 1 · From Twist to wheel PWM

Anatomy figure (Figure tier). It shows how the Scout base bridge turns a ROS `cmd_vel` (v, ω) into `M <left> <right>` for the UNO R4.

- `twist-to-pwm.html` / `twist-to-pwm-light.html`: standalone interactive pages. The pointer steers: left/right sets ω, up/down sets v. Arrow keys work too. When nobody touches it, it loops through six presets: straight, gentle left, saturated left, spin in place, right turn, deadband creep.
- Add `#v=0.45&w=2` to the URL to freeze the figure at a given twist, for screenshots.
- `twist-to-pwm.png`: desktop screenshot. `shots/` holds the turn, spin and light-theme states and the line-check images.
- `model.mjs`: the pure model. The page inlines this same code, so every number it shows comes from the model. Rebuild with `node build.mjs && node build.mjs --light`.

## Values and their sources

| Value | Used as | Source |
|---|---|---|
| v_L = v − ω·W/2, v_R = v + ω·W/2 | kinematics | spec: DD-SCOUT-BRIDGE §5.1 |
| ω > 0 = left turn, right side faster; x forward, y left | signs, axes | spec: §5.1 (REP-103) |
| s = max(\|v_L\|,\|v_R\|)/v_max, both sides ÷ s if over v_max | saturation | spec: §5.2 |
| \|v\| < 0.005 m/s → 0; PWM = sign·round(pwm_min + (255−pwm_min)·\|v\|/v_max), clamp ±255 | deadband → PWM | spec: §5.3 |
| re-sent at 20 Hz; no cmd_vel 0.25 s → `S`; R4 watchdog 300 ms | caption (safety) | spec: §5.4 |
| `M <left> <right>`, PWM −255…255, sign = direction | terminal line | spec: serial-r4-v1 |
| TB6612 on the Elegoo V4 shield, left pair / right pair | panel text | spec: serial-r4-v1 (Hardware) |
| plates 230.8 × 132.5 mm, 4 TT motors, wheel radius 32.5 mm | chassis geometry | task brief (Elegoo V4 / datasheet) |
| UNO R4 board 68.6 × 53.4 mm | deck board size | datasheet (Arduino UNO footprint) |
| **W = 0.16 m** | live numbers | **assumed: example value, not measured** |
| **v_max = 0.5 m/s** | live numbers, ruler scale | **assumed: example value, not measured** |
| **pwm_min = 60** | live numbers | **assumed: example value, not measured** |
| wheel positions (x ±62, y ±82 mm), motor boxes, servo / HC-SR04 placement, mat, terminal | drawing only | assumed (drawing layout, not used in the maths) |

The figure labels W, v_max and pwm_min as example values. Lab 1 (§5.5) will measure them. Once it has, change `EXAMPLE` in `model.mjs` and rebuild.

## Checks

- `scripts/lines.mjs`: `line ends: 0` in both themes (default pose). Console clean.
- The geometry audit (`--audit`, `A.recorder`) was not wired up. The figure has no pipes or harness, and its painter's order is hand-sorted by `depthOf`.
- The PWM values were checked by hand: (0.30, 1.5) → `M 130 224`, (0, 3) → `M -154 154`, (0.45, 2) → s = 1.22 → `M 153 255`.
