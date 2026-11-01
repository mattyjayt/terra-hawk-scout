# Scout layout: fit check v1 (cad skill, build123d / cadgen 0.8.4)

**Question:** does the planned hardware fit on the Elegoo Smart Robot Car V4.0 plates, and where should each part go?

**Answer: yes, if the battery is flat (about 20 mm tall).** A 40 mm tall 3S pack collides with the top plate, because the gap between the decks is only 39.7 mm.

## Files
| Path | What |
|---|---|
| `src/scout_dims.py` | every dimension, with its source |
| `src/scout_layout.py` | the assembly (`scout_layout` = flat battery, `scout_layout_tall_battery` = the 40 mm variant) |
| `src/plate_outline.py` | plate outlines measured from the Elegoo STLs |
| `checks/fit_check.py` → `checks/fit_check_results.json` | overlaps, gaps, overhangs, LiDAR clearance, centre of mass |
| `GLB/scout_layout.glb` | web and viewer export (committed) |
| `STEP/scout_layout.step` | CAD exchange (18.5 MB, git-ignored; regenerate with `cadgen +0.8.4 export <id> STEP/scout_layout.step`) |
| `vendor/raspberry_pi_5.*` | real Pi 5 model from step-parts (git-ignored) |
| `shots/scout_iso.png`, `shots/scout_top.png` | review snapshots |

Rebuild: `~/.local/bin/cadgen +0.8.4 run src/scout_layout.py`, then `cadgen +0.8.4 run checks/fit_check.py`.

## Dimension sources
- **Measured from the STLs:** plates 132.52 × 230.78 × 4.5 mm; **39.7 mm clear gap between the decks**; wheel-well positions; spine half-width 47 mm.
- **From datasheets:** Pi 5 (vendor STEP); Camera Module 3 25 × 24 × 11.5; LD19 LiDAR 54 × 46.3 × 34.8; GY-521 21 × 16; HC-SR04 45 × 20 × 15; ESP32 DevKit 55 × 28 × 13; 65 mm wheels.
- **Assumed (measure these):** battery envelope, buck converter 60 × 35 × 20, active-cooler envelope, standoff heights (5 and 6 mm), LiDAR mast 12 × 12 and platform 60 × 60 × 3, height of the LiDAR's optical centre (24 mm above its base), wheel width 26, and the masses used for the centre of mass.

## Fit-check results (`scout_layout`, flat battery)
| Check | Result |
|---|---|
| Overlaps between solids | **none** |
| Tightest gaps | battery ↔ front/rear-left motors 1.5 mm · wheels ↔ plates 1.6 mm · Pi 5 ↔ payload bay 1.3 mm · Pi 5 ↔ LiDAR mast 4.0 mm |
| Overhangs | the wheels stick out ~11 mm beyond the plate outline (expected, as on the stock car); payload keep-out overhangs by 9.4 mm at the rear |
| LiDAR | scan plane 131 mm above the ground; clears the tallest other part (payload bay) by **5.5 mm** |
| Mass (estimate) | ~794 g without payload |
| Centre of mass | empty: 48 mm high, on the centre of rotation (y ≈ −2). With the bay full, it moves 20 mm rearward (y −22) and rises to 58 mm, so watch for tipping on braking |

**Tall-battery variant (70 × 55 × 40):** overlaps the top plate (1133 mm³) and the IMU (1344 mm³). **It does not fit.**

## Open questions for Jaime
1. Measure the blue 3S pack (L × W × H). If it's taller than ~34 mm, it can't go between the decks: mount it on top, or use the Elegoo 2S box for the motors.
2. Buck converter choice: its size decides the lower-deck layout.
3. Which LiDAR (LD19 or RPLIDAR C1)? The C1 is taller (41 mm) and would raise the scan plane.
4. Payload bay: is 80 × 60 × 60 mm right for a water bottle or a seedling cup?
