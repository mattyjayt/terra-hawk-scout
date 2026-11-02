"""Markdown tables for docs/cad-round3.md (BOM, drilling table, check results) from checks/results/*.json.

Run:  python3 checks/report_tables.py  -> checks/results/tables.md   (plain Python, no kernel)
"""
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
import dims as D  # noqa: E402
from cables import CABLE_CHOICE  # noqa: E402


def product_key(label):
    if label in D.PRODUCTS:
        return label
    best = None
    for p in D.PRODUCTS:
        if p.endswith("_") and label.startswith(p) and (best is None or len(p) > len(best)):
            best = p
    assert best, f"no product reference for {label}"
    return best


def bom(r):
    groups = {}
    for label, g in sorted(r["mass_breakdown_g"].items()):
        if label.startswith("cable_"):
            continue
        k = product_key(label)
        groups.setdefault(k, []).append((label, g))
    rows = ["| Label(s) | Qty | Product | Source / URL | Mass each (g) | Dim./mass source |", "|---|---|---|---|---|---|"]
    for k, items in groups.items():
        prod, url, tag = D.PRODUCTS[k]
        labels = items[0][0] if len(items) == 1 else f"{k}1..{len(items)}" if k.endswith("_") and re.search(r"\d$", items[0][0]) \
            else ", ".join(l for l, _ in items)
        m = items[0][1]
        rows.append(f"| `{labels}` | {len(items)} | {prod} | {url} | {m:g} | {tag} |")
    cab = ["| Route | Kind | Path (mm) | +15 % (mm) | Cable |", "|---|---|---|---|---|"]
    for c in r["cables"]:
        cab.append(f"| `{c['route']}` | {c['kind']} | {c['path_mm']} | {c['with_15pct_slack_mm']} | {c['cable']} |")
    return "\n".join(rows), "\n".join(cab)


def drilling(r):
    rows = ["| # | Part | Purpose | X (mm) | Y (mm) | Hole |", "|---|---|---|---|---|---|"]
    for i, d in enumerate(r["drilling_table"], 1):
        dia = f"{d['dia_mm'][0]:g} x {d['dia_mm'][1]:g} mm cut-out" if isinstance(d["dia_mm"], list) else f"Ø{d['dia_mm']:g}"
        rows.append(f"| {i} | {d['part']} | {d['purpose']} | {d['x_mm']:+.1f} | {d['y_mm']:+.1f} | {dia} |")
    return "\n".join(rows)


def checks(rs):
    hdr = "| Check | " + " | ".join(f"`{r['model']}`" for r in rs) + " | Requirement |"
    rows = [hdr, "|---" * (len(rs) + 2) + "|"]

    def row(name, f, req):
        rows.append(f"| {name} | " + " | ".join(str(f(r)) for r in rs) + f" | {req} |")
    row("Unintended overlaps", lambda r: len(r["overlaps_unintended"]), "0")
    row("Intended overlaps", lambda r: "; ".join(f"{o['a']}/{o['b']} {o['volume_mm3']} mm³" for o in r["overlaps_intended"]), "envelope only")
    row("Tightest gap (non-cable)", lambda r: f"{r['tightest_gaps_mm'][0]['a']}–{r['tightest_gaps_mm'][0]['b']} {r['tightest_gaps_mm'][0]['gap_mm']} mm", "> 0")
    row("Power-part min gap", lambda r: r["required_clearances"]["power_min_gap_mm"], "≥ 2 mm")
    row("LiDAR tower min gap", lambda r: r["required_clearances"]["lidar_tower_min_gap_mm"], "≥ 2 mm")
    row("Scan plane z (mm)", lambda r: r["lidar_check"]["scan_plane_z_mm"], "")
    row("Scan margin over highest part (mm)", lambda r: f"{r['lidar_check']['margin_mm']} ({r['lidar_check']['highest_other_part']})", "≥ 5")
    row("Scan margin over payload keep-out (mm)", lambda r: r["lidar_check"]["margin_over_payload_keepout_mm"], "≥ 5")
    row("Blind sectors (parts above the plane)", lambda r: r["lidar_check"]["blind_sectors_deg"] or "none", "documented")
    row("Smallest dip that grazes a part (deg)", lambda r: f"{r['lidar_check']['dip_tolerance_deg'][0]['deg']} ({r['lidar_check']['dip_tolerance_deg'][0]['part']})", "> 1.5 (C1 flatness)")
    row("Tower legs (catalogue stack, mm)", lambda r: "+".join(str(x) for x in r["lidar_check"]["tower_leg_stack_mm"]), "")
    row("LiDAR board − cooler top (mm)", lambda r: r["jaime_numbers_on_geometry"]["board_minus_cooler_ge_3"], "≥ 3")
    row("Camera floor visible ahead of front (mm)", lambda r: r["camera_check"]["nearest_visible_floor_ahead_of_robot_front_mm"], "≤ 150")
    row("Camera lower FOV edge blocked by", lambda r: r["camera_check"]["lower_fov_edge_hits_parts_aabb"] or "nothing", "nothing")
    row("Ground clearance (mm)", lambda r: f"{r['ground_check']['ground_clearance_mm']} ({r['ground_check']['lowest_non_wheel_part']})", "≥ 12 (10 mm step + 2)")
    row("Tail clearance, front wheels on 10 mm step", lambda r: r["ground_check"]["rear_overhang_tail_clearance_on_step_mm"], "> 0")
    row("Motor fits", lambda r: r["motor_fit"]["fits"], "True")
    row("Mass empty (kg)", lambda r: r["mass_empty_kg"], "")
    row("CoM empty (mm)", lambda r: r["com_empty_mm"], "|y| < 10, x ≥ −22 (a6)")
    row("Mass + 300 g (kg)", lambda r: r["mass_with_payload_kg"], "")
    row("CoM + 300 g (mm)", lambda r: r["com_with_payload_mm"], "|y| < 10, x ≥ −22 (a6)")
    row("Front/rear load empty", lambda r: f"{r['wheel_loads_empty']['front_share']:.3f} / {r['wheel_loads_empty']['rear_share']:.3f}", "")
    row("Front/rear load + 300 g", lambda r: f"{r['wheel_loads_with_payload']['front_share']:.3f} / {r['wheel_loads_with_payload']['rear_share']:.3f}", "≈ 0.47 front (finding 10)")
    row("Wheel loads + 300 g (N, fl/fr/rl/rr)", lambda r: "/".join(f"{v:g}" for v in r["wheel_loads_with_payload"]["per_wheel_N"].values()), "")
    row("IMU chip centre (mm), dist. to CoR", lambda r: f"{r['imu_frame_mm']['chip_centre']}, {r['imu_frame_mm']['distance_from_cor_xy_mm']}", "≤ 25")
    row("Jaime's numbers on the geometry", lambda r: ", ".join(f"{k}={v}" for k, v in r["jaime_numbers_on_geometry"].items() if v is not None), "match")
    return "\n".join(rows)


if __name__ == "__main__":
    rs = [json.loads((HERE / "results" / f"{n}.json").read_text()) for n in ("scout_v2", "scout_v2__c1_hiwonder_nopay")]
    b, c = bom(rs[0])
    out = ["## Checks\n", checks(rs), "\n## BOM\n", b, "\n## Cables\n", c, "\n## Drilling\n", drilling(rs[0])]
    (HERE / "results" / "tables.md").write_text("\n".join(out) + "\n")
    print("wrote", HERE / "results" / "tables.md")
