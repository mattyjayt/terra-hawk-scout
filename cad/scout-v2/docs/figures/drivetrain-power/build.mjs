// Fig 2 · Drivetrain and power: a cut-away of Scout v2's left side on a bench.
// node build.mjs [--light] [--audit]  ->  drivetrain-power.html / drivetrain-power-light.html
//
// Concept. The robot itself is the explanation. The near half of the top deck is cut
// away (contents left in place) and the near front wheel is pulled off and laid on
// the bench, so the whole chain is in view: 3S pack -> XT30 -> 7.5 A fuse -> 10 A
// switch -> Cytron MDD3A (PWM capped at 8.4 V average) with a branch to the Pololu
// D36V50F5 5 V buck and up to the Pi 5 header; MDD3A -> Hiwonder TT 1:42 motor whose
// hall encoder (lit) sits on the motor shaft; its harness runs back to the ESP32,
// which counts with PCNT and writes PWM back to the MDD3A. The loop runs a tour:
// straight, a skid turn (near wheels slow down; readout shows what the wheels claim
// vs the gyro), then /cmd_vel goes quiet and the 300 ms watchdog brakes LOW/LOW.
//
// World: 1 unit = 1 mm, x forward, y left, z up, origin at the centre of rotation on
// the floor. Geometry from src/dims.py (CAD r1) unless marked "approx" below.
//
// Parts list (written before geometry), ~85 parts:
//  Bench (13): plate, 4 feet (flange + foot), groove, 4 corner screws, ruler strip,
//    label plate (engraved, 3 rule lines), halo
//  Chassis (10): bottom plate with 4 wheel wells, top plate cut at y = -4 with 2 wells,
//    6 M3 hex deck pillars (39.7 mm)
//  Motors (4 x 6 + 8): bracket wall, 2 nylon spacers, gearbox, can, encoder cap,
//    6-pin connector, output shaft; front-left adds the open encoder cover, magnet
//    (lit), 2 hall sensors, shaft flat, gearbox screws
//  Wheels (4): near-rear and two far wheels on their shafts, the near-front wheel
//    lying on the bench (hub, bore, spokes, tread)
//  Lower deck (12): battery + pad, XT30 pair, inline fuse holder + lid, D36V50F5 buck
//    (PCB, inductor, 2 caps, pin rows), ESP32 DevKit (PCB, 2 headers, WROOM can, USB-C)
//  Under the top deck (12): MDD3A (4 standoffs, board, VM / M1 / M2 terminal blocks,
//    input header), power switch (body, 2 lugs, bushing nut, lever)
//  Top deck (9): Pi 5 (4 standoffs, PCB, GPIO header, 5 V housing, Ethernet, 2 USB),
//    Active Cooler (base, fin block, fan)
//  Cables (9): pack lead, XT30-fuse, fuse-switch, switch-VM, VM-buck, buck-Pi,
//    motor pair, encoder lead, PWM lead

import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import * as k from "./kit/iso-kit.mjs";
import * as G from "./kit/lathe.mjs";
import { tubePieces, tubeSvg } from "./kit/tube.mjs";
import * as A from "./kit/audit.mjs";
import { kitScript } from "./scripts/inline-kit.mjs";
import * as M from "./model.mjs";

const HERE = dirname(fileURLToPath(import.meta.url));
const THEME = process.argv.includes("--light") ? "light" : "dark";

const W = 600;
const H = 350;

// ---- world plan ------------------------------------------------------------------
const BENCH = { x: -132, y: -80, w: 258, d: 230, r: 10 };
const BENCH_H = 10;
const FOOT_H = 5;
const PX = 115.39; // plate half-length (230.78)
const PY = 66.26; // plate half-width (132.52)
const SPINE = 47; // plate half-width beside the wheel wells
const WELL = [5, 75]; // |x| span of the wheel wells (approx, from the plate STL proportions)
const BP = [13.8, 18.3]; // bottom plate z (shaft 32.5 = 11.2 + 3 mm spacer above its top)
const TP = [58.0, 62.5]; // top plate z: 39.7 mm clear above the bottom plate
const CUT = -4; // the top plate is cut away for y > CUT
const SHAFT_Z = 32.5;
const AXLE = 40;
const WHEEL = { r: 32.5, w: 26, y0: 49 }; // wheel from |y| 49 to 75 (track 124)
const GB = { y0: 27.2, y1: 45.95, z0: 21.3, z1: 43.7 }; // gearbox inboard/wheel-side faces
const LAID = [78, 113];
const STRIP_X = 96; // the top plate keeps a front strip (x > 92) that carries the switch // where the near-front wheel lies on the bench

const P = k.fitProjection(
  [...k.boxCorners(BENCH, -BENCH_H - FOOT_H, 0), [0, 0, 88], [PX, -PY, 66], [-PX, -PY, 66], [-PX, 75, 66]],
  W,
  H,
  { pad: 20, azimuth: 52, elevation: 30 },
);
const V = G.viewOf(P);
const KS = G.scaleOf(P);
const at = (p) => k.iso(p, P);
const S = (paths, style) => k.solidSvg(paths, style);
const Ln = (d, style) => k.lineSvg(d, style);
const Dots = (points, style) => k.dotsSvg(points, style);
const Faded = (list, style) => k.fadedSvg(list, style);
const path3 = (points, closed = false) => k.pathOf(points.map(at), closed);
const f4 = (v) => v.toFixed(4);

const R = A.recorder(P);
const record = R.solid;
let parts = 0;
function put(name, ref, svg, shapes = [], { bias = 0, count = 1, ...rest } = {}) {
  parts += count;
  return R.put({ name, at: ref, bias, svg, shapes, piece: "car", ...rest });
}
function planeMatrix(o, ex, ey) {
  const p0 = at(o);
  const px = at(G.add3(o, ex));
  const py = at(G.add3(o, ey));
  return `matrix(${f4(px[0] - p0[0])} ${f4(px[1] - p0[1])} ${f4(py[0] - p0[0])} ${f4(py[1] - p0[1])} ${p0[0].toFixed(2)} ${p0[1].toFixed(2)})`;
}

// rounded box from corners
function box(name, [x0, y0, z0], [x1, y1, z1], style = {}, { r = 0.5, steps = 2, bevel = 0, extra = "", inner = "", bias = 0, touch } = {}) {
  const [ax, bx] = [Math.min(x0, x1), Math.max(x0, x1)];
  const [ay, by] = [Math.min(y0, y1), Math.max(y0, y1)];
  const plan = { x: ax, y: ay, w: bx - ax, d: by - ay, r };
  const shape = record(A.slab(plan, z0, z1 - z0, { name, touch }, steps));
  put(name, [(ax + bx) / 2, (ay + by) / 2, (z0 + z1) / 2], S(k.slabOf(plan, z0, z1 - z0, P, steps, bevel), { tone: "mid", ...style, inner }) + extra, [shape], { bias });
  return plan;
}
function turned(name, F, profile, style = {}, { extra = "", inner = "", bias = 0, steps, ref } = {}) {
  const paths = G.solidOf(profile, F, P, steps ? { steps } : {});
  const mid = (profile[0][0] + profile[profile.length - 1][0]) / 2;
  const shape = record(A.solid(F, profile, { name }));
  put(name, ref ?? G.add3(F.o, G.mul3(F.a, mid)), S(paths, { tone: "mid", ...style, inner }) + extra, [shape], { bias });
  return shape;
}
const disc = (name, F, s0, s1, r, style = {}, options = {}) => turned(name, F, [[s0, r], [s1, r]], style, options);
function prism(name, F, poly, s0, s1, style = {}, { extra = "", inner = "", bias = 0 } = {}) {
  const shape = record(A.prism(F, poly, s0, s1, { name }));
  const cx = poly.reduce((a, p) => a + p[0], 0) / poly.length;
  const cy = poly.reduce((a, p) => a + p[1], 0) / poly.length;
  const centre = G.add3(G.add3(G.add3(F.o, G.mul3(F.a, (s0 + s1) / 2)), G.mul3(F.u, cx)), G.mul3(F.v, cy));
  put(name, centre, S(G.prismOf(F, poly, s0, s1, P), { tone: "mid", ...style, inner }) + extra, [shape], { bias });
}
const roundRect = (u0, u1, v0, v1, r, steps = 3) => k.roundedPlan({ x: u0, y: v0, w: u1 - u0, d: v1 - v0, r }, steps);
const frame = (o, a, u, v) => G.frameOf(o, a, u, v);
const XA = (o, sx = 1) => frame(o, [sx, 0, 0], [0, 1, 0], [0, 0, 1]);
const YA = (o, sy = 1) => frame(o, [0, sy, 0], [1, 0, 0], [0, 0, 1]);
const ZA = (o) => frame(o, [0, 0, 1], [1, 0, 0], [0, 1, 0]);

// ---- cables -------------------------------------------------------------------------
const CABLES = [];
function cable(name, points, r = 1.2, { tone = "mid", bend = 3, maxLength = 14, bias = 0.05, bundle, touch, ends = [true, true] } = {}) {
  const route = G.fillet(points, Math.max(bend, r * 2.6), 10, { P, tube: r });
  R.route(name, route, r, { limp: true, touch, bundle, piece: "car" });
  CABLES.push(name);
  tubePieces(route, r, P, { maxLength, spacing: Math.max(0.6, Math.min(1.6, r)) }).forEach((piece, index) =>
    put(`${name}:${index}`, piece.mid, tubeSvg(piece, { tone }), [A.tube(piece.points, piece.radii)], { bias, route: name, chunk: index, count: 0, attrs: ` data-route="${name}"` }),
  );
  ends.forEach((on, index) => {
    if (!on) return;
    const end = index ? route[route.length - 1] : route[0];
    const size = Math.max(0.55, r * KS * 0.9);
    put(`${name}.end${index}`, end, Dots([at(end)], { size, tone: "hi" }), [A.ball(end, size / KS, { name: `${name}.end${index}`, owner: name })], { bias: bias + 0.15, count: 0 });
  });
  parts += 1;
  return route;
}

// ---- bench (static back layer) ---------------------------------------------------------
const back = [];
const byDepth = (points) => [...points].sort((a, b) => k.depthOf([a[0], a[1], 0], P) - k.depthOf([b[0], b[1], 0], P));
back.push(`<path class="iso-halo" d="${k.haloOf(BENCH, -BENCH_H - FOOT_H, BENCH_H, P)}"/>`);
for (const [x, y] of byDepth(k.corners(BENCH, 24))) {
  back.push(S(k.cylinder(x, y, 7.5, -BENCH_H - FOOT_H, 1.6, P, 28), { tone: "lo" }));
  back.push(S(k.cylinder(x, y, 5.8, -BENCH_H - FOOT_H + 1.6, FOOT_H - 1.6, P, 28), { tone: "mid" }));
}
back.push(S(k.slabOf(BENCH, -BENCH_H, BENCH_H, P, 8, 1.6), { tone: "mid", inner: Ln(k.sideSeam(BENCH, -4, P, 8), { tone: "faint" }) }));
back.push(Ln(k.planOutline(k.insetPlan(BENCH, 5), 0, P), { tone: "lo" }));
const benchScrews = k.corners(BENCH, 11);
back.push(Ln(benchScrews.map(([x, y]) => k.ring(x, y, 2.2, 0, P, 16)).join(""), { tone: "lo" }));
back.push(Dots(benchScrews.map(([x, y]) => at([x, y, 0])), { size: 0.5 }));
record(A.slab(BENCH, -BENCH_H, BENCH_H, { name: "stand.bench", ground: true }, 8));
parts += 13;
// ruler strip along the near edge: 5 mm minor, 25 mm major ticks
const STRIP = { x: -116, y: 134, w: 140, d: 7, r: 1 };
back.push(S(k.slabOf(STRIP, 0, 0.6, P, 3), { tone: "lo", crease: "none" }));
{
  const t = k.topTicks(STRIP.x + 5, STRIP.x + STRIP.w - 5, 5, 5, STRIP.y + 0.01, 0.6, [2, 3.8], P);
  back.push(Ln(t.minor, { tone: "lo" }), Ln(t.major, { tone: "mid" }));
  back.push(Dots([at([STRIP.x + 2.4, STRIP.y + 3.5, 0.6]), at([STRIP.x + STRIP.w - 2.4, STRIP.y + 3.5, 0.6])], { size: 0.45 }));
}
// label plate with the one engraved label
const LABEL = { x: -118, y: 104, w: 38, d: 15, r: 1.5 };
back.push(S(k.slabOf(LABEL, 0, 0.8, P, 4), { tone: "lo", crease: "none" }));
back.push(Ln(k.planOutline(k.insetPlan(LABEL, 1.6), 0.8, P, 4), { tone: "faint" }));
back.push(k.faceTextSvg(k.topMatrix([LABEL.x + 4, LABEL.y + 5.6], 0.8, P, "x"), "SCOUT v2", { size: 4, tone: "mid" }));
back.push(Ln([24, 17, 21].map((w, i) => k.lineOnTop([LABEL.x + 4, LABEL.y + 7.8 + i * 2.2], [LABEL.x + 4 + w, LABEL.y + 7.8 + i * 2.2], 0.8, P)).join(""), { tone: "lo", free: true }));
back.push(Dots([at([LABEL.x + LABEL.w - 2.6, LABEL.y + 2.6, 0.8]), at([LABEL.x + LABEL.w - 2.6, LABEL.y + LABEL.d - 2.6, 0.8])], { size: 0.45 }));
// soft shadow of the car and of the laid wheel
back.push(`<path class="iso-halo" d="${k.haloOf({ x: -112, y: -76, w: 224, d: 152, r: 24 }, 0, 0.1, P)}" style="filter:blur(6px);opacity:.7"/>`);
back.push(`<path class="iso-halo" d="${k.haloOf({ x: LAID[0] - 31, y: LAID[1] - 31, w: 62, d: 62, r: 31 }, 0, 0.1, P)}" style="filter:blur(4px);opacity:.6"/>`);

// ---- chassis plates ---------------------------------------------------------------------
// Each plate is a spine (|y| <= 47) plus the tabs between and beyond the wheel wells, drawn as
// separate pieces so every wheel sorts cleanly against the plate around its well.
function plateTabs(side) {
  const [a, b] = WELL;
  const y0 = side > 0 ? SPINE : -PY;
  const d = PY - SPINE;
  return [
    { x: -PX, y: y0, w: PX - b, d, r: 0 },
    { x: -a, y: y0, w: 2 * a, d, r: 0 },
    { x: b, y: y0, w: PX - b, d, r: 0 },
  ];
}
function plateHoles() {
  // zip-tie holes beside the battery (dims.py, measured) and two harness holes (approx)
  return [[19.3, 18.2], [-17.3, 18.2], [19.3, -18.4], [-17.3, -18.4], [-104, 30], [-104, -30], [104, -36], [-108, 0]];
}
{
  const spine = { x: -PX, y: -SPINE, w: 2 * PX, d: 2 * SPINE, r: 0 };
  put("plate.bottom", [0, 0, BP[0]], S(k.slabOf(spine, BP[0], BP[1] - BP[0], P, 2, 0.6), { tone: "mid" }) + Dots(plateHoles().map((p) => at([...p, BP[1]])), { size: 0.5, tone: "lo" }), [record(A.slab(spine, BP[0], BP[1] - BP[0], { name: "plate.bottom" }, 2))]);
  for (const side of [-1, 1]) plateTabs(side).forEach((tab, i) => put("plate.bottom.tab", [tab.x + tab.w / 2, tab.y + tab.d / 2, BP[0]], S(k.slabOf(tab, BP[0], BP[1] - BP[0], P, 2, 0.6), { tone: "mid" }), [record(A.slab(tab, BP[0], BP[1] - BP[0], { name: "plate.bottom" }, 2))], { count: 0 }));
}
{
  const spine = { x: -PX, y: -SPINE, w: 2 * PX, d: SPINE + CUT, r: 0 };
  const saw = (x0, x1) => Ln(path3([[x0, CUT, (TP[0] + TP[1]) / 2], [x1, CUT, (TP[0] + TP[1]) / 2]]), { tone: "faint" });
  put("plate.top", [0, (CUT - SPINE) / 2, TP[0]], S(k.slabOf(spine, TP[0], TP[1] - TP[0], P, 2, 0.6), { tone: "mid", inner: saw(-PX, STRIP_X) }), [record(A.slab(spine, TP[0], TP[1] - TP[0], { name: "plate.top" }, 2))]);
  plateTabs(-1).forEach((tab) => put("plate.top.tab", [tab.x + tab.w / 2, tab.y + tab.d / 2, TP[0]], S(k.slabOf(tab, TP[0], TP[1] - TP[0], P, 2, 0.6), { tone: "mid" }), [record(A.slab(tab, TP[0], TP[1] - TP[0], { name: "plate.top" }, 2))], { count: 0 }));
  const front = { x: STRIP_X, y: CUT, w: PX - STRIP_X, d: PY - CUT, r: 0 };
  put("plate.top.front", [front.x + front.w / 2, front.y + front.d / 2, TP[0]], S(k.slabOf(front, TP[0], TP[1] - TP[0], P, 2, 0.6), { tone: "mid" }), [record(A.slab(front, TP[0], TP[1] - TP[0], { name: "plate.top" }, 2))], { count: 0 });
}
parts += 2;
// deck pillars: M3 brass hex, 39.7 mm, the clear gap between the decks
const PILLARS = [[-80.7, -56.1], [-80.7, 55.9], [-0.7, 55.9], [0.3, -56.1], [84.4, -52.3], [84.4, 52.7]];
for (const [x, y] of PILLARS) {
  const F = ZA([x, y, 0]);
  const top = y > CUT ? Dots([at([x, y, TP[0]])], { size: 0.5, tone: "lo" }) : "";
  turned("pillar", F, [[BP[1], 2.75], [TP[0], 2.75]], { tone: "mid" }, { steps: 6, extra: top, inner: Ln(G.arcOf(F, BP[1] + 6, 2.75, P) + G.arcOf(F, TP[0] - 6, 2.75, P), { tone: "faint" }) });
}

// ---- motors ---------------------------------------------------------------------------------
// u along the motor from the output shaft (+ toward the can, which points to the plate end).
function motor(sx, sy, front) {
  const X = (u) => sx * (AXLE + u);
  const Y = (v) => sy * v;
  const name = `motor.${sx > 0 ? "f" : "r"}${sy > 0 ? "l" : "r"}`;
  // Elegoo bracket wall on two 3 mm nylon spacers
  for (const hx of [57.7, 65.5]) disc(`${name}.spacer`, ZA([sx * hx, Y(24.7), 0]), BP[1], BP[1] + 3, 2.4, { tone: "lo" });
  box(`${name}.bracket`, [sx * 55.6, Y(22.2), BP[1] + 3], [sx * 67.6, Y(27.2), 43.3], { tone: "lo" }, { extra: Dots([at([sx * 57.7, Y(27.25), 38]), at([sx * 65.5, Y(27.25), 38])].map((p) => p), { size: 0.45, tone: "hi" }) });
  // gearbox 37 long, 18.75 thick, 22.4 tall
  const gb = { x0: X(-9.25), x1: X(27.75) };
  const screws = front && sy > 0 ? Dots([[X(-4), GB.y1 + 0.02, 25], [X(-4), GB.y1 + 0.02, 40], [X(22), GB.y1 + 0.02, 25], [X(22), GB.y1 + 0.02, 40]].map(at), { size: 0.5, tone: "lo" }) : "";
  box(`${name}.gearbox`, [gb.x0, Y(GB.y0), GB.z0], [gb.x1, Y(GB.y1), GB.z1], { tone: "mid" }, { r: 1.2, steps: 3, bevel: 0.5, extra: screws, inner: Ln(k.sideSeam({ x: Math.min(gb.x0, gb.x1), y: Math.min(Y(GB.y0), Y(GB.y1)), w: 37, d: 18.75, r: 1.2 }, 36, P, 3), { tone: "faint" }) });
  // 130-size can
  const canF = XA([X(27.75), Y(36.6), SHAFT_Z], sx);
  disc(`${name}.can`, canF, 0, 31.25, 10.6, { tone: "hi" }, { inner: Ln(G.arcOf(canF, 6, 10.6, P) + G.arcOf(canF, 26, 10.6, P), { tone: "faint" }) });
  // encoder cap: 6.9 long, 18.45 wide (y 28.5-46.95), 25.2 tall (z 19.9-45.1)
  const capF = frame([X(59), 0, 0], [sx, 0, 0], [0, 1, 0], [0, 0, 1]);
  const capPoly = roundRect(Y(28.5) * (sy > 0 ? 1 : 1), Y(46.95), 19.9, 45.1, 3).map(([u, v]) => [u, v]);
  const fixed = sy > 0 ? capPoly : roundRect(Y(46.95), Y(28.5), 19.9, 45.1, 3);
  const open = front && sy > 0 && sx > 0;
  let extra = "";
  if (open) {
    // cover off: a round opening in the cap's end face with the magnet inside
    const face = X(65.9) + 0.02;
    const o = [face, Y(37.7), SHAFT_Z];
    const m = planeMatrix(o, [0, 1, 0], [0, 0, -1]);
    extra =
      `<g transform="${m}"><circle class="iso-shade" data-shade="3" r="8.6"/><circle class="iso-line" data-tone="mid" r="8.6" fill="none"/>` +
      `<g data-part="magnet" data-lit=""><circle class="dp-magnet" r="6.8"/>${Array.from({ length: 12 }, (_, i) => {
        const a = (i * Math.PI) / 6;
        return `<path class="iso-line" data-tone="${i % 2 ? "lo" : "mid"}" d="M${(1.6 * Math.cos(a)).toFixed(2)} ${(1.6 * Math.sin(a)).toFixed(2)}L${(6.8 * Math.cos(a)).toFixed(2)} ${(6.8 * Math.sin(a)).toFixed(2)}"/>`;
      }).join("")}<circle class="iso-line" data-tone="hi" r="6.8" fill="none"/><circle class="iso-line" data-tone="mid" r="1.6" fill="none"/></g>` +
      // two hall sensors 90 degrees apart on the rim: channels A and B
      `<rect class="dp-hall" x="-1.3" y="-9.6" width="2.6" height="1.8" rx="0.3"/><rect class="dp-hall" x="7.8" y="-1.3" width="1.8" height="2.6" rx="0.3"/>` +
      `<circle class="iso-line" data-tone="faint" r="10.4" fill="none"/></g>`;
  }
  prism(`${name}.encoder`, capF, sy > 0 ? capPoly : fixed, 0, 6.9, { tone: open ? "hi" : "mid", lit: false }, { extra, bias: open ? 0.4 : 0 });
  // 6-pin connector block on the wheel side of the cap
  box(`${name}.conn`, [X(53.3), Y(46.95), 27.5], [X(65.9), Y(54.25), 37.5], { tone: "lo" }, { r: 0.4, extra: front && sy > 0 ? Dots([0, 1, 2, 3, 4, 5].map((i) => at([X(54.8 + i * 1.9), Y(50.6), 37.52])), { size: 0.42, tone: "hi" }) : "" });
  // output shaft (double-D, 5.4); on a mounted wheel it disappears into the hub at |y| 49
  const shF = YA([X(0), Y(GB.y1), SHAFT_Z], sy);
  const bare = front && sy > 0;
  const flat = bare
    ? `<g transform="${planeMatrix([X(0), GB.y1 + 9.3 + 0.02, SHAFT_Z], [1, 0, 0], [0, 0, -1])}"><g data-part="spinShaft"><path class="iso-line" data-tone="lo" d="M-1.9 -1.6L1.9 -1.6"/></g></g>`
    : "";
  disc(`${name}.shaft`, shF, 0, bare ? 9.3 : WHEEL.y0 - GB.y1, 2.7, { tone: "hi" }, { extra: flat, bias: bare ? 0.3 : 0 });
}
motor(-1, -1, false);
motor(1, -1, true);
motor(-1, 1, false);
motor(1, 1, true);

// ---- wheels ----------------------------------------------------------------------------------
const TYRE = [[0, 26.5], [0.9, 30.6, 1], [2.6, 32.5, 1], [23.4, 32.5, 1], [25.1, 30.6, 1], [26, 26.5]];
function spokes(n = 6) {
  // drawn in the wheel's face plane: rim lip, hub, 6 spokes, a double-D bore
  const sp = Array.from({ length: n }, (_, i) => {
    const a = (i * 2 * Math.PI) / n;
    const [c, s] = [Math.cos(a), Math.sin(a)];
    const q = (r, d) => `${(r * c - d * s).toFixed(2)} ${(r * s + d * c).toFixed(2)}`;
    return `M${q(7.6, -1.6)}L${q(22, -2.4)}M${q(7.6, 1.6)}L${q(22, 2.4)}`;
  }).join("");
  return `<circle class="iso-line" data-tone="lo" r="22" fill="none"/><circle class="iso-line" data-tone="mid" r="7.6" fill="none"/><path class="iso-line" data-tone="lo" d="${sp}"/><path class="iso-line" data-tone="mid" d="M-1.75 -2.15A2.7 2.7 0 0 1 1.75 -2.15ZM-1.75 2.15A2.7 2.7 0 0 0 1.75 2.15Z" fill="none"/>`;
}
function wheel(x, sy, live) {
  const F = YA([x, sy * WHEEL.y0, SHAFT_Z], sy);
  const tread = Faded(G.ribsOf(F, 2.6, 23.4, 32.5, 44, P, { fade: [0.08, 0.5], seams: true }), { tone: "lo" });
  let face = "";
  if (live) {
    const m = planeMatrix([x, sy * (WHEEL.y0 + WHEEL.w) + 0.02 * sy, SHAFT_Z], [1, 0, 0], [0, 0, -1]);
    face = `<g transform="${m}"><g data-part="${live}">${spokes()}</g></g>`;
  }
  turned(`wheel.${x > 0 ? "f" : "r"}${sy > 0 ? "l" : "r"}`, F, TYRE, { tone: "mid" }, { extra: tread + face, steps: 72 });
}
wheel(-AXLE, -1, "");
wheel(AXLE, -1, "");
wheel(-AXLE, 1, "spinRear");
{
  // the near-front wheel, pulled off its shaft and laid on the bench
  const F = ZA([LAID[0], LAID[1], 0]);
  const tread = Faded(G.ribsOf(F, 2.6, 23.4, 32.5, 44, P, { fade: [0.08, 0.5], seams: true }), { tone: "lo" });
  const m = planeMatrix([LAID[0], LAID[1], WHEEL.w + 0.02], [1, 0, 0], [0, 1, 0]);
  turned("wheel.laid", F, TYRE, { tone: "mid" }, { extra: tread + `<g transform="${m}">${spokes()}</g>`, steps: 72 });
}
// ---- lower deck ---------------------------------------------------------------------------------
const PAD = BP[1] + 1.1; // battery foam pad
const DECK = BP[1];
box("battery.pad", [-33, -23, BP[1]], [33, 23, PAD], { tone: "lo" }, { r: 1 });
box("battery", [-35, -25, PAD], [35, 25, PAD + 15], { tone: "hi" }, {
  r: 2,
  steps: 4,
  bevel: 0.8,
  inner: Ln(k.sideSeam({ x: -35, y: -25, w: 70, d: 50, r: 2 }, PAD + 7.5, P, 4), { tone: "faint" }),
  extra: Ln([30, 22, 26].map((w, i) => k.lineOnTop([-24, -6 + i * 4], [-24 + w, -6 + i * 4], PAD + 15, P)).join(""), { tone: "lo", free: true }),
});
// XT30 pair along x on the near spine, fuse holder after it
box("xt30.male", [-24, 31, DECK], [-13, 37, DECK + 5.2], { tone: "mid" }, { r: 0.6 });
box("xt30.female", [-13, 31.4, DECK], [-1, 36.6, DECK + 5.2], { tone: "mid" }, { r: 0.6 });
const FUSE = { x0: 4, x1: 25, y0: 29, y1: 39 };
box("fuse.holder", [FUSE.x0, FUSE.y0, DECK], [FUSE.x1, FUSE.y1, DECK + 7.5], { tone: "mid" }, { r: 1.2, steps: 3 });
box("fuse.lid", [FUSE.x0 + 2, FUSE.y0 + 1, DECK + 7.5], [FUSE.x1 - 2, FUSE.y1 - 1, DECK + 9.5], { tone: "hi" }, { r: 1, steps: 3, bevel: 0.3, extra: Ln(k.planOutline({ x: FUSE.x0 + 6, y: FUSE.y0 + 3.5, w: FUSE.x1 - FUSE.x0 - 12, d: 3, r: 1.5 }, DECK + 9.5, P, 3), { tone: "lo" }) });
// Pololu D36V50F5: 25.4 x 25.4 x 9.5
const BUCK = { x0: 38, x1: 63.4, y0: -12.7, y1: 12.7 };
box("buck.pcb", [BUCK.x0, BUCK.y0, DECK], [BUCK.x1, BUCK.y1, DECK + 1.6], { tone: "lo" }, { r: 0.6, extra: Dots([-9, -6, -3, 0, 3, 6, 9].map((y) => at([61.6, y, DECK + 1.6])), { size: 0.42, tone: "hi" }) });
box("buck.inductor", [41, -9, DECK + 1.6], [53, 3, DECK + 9.5], { tone: "mid" }, { r: 1.4, steps: 3, bevel: 0.4, extra: Ln(k.ring(47, -3, 3.2, DECK + 9.5, P, 20), { tone: "faint" }) });
disc("buck.cap", ZA([45, 8, 0]), DECK + 1.6, DECK + 6.5, 2.4, { tone: "hi" }, { inner: "" });
disc("buck.cap", ZA([52, 8, 0]), DECK + 1.6, DECK + 6.5, 2.4, { tone: "hi" });
// ESP32 DevKit in the rear bay
const ESP = { x0: -94, x1: -39, y0: -14, y1: 14 };
const EZ = DECK + 8.5; // board underside: the DevKit stands on its own pin headers
for (const y of [-12.7, 12.7]) box("esp.header", [ESP.x0 + 6, y - 1.25, DECK], [ESP.x1 - 4, y + 1.25, EZ], { tone: "lo" }, { r: 0.3 });
box("esp.pcb", [ESP.x0, ESP.y0, EZ], [ESP.x1, ESP.y1, EZ + 1.6], { tone: "mid" }, {
  r: 0.8,
  extra: Dots([-12.7, 12.7].flatMap((y) => Array.from({ length: 19 }, (_, i) => at([ESP.x0 + 7 + i * 2.54, y, EZ + 1.6]))), { size: 0.38, tone: "hi" }),
});
box("esp.module", [-64, -9, EZ + 1.6], [-45, 9, EZ + 4.7], { tone: "hi" }, { r: 0.6, bevel: 0.3, extra: Ln(k.planOutline({ x: -62.5, y: -7.5, w: 16, d: 15, r: 0.6 }, EZ + 4.7, P, 2), { tone: "faint" }) });
box("esp.usb", [-95.5, -4.5, EZ + 1.6], [-88, 4.5, EZ + 4.8], { tone: "lo" }, { r: 1.2, steps: 3 });

// ---- under the top deck --------------------------------------------------------------------------
// Cytron MDD3A hung on 5 mm standoffs, components facing down (CAD r1)
const MDD = { x0: -37.13, x1: 11.13, y0: -20.83, y1: 20.83, z0: TP[0] - 5 - 1.6, z1: TP[0] - 5 };
for (const [x, y] of byDepth([[-34.6, -18.3], [8.6, -18.3], [-34.6, 18.3], [8.6, 18.3]])) {
  const top = y > CUT ? Dots([at([x, y, TP[0]])], { size: 0.45, tone: "lo" }) : "";
  turned("mdd.standoff", ZA([x, y, 0]), [[MDD.z1, 2.2], [TP[0], 2.2]], { tone: "lo" }, { steps: 6, extra: top });
}
box("mdd.board", [MDD.x0, MDD.y0, MDD.z0], [MDD.x1, MDD.y1, MDD.z1], { tone: "mid" }, {
  r: 1,
  steps: 3,
  bevel: 0.3,
  extra:
    Ln([[-34.6, -18.3], [8.6, -18.3], [-34.6, 18.3], [8.6, 18.3]].map(([x, y]) => k.ring(x, y, 2.6, MDD.z1, P, 14)).join(""), { tone: "faint" }) +
    Dots([...Array.from({ length: 8 }, (_, i) => at([-26 + i * 2.54, 16.5, MDD.z1])), ...Array.from({ length: 6 }, (_, i) => at([-30, -6.35 + i * 2.54, MDD.z1])), at([5, 6, MDD.z1]), at([5, 12, MDD.z1])], { size: 0.38, tone: "lo" }),
});
const TB_Z = [MDD.z0 - 11, MDD.z0];
box("mdd.vm", [3, 3, TB_Z[0]], [11.1, 15, TB_Z[1]], { tone: "hi" }, { r: 0.4, extra: Dots([at([11.15, 6, TB_Z[0] + 4]), at([11.15, 12, TB_Z[0] + 4])], { size: 0.45, tone: "lo" }) });
box("mdd.m1", [-9, 13, TB_Z[0]], [1, 20.8, TB_Z[1]], { tone: "hi" }, { r: 0.4, extra: Dots([at([-6.5, 20.85, TB_Z[0] + 4]), at([-1.5, 20.85, TB_Z[0] + 4])], { size: 0.45, tone: "lo" }) });
box("mdd.m2", [-21, 13, TB_Z[0]], [-11, 20.8, TB_Z[1]], { tone: "hi" }, { r: 0.4, extra: Dots([at([-18.5, 20.85, TB_Z[0] + 4]), at([-13.5, 20.85, TB_Z[0] + 4])], { size: 0.45, tone: "lo" }) });
box("mdd.input", [-36.8, -14, MDD.z0 - 6], [-34, 6, MDD.z0], { tone: "lo" }, { r: 0.3 });
// GY-521 (MPU-6050) hung under the top deck on 10 mm nylon standoffs (CAD r1: x 25, y 0)
const IMU = { x0: 14.4, x1: 35.6, y0: -8.2, y1: 8.2, z0: TP[0] - 10 - 1.6, z1: TP[0] - 10 };
for (const [x, y] of byDepth([[16.9, -5.7], [33.1, -5.7], [16.9, 5.7], [33.1, 5.7]])) {
  const top = y > CUT ? Dots([at([x, y, TP[0]])], { size: 0.4, tone: "lo" }) : "";
  turned("imu.standoff", ZA([x, y, 0]), [[IMU.z1, 1.6], [TP[0], 1.6]], { tone: "lo" }, { steps: 12, extra: top });
}
box("imu.board", [IMU.x0, IMU.y0, IMU.z0], [IMU.x1, IMU.y1, IMU.z1], { tone: "mid" }, {
  r: 0.6,
  extra: Dots(Array.from({ length: 8 }, (_, i) => at([16.1 + i * 2.54, 7.1, IMU.z1])), { size: 0.36, tone: "hi" }),
});
box("imu.chip", [22.5, -2, IMU.z0 - 0.9], [26.5, 2, IMU.z0], { tone: "lo" }, { r: 0.2 });
// main switch, >= 10 A DC: body under the plate, lever above (front-left, CAD r1)
const SW = { x0: 98.5, x1: 111.5, y0: 28, y1: 36, z0: TP[0] - 10, z1: TP[0] };
box("switch.body", [SW.x0, SW.y0, SW.z0], [SW.x1, SW.y1, SW.z1], { tone: "mid" }, { r: 0.8, bevel: 0.3, inner: Ln(k.sideSeam({ x: SW.x0, y: SW.y0, w: 13, d: 8, r: 0.8 }, SW.z0 + 6.5, P, 2), { tone: "faint" }) });
box("switch.lugA", [SW.x0 - 2.4, 32.4, SW.z0 + 3], [SW.x0, 35.6, SW.z0 + 8], { tone: "hi" }, { r: 0.2 });
box("switch.lugB", [SW.x0 - 2.4, 28.4, SW.z0 + 0.5], [SW.x0, 31.6, SW.z0 + 5], { tone: "hi" }, { r: 0.2 });
disc("switch.nut", ZA([105, 32, 0]), TP[1], TP[1] + 2.2, 4.4, { tone: "lo" }, { steps: 6 });
disc("switch.bushing", ZA([105, 32, 0]), TP[1] + 2.2, TP[1] + 4, 3, { tone: "mid" });
turned("switch.lever", frame([105, 32, TP[1] + 4], G.unit3([0.35, 0, 1]), [0, 1, 0], G.unit3([1, 0, -0.35])), [[0, 1.4], [9, 1.2], [9.6, 0.8, 1], [10, 0.3, 1]], { tone: "hi" });

// ---- top deck: Pi 5 + Active Cooler -----------------------------------------------------------------
const PI = { x0: -7.5, x1: 77.5, y0: -28, y1: 28, z0: TP[1] + 6, z1: TP[1] + 7.5 };
for (const [x, y] of byDepth([[16, -24.5], [74, -24.5], [16, 24.5], [74, 24.5]])) turned("pi.standoff", ZA([x, y, 0]), [[TP[1], 2.4], [PI.z0, 2.4]], { tone: "lo" }, { steps: 6 });
box("pi.pcb", [PI.x0, PI.y0, PI.z0], [PI.x1, PI.y1, PI.z1], { tone: "mid" }, {
  r: 3,
  steps: 4,
  extra: Ln([[16, -24.5], [74, -24.5], [16, 24.5], [74, 24.5]].map(([x, y]) => k.ring(x, y, 2.9, PI.z1, P, 14)).join(""), { tone: "faint" }),
});
box("pi.gpio", [23, -27.3, PI.z1], [74, -22.3, PI.z1 + 2.5], { tone: "lo" }, {
  r: 0.3,
  extra: Dots([-26, -23.6].flatMap((y) => Array.from({ length: 18 }, (_, i) => at([24.3 + i * 2.54, y, PI.z1 + 2.5]))), { size: 0.36, tone: "hi" }),
});
box("pi.5v", [68.4, -27.6, PI.z1 + 2.5], [74.6, -21.4, PI.z1 + 11], { tone: "hi" }, { r: 0.5, bevel: 0.3 });
box("pi.eth", [-9.5, -26.5, PI.z1], [11.5, -10.5, PI.z1 + 13.5], { tone: "mid" }, { r: 0.6, bevel: 0.3 });
box("pi.usb", [-9.5, -8.5, PI.z1], [8, 5.5, PI.z1 + 16], { tone: "mid" }, { r: 0.6, bevel: 0.3, inner: Ln(k.sideSeam({ x: -9.5, y: -8.5, w: 17.5, d: 14, r: 0.6 }, PI.z1 + 8, P, 2), { tone: "faint" }) });
box("pi.usb", [-9.5, 8.5, PI.z1], [8, 22.5, PI.z1 + 16], { tone: "mid" }, { r: 0.6, bevel: 0.3, inner: Ln(k.sideSeam({ x: -9.5, y: 8.5, w: 17.5, d: 14, r: 0.6 }, PI.z1 + 8, P, 2), { tone: "faint" }) });
box("cooler.base", [12, -19, PI.z1], [66, 21, PI.z1 + 3], { tone: "lo" }, { r: 1, steps: 3 });
box("cooler.fins", [14, -17, PI.z1 + 3], [64, 19, PI.z1 + 10], { tone: "mid" }, {
  r: 1.4,
  steps: 3,
  bevel: 0.4,
  extra:
    Ln(Array.from({ length: 6 }, (_, i) => k.planOutline({ x: 16.5 + i * 3.2, y: -14.5, w: 1.4, d: 31, r: 0.7 }, PI.z1 + 10, P, 3)).join(""), { tone: "faint" }) +
    Ln(k.ring(49, 1, 11, PI.z1 + 10, P, 40) + k.ring(49, 1, 3.4, PI.z1 + 10, P, 20), { tone: "lo" }) +
    Ln(Array.from({ length: 7 }, (_, i) => {
      const a = (i * 2 * Math.PI) / 7;
      return k.lineOnTop([49 + 3.4 * Math.cos(a), 1 + 3.4 * Math.sin(a)], [49 + 11 * Math.cos(a + 0.5), 1 + 11 * Math.sin(a + 0.5)], PI.z1 + 10, P);
    }).join(""), { tone: "faint" }),
});

// ---- cables (fillet-routed, depth-sorted, audited) ----------------------------------------------------
const Z5 = DECK + 2.6; // XT30 / fuse lead height
const LUG_X = SW.x0 - 2.4;
cable("lead.pack", [[-27.6, 25, Z5], [-27.6, 34, Z5], [-24, 34, Z5]], 1.2, { bend: 2.4, touch: ["battery", "xt30.male"] });
cable("lead.xt30", [[-1, 34, Z5], [FUSE.x0, 34, Z5]], 1.2, { touch: ["xt30.female", "fuse.holder"], ends: [false, false] });
cable("lead.fuse", [[FUSE.x1, 34, Z5], [28.4, 34, Z5], [28.4, 34, 53.5], [LUG_X, 34, 53.5]], 1.2, { bend: 2.6, touch: ["fuse.holder", "switch.lugA"] });
cable("lead.vm", [[LUG_X, 30, 50.75], [86, 30, 50.75], [76, 12, 50.75], [30, 12, 50.75], [18, 12, 44.5], [11.1, 12, 44.5]], 1.2, { bend: 5, touch: ["switch.lugB", "mdd.vm"] });
cable("lead.buckin", [[11.1, 6, 44.5], [39.5, 6, 44.5], [39.5, 6, DECK + 1.6]], 1.0, { bend: 5, touch: ["mdd.vm", "buck.pcb"] });
cable("lead.5v", [[61.6, -6, DECK + 1.6], [61.6, -6, 40], [89, -0.5, 46], [89, -0.5, 77], [89, -24.5, 77], [74.6, -24.5, 77]], 0.9, { bend: 8, touch: ["buck.pcb", "pi.5v"] });
// Hiwonder 6-pin harness: motor pair to MDD3A M1, encoder (VCC GND A B) to the ESP32
cable("harness.motor", [[98.5, 50.6, 37.5], [98.5, 50.6, 47], [83.5, 18, 47], [1, 18, 47]], 1.0, { tone: "hi", bend: 5, bundle: "harness", touch: ["motor.fl.conn", "mdd.m1"] });
cable("harness.enc", [[95.5, 50.6, 37.5], [95.5, 50.6, 47], [83.5, 25, 47], [-30, 25, 47], [-44, 12.7, 47], [-52, 12.7, 47], [-52, 12.7, EZ + 1.6]], 1.0, { tone: "hi", bend: 5, bundle: "harness", touch: ["motor.fl.conn", "esp.pcb"] });
cable("lead.pwm", [[-47, -12.7, EZ + 1.6], [-47, -12.7, 48], [-36.8, -12.7, 48]], 0.8, { tone: "lo", bend: 3, touch: ["esp.pcb", "mdd.input"] });

// ---- settle the painter's order --------------------------------------------------------------------------
A.settle(R, P);
R.items.sort((a, b) => a.key - b.key);
const groups = R.items.map((item) => `<g class="it" data-name="${item.name}"${item.attrs ?? ""}>${item.svg}</g>`);

// ---- live script ----------------------------------------------------------------------------------------------
const modelSource = readFileSync(join(HERE, "model.mjs"), "utf8").replace(/^export /gm, "");
const LIVE = `
const M = (() => { ${modelSource}
 return { SCOUT, stateAt, wheelYaw, countsPerSecond, dutyCap, LOOP }; })();
const stage = document.querySelector(".iso-stage");
const svg = stage.querySelector("svg");
const $ = (name) => svg.querySelector('[data-part="' + name + '"]');
const el = { rear: $("spinRear"), shaft: $("spinShaft"), magnet: $("magnet") };
const out = document.querySelector("[data-readout]");
const still = matchMedia("(prefers-reduced-motion: reduce)");
const R = M.SCOUT.wheelD / 2;
const MAGNET_SLOW = 14; // the magnet turns 42x the wheel; drawn at 42/14 = 3x so it stays readable
let t = 0, last = 0, frame = 0, visible = false, held = false, heldFor = 0, angle = 0, shown = { left: 0, right: 0 };
const deg = (r) => (r * 180 / Math.PI).toFixed(2);
const f2 = (v) => (Math.abs(v) < 0.005 ? "0.00" : v.toFixed(2));
function current() {
  if (!held) return M.stateAt(t);
  // the user cut /cmd_vel: watchdog then brake, from whatever the robot was doing
  const base = M.stateAt(t);
  const since = heldFor;
  let gain = 1, mode = "waiting";
  if (since >= M.SCOUT.watchdog) { gain = Math.max(0, 1 - (since - M.SCOUT.watchdog) / M.SCOUT.brake); mode = gain > 0 ? "braking" : "stopped"; }
  return { ...base, left: base.left * gain, right: base.right * gain, mode, cmd: false, since };
}
function text(s) {
  const lr = "L " + f2(s.left) + " \\u00b7 R " + f2(s.right) + " m/s";
  if (s.mode === "waiting") return "no /cmd_vel " + Math.round((s.since ?? (s.t - 9)) * 1000) + " ms \\u00b7 " + lr;
  if (s.mode === "braking") return "watchdog: brake LOW/LOW \\u00b7 " + lr;
  if (s.mode === "stopped") return "stopped \\u00b7 MDD3A inputs LOW/LOW";
  if (s.mode === "idle") return "waiting for /cmd_vel";
  if (s.phase === "turn") return "L " + f2(s.left) + " R " + f2(s.right) + " m/s \\u00b7 yaw " + M.wheelYaw(s).toFixed(2) + " vs gyro " + s.w.toFixed(2);
  return lr + " \\u00b7 " + Math.round(M.countsPerSecond(s.left)) + " counts/s";
}
function draw(s) {
  el.rear.setAttribute("transform", "rotate(" + deg(angle) + ")");
  el.shaft.setAttribute("transform", "rotate(" + deg(angle) + ")");
  el.magnet.setAttribute("transform", "rotate(" + deg(angle * 42 / MAGNET_SLOW) + ")");
  const words = text(s);
  if (out.textContent !== words) out.textContent = words;
}
function tick(now) {
  const dt = last ? Math.min((now - last) / 1000, 1 / 20) : 1 / 60;
  last = now;
  if (held) heldFor += dt; else t = (t + dt) % M.LOOP;
  const s = current();
  const ease = 1 - Math.exp(-dt / 0.06);
  shown.left += (s.left - shown.left) * ease;
  angle += (shown.left / R) * dt;
  draw(s);
  frame = visible && !still.matches ? requestAnimationFrame(tick) : 0;
}
const run = () => { if (!frame && visible && !still.matches) { last = 0; frame = requestAnimationFrame(tick); } };
const hold = (on) => { held = on; heldFor = 0; if (!on) { /* resume the tour after the stop */ } stage.setAttribute("aria-valuetext", on ? "cmd_vel cut: watchdog brakes after 300 ms" : "touring"); if (still.matches) { const s = on ? { ...current(), mode: "stopped" } : M.stateAt(1); draw(s); } else run(); };
new IntersectionObserver((entries) => { visible = entries[entries.length - 1].isIntersecting; if (visible) run(); }, { rootMargin: "120px 0px" }).observe(stage);
stage.addEventListener("pointerdown", (e) => { stage.setPointerCapture?.(e.pointerId); hold(true); });
stage.addEventListener("pointerup", () => hold(false));
stage.addEventListener("pointercancel", () => hold(false));
stage.addEventListener("keydown", (e) => { if ((e.key === " " || e.key === "Enter") && !e.repeat) { e.preventDefault(); hold(true); } });
stage.addEventListener("keyup", (e) => { if (e.key === " " || e.key === "Enter") hold(false); });
const params = new URLSearchParams(location.search);
if (params.has("t")) { t = +params.get("t"); }
draw(current());
window.__drive = { get t() { return t; }, set t(v) { t = v; }, hold, current };
`;

const CSS = `
.dp-magnet{fill:var(--anatomy-paper,#111);stroke:none}
[data-theme="dark"] .dp-magnet,.iso[data-theme="dark"] .dp-magnet{fill:#2a2a2a}
[data-theme="light"] .dp-magnet,.iso[data-theme="light"] .dp-magnet{fill:#e9e9e6}
.dp-hall{fill:currentColor;opacity:.55}
[data-part="magnet"] .iso-line{stroke-width:.7px}
.iso-legend .iso-key b{font-weight:600;margin-right:.35em}
@media (max-width:520px){.iso-plate{overflow:hidden}.iso-stage svg{width:108%;margin:0 -4%}}
`;

const svg = k.figureSvg({
  width: W,
  height: H,
  label:
    "Scout v2 on a bench, cut away on its left side. The near half of the top deck is removed and the near front wheel lies on the bench, showing the power path from the 3S battery through the XT30, the 7.5 A fuse and the 10 A switch to the Cytron MDD3A motor driver hung under the top deck, a branch to the Pololu 5 V buck and up to the Raspberry Pi 5 header, and the Hiwonder TT gear motor with its hall encoder cover off, whose harness runs back to the ESP32 and the driver.",
  body: [...back, `<g class="dp-car">${groups.join("")}</g>`],
});

const mm = M.mmPerCount();
const plate = k.plateHtml({
  fig: "Fig 2",
  title: "Drive and power",
  hint: "Hold to cut /cmd_vel",
  readout: "L 0.30 \u00b7 R 0.30 m/s \u00b7 1481 counts/s",
  keys: [
    { mark: "raised", label: `1 Power: 3S pack 9.0\u201312.6 V \u2192 7.5 A fuse \u2192 10 A DC switch \u2192 Cytron MDD3A. Duty capped at 8.4 V \u00f7 V_pack (67 % full, 93 % at 9.0 V), so the motors see \u2264 8.4 V average. Branch: Pololu D36V50F5 \u2192 5 V to the Pi 5 header` },
    { mark: "lit", label: `2 Drive: Hiwonder TT motor, 1:42 gearbox; hall encoder on the motor shaft, 1008 counts per wheel turn claimed (\u00d74 quadrature) \u2192 \u03c0 \u00d7 65 mm \u00f7 1008 = ${mm.toFixed(2)} mm per count` },
    { mark: "edge", label: `3 Skid steer: in a turn the tyres scrub sideways, so the wheels run as if the track were 1.7\u20131.9 \u00d7 the 124 mm geometric one. Wheel odometry is good for distance, bad for heading: the EKF takes yaw from the MPU-6050 gyro` },
    { mark: "dashed", label: `4 Loop: ESP32 counts with PCNT and runs the wheel PID at \u2265 50 Hz (20 ms); no /cmd_vel for 300 ms \u2192 MDD3A inputs LOW/LOW: braked to a stop in 0.16\u20130.19 s (coasting takes 0.32\u20130.42 s)` },
  ],
  caption:
    "Scout v2's left side with the near half of the top deck cut away (the parts it carries stay in place) and the near front wheel laid on the bench. The tour drives straight at 0.3 m/s (1.47 wheel turns, about 1481 counts per second), then turns left at 0.5 rad/s with 0.2 m/s forward: the near wheels slow to 0.14 m/s and the far ones speed up to 0.26 m/s, because the scrubbing tyres need an effective track about 1.8 \u00d7 124 mm. Read through the geometric track those wheel speeds claim 0.90 rad/s, so heading comes from the gyro. Then /cmd_vel goes quiet: after 300 ms the ESP32 drives both MDD3A inputs LOW and the motors brake. Hold the figure to cut /cmd_vel yourself. The encoder magnet is drawn turning 3 \u00d7 the wheel; the real one turns 42 \u00d7. The 1008-count figure is Hiwonder's claim (one buyer measured 600): measure it on arrival. The MPU-6050 hangs under the top deck between the driver and the buck. The right-side wiring, the I\u00b2C lead and the LiDAR tower are left out; layout after CAD r1, with the fuse holder moved beside the XT30 so the series path is in view.",
  body: `<style>${CSS}</style><div class="iso-stage" tabindex="0" role="img" aria-label="Scout drivetrain and power cut-away. Press and hold, or hold Space, to stop sending /cmd_vel and watch the watchdog brake the motors." style="touch-action:pan-y;cursor:pointer">${svg}</div>`,
});

const name = THEME === "light" ? "drivetrain-power-light.html" : "drivetrain-power.html";
const page = k.pageHtml({ title: "Scout v2: drivetrain and power path", theme: THEME, body: plate, script: kitScript() + LIVE });
writeFileSync(join(HERE, name), page);
console.log("wrote", name, `${(page.length / 1024).toFixed(0)} KB`, `${R.items.length} items`, `parts ~${parts}`, `scale ${KS.toFixed(3)}`, `${CABLES.length} cables`, `mm/count ${mm.toFixed(4)}`);
// Excused after an 8x close-up (shots/zoom-5v.png): the 5 V lead ends on the housing face; the
// far top-plate tab edge only passes behind its bend, 10 mm deeper.
const CROWD_OK = [["lead.5v", "plate.top"]];
A.auditOrExit(R, P, { crowd: CROWD_OK });
