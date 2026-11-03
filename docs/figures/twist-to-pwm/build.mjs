// Fig 1 · From Twist to wheel PWM — Scout base bridge, DD-SCOUT-BRIDGE §5.
// node build.mjs [--light]  ->  twist-to-pwm.html / twist-to-pwm-light.html
//
// Concept. Scout (Elegoo Smart Robot Car V4 chassis) stands on a test mat, seen
// in its own frame (REP-103: x forward, y left). A cmd_vel twist (v, ω) puts the
// instantaneous centre of rotation (ICR, the lit pin) at y = v/ω on the axle
// line; the lit arc is the path the centre would drive. Each side's speed is its
// distance to the ICR times ω: v_L = v − ω·W/2, v_R = v + ω·W/2. Two speed rulers
// on the mat, one per side, carry a slider at that side's speed (after
// saturation; a faint tick marks the raw value when it was scaled). A serial
// terminal on the mat shows the `M l r` line that the bridge sends to the UNO R4
// over USB. Units: 1 world unit = 1 mm.
//
// Parts list (≈70): mat (slab, 4 feet, halo, grid, 4 corner screws, label plate),
// 2 speed rulers (strip, ticks, zero mark, v_max stops, end screws, slider, bar,
// raw tick), ICR pin (base ring, post, cap), radius line, arc + end dots,
// chassis (bottom plate, top plate, 4 standoffs, 4 TT motors, 4 axle stubs,
// 4 wheels with hub, rim ring, 3 screws), UNO R4 + shield (board, 4 spacers,
// 2 header rows, TB6612 chip, USB-C port, header pins), twist dial (inlay, v
// needle, ω arc), pan servo + HC-SR04 (servo, board, 2 transducers), serial
// terminal (case, bezel, screen, text, 2 feet, cable).

import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import * as k from "./kit/iso-kit.mjs";
import * as G from "./kit/lathe.mjs";
import { kitScript } from "./scripts/inline-kit.mjs";
import { EXAMPLE, twistToPwm } from "./model.mjs";

const HERE = dirname(fileURLToPath(import.meta.url));
const theme = process.argv.includes("--light") ? "light" : "dark";

const WIDTH = 600;
const HEIGHT = 400;

// ---- world plan (mm) ---------------------------------------------------------
const MAT = { x: -180, y: -185, w: 360, d: 450, r: 10 };
const MAT_H = 6;
const FOOT_H = 4;
const PLATE = { x: -115.4, y: -66.25, w: 230.8, d: 132.5, r: 8 }; // Elegoo V4 plates
const LOW = [19, 22]; // bottom plate z
const TOP = [46, 49]; // top plate z
const WHEEL_R = 32.5; // spec
const WHEEL_Y = 82; // drawing only (assumed)
const WHEEL_X = 62; // drawing only (assumed)
const WHEEL_W = 26;
const RULER_Y = 138;
const RULER = { from: -172, to: 172, d: 12, h: 3 };
const PER_V = 130; // mm of ruler per v_max
const DIAL = [42, 0, TOP[1] + 1];
const TERM = { x: -172, y: 182, w: 76, d: 48, r: 4 };
const TERM_H = 22;
const USB = [-63, 0, 58];

const P = k.fitProjection(
  [...k.boxCorners(MAT, -MAT_H - FOOT_H, 0), [0, 0, 92], [115, 0, 92], [-115, 0, 92]],
  WIDTH,
  HEIGHT,
  { pad: 26, azimuth: 52, elevation: 36 },
);

const at = (p) => k.iso(p, P);
const S = (paths, style) => k.solidSvg(paths, style);
const L = (d, style) => k.lineSvg(d, style);
const D = (pts, style) => k.dotsSvg(pts, style);
const depth = (p) => k.depthOf(p, P);
const byDepth = (list, key = (p) => p) => [...list].sort((a, b) => depth(key(a)) - depth(key(b)));
const NEAR = depth([0, -1, 0]) > depth([0, 1, 0]) ? -1 : 1; // sign of y on the near side

// ---- mat --------------------------------------------------------------------
function mat() {
  const out = [`<path class="iso-halo" d="${k.haloOf(MAT, -MAT_H - FOOT_H, MAT_H, P)}"/>`];
  for (const [x, y] of byDepth(k.corners(MAT, 24), ([x, y]) => [x, y, 0])) {
    out.push(S(k.cylinder(x, y, 8, -MAT_H - FOOT_H, FOOT_H, P, 28), { tone: "lo" }));
  }
  out.push(S(k.slabOf(MAT, -MAT_H, MAT_H, P, 8, 1.4), { tone: "mid" }));
  out.push(L(k.planOutline(k.insetPlan(MAT, 6), 0, P), { tone: "lo" }));
  // 50 mm grid, edge to edge inside the groove
  const g = k.insetPlan(MAT, 6);
  const grid = [];
  for (let x = -150; x <= 150; x += 50) grid.push(k.lineOnTop([x, g.y], [x, g.y + g.d], 0, P));
  for (let y = -150; y <= 250; y += 50) grid.push(k.lineOnTop([g.x, y], [g.x + g.w, y], 0, P));
  out.push(L(grid.join(""), { tone: "faint" }));
  // axle line (y axis through the robot centre) a touch stronger
  out.push(L(k.lineOnTop([0, g.y], [0, g.y + g.d], 0, P), { tone: "lo" }));
  const screws = k.corners(MAT, 12);
  out.push(L(screws.map(([x, y]) => k.ring(x, y, 3, 0, P, 16)).join(""), { tone: "lo" }));
  out.push(D(screws.map(([x, y]) => at([x, y, 0])), { size: 0.5 }));
  // label plate, back-left corner, engraved REP-103 axes mark
  const lp = { x: 100, y: -163, w: 58, d: 30, r: 2 };
  out.push(S(k.slabOf(lp, 0, 0.8, P, 3), { tone: "lo", crease: "none" }));
  out.push(k.faceTextSvg(k.topMatrix([lp.x + 5, lp.y + 9], 0.8, P, "x"), "REP-103", { size: 6, tone: "mid" }));
  for (const y of [lp.y + 15, lp.y + 20, lp.y + 25]) out.push(L(k.lineOnTop([lp.x + 5, y], [lp.x + lp.w - 5, y], 0.8, P), { tone: "faint", free: "both" }));
  out.push(D([at([lp.x + 2.5, lp.y + 2.5, 0.8]), at([lp.x + lp.w - 2.5, lp.y + lp.d - 2.5, 0.8])], { size: 0.4 }));
  return out.join("");
}

// ---- speed rulers -------------------------------------------------------------
function ruler(sign, name) {
  const y0 = sign * RULER_Y - RULER.d / 2;
  const strip = { x: RULER.from, y: y0, w: RULER.to - RULER.from, d: RULER.d, r: 1.5 };
  const z = RULER.h;
  const edge = sign === NEAR ? y0 + RULER.d : y0; // ticks hang from the viewer-facing long edge... on top
  const out = [S(k.slabOf(strip, 0, z, P, 3, 0.4), { tone: "mid" })];
  const t = k.topTicks(-130, 130, 13, 5, y0 + 0.6, z, [2.4, 4.4], P);
  out.push(L(t.minor, { tone: "lo" }), L(t.major, { tone: "mid" }));
  // v_max stops and zero mark run across the strip
  out.push(L([-PER_V, PER_V].map((x) => k.lineOnTop([x, y0], [x, y0 + RULER.d], z, P)).join(""), { tone: "hi" }));
  out.push(L(k.lineOnTop([0, y0], [0, y0 + RULER.d], z, P), { tone: "hi" }));
  out.push(D([at([RULER.from + 3, sign * RULER_Y, z]), at([RULER.to - 3, sign * RULER_Y, z])], { size: 0.45 }));
  void edge;
  // raw (pre-saturation) tick, bar and slider: live
  out.push(`<path data-part="${name}-raw" class="iso-line" data-tone="lo" d="" style="opacity:0"/>`);
  out.push(`<path data-part="${name}-bar" class="iso-line" data-tone="hi" d="" style="stroke-width:2.2;stroke-linecap:butt"/>`);
  const sl = { x: -6, y: y0 - 2, w: 12, d: RULER.d + 4, r: 1.5 };
  const slider = [S(k.slabOf(sl, z, 6, P, 3, 0.5), { tone: "hi", crease: "lo" })];
  slider.push(L(k.lineOnTop([0, sl.y], [0, sl.y + sl.d], z + 6, P), { tone: "lo" }));
  slider.push(D([at([0, sl.y + 2, z + 6]), at([0, sl.y + sl.d - 2, z + 6])], { size: 0.45, tone: "lo" }));
  out.push(`<g data-part="${name}-slider">${slider.join("")}</g>`);
  return out.join("");
}

// ---- ICR pin (built at the origin, moved along y) --------------------------
function icr(slot) {
  const out = [];
  out.push(L(k.ring(0, 0, 9, 0, P, 32), { tone: "hi" }));
  out.push(S(k.cylinder(0, 0, 5, 0, 2, P, 28), { tone: "hi", lit: true }));
  out.push(S(k.cylinder(0, 0, 2, 2, 20, P, 16), { tone: "hi", crease: "none", lit: true }));
  out.push(S(k.cylinder(0, 0, 3.4, 22, 3, P, 20), { tone: "hi", lit: true }));
  return `<g data-part="icr-${slot}" style="opacity:0">${out.join("")}</g>`;
}

// ---- chassis ----------------------------------------------------------------
function wheel(x, side) {
  const F = G.frameAlong([x, side * WHEEL_Y, WHEEL_R], [0, side, 0]);
  const out = [S(G.disc(-WHEEL_W / 2, WHEEL_W / 2, WHEEL_R, F, P, { steps: 56, bevel: 1.2 }), { tone: "lo" })];
  const near = side === NEAR;
  if (near) {
    out.push(L(G.circleOf(F, WHEEL_W / 2, WHEEL_R - 6, P, 56), { tone: "faint" }));
    out.push(S(G.disc(WHEEL_W / 2, WHEEL_W / 2 + 2, 11, F, P, { steps: 32 }), { tone: "mid" }));
    const hub = [0, 1, 2].map((i) => at(G.pointOf(F, WHEEL_W / 2 + 2, 6, (i / 3) * Math.PI * 2 + 0.4)));
    out.push(D(hub, { size: 0.5, tone: "lo" }));
    out.push(D([at(G.pointOf(F, WHEEL_W / 2 + 2, 0, 0))], { size: 0.7, tone: "hi" }));
  }
  return out.join("");
}
const axle = (x, side) => S(G.disc(0, 3.5, 3, G.frameAlong([x, side * 66, WHEEL_R], [0, side, 0]), P, { steps: 16 }), { tone: "mid" });
function motor(x, side) {
  const front = x > 0;
  const plan = { x: front ? x - 50 : x - 14, y: side > 0 ? 44 : -66, w: 64, d: 22, r: 3 };
  const out = [S(k.slabOf(plan, LOW[1], 22, P, 4, 0.6), { tone: "mid" })];
  out.push(k.sideSeam(plan, LOW[1] + 11, P) ? L(k.sideSeam(plan, LOW[1] + 11, P), { tone: "faint" }) : "");
  out.push(D([at([plan.x + 6, plan.y + 11, LOW[1] + 22]), at([plan.x + plan.w - 6, plan.y + 11, LOW[1] + 22])], { size: 0.45, tone: "lo" }));
  return out.join("");
}
function plateOf(z, tone) {
  const out = [S(k.slabOf(PLATE, z[0], z[1] - z[0], P, 8, 0.6), { tone })];
  return out.join("");
}
function deck() {
  const z = TOP[1];
  const out = [];
  // UNO R4 (68.6 × 53.4) on spacers, Elegoo shield on top
  const board = { x: -63, y: -26.7, w: 68.6, d: 53.4, r: 2 };
  for (const [x, y] of byDepth(k.corners(board, 4), ([x, y]) => [x, y, 0])) out.push(S(k.cylinder(x, y, 2, z, 6, P, 16), { tone: "mid" }));
  out.push(S(k.slabOf(board, z + 6, 1.6, P, 4), { tone: "mid" }));
  out.push(S(k.slabOf({ x: -63, y: -4.5, w: 7, d: 9, r: 1 }, z + 7.6, 3.2, P, 2), { tone: "hi" })); // USB-C
  const hdr = (y) => ({ x: -50, y, w: 52, d: 3, r: 0.6 });
  for (const y of byDepth([-25, 22], (y) => [0, y, 0])) out.push(S(k.slabOf(hdr(y), z + 7.6, 8.5, P, 2), { tone: "lo" }));
  const shield = { x: -60, y: -26.7, w: 64, d: 53.4, r: 2 };
  out.push(S(k.slabOf(shield, z + 16.1, 1.6, P, 4), { tone: "mid" }));
  const top = z + 17.7;
  out.push(L(k.planOutline(k.insetPlan(shield, 3), top, P, 3), { tone: "faint" }));
  // TB6612 motor driver chip
  const chip = { x: -22, y: -8, w: 10, d: 8, r: 0.6 };
  out.push(S(k.slabOf(chip, top, 1.4, P, 2), { tone: "lo", crease: "none" }));
  out.push(D([0, 1, 2, 3].map((i) => at([chip.x + 1.5 + i * 2.3, chip.y - 0.8, top])), { size: 0.3, tone: "mid" }));
  // motor screw terminals (left pair / right pair)
  for (const y of byDepth([-20, 14], (y) => [0, y, 0])) {
    const t = { x: -54, y, w: 10, d: 6, r: 0.8 };
    out.push(S(k.slabOf(t, top, 5, P, 2, 0.3), { tone: "mid" }));
    out.push(D([at([t.x + 3, t.y + 3, top + 5]), at([t.x + 7, t.y + 3, top + 5])], { size: 0.45, tone: "lo" }));
  }
  out.push(D(k.dotGrid(-40, 20, 8, 1, 4, top, P).map ? k.dotGrid(-40, 20, 8, 1, 4, top, P) : [], { size: 0.3, tone: "lo" }));
  // twist dial inlay: v needle + ω arc are live
  out.push(S(k.cylinder(DIAL[0], DIAL[1], 26, z, 1, P, 48), { tone: "lo", crease: "none" }));
  const ticks = k.radialTicks(DIAL[0], DIAL[1], 25, 24, 6, z + 1, [1.6, 3.2], P);
  out.push(L(ticks.minor, { tone: "faint" }), L(ticks.major, { tone: "mid" }));
  out.push(`<path data-part="dial-w" class="iso-line" data-tone="hi" d="" style="stroke-width:1.4"/>`);
  out.push(`<path data-part="dial-v" class="iso-line" data-tone="hi" d="" style="stroke-width:1.4"/>`);
  out.push(`<g data-part="dial-ends"></g>`);
  out.push(D([at(DIAL)], { size: 0.9, tone: "hi" }));
  return out.join("");
}
function sensor() {
  const z = TOP[1];
  const out = [];
  out.push(S(k.slabOf({ x: 84, y: -6, w: 23, d: 12, r: 1 }, z, 22, P, 3, 0.4), { tone: "mid" })); // pan servo
  out.push(S(k.cylinder(95, 0, 3, z + 22, 2, P, 16), { tone: "mid" }));
  out.push(S(k.slabOf({ x: 98, y: -22.5, w: 2, d: 45, r: 0.4 }, z + 24, 20, P, 2), { tone: "mid" })); // HC-SR04 board
  for (const y of byDepth([-12, 12], (y) => [0, y, 0])) {
    const F = G.frameAlong([100, y, z + 34], [1, 0, 0]);
    out.push(S(G.disc(0, 11, 8, F, P, { steps: 32 }), { tone: "hi" }));
    out.push(L(G.circleOf(F, 11, 5, P, 28), { tone: "lo" }));
  }
  return out.join("");
}
function chassis() {
  const out = [];
  out.push(`<path class="iso-halo" d="${k.haloOf(k.insetPlan({ x: -100, y: -95, w: 200, d: 190, r: 20 }, 0), 0, 0.1, P)}" style="filter:blur(6px)"/>`);
  out.push(plateOf(LOW, "lo"));
  const wheels = [WHEEL_X, -WHEEL_X].flatMap((x) => [1, -1].map((side) => ({ x, side })));
  const far = byDepth(wheels.filter((w) => w.side !== NEAR), (w) => [w.x, w.side * WHEEL_Y, 0]);
  const near = byDepth(wheels.filter((w) => w.side === NEAR), (w) => [w.x, w.side * WHEEL_Y, 0]);
  for (const w of far) out.push(wheel(w.x, w.side), axle(w.x, w.side));
  for (const w of byDepth(wheels, (w) => [w.x, w.side * 55, 0])) out.push(motor(w.x, w.side));
  for (const [x, y] of byDepth(k.corners(PLATE, 12), ([x, y]) => [x, y, 0])) out.push(S(k.cylinder(x, y, 2.8, LOW[1], TOP[0] - LOW[1], P, 16), { tone: "hi", crease: "none" }));
  out.push(plateOf(TOP, "mid"));
  out.push(L(k.planOutline(k.insetPlan(PLATE, 4), TOP[1], P), { tone: "faint" }));
  out.push(D(k.corners(PLATE, 12).map(([x, y]) => at([x, y, TOP[1]])), { size: 0.5 }));
  out.push(deck(), sensor());
  for (const w of near) out.push(axle(w.x, w.side), wheel(w.x, w.side));
  return out.join("");
}

// ---- serial terminal ----------------------------------------------------------
function terminal() {
  const out = [`<path class="iso-halo" d="${k.haloOf(TERM, 0, 0.1, P)}" style="filter:blur(4px)"/>`];
  out.push(S(k.slabOf({ x: TERM.x + 32, y: TERM.y - 4, w: 10, d: 4, r: 1 }, 6, 6, P, 2), { tone: "hi" })); // USB port (far face)
  out.push(S(k.slabOf(TERM, 0, TERM_H, P, 6, 0.8), { tone: "mid" }));
  const screen = k.insetPlan(TERM, 6);
  out.push(S(k.slabOf(screen, TERM_H, 0.5, P, 3), { tone: "lo", crease: "none" }));
  out.push(k.faceTextSvg(k.topMatrix([screen.x + 4, screen.y + screen.d / 2 + 4], TERM_H + 0.5, P, "x"), "M 177 177", { size: 10, tone: "hi" }).replace("<text ", '<text data-part="term" '));
  out.push(k.faceTextSvg(k.topMatrix([screen.x + 4, screen.y + 9], TERM_H + 0.5, P, "x"), "→ R4 · 20 Hz", { size: 5, tone: "lo" }));
  out.push(D(k.corners(TERM, 3).map(([x, y]) => at([x, y, TERM_H])), { size: 0.45 }));
  return out.join("");
}
// cable: along the floor to the back of the chassis, up its back edge (drawn
// before the chassis), then over the deck to the R4's USB-C (drawn after)
const cableLow = () =>
  k.wireSvg(at([TERM.x + 37, TERM.y - 4, 9]), at([-115.4, 0, TOP[1]]), {
    via: [[TERM.x + 37, TERM.y - 22, 1], [-130, 60, 1], [-116, 20, 1], [-116, 0, 1], [-116, 0, TOP[1] - 1]].map(at),
    tone: "mid", size: 1.4, ends: "from",
  });
const cableHigh = () =>
  k.wireSvg(at([-115.4, 0, TOP[1] + 0.5]), at(USB), { via: [[-80, 0, TOP[1] + 0.5]].map(at), tone: "mid", size: 1.4, ends: "to" });

// ---- arc + radius (live) ---------------------------------------------------
const floorLive = `<path data-part="radius" class="iso-line" data-tone="mid" d="" style="opacity:0"/><path data-part="arc" class="iso-line" data-tone="lit" d="" style="stroke-width:1.6"/><g data-part="floor-ends"></g>`;

const nearRuler = NEAR > 0 ? "left" : "right";
const farRuler = NEAR > 0 ? "right" : "left";
const svg = k.figureSvg({
  width: WIDTH,
  height: HEIGHT,
  label:
    "Scout, an Elegoo Smart Robot Car V4, on a test mat seen in its own frame. A lit pin marks the centre of rotation on the axle line, a lit arc the path the robot drives. Two rulers beside the car carry a slider each for the left and right side speed. A small terminal on the mat shows the M left right line sent to the UNO R4.",
  body: [
    mat(),
    floorLive,
    icr("far"),
    ruler(-NEAR, farRuler),
    terminal(),
    cableLow(),
    chassis(),
    cableHigh(),
    ruler(NEAR, nearRuler),
    icr("near"),
  ],
});

const panel = `
<style>
.tw-steps{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin:18px 0 0;font:12px/1.5 var(--anatomy-mono,ui-monospace,monospace);color:var(--anatomy-mid)}
.tw-step{border:1px solid var(--anatomy-faint);padding:8px 10px}
.tw-step b{display:block;color:var(--anatomy-hi);font-weight:600;margin-bottom:2px}
.tw-step .f{color:var(--anatomy-lo)}
.tw-step [data-v]{color:var(--anatomy-hi)}
.tw-out{display:flex;gap:12px;align-items:center;justify-content:space-between;margin-top:10px;font:12px/1.5 var(--anatomy-mono,ui-monospace,monospace);color:var(--anatomy-lo)}
.tw-out strong{font-size:15px;color:var(--anatomy-lit,var(--anatomy-hi));border:1px solid var(--anatomy-mid);padding:3px 10px;letter-spacing:.04em}
@media (max-width:560px){.tw-steps{grid-template-columns:1fr}.tw-out{flex-direction:column;align-items:flex-start}}
</style>
<div class="tw-steps" aria-live="off">
  <div class="tw-step"><b>1 · Kinematics</b><span class="f">v_L = v − ω·W/2<br>v_R = v + ω·W/2</span><br><span data-v="k">v_L 0.30 · v_R 0.30 m/s</span></div>
  <div class="tw-step"><b>2 · Saturation</b><span class="f">s = max(|v_L|,|v_R|)/v_max<br>if s &gt; 1: both ÷ s (same curve)</span><br><span data-v="s">s ≤ 1 · unchanged</span></div>
  <div class="tw-step"><b>3 · Deadband → PWM</b><span class="f">|v| &lt; 0.005 → 0, else sign·round(pwm_min<br>+ (255−pwm_min)·|v|/v_max), ≤ 255</span><br><span data-v="p">L 177 · R 177</span></div>
</div>
<div class="tw-out"><span>USB serial → UNO R4 → TB6612 (left pair / right pair) · re-sent at 20 Hz</span><strong data-v="m">M 177 177</strong></div>
<div class="tw-out"><span>Example values, not measured: W = ${EXAMPLE.W} m · v_max = ${EXAMPLE.vmax} m/s · pwm_min = ${EXAMPLE.pwmMin}</span></div>`;

const model = readFileSync(join(HERE, "model.mjs"), "utf8").replace(/^export /gm, "").replace(/^\/\/.*$/gm, "");

const live = `
${model}
const P = ${JSON.stringify(P)};
const MAT = ${JSON.stringify(MAT)}, RULER_Y = ${RULER_Y}, RULER_Z = ${RULER.h}, RULER_D = ${RULER.d}, PER_V = ${PER_V}, DIAL = ${JSON.stringify(DIAL)}, NEAR = ${NEAR};
const stage = document.querySelector(".iso-stage");
const svg = stage.querySelector("svg");
const $ = (n) => svg.querySelector('[data-part="' + n + '"]');
const parts = { arc: $("arc"), radius: $("radius"), ends: $("floor-ends"), far: $("icr-far"), near: $("icr-near"), term: $("term"),
  dv: $("dial-v"), dw: $("dial-w"), dends: $("dial-ends") };
const side = (n) => ({ bar: $(n + "-bar"), raw: $(n + "-raw"), slider: $(n + "-slider") });
const left = side("left"), right = side("right");
const readout = document.querySelector("[data-readout]");
const out = (n) => document.querySelector('[data-v="' + n + '"]');
const still = matchMedia("(prefers-reduced-motion: reduce)");
const f2 = (x) => (Math.abs(x) < 0.005 ? 0 : x).toFixed(2);
const dot = (p, cls = "iso-wire-end", r = 1.3) => { const [x, y] = iso(p, P); return '<circle class="' + cls + '" cx="' + x.toFixed(1) + '" cy="' + y.toFixed(1) + '" r="' + r + '"/>'; };
const inMat = (x, y) => x > MAT.x + 8 && x < MAT.x + MAT.w - 8 && y > MAT.y + 8 && y < MAT.y + MAT.d - 8;
const TOUR = [[0.30, 0], [0.30, 1.5], [0.45, 2.0], [0, 3.0], [0.25, -2.2], [0.004, 0]];
let cur = { v: 0.30, w: 0 }, goal = { v: 0.30, w: 0 }, clock = 0, last = 0, frame = 0, touring = true, visible = false, idle = 0;
function setSide(s, raw, val, y) {
  const x = PER_V * val / EXAMPLE.vmax;
  s.slider.setAttribute("transform", translateAlong(P, [x, 0, 0]));
  const y0 = y - RULER_D / 2;
  s.bar.setAttribute("d", Math.abs(x) > 0.5 ? onTop([[0, y], [x, y]], RULER_Z, P) : "");
  const rx = Math.max(-170, Math.min(170, PER_V * raw / EXAMPLE.vmax));
  const scaled = Math.abs(raw - val) > 0.002;
  s.raw.setAttribute("d", onTop([[rx, y0], [rx, y0 + RULER_D]], RULER_Z, P));
  s.raw.style.opacity = scaled ? 1 : 0;
}
function draw() {
  const { v, w } = cur;
  const r = twistToPwm(v, w);
  setSide(left, r.rawL, r.vL, RULER_Y);
  setSide(right, r.rawR, r.vR, -RULER_Y);
  // arc: path of the centre over the next 1.2 s, clipped to the mat
  const pts = [];
  const N = 60, T = 1.2;
  for (let i = 0; i <= N; i++) {
    const t = (i / N) * T, th = w * t;
    const x = Math.abs(w) < 1e-4 ? v * 1000 * t : (v / w) * 1000 * Math.sin(th);
    const y = Math.abs(w) < 1e-4 ? 0 : (v / w) * 1000 * (1 - Math.cos(th));
    if (!inMat(x, y)) break;
    pts.push([x, y]);
  }
  const ends = [];
  const moving = Math.abs(v) > 0.01 && pts.length > 2;
  parts.arc.setAttribute("d", moving ? onTop(pts, 0, P) : "");
  if (moving) ends.push(dot([...pts[pts.length - 1], 0]));
  // ICR + radius line on the axle line
  const R = r.R * 1000;
  const show = isFinite(R) && Math.abs(R) > 100 && inMat(0, R);
  const sgn = R >= 0 ? 1 : -1;
  for (const [g, slot] of [[parts.far, -NEAR], [parts.near, NEAR]]) {
    const on = show && sgn === slot;
    g.style.opacity = on ? 1 : 0;
    if (on) g.setAttribute("transform", translateAlong(P, [0, R, 0]));
  }
  if (show) {
    const y1 = sgn * ${WHEEL_Y + WHEEL_W / 2};
    parts.radius.setAttribute("d", onTop([[0, y1], [0, R]], 0, P));
    parts.radius.style.opacity = 1;
    ends.push(dot([0, y1, 0], "iso-wire-end", 1));
  } else parts.radius.style.opacity = 0;
  parts.ends.innerHTML = ends.join("");
  // twist dial on the deck: v needle along x, ω arc (counter-clockwise = left)
  const vx = Math.max(-22, Math.min(22, 22 * v / EXAMPLE.vmax));
  parts.dv.setAttribute("d", Math.abs(vx) > 0.6 ? onTop([[DIAL[0], 0], [DIAL[0] + vx, 0]], DIAL[2], P) : "");
  const sweep = Math.max(-1, Math.min(1, w / 3)) * Math.PI * 0.9;
  const arc = [];
  for (let i = 0; i <= 24; i++) { const a = (i / 24) * sweep; arc.push([DIAL[0] + 17 * Math.cos(a), 17 * Math.sin(a)]); }
  parts.dw.setAttribute("d", Math.abs(sweep) > 0.05 ? onTop(arc, DIAL[2], P) : "");
  const de = [];
  if (Math.abs(vx) > 0.6) de.push(dot([DIAL[0] + vx, 0, DIAL[2]], "iso-wire-end", 0.9));
  if (Math.abs(sweep) > 0.05) { de.push(dot([...arc[arc.length - 1], DIAL[2]], "iso-wire-end", 0.9)); de.push(dot([...arc[0], DIAL[2]], "iso-wire-end", 0.7)); }
  parts.dends.innerHTML = de.join("");
  // text
  if (parts.term.textContent !== r.line) parts.term.textContent = r.line;
  out("k").textContent = "v_L " + f2(r.rawL) + " · v_R " + f2(r.rawR) + " m/s";
  out("s").textContent = r.s > 1 ? "s " + r.s.toFixed(2) + " → L " + f2(r.vL) + " · R " + f2(r.vR) : "s ≤ 1 · unchanged";
  out("p").textContent = (r.l === 0 && r.r === 0 && Math.abs(v) > 0 ? "deadband · " : "") + "L " + r.l + " · R " + r.r;
  out("m").textContent = r.line;
  const text = "v " + f2(v) + " m/s · ω " + f2(w) + " rad/s → " + r.line;
  if (readout.textContent !== text) readout.textContent = text;
  stage.setAttribute("aria-valuetext", text);
}
function tick(now) {
  const dt = last ? Math.min((now - last) / 1000, 1 / 30) : 1 / 60;
  last = now;
  if (touring) { clock += dt; const g = TOUR[Math.floor(clock / 2.6) % TOUR.length]; goal = { v: g[0], w: g[1] }; }
  const a = still.matches ? 1 : 1 - Math.exp(-dt / 0.35);
  cur.v += (goal.v - cur.v) * a; cur.w += (goal.w - cur.w) * a;
  if (Math.abs(goal.v - cur.v) < 1e-4) cur.v = goal.v;
  if (Math.abs(goal.w - cur.w) < 1e-4) cur.w = goal.w;
  draw();
  const settled = !touring && cur.v === goal.v && cur.w === goal.w;
  frame = visible && !settled ? requestAnimationFrame(tick) : 0;
}
const run = () => { if (!frame && visible) { last = 0; frame = requestAnimationFrame(tick); } };
const steer = (v, w) => { touring = false; goal = { v: Math.max(-0.5, Math.min(0.5, v)), w: Math.max(-3, Math.min(3, w)) }; clearTimeout(idle); idle = setTimeout(() => { touring = !still.matches; clock = 0; run(); }, 4000); run(); };
new IntersectionObserver((e) => { visible = e[e.length - 1].isIntersecting; if (visible) run(); }, { rootMargin: "120px 0px" }).observe(stage);
if (still.matches) touring = false;
stage.addEventListener("pointermove", (e) => {
  const b = svg.getBoundingClientRect();
  const sx = Math.max(0, Math.min(1, (e.clientX - b.left) / b.width));
  const sy = Math.max(0, Math.min(1, (e.clientY - b.top) / b.height));
  steer(Math.round((0.5 - sy) * 2 * 0.5 * 100) / 100, Math.round((0.5 - sx) * 2 * 3 * 10) / 10);
});
stage.addEventListener("keydown", (e) => {
  const m = { ArrowUp: [0.05, 0], ArrowDown: [-0.05, 0], ArrowLeft: [0, 0.25], ArrowRight: [0, -0.25] }[e.key];
  if (!m) return;
  e.preventDefault();
  steer(goal.v + m[0], goal.w + m[1]);
});
const hp = new URLSearchParams(location.hash.slice(1));
if (hp.has("v")) { touring = false; cur = { v: +hp.get("v"), w: +(hp.get("w") || 0) }; goal = { ...cur }; }
draw();
`;

const first = twistToPwm(0.3, 0);
const body = k.plateHtml({
  fig: "Fig 1",
  title: "From Twist to wheel PWM",
  hint: "Pointer: left/right sets ω, up/down sets v",
  readout: `v 0.30 m/s · ω 0.00 rad/s → ${first.line}`,
  keys: [
    { mark: "lit", label: "ICR pin and the arc the centre drives" },
    { mark: "raised", label: "Side-speed sliders, left (+y) and right (−y), ± v_max stops" },
  ],
  caption:
    "The Scout bridge (DD-SCOUT-BRIDGE §5) turns cmd_vel into one serial line. Kinematics splits (v, ω) into side speeds v_L = v − ω·W/2 and v_R = v + ω·W/2, so the ICR sits at v/ω on the axle line and a left turn (ω > 0) makes the right side faster. If a side exceeds v_max, both are divided by the same s, so the arc keeps its curvature (a faint tick shows the raw value). Each side then maps to PWM with a deadband and goes out as M <left> <right>, PWM −255…255, to the UNO R4 and the shield's TB6612. Safety: the line is re-sent at 20 Hz; no cmd_vel for 0.25 s → S; the R4 stops the motors if no M arrives for 300 ms. W, v_max and pwm_min are example values until Lab 1 measures them.",
  body: `<div class="iso-stage" tabindex="0" role="application" aria-label="Twist to PWM: arrow keys change v and ω">${svg}</div>${panel}`,
});

const file = theme === "light" ? "twist-to-pwm-light.html" : "twist-to-pwm.html";
writeFileSync(join(HERE, file), k.pageHtml({ title: "From Twist to wheel PWM", theme, body, script: kitScript() + live, width: 760 }));
console.log("wrote", file, "near side y sign", NEAR);
