// Fig 1 · Fusing heading: a summing-beam "fusion bench" for Scout.
// node build.mjs [--light]  ->  sensor-fusion.html / sensor-fusion-light.html
//
// Concept. The scalar Kalman update  fused = gyro + K·(wheels − gyro),
// K = Pg/(Pg + Pw), is a point a fraction K of the way along a straight bar
// from one estimate to the other. So: two carriages ride two rails, each set to
// one source's heading error (back: wheel encoders, front: MPU-6050 gyro). A
// telescoping beam joins their pins. A cross-rail rides two tracks and holds the
// lit fused carriage at the fraction K along the beam, its pin in the beam's
// slot, so the fused carriage reads the inverse-variance average on its rail.
// Holding the slip button drives Scout over a carpet tile: the right encoder
// disc over-spins, the wheels carriage runs away, the filter sees the
// disagreement, swings K toward the gyro and the fused carriage barely moves.
//
// Parts list (written before geometry), 72 parts:
//  Base (19): plate, 4 foot flanges, 4 feet, perimeter groove, 4 corner screws
//    (rings + dots), back ruler strip (wheels), front ruler strip (gyro),
//    label plate (engraved "EKF", 3 rule lines, 2 screws), halo
//  Structure (16): back rail, front rail, 4 rail end stops, 2 cross tracks,
//    4 track end stops, cross-rail with tick row, 2 cross-rail riders, rider caps
//  Wheels carriage (12): body, top plate, 4 top screws, pointer tongue + index,
//    left & right encoder discs with slots, 2 hubs, 2 optical forks (2 jaws each)
//  Gyro carriage (11): body, top plate, MPU-6050 board, chip, 8-pin header,
//    2 mounting holes, crystal, pointer tongue + index
//  Fused carriage (6, lit): saddle, 2 side gibs, pointer tongue + index, pin
//  Beam (8): outer sleeve, inner slotted bar, slot, sleeve collar,
//    3 pins with collars and caps (wheel pivot, fused slider, gyro swivel)
//  Controls (4): slip button housing, knurled ring, plunger, carpet-tile inlay

import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import * as k from "./kit/iso-kit.mjs";
import { kitScript } from "./scripts/inline-kit.mjs";
import * as M from "./model.mjs";

const HERE = dirname(fileURLToPath(import.meta.url));
const theme = process.argv.includes("--light") ? "light" : "dark";

const WIDTH = 600;
const HEIGHT = 330;

// ---- world plan (1 unit ≈ 1 px of the bench) ---------------------------------
const PLATE = { x: -164, y: -64, w: 328, d: 128, r: 9 };
const PLATE_H = 12;
const FOOT_H = 5;
const PER_DEG = 9; // world units per degree of heading error
const RANGE = 12; // ± degrees on the rulers
const TRAVEL = RANGE * PER_DEG; // 108
const YC = 30; // rail centre lines: back (wheels) −30, front (gyro) +30
const PIN = 34; // pin lines: wheels pin y −34, gyro pin y +34
const CAR = { x: -17, y: -10, w: 34, d: 20, r: 2.5 }; // fixed carriages, local
const CAR_Z = 3;
const CAR_TOP = 12;
const TRACK_X = 146; // cross tracks
const XR_Z = 14; // cross-rail bottom
const XR_TOP = 18;
const FUSED = { x: -9, y: -5, w: 18, d: 10, r: 2 }; // fused saddle, local
const FUSED_TOP = 22;
const BEAM_Z = 30;
const BEAM_TOP = 33;
const K_MIN = 7 / (2 * PIN); // fused pin reaches within 7 of either pin
const K_MAX = 1 - K_MIN;
const BUTTON = [129.5, 50.5];

const P = k.fitProjection(
  [...k.boxCorners(PLATE, -PLATE_H - FOOT_H, 0), [0, -PIN, BEAM_TOP + 2], [TRAVEL, -PIN, BEAM_TOP + 2], [-TRAVEL, PIN, BEAM_TOP]],
  WIDTH,
  HEIGHT,
  { pad: 22, azimuth: 60, elevation: 34 },
);

const at = (point) => k.iso(point, P);
const S = (paths, style) => k.solidSvg(paths, style);
const L = (d, style) => k.lineSvg(d, style);
const D = (points, style) => k.dotsSvg(points, style);
const byDepth = (points) => [...points].sort((a, b) => k.depthOf([a[0], a[1], 0], P) - k.depthOf([b[0], b[1], 0], P));
let parts = 0;
const count = (n = 1) => (parts += n);

// ---- base ------------------------------------------------------------------
function base() {
  const out = [`<path class="iso-halo" d="${k.haloOf(PLATE, -PLATE_H - FOOT_H, PLATE_H, P)}"/>`];
  for (const [x, y] of byDepth(k.corners(PLATE, 24))) {
    out.push(S(k.cylinder(x, y, 7.5, -PLATE_H - FOOT_H, 1.6, P, 28), { tone: "lo" }));
    out.push(S(k.cylinder(x, y, 5.8, -PLATE_H - FOOT_H + 1.6, FOOT_H - 1.6, P, 28), { tone: "mid" }));
  }
  count(8);
  out.push(S(k.slabOf(PLATE, -PLATE_H, PLATE_H, P, 8, 1.6), { tone: "mid" }));
  out.push(L(k.planOutline(k.insetPlan(PLATE, 5), 0, P), { tone: "lo" }));
  const screws = k.corners(PLATE, 11);
  out.push(L(screws.map(([x, y]) => k.ring(x, y, 2.2, 0, P, 16)).join(""), { tone: "lo" }));
  out.push(D(screws.map(([x, y]) => at([x, y, 0])), { size: 0.5 }));
  count(1 + 1 + 4);
  return out.join("");
}

// A ruler strip whose ticks hang from the edge facing its carriage.
function ruler(y, towardPlus) {
  const strip = { x: -116, y, w: 232, d: 7, r: 1 };
  const out = [S(k.slabOf(strip, 0, 0.6, P, 3), { tone: "lo", crease: "none" })];
  const edge = towardPlus ? y + 0.01 : y + strip.d - 0.01;
  const sign = towardPlus ? 1 : -1;
  const t = k.topTicks(-TRAVEL, TRAVEL, PER_DEG, 4, edge, 0.6, [2 * sign, 3.8 * sign], P);
  out.push(L(t.minor, { tone: "lo" }), L(t.major, { tone: "mid" }));
  out.push(L(k.lineOnTop([0, edge], [0, edge + 5.4 * sign], 0.6, P), { tone: "hi" }));
  out.push(D([at([strip.x + 2.4, y + 3.5, 0.6]), at([strip.x + strip.w - 2.4, y + 3.5, 0.6])], { size: 0.45 }));
  count();
  return out.join("");
}

function labelPlate() {
  const plan = { x: -152, y: 44, w: 30, d: 13, r: 1.5 };
  const out = [S(k.slabOf(plan, 0, 0.8, P, 4), { tone: "lo" })];
  out.push(L(k.planOutline(k.insetPlan(plan, 1.6), 0.8, P), { tone: "faint" }));
  out.push(k.faceTextSvg(k.topMatrix([plan.x + 4, plan.y + 4.4], 0.8, P, "x"), "EKF", { size: 3.6, tone: "mid" }));
  out.push(L([[plan.x + 4, plan.y + 5.6, 22], [plan.x + 4, plan.y + 8.4, 16]].map(([x, yy, len]) => k.lineOnTop([x, yy + 1.4], [x + len, yy + 1.4], 0.8, P)).join(""), { tone: "lo", free: true }));
  out.push(D([at([plan.x + 26.5, plan.y + 3, 0.8]), at([plan.x + 26.5, plan.y + plan.d - 3, 0.8])], { size: 0.45 }));
  count();
  return out.join("");
}

function carpetTile() {
  // a square of carpet under the bench's right corner: where slip happens
  const plan = { x: 118.5, y: 44, w: 22, d: 13, r: 1 };
  const out = [S(k.slabOf(plan, 0, 0.5, P, 3), { tone: "lo", crease: "none" })];
  out.push(D(k.dotGrid(plan.x + 1.6, plan.y + 1.6, 9, 5, 2.35, 0.5, P).filter(([x, y]) => Math.hypot(x - BUTTON[0], y - BUTTON[1]) > 7.4), { size: 0.35, tone: "lo" }));
  count();
  return out.join("");
}

// ---- structure ---------------------------------------------------------------
function fixedRail(yc) {
  const out = [S(k.slabOf({ x: -127, y: yc - 2.5, w: 254, d: 5, r: 1 }, 0, 3, P, 3, 0.4), { tone: "mid" })];
  out.push(D([-122, 122].map((x) => at([x, yc, 3])), { size: 0.4 }));
  for (const x of [-133, 127]) {
    out.push(S(k.slabOf({ x, y: yc - 8, w: 6, d: 16, r: 1.2 }, 0, 8, P, 3, 0.5), { tone: "mid" }));
    out.push(D([at([x + 3, yc - 4.5, 8]), at([x + 3, yc + 4.5, 8])], { size: 0.45 }));
  }
  count(3);
  return out.join("");
}

function crossTrack(xc) {
  const out = [S(k.slabOf({ x: xc - 3, y: -36, w: 6, d: 72, r: 1 }, 0, 3, P, 3, 0.4), { tone: "mid" })];
  out.push(D([-31, 0, 31].map((y) => at([xc, y, 3])), { size: 0.4 }));
  for (const y of [-42, 36]) {
    out.push(S(k.slabOf({ x: xc - 7, y, w: 14, d: 6, r: 1.2 }, 0, 7, P, 3, 0.5), { tone: "mid" }));
    out.push(D([at([xc - 4, y + 3, 7]), at([xc + 4, y + 3, 7])], { size: 0.45 }));
  }
  count(3);
  return out.join("");
}

// Rider on a cross track, built at y = 0.
function rider(xc) {
  const plan = { x: xc - 6, y: -6, w: 12, d: 12, r: 1.6 };
  const out = [S(k.slabOf(plan, 3, XR_TOP - 3, P, 4, 0.6), { tone: "mid" })];
  out.push(S(k.slabOf(k.insetPlan(plan, 2), XR_TOP, 1.2, P, 3, 0.3), { tone: "mid" }));
  out.push(D([at([xc - 2.4, -2.4, XR_TOP + 1.2]), at([xc + 2.4, 2.4, XR_TOP + 1.2])], { size: 0.45 }));
  count(2);
  return out.join("");
}

// Cross-rail between the riders, built at y = 0, with its own tick row.
function crossRail() {
  const plan = { x: TRACK_X - 6 - 2 * (TRACK_X - 6) - 0, y: -3, w: 2 * (TRACK_X - 6), d: 6, r: 0.8 };
  const out = [`<path class="iso-halo" d="${k.haloOf(k.insetPlan(plan, 1), 3, 0.1, P)}" style="filter:blur(5px);opacity:.5"/>`];
  out.push(S(k.slabOf(plan, XR_Z, XR_TOP - XR_Z, P, 3, 0.4), { tone: "mid" }));
  const t = k.topTicks(-TRAVEL, TRAVEL, PER_DEG, 4, 2.99, XR_TOP, [-1.3, -2.4], P);
  out.push(L(t.minor, { tone: "lo" }), L(t.major, { tone: "mid" }));
  out.push(L(k.lineOnTop([0, 2.99], [0, -0.4], XR_TOP, P), { tone: "hi" }));
  count();
  return out.join("");
}

// ---- carriages (built at x = 0 on their rail line) ---------------------------
function carriageBody(yc, outward) {
  const plan = { ...CAR, y: yc + CAR.y };
  const out = [`<path class="iso-halo" d="${k.haloOf(k.insetPlan(plan, 3), CAR_Z, 0.1, P)}" style="filter:blur(4px)"/>`];
  // pointer tongue over the ruler (outward side)
  const ty = outward < 0 ? plan.y - 6 : plan.y + plan.d;
  out.push(S(k.slabOf({ x: -1.8, y: ty, w: 3.6, d: 6, r: 1 }, CAR_Z, 1, P, 2), { tone: "mid", crease: "none" }));
  out.push(S(k.slabOf(plan, CAR_Z, 6, P, 5, 0.6), { tone: "mid" }));
  out.push(S(k.slabOf(k.insetPlan(plan, 1.4), CAR_Z + 6, CAR_TOP - CAR_Z - 6, P, 5, 0.5), { tone: "hi", crease: "lo" }));
  const tip = outward < 0 ? ty : ty + 6;
  const root = outward < 0 ? ty + 6 : ty;
  out.push(L(k.lineOnTop([0, root], [0, tip], CAR_Z + 1, P), { tone: "hi" }));
  const top = k.insetPlan(plan, 1.4);
  out.push(D(k.corners(top, 2.2).map(([x, y]) => at([x, y, CAR_TOP])), { size: 0.42 }));
  count(4);
  return out.join("");
}

const DISC_R = 6;
const DISCS = [-9.5, 9.5]; // left, right encoder at local x, y = −56 + 1
function encoderSlots(angle, side) {
  const [x, y] = [DISCS[side], -YC + 0.5];
  const t = k.radialTicks(x, y, DISC_R - 0.2, 20, 20, CAR_TOP + 1.4, [1.6, 2.6], P, angle);
  return t.minor + t.major;
}
function wheelsCarriage() {
  const out = [carriageBody(-YC, -1)];
  for (const side of [0, 1]) {
    const [x, y] = [DISCS[side], -YC + 0.5];
    out.push(S(k.cylinder(x, y, 1.6, CAR_TOP, 0.6, P, 16), { tone: "mid", crease: "none" }));
    out.push(S(k.cylinder(x, y, DISC_R, CAR_TOP + 0.6, 0.8, P, 40), { tone: "hi", crease: "lo" }));
    out.push(L(encoderSlots(0, side), { tone: "lo" }).replace("<path", `<path data-part="slots${side}"`));
    out.push(L(k.ring(x, y, 1.5, CAR_TOP + 1.4, P, 16), { tone: "mid" }));
    // optical fork straddling the disc rim, on the inner (toward-rail) side
    const fx = x + (side ? 1 : -1) * 0;
    out.push(S(k.slabOf({ x: fx - 2, y: y + DISC_R - 1.6, w: 4, d: 3.6, r: 0.6 }, CAR_TOP + 1.6, 1.4, P, 2), { tone: "mid" }));
    out.push(S(k.slabOf({ x: fx - 2, y: y + DISC_R + 0.4, w: 4, d: 1.6, r: 0.4 }, CAR_TOP, 1.6, P, 2), { tone: "mid" }));
    count(5);
  }
  return out.join("");
}

function gyroCarriage() {
  const out = [carriageBody(YC, 1)];
  const board = { x: -11, y: YC - 8.4, w: 22, d: 10, r: 0.8 };
  out.push(S(k.slabOf(board, CAR_TOP, 0.8, P, 3), { tone: "mid", crease: "none" }));
  out.push(L(k.ring(board.x + 2, board.y + 2, 0.9, CAR_TOP + 0.8, P, 12) + k.ring(board.x + board.w - 2, board.y + 2, 0.9, CAR_TOP + 0.8, P, 12), { tone: "lo" }));
  out.push(S(k.slabOf({ x: -2.6, y: YC - 6.6, w: 5.2, d: 5.2, r: 0.4 }, CAR_TOP + 0.8, 1, P, 2), { tone: "hi", crease: "lo" }));
  out.push(D([at([-1.6, YC - 5.6, CAR_TOP + 1.8])], { size: 0.35, tone: "lo" }));
  out.push(S(k.slabOf({ x: 4.8, y: YC - 6.2, w: 3.2, d: 1.8, r: 0.4 }, CAR_TOP + 0.8, 0.8, P, 2), { tone: "mid", crease: "none" }));
  const header = { x: -9, y: YC - 0.2, w: 18, d: 1.8, r: 0.3 };
  out.push(S(k.slabOf(header, CAR_TOP + 0.8, 1.4, P, 2), { tone: "lo", crease: "none" }));
  out.push(D(Array.from({ length: 8 }, (_, i) => at([-7.7 + i * 2.2, YC + 0.7, CAR_TOP + 2.2])), { size: 0.42, tone: "hi" }));
  count(7);
  return out.join("");
}

function fusedCarriage() {
  const out = [];
  for (const y of [-4.6, 3]) out.push(S(k.slabOf({ x: -7, y, w: 7 * 2, d: 1.6, r: 0.4 }, XR_Z + 1, XR_TOP - XR_Z - 1, P, 2), { tone: "mid", crease: "none" }));
  // tongue reaching to the tick row, lying on the rail's top
  out.push(S(k.slabOf({ x: -9, y: -5, w: 18, d: 7, r: 2 }, XR_TOP, FUSED_TOP - XR_TOP, P, 5, 0.6), { tone: "hi", crease: "lo", lit: true }));
  out.push(S(k.slabOf({ x: -1.5, y: 1.4, w: 3, d: 1.59, r: 0.5 }, XR_TOP, 0.6, P, 2), { tone: "hi", crease: "none", lit: true }));
  out.push(D(k.corners({ x: -9, y: -5, w: 18, d: 7 }, 2).map(([x, y]) => at([x, y, FUSED_TOP])), { size: 0.42 }));
  count(5);
  return out.join("");
}

// pin from z0 to BEAM_Z with a collar under the beam, at plan point (x, y)
function pinLower(x, y, z0, lit = false) {
  const out = [S(k.cylinder(x, y, 1.7, z0, BEAM_Z - 2.2 - z0, P, 20), { tone: "hi", crease: "none", lit })];
  out.push(S(k.cylinder(x, y, 3.4, BEAM_Z - 2.2, 2.2, P, 28, 0.3), { tone: lit ? "hi" : "mid", lit }));
  count();
  return out.join("");
}
function pinCap(x, y, lit = false) {
  const out = [S(k.cylinder(x, y, 2.9, BEAM_TOP, 1.6, P, 24, 0.3), { tone: lit ? "hi" : "mid", lit })];
  out.push(L(k.ring(x, y, 1.1, BEAM_TOP + 1.6, P, 12), { tone: "lo" }));
  return out.join("");
}

// The telescoping beam between the two pins, rebuilt every frame.
function stadium([ax, ay], [bx, by], r, n = 10) {
  const a = Math.atan2(by - ay, bx - ax);
  const ring = [];
  for (let i = 0; i <= n; i++) {
    const t = a + Math.PI / 2 + (i / n) * Math.PI;
    ring.push([ax + r * Math.cos(t), ay + r * Math.sin(t)]);
  }
  for (let i = 0; i <= n; i++) {
    const t = a - Math.PI / 2 + (i / n) * Math.PI;
    ring.push([bx + r * Math.cos(t), by + r * Math.sin(t)]);
  }
  return ring;
}
function beamSvg(xw, xg, P, k) {
  const W = [xw, -PIN];
  const G = [xg, PIN];
  const lerp = (t) => [W[0] + (G[0] - W[0]) * t, W[1] + (G[1] - W[1]) * t];
  const out = [];
  const inner = k.extrude(stadium(lerp(0.36), G, 2.6), BEAM_Z + 0.5, 2, P, { bevel: 0.3 });
  out.push(k.solidSvg(inner, { tone: "hi", crease: "lo" }));
  const slotA = lerp(0.12);
  const slotB = lerp(0.96);
  out.push(k.lineSvg(k.onTop(stadium(lerp(0.5), slotB, 1.1, 6), BEAM_Z + 2.5, P, true), { tone: "lo" }));
  const outer = k.extrude(stadium(W, lerp(0.5), 3.6), BEAM_Z, BEAM_TOP - BEAM_Z, P, { bevel: 0.4 });
  out.push(k.solidSvg(outer, { tone: "hi", crease: "lo" }));
  out.push(k.lineSvg(k.onTop(stadium(slotA, lerp(0.5), 1.1, 6), BEAM_TOP, P, true), { tone: "lo" }));
  return out.join("");
}
count(4); // beam: outer sleeve, inner bar, two slots

function slipButton() {
  const [x, y] = BUTTON;
  const out = [S(k.cylinder(x, y, 6.4, 0, 1.2, P, 32), { tone: "lo" })];
  out.push(S(k.cylinder(x, y, 5.2, 1.2, 4.6, P, 32, 0.4), { tone: "mid" }));
  out.push(L(k.knurl(x, y, 5.2, 1.6, 5.4, 36, P), { tone: "lo" }));
  out.push(L(k.ring(x, y, 3.9, 5.8, P, 28), { tone: "faint" }));
  count(3);
  return out.join("");
}
function plunger() {
  const [x, y] = BUTTON;
  count();
  return S(k.cylinder(x, y, 3.4, 5.8, 3, P, 28, 0.4), { tone: "hi", crease: "lo" });
}

// ---- rest pose from the model -----------------------------------------------
const REST = M.readout(M.start());
const posOf = (deg) => Math.max(-TRAVEL, Math.min(TRAVEL, deg * PER_DEG));

const g = (part, transform, inner) => `<g data-part="${part}"${transform ? ` transform="${transform}"` : ""}>${inner}</g>`;
const T0 = k.translateAlong(P, [0, 0, 0]);
const yOfK = (gain) => PIN - 2 * PIN * Math.min(K_MAX, Math.max(K_MIN, gain));
const restY = yOfK(REST.gain);

const body = [
  base(),
  crossTrack(-TRACK_X),
  g("riderL", k.translateAlong(P, [0, restY, 0]), rider(-TRACK_X)),
  ruler(-YC - 23, false),
  fixedRail(-YC),
  g("wheels", T0, wheelsCarriage()),
  g("wpin", T0, pinLower(0, -PIN, CAR_TOP)),
  fixedRail(YC),
  ruler(YC + 16, true),
  labelPlate(),
  carpetTile(),
  g("gyro", T0, gyroCarriage()),
  g("rail", k.translateAlong(P, [0, restY, 0]), crossRail()),
  g("fused", k.translateAlong(P, [0, restY, 0]), fusedCarriage() + pinLower(0, 0, FUSED_TOP, true)),
  g("gpin", T0, pinLower(0, PIN, CAR_TOP)),
  slipButton(),
  g("plunger", T0, plunger()),
  crossTrack(TRACK_X),
  g("riderR", k.translateAlong(P, [0, restY, 0]), rider(TRACK_X)),
  g("beam", "", beamSvg(0, 0, P, k)),
  g("wcap", T0, pinCap(0, -PIN)),
  g("fcap", k.translateAlong(P, [0, restY, 0]), pinCap(0, 0, true)),
  g("gcap", T0, pinCap(0, PIN)),
];
count(3); // three caps

const svg = k.figureSvg({
  width: WIDTH,
  height: HEIGHT,
  label:
    "A fusion bench. Two carriages on two rails each show one heading estimate's error: the back one carries two wheel-encoder discs, the front one an MPU-6050 gyro board. A telescoping beam joins their pins. A lit carriage on a cross-rail holds its pin in the beam's slot at a fraction K of the way from gyro to wheels, so it reads the inverse-variance weighted heading.",
  body,
});

// ---- live code ----------------------------------------------------------------
const modelSource = readFileSync(join(HERE, "model.mjs"), "utf8").replace(/^export /gm, "");
const live = `
const M = (() => { ${modelSource}
 return { SCOUT, start, step, readout, tourSlip }; })();
const P = ${JSON.stringify(P)};
const PIN = ${PIN}, BEAM_Z = ${BEAM_Z}, BEAM_TOP = ${BEAM_TOP}, PER_DEG = ${PER_DEG}, TRAVEL = ${TRAVEL};
const K_MIN = ${K_MIN}, K_MAX = ${K_MAX}, YC = ${YC}, DISC_R = ${DISC_R}, CAR_TOP = ${CAR_TOP};
const DISCS = ${JSON.stringify(DISCS)};
const k = { extrude, solidSvg, lineSvg, onTop };
${stadium.toString()}
${beamSvg.toString()}
${encoderSlots.toString().replace(/k\.radialTicks/g, "radialTicks")}
const stage = document.querySelector(".iso-stage");
const svg = stage.querySelector("svg");
const $ = (name) => svg.querySelector('[data-part="' + name + '"]');
const el = { wheels: $("wheels"), wpin: $("wpin"), wcap: $("wcap"), gyro: $("gyro"), gpin: $("gpin"), gcap: $("gcap"), rail: $("rail"), riderL: $("riderL"), riderR: $("riderR"), fused: $("fused"), fcap: $("fcap"), beam: $("beam"), plunger: $("plunger"), slots: [$("slots0"), $("slots1")] };
const readoutEl = document.querySelector("[data-readout]");
const still = matchMedia("(prefers-reduced-motion: reduce)");
const SPEED = 6; // simulated seconds per real second
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const posOf = (deg) => clamp(deg * PER_DEG, -TRAVEL, TRAVEL);
const sign = (v) => (v < -0.05 ? "\\u2212" : v > 0.05 ? "+" : "\\u00b1") + Math.abs(v).toFixed(1) + "\\u00b0";
let sim = M.start(), out = M.readout(sim);
const shown = { xw: 0, xg: 0, gain: out.gain, press: 0 };
let held = false, touring = !still.matches, idleTimer = 0, pause = 0, frame = 0, last = 0, visible = false;
function draw() {
  const xw = shown.xw, xg = shown.xg, gain = clamp(shown.gain, K_MIN, K_MAX);
  const yf = PIN - 2 * PIN * gain;
  const xf = xg + gain * (xw - xg);
  const T = (v) => translateAlong(P, v);
  el.wheels.setAttribute("transform", T([xw, 0, 0]));
  el.wpin.setAttribute("transform", T([xw, 0, 0]));
  el.wcap.setAttribute("transform", T([xw, 0, 0]));
  el.gyro.setAttribute("transform", T([xg, 0, 0]));
  el.gpin.setAttribute("transform", T([xg, 0, 0]));
  el.gcap.setAttribute("transform", T([xg, 0, 0]));
  el.rail.setAttribute("transform", T([0, yf, 0]));
  el.riderL.setAttribute("transform", T([0, yf, 0]));
  el.riderR.setAttribute("transform", T([0, yf, 0]));
  el.fused.setAttribute("transform", T([xf, yf, 0]));
  el.fcap.setAttribute("transform", T([xf, yf, 0]));
  el.plunger.setAttribute("transform", T([0, 0, -2.2 * shown.press]));
  el.beam.innerHTML = beamSvg(xw, xg, P, k);
  const turn = 1 / 18; // discs drawn geared down so the slots stay readable
  el.slots[0].setAttribute("d", encoderSlots(sim.left * turn, 0));
  el.slots[1].setAttribute("d", encoderSlots(sim.right * turn, 1));
  const text = "wheels " + sign(out.wheels) + " \\u00b7 gyro " + sign(out.gyro) + " \\u00b7 fused " + sign(out.fused);
  if (readoutEl.textContent !== text) readoutEl.textContent = text;
}
function describe() {
  stage.setAttribute("aria-valuetext", "Heading error after " + Math.round(out.t) + " s: wheels " + out.wheels.toFixed(1) + " degrees, gyro " + out.gyro.toFixed(1) + ", fused " + out.fused.toFixed(1) + "; wheel weight K " + out.gain.toFixed(2));
}
function advance(dt) {
  if (pause > 0) { pause -= dt; if (pause <= 0) { sim = M.start(); } return; }
  const slip = held ? 1 : touring ? M.tourSlip(sim.t) : 0;
  let left = dt * SPEED;
  while (left > 1e-9) { const h = Math.min(0.05, left); sim = M.step(sim, h, slip); left -= h; }
  out = M.readout(sim);
  if (sim.t >= M.SCOUT.run) { pause = 2.2; describe(); }
}
function tick(now) {
  const dt = last ? Math.min((now - last) / 1000, 1 / 30) : 1 / 60;
  last = now;
  advance(dt);
  const resetting = pause > 0;
  const tw = resetting ? 0 : posOf(out.wheels), tg = resetting ? 0 : posOf(out.gyro), tk = resetting ? 0.5 : out.gain;
  const slipNow = held || (touring && pause <= 0 && M.tourSlip(sim.t) > 0) ? 1 : 0;
  const ease = (from, to, tau) => from + (to - from) * (1 - Math.exp(-dt / tau));
  const tau = resetting ? 0.45 : 0.12;
  shown.xw = ease(shown.xw, tw, tau);
  shown.xg = ease(shown.xg, tg, tau);
  shown.gain = ease(shown.gain, tk, resetting ? 0.45 : 0.25);
  shown.press = ease(shown.press, slipNow, 0.08);
  draw();
  frame = visible ? requestAnimationFrame(tick) : 0;
}
const run = () => { if (!frame && visible && !still.matches) { last = 0; frame = requestAnimationFrame(tick); } };
function stillFrame() {
  // reduced motion: show the end of a run with the current slip history, no loop
  shown.xw = posOf(out.wheels); shown.xg = posOf(out.gyro); shown.gain = out.gain; shown.press = held ? 1 : 0;
  draw(); describe();
}
const hold = (on) => {
  held = on; touring = false; clearTimeout(idleTimer);
  if (!on) idleTimer = setTimeout(() => { touring = !still.matches; run(); }, 3600);
  if (on && pause > 0) { pause = 0; sim = M.start(); }
  describe();
  if (still.matches) {
    if (on) { for (let i = 0; i < 30; i++) sim = M.step(sim, 0.05, 1); out = M.readout(sim); }
    stillFrame();
  } else run();
};
new IntersectionObserver((entries) => { visible = entries[entries.length - 1].isIntersecting; if (visible) run(); }, { rootMargin: "120px 0px" }).observe(stage);
stage.addEventListener("pointerdown", (e) => { stage.setPointerCapture?.(e.pointerId); hold(true); });
stage.addEventListener("pointerup", () => hold(false));
stage.addEventListener("pointercancel", () => hold(false));
stage.addEventListener("keydown", (e) => {
  if (e.key === " " || e.key === "Enter") { e.preventDefault(); if (!e.repeat) hold(true); }
  if (e.key === "r" || e.key === "R" || e.key === "Home") { e.preventDefault(); sim = M.start(); out = M.readout(sim); pause = 0; describe(); if (still.matches) stillFrame(); }
});
stage.addEventListener("keyup", (e) => { if (e.key === " " || e.key === "Enter") hold(false); });
if (still.matches) {
  touring = false;
  while (sim.t < M.SCOUT.run) sim = M.step(sim, 0.05, M.tourSlip(sim.t));
  out = M.readout(sim);
  stillFrame();
} else { draw(); describe(); }
window.__fusion = { get state() { return { sim, out, shown }; }, hold };
`;

const plate = k.plateHtml({
  fig: "Fig 1",
  title: "Fusing heading",
  hint: "Hold for carpet slip",
  readout: "wheels \u00b10.0\u00b0 \u00b7 gyro \u00b10.0\u00b0 \u00b7 fused \u00b10.0\u00b0",
  keys: [
    { mark: "lit", label: "Fused carriage: the EKF's heading, on the beam at K = Pg / (Pg + Pw)" },
    { mark: "raised", label: "Back carriage: wheel encoders, (dR \u2212 dL) / track" },
    { mark: "raised", label: "Front carriage: MPU-6050 gyro, \u222b yaw rate" },
    { mark: "line", label: "Rulers: heading error, 1\u00b0 per tick, \u00b116\u00b0" },
  ],
  caption:
    "Scout drives a 90 s loop of the room, shown six times faster. Each carriage sits at its heading error. The wheel encoders' heading, (dR \u2212 dL) / track width, stays true until a wheel spins on a carpet tile; then it jumps and keeps the error. The gyro integrates yaw rate smoothly but its 0.08\u00b0/s residual bias drifts it steadily. The filter weights each by the inverse of its variance: the fused carriage sits a fraction K = Pg / (Pg + Pw) of the way from the gyro to the wheels. A slip makes the wheels disagree with the gyro beyond the gate, Pw jumps and K swings to the gyro, so the fused heading follows the gyro short-term; as the gyro's variance grows with its drift, K creeps back and the fused error stays bounded, within about 2\u00b0 here while the wheels are off by 10\u00b0 and the gyro by 7\u00b0. This is a 1-D picture of the robot_localization EKF's update.",
  body: `<style>@media (max-width:520px){.iso-plate{overflow:hidden}.iso-stage svg{width:122%;margin:0 -11%}}</style><div class="iso-stage" tabindex="0" role="slider" aria-label="Fusion bench: press and hold, or hold Space, to drive a wheel over carpet and make it slip; R restarts the run" aria-valuemin="-16" aria-valuemax="16" aria-valuenow="0" style="touch-action:pan-y;cursor:pointer">${svg}</div>`,
});

const name = theme === "light" ? "sensor-fusion-light.html" : "sensor-fusion.html";
writeFileSync(join(HERE, name), k.pageHtml({ title: "Fusing heading: wheel encoders + gyro", theme, body: plate, script: kitScript() + live }));
console.log("wrote", name, "· parts", parts);
