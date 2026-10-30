// Pure model: heading error of wheel odometry, gyro integration and their
// inverse-variance (scalar Kalman) fusion for Scout, a small differential-drive
// robot (TT motors, 65 mm wheels, MPU-6050). No DOM, no drawing: build.mjs
// imports it for the rest pose and inlines it into the page for the live loop.
// (Revised from the first draft's closed-form stateAt(t, slip): same fusion
// rule, but stepped in time so slip can be injected while the robot drives.)
//
// Wheels:  heading = (dR - dL) / track. On a carpet tile the right wheel spins:
//          its encoder counts travel the robot never made, so the heading rate
//          the encoders report is too high by  slip * v / track  while it lasts.
//          The error stays afterwards: odometry alone never sees it.
// Gyro:    heading = integral of yaw rate. A residual bias b after start-up
//          calibration makes the error grow without limit: error = b * t.
// Fusion:  the scalar Kalman update of two independent heading estimates with
//          variances Pg (gyro) and Pw (wheels):
//            K = Pg / (Pg + Pw),   fused = gyro + K * (wheels - gyro)
//          i.e. each is weighted by 1 / its variance, and the fused variance
//          Pg*Pw/(Pg+Pw) is smaller than either. Pg grows with t^2 (bias);
//          Pw starts at the encoders' count quantum and grows with angle turned, plus the square of every rate
//          disagreement that fails the gate (the slip the filter can see, as
//          robot_localization's rejection threshold does).
//          Short-term the fused heading follows the gyro; long-term the gyro's
//          variance outgrows the wheels' and the fused heading stays bounded.

const DEG = Math.PI / 180;

export const SCOUT = {
  track: 0.135, // m, wheel centre to wheel centre
  wheelR: 0.0325, // m, 65 mm wheels
  speed: 0.2, // m/s forward
  turnRate: 0.1, // rad/s: Scout loops the room, always turning left (5.7 deg/s)
  bias: -0.08, // deg/s residual MPU-6050 gyro bias after warm-up and calibration
  arw: 0.005, // deg/sqrt(s) angle random walk
  wheelSigma: 0.02, // rad of heading error per rad turned (track-width calibration)
  counts: 40, // encoder counts per wheel turn (20-slot disc, both edges)
  gate: 0.5, // deg/s: wheel/gyro rate disagreement the filter accepts as noise
  slip: 0.04, // fraction of the right wheel's travel lost on a carpet tile
  run: 90, // s per simulated run
};

export function start() {
  return { t: 0, turned: 0, wheels: 0, gyro: 0, seen: 0, slipping: 0, left: 0, right: 0 };
}

// Advance the state by dt seconds; slip is 0..1 of SCOUT.slip (pointer/key held).
export function step(state, dt, slip, s = SCOUT) {
  const v = s.speed;
  const w = s.turnRate;
  const slipRate = (slip * s.slip * v) / s.track / DEG; // deg/s of false turn
  const wheelRate = w / DEG + slipRate; // what the encoders report
  const gyroRate = w / DEG + s.bias; // what the MPU-6050 reports
  const disagree = Math.abs(wheelRate - gyroRate);
  const t = state.t + dt;
  return {
    t,
    turned: state.turned + w * dt, // rad
    wheels: state.wheels + slipRate * dt, // deg error
    gyro: s.bias * t, // deg error
    seen: state.seen + Math.max(0, disagree - s.gate) * dt, // deg the gate rejected
    slipping: slip,
    left: state.left + ((v - (w * s.track) / 2) * dt) / s.wheelR, // rad of wheel rotation
    right: state.right + ((v + (w * s.track) / 2 + slip * s.slip * v) * dt) / s.wheelR,
  };
}

// What the figure shows for a state (all angles in degrees).
export function readout(state, s = SCOUT) {
  const t = state.t;
  const pg = (s.bias * t) ** 2 + s.arw ** 2 * t + 1e-6;
  const quantum = (2 * Math.PI * s.wheelR) / s.counts / s.track / DEG; // deg per count of dR - dL
  const pw = quantum ** 2 / 12 + (s.wheelSigma / DEG) ** 2 * state.turned * DEG + state.seen ** 2;
  const gain = pg / (pg + pw);
  const fused = state.gyro + gain * (state.wheels - state.gyro);
  return {
    t,
    wheels: state.wheels,
    gyro: state.gyro,
    fused,
    gain,
    sigmaG: Math.sqrt(pg),
    sigmaW: Math.sqrt(pw),
    sigmaF: Math.sqrt((pg * pw) / (pg + pw)),
  };
}

// The idle tour's carpet tiles: [from, to] seconds of a run with slip held.
export const TILES = [
  [20, 21.5],
  [50, 51.5],
];
export const tourSlip = (t) => (TILES.some(([a, b]) => t >= a && t < b) ? 1 : 0);

// Run a whole tour run and sample it (used by build.mjs to check ranges).
export function sweep(dt = 0.05, s = SCOUT) {
  let st = start();
  const out = [];
  while (st.t < s.run) {
    st = step(st, dt, tourSlip(st.t), s);
    out.push(readout(st, s));
  }
  return out;
}
