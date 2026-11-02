// Pure model of Scout v2's drive loop: /cmd_vel -> wheel speeds -> encoder counts,
// the PWM cap on the 3S pack, skid-steer wheel speeds and the /cmd_vel watchdog.
// No DOM. build.mjs imports it for the rest pose and inlines it into the page.
//
// Sources (docs/physics-round2.md, BRIEF.md):
//   wheel Ø65 mm; geometric track 124 mm; Hiwonder TT 1:42, 1008 counts per wheel
//   revolution claimed (x4 quadrature; one buyer measured 600): finding 24, section 5.
//   PWM duty capped at V_cap / V_batt with V_cap 8.4 V, pack 9.0-12.6 V: section 3.4.
//   Effective spin track 1.70x (carpet) to 1.93x (hard floor) geometric: finding 9, section 9.
//   Encoders read >= 50 Hz (20 ms samples) with PCNT: REQ-SCOUT-003, section 5.
//   Watchdog 300 ms; braking LOW/LOW stops in 0.16-0.19 s, coasting 0.32-0.42 s: finding 25.

export const SCOUT = {
  wheelD: 0.065, // m
  track: 0.124, // m, geometric (wheel mid-planes)
  scrub: [1.7, 1.93], // effective / geometric track while skid-turning
  ratio: 42, // gearbox
  counts: 1008, // per wheel revolution, x4 quadrature (claimed)
  vCap: 8.4, // V average at the motor
  pack: [9.0, 12.6], // V, 3S floor and full
  sample: 0.02, // s, 50 Hz encoder / PID period
  watchdog: 0.3, // s without /cmd_vel before the brake
  brake: 0.175, // s to stop braking LOW/LOW (0.16-0.19)
};

export const mmPerCount = (s = SCOUT) => (Math.PI * s.wheelD * 1000) / s.counts; // 0.2026
export const pulsesPerMotorTurn = (s = SCOUT) => s.counts / s.ratio / 4; // 6 per channel
export const dutyCap = (vBatt, s = SCOUT) => Math.min(1, s.vCap / vBatt); // 0.667 at 12.6 V, 0.933 at 9.0 V

// Wheel surface speeds for a body twist (v m/s, w rad/s). On a skid-steer robot the
// wheels must run as if the track were k times wider, because the tyres scrub sideways.
export function wheels(v, w, k = 1.8, s = SCOUT) {
  const half = (k * s.track) / 2;
  return { left: v - w * half, right: v + w * half };
}
// Yaw rate the wheels would report if odometry trusted the geometric track.
export const wheelYaw = ({ left, right }, s = SCOUT) => (right - left) / s.track;
export const countsPerSecond = (speed, s = SCOUT) => (speed / (Math.PI * s.wheelD)) * s.counts;

// The idle tour: a 14 s loop of commands. Each phase is { from, to, v, w, cmd }.
// cmd false = /cmd_vel stops arriving (the Pi or the link has gone quiet).
export const TOUR = [
  { name: "straight", from: 0, to: 4.5, v: 0.3, w: 0, cmd: true },
  { name: "turn", from: 4.5, to: 9, v: 0.2, w: 0.5, cmd: true },
  { name: "lost", from: 9, to: 12.5, v: 0.2, w: 0.5, cmd: false },
  { name: "idle", from: 12.5, to: 14, v: 0, w: 0, cmd: true },
];
export const LOOP = 14;

// State of the drive at loop time t (or with /cmd_vel held off from tHeld seconds ago).
// Returns commanded and actual wheel speeds, the phase and what the firmware is doing.
export function stateAt(t, s = SCOUT) {
  const tt = ((t % LOOP) + LOOP) % LOOP;
  const phase = TOUR.find((p) => tt >= p.from && tt < p.to) ?? TOUR[0];
  const target = wheels(phase.v, phase.w);
  let gain = 1;
  let mode = phase.v === 0 ? "idle" : "drive";
  if (!phase.cmd) {
    const since = tt - phase.from; // s since the last /cmd_vel
    if (since < s.watchdog) mode = "waiting";
    else {
      const braking = since - s.watchdog;
      gain = Math.max(0, 1 - braking / s.brake);
      mode = gain > 0 ? "braking" : "stopped";
    }
  }
  const left = target.left * gain;
  const right = target.right * gain;
  return { t: tt, phase: phase.name, mode, v: phase.v * gain, w: phase.w * gain, left, right, cmd: phase.cmd };
}

// Wheel angle (rad) integrated over the loop, for the drawn wheels; sampled.
export function angleTable(dt = 0.02, s = SCOUT) {
  const out = { left: [0], right: [0] };
  let l = 0;
  let r = 0;
  for (let t = dt; t <= LOOP + 1e-9; t += dt) {
    const st = stateAt(t - dt / 2);
    l += (st.left / (s.wheelD / 2)) * dt;
    r += (st.right / (s.wheelD / 2)) * dt;
    out.left.push(l);
    out.right.push(r);
  }
  return out;
}
