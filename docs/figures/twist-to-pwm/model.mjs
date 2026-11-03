// Pure model of DD-SCOUT-BRIDGE §5: cmd_vel (v, ω) → `M <left> <right>`.
// W, v_max and pwm_min are NOT measured yet (Lab 1). EXAMPLE holds example
// values only, so the figure can move; every rule below is from the spec.
export const EXAMPLE = { W: 0.16, vmax: 0.5, pwmMin: 60 }; // example values, not measured
export const DEADBAND = 0.005; // m/s, §5.3

export function twistToPwm(v, w, { W, vmax, pwmMin } = EXAMPLE) {
  // 5.1 kinematics (REP-103: ω > 0 is a left turn → right side faster)
  const rawL = v - (w * W) / 2;
  const rawR = v + (w * W) / 2;
  // 5.2 saturation: scale BOTH sides by s = max(|vL|,|vR|)/v_max (keeps the curvature)
  const most = Math.max(Math.abs(rawL), Math.abs(rawR));
  const s = most > vmax ? most / vmax : 1;
  const vL = rawL / s;
  const vR = rawR / s;
  // 5.3 deadband → PWM
  const pwm = (x) => {
    if (Math.abs(x) < DEADBAND) return 0;
    const p = Math.round(pwmMin + ((255 - pwmMin) * Math.abs(x)) / vmax);
    return Math.sign(x) * Math.min(255, p);
  };
  const l = pwm(vL);
  const r = pwm(vR);
  return { rawL, rawR, s, vL, vR, l, r, line: `M ${l} ${r}`, R: Math.abs(w) < 1e-6 ? Infinity : v / w };
}
