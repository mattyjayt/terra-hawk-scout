"""Wheel-load models shared by a2 (drivetrain), a3 (skid steer), a4 (driver), a5 (power)."""
import math

from common import ENV, G, geometry, load_results

GEO = geometry()
R = GEO["r_wheel"]
L = GEO["wheelbase"]
B = GEO["track"]
AX_F, AX_R = GEO["axle_x"]


def design_case(name="proposed_c1_pay300"):
    """Mass [kg] and CoM [m] from a1 results, or from the CAD fit check for names 'cad_*'."""
    if name.startswith("cad_") or name == "robot_params":
        from common import cad_cases
        c = cad_cases()[name]
        return c["mass_kg"], list(c["com_m"])
    c = load_results()["mass"]["cases"][name]
    return c["mass_kg"], [v / 1000 for v in c["com_mm"]]


def axle_loads(m, com_x):
    W = m * G
    nf = (com_x - AX_R) / (AX_F - AX_R)
    return W * nf, W * (1 - nf)


def rpm_from_v(v):
    return v / R * 60 / (2 * math.pi)


def w_from_v(v):
    return v / R


def straight_force(m, surface, a=0.0):
    """Total traction force [N] for straight motion (rolling + inertia)."""
    return ENV["crr"][surface] * m * G + m * a


def skid_moment(m, mu_lat, com_x=0.0):
    """Turning-resistance moment about the CoR for a 4-wheel skid steer (Coulomb lateral
    friction, wheels at x = +/- L/2): M = mu * sum(N_i |x_i|). Wong, Theory of Ground
    Vehicles ch. 7; for axles at +/-L/2 this is mu*W*L/2 regardless of fore/aft split."""
    nf, nr = axle_loads(m, com_x)
    return mu_lat * (nf * abs(AX_F) + nr * abs(AX_R))


def skid_side_forces(m, mu_lat, surface, com_x=0.0):
    """Outer/inner side forward forces [N] for a turn (pivot or arc): F_o - F_i = 2M/B... with
    F_o + F_i = rolling; moment (F_o - F_i) * B/2 = M."""
    M = skid_moment(m, mu_lat, com_x)
    roll = ENV["crr"][surface] * m * G
    f_o = roll / 2 + M / B
    f_i = roll / 2 - M / B
    return f_o, f_i, M


def step_climb(n_climb, n_push, h, mu, crr, t_wheel):
    """Quasi-static step climb of one axle (2 wheels) over a square edge of height h.
    n_climb: vertical load on the climbing axle [N]; n_push: load on the other axle.
    t_wheel: torque available per wheel [N*m]. Returns (ok, margin_N, theta_deg, f_trac_max).
    Balance on the climbing wheel pair (edge contact angle theta from vertical):
        P + f / cos(theta) = N_c tan(theta)
    f <= mu N_c / (cos th + mu sin th)  (edge traction),   P <= mu N_p - crr N_p (pusher traction)."""
    if h >= R:
        return False, -1e9, 90.0, 0.0
    th = math.acos((R - h) / R)
    need = n_climb * math.tan(th)
    f_trac = mu * n_climb / (math.cos(th) + mu * math.sin(th))
    f = min(2 * t_wheel / R, f_trac)
    p = max(min(2 * t_wheel / R, mu * n_push) - crr * n_push, 0.0)
    have = p + f / math.cos(th)
    return have >= need, have - need, math.degrees(th), f_trac


def step_torque_required(n_climb, n_push, h, mu, crr):
    """Smallest per-wheel torque that climbs (both axles drive equally). None if traction-limited."""
    ok, _, _, _ = step_climb(n_climb, n_push, h, mu, crr, 10.0)
    if not ok:
        return None
    lo, hi = 0.0, 10.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if step_climb(n_climb, n_push, h, mu, crr, mid)[0]:
            hi = mid
        else:
            lo = mid
    return hi
