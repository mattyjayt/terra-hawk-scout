# Lab 3 — Python bridge core, test first
**Objectives:** write the parts of the bridge that make decisions (parsing, kinematics, conversions, fault detection) as **pure functions**, with no ROS and no serial, and prove them with unit tests **before** writing them.
**Reading:** DD-SCOUT-BRIDGE §3, §5, §6, §8.1 · Modern Robotics §13.3 · Gary Bernhardt, "Boundaries" · ROS 2 tutorial "Testing your code with Python" (Intermediate → Testing).

## Background: functional core, imperative shell
Bugs hide where logic and I/O are tangled. With pure functions (same input → same output, no side effects) you can test thousands of cases in milliseconds. The node (Lab 4) only wires the core to ROS and the port. If you put an `if` with real logic in the node, ask yourself whether it belongs in the core.

**Workflow per task:** write the test file → run it and watch it fail (RED) → write the minimum code (GREEN) → clean up (REFACTOR). Fast loop without colcon:
```bash
cd ~/Projects/terra-hawk-scout/ros2_ws/src/scout_base_py
python3 -m pytest -q test/          # seconds
```
Full check (linters too): `colcon build --packages-select scout_base_py && colcon test --packages-select scout_base_py && colcon test-result --verbose`.

---

## Task 3.1 — `protocol.py`: parse every line the R4 can send 🧠
**Interface (suggested):**
```python
@dataclass(frozen=True)
class ImuSample:      # one 'I' line, raw units as sent (g, deg/s, deg, deg, V)
    t_ms: int; ax: float; ay: float; az: float
    gx: float; gy: float; gz: float
    yaw_deg: float; servo_deg: int; batt_v: float

@dataclass(frozen=True)
class Ack:            # 'A ...' lines: 'A S ok', 'A err ...', 'A boot ready'
    text: str         # everything after 'A '

@dataclass(frozen=True)
class WatchdogStop:   # 'W stop'
    pass

def parse_line(line: str) -> ImuSample | Ack | WatchdogStop | None: ...
def format_drive(left: int, right: int) -> str: ...      # 'M 120 -80\n', clamped to ±255
def format_servo(deg: int) -> str: ...                   # 'V 119\n', clamped 0..180
```
**Tests first** (`test/test_protocol.py`, cases U1–U3 of DD-SCOUT-BRIDGE §8.1):
- the example line from serial-r4-v1 parses to the right values *and types* (`servo_deg` is an `int`)
- `\r\n` endings and surrounding spaces are tolerated
- **never raises** for: `""`, `"I"`, `"I 1 2 3"` (too few fields), 12 fields, `"I 1 a 2 …"`, 200 random characters, `"M 1 2"` (a command echoed back)
- `A S ok` → `Ack("S ok")`, `A boot ready` → `Ack("boot ready")`, `W stop` → `WatchdogStop()`
- `format_drive(300, -300) == "M 255 -255\n"`

Tip: `pytest.mark.parametrize` turns the list of bad lines into one test per case.

**Check:** all protocol tests green, and `parse_line` is the only place in the package that knows the field order.

## Task 3.2 — `kinematics.py`: Twist → PWM, pan angle ↔ servo degrees 🧠
**Interface:**
```python
def twist_to_pwm(v: float, w: float, track_width: float, v_max: float, pwm_min: int) -> tuple[int, int]: ...
def pan_rad_to_servo_deg(rad: float, pan_sign: int, pan_min: float, pan_max: float) -> int: ...
def servo_deg_to_pan_rad(deg: int, pan_sign: int) -> float: ...
```
Algorithm: DD-SCOUT-BRIDGE §5.1–5.3 and §7. Work the maths on paper before you write the code.
**Tests first** (U4, U5), with `track_width = 0.15`, `v_max = 0.5`, `pwm_min = 60` as test values:
- `(0, 0) → (0, 0)`
- `(0.25, 0)` → equal, positive. Compute the exact expected PWM by hand.
- `(0, +1.0)` → left negative, right positive (a left turn spins the right side forward)
- `(0.5, 2.0)` saturates: both scaled, and `right/left` equals `v_R/v_L` before scaling (`pytest.approx`, with a tolerance for rounding)
- `(0.001, 0)` → `(0, 0)` (below the 5 mm/s threshold)
- `(0.006, 0)` → `(pwm_min, pwm_min)`
- for 1000 random `(v, w)`: `|pwm| ≤ 255` and `sign(pwm) == sign(v_side)` (a *property test*; the `hypothesis` library does this properly, a loop is fine)
- servo: `0 rad → 90`, `+π/4 → 135` (pan_sign +1) and `45` (pan_sign −1), `+3.0 rad → clamped`, round trip deg → rad → deg

**Check:** all green; explain each number in the forward test from the formula.

## Task 3.3 — `conversions.py`: units, stale detector, rate meter 🧠
```python
G = 9.80665
def g_to_mps2(a_g: float) -> float: ...
def dps_to_radps(w_dps: float) -> float: ...

class StaleDetector:          # True once the same tuple was seen `limit` times in a row
    def __init__(self, limit: int = 10): ...
    def update(self, values: tuple) -> bool: ...

class RateMeter:              # lines per second over a sliding window (pass `now` in: testable!)
    def __init__(self, window_s: float = 1.0): ...
    def tick(self, now: float) -> None: ...
    def rate(self, now: float) -> float: ...

def link_level(connected: bool, rate_hz: float, expected_hz: float, imu_frozen: bool, s_since_line: float) -> int: ...
    # 0 OK / 1 WARN / 2 ERROR, rules in IF-ROS-TOPICS T6
```
**Tests first** (U6–U8): `g_to_mps2(1) == 9.80665`; `dps_to_radps(180) == approx(pi)`; 9 identical → False, 10th → True, one different → resets; RateMeter with fake times 0.00, 0.05, … → 20 Hz; `link_level` table: (True, 19, 20, False, 0.05) → 0, (True, 15, 20, …) → 1, (False, …) → 2, frozen → 2, 0.6 s silence → 2.

**Why `now` is a parameter:** a function that calls `time.time()` inside can't be tested without sleeping. Passing time in is *dependency injection* in its simplest form.

**Check:** `colcon test` green: your tests + flake8 + pep257. Commit: `feat(scout_base_py): bridge core with unit tests`.

## Questions (lab report)
1. Give a numeric example where clipping each wheel separately (instead of scaling both) changes the turn radius. By how much?
2. Why must `parse_line` never raise? What would happen to the node's reader thread if it did?
3. Which of your tests would have caught the 3 Nov frozen-IMU fault, and at which level (unit, SIL, HITL) is it finally proven?
4. Your tests passed RED → GREEN. Name one bug the tests *cannot* catch.
