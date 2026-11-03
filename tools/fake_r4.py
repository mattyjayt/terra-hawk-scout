#!/usr/bin/env python3
"""Fake Arduino UNO R4 base controller: speaks serial-r4-v1 on a pseudo-terminal.

Use it to run and test the ROS 2 base bridge without hardware (software in the loop).
It mirrors firmware/r4_base: same commands, replies, 20 Hz telemetry, 300 ms watchdog.

    python3 tools/fake_r4.py                       # creates /tmp/fake_r4 -> /dev/pts/N
    python3 tools/fake_r4.py --log /tmp/r4.jsonl   # also log every received command
    ros2 run scout_base_py base_bridge --ros-args -p port:=/tmp/fake_r4

Log format (one JSON object per line): {"t": <unix s>, "rx": "<command line>"}
plus {"t": ..., "event": "watchdog_stop"} when the watchdog fires. Tests read this file.

Use --freeze-imu to reproduce the frozen-IMU fault seen on 3 Nov (identical values every line).

Axis and sign conventions follow firmware v1 (YAW_SIGN = -1, gravity on chip +Z); whether that
matches the real chip mounting is Lab 2.2. Physics is deliberately crude: yaw rate is proportional to (right - left) PWM, so a
turn command shows up in the gyro. Gravity is on chip +Z. Noise is Gaussian.
Spec: docs/specs/interfaces/serial-r4-v1.md
"""
import argparse
import json
import os
import random
import select
import signal
import sys
import time
import tty

WATCHDOG_S = 0.300
MAX_TIMED_MS = 5000
TELEMETRY_S = 0.050
YAW_RATE_PER_PWM = 0.6  # deg/s of yaw rate per PWM unit of (right - left); crude
YAW_SIGN = -1.0         # same convention as the firmware: chip gz * YAW_SIGN = left-positive yaw rate


class FakeR4:
    def __init__(self, fd, log_path=None, noise=True, boot_s=3.6, battery=7.6, seed=None, freeze_imu=False):
        self.fd = fd
        self.freeze_imu = freeze_imu  # mimic firmware v1 after an I2C failure: repeat the last sample
        self.frozen = None
        self.log = open(log_path, "a", buffering=1) if log_path else None
        self.noise = noise
        self.rng = random.Random(seed)
        self.t0 = time.monotonic()
        self.boot_until = self.t0 + boot_s
        self.booted = False
        self.left = self.right = 0
        self.drive_until = None
        self.yaw = 0.0
        self.servo = 90
        self.battery = battery
        self.last_tel = 0.0
        self.last_imu = time.monotonic()
        self.buf = b""
        self.calibrating_until = None
        self.gz_chip = 0.0
        self._send("A boot calibrating gyro, keep still 2 s")

    # ---- I/O ----
    def _send(self, line):
        try:
            os.write(self.fd, (line + "\r\n").encode())  # Arduino println ends with \r\n
        except OSError:
            pass  # nobody has the port open; a real USB CDC port drops data too

    def _log(self, **kw):
        if self.log:
            kw["t"] = time.time()
            self.log.write(json.dumps(kw) + "\n")

    def _n(self, sigma):
        return self.rng.gauss(0.0, sigma) if self.noise else 0.0

    # ---- commands ----
    def handle(self, cmd):
        self._log(rx=cmd)
        now = time.monotonic()
        if not self.booted:
            return  # the real R4 is still in setup() and not reading the port
        if self.calibrating_until:
            return  # blocking calibration on the real R4: commands wait (we drop them)
        c = cmd[0]
        if c == "M":
            parts = cmd[1:].split()
            try:
                vals = [int(p) for p in parts]
            except ValueError:
                vals = []
            if len(vals) < 2:
                self._send("A err M needs: M <left> <right> [ms]")
                return
            self.left = max(-255, min(255, vals[0]))
            self.right = max(-255, min(255, vals[1]))
            hold = WATCHDOG_S if len(vals) == 2 else max(0, min(MAX_TIMED_MS, vals[2])) / 1000.0
            self.drive_until = now + hold
            if len(vals) >= 3:
                self._send("A M ok")
        elif c == "S":
            self._stop()
            self._send("A S ok")
        elif c == "Z":
            self.yaw = 0.0
            self._send("A Z ok")
        elif c == "C":
            self._stop()
            self._send("A C calibrating 2 s")
            self.calibrating_until = now + 2.0
        elif c == "V":
            try:
                a = int(cmd[1:].split()[0])
            except (ValueError, IndexError):
                self._send("A err V needs: V <angle 0-180>")
                return
            self.servo = max(0, min(180, a))
            self._send("A V ok")
        else:
            self._send("A err unknown: " + cmd)

    def _stop(self):
        self.left = self.right = 0
        self.drive_until = None

    # ---- main loop step ----
    def step(self):
        now = time.monotonic()
        if not self.booted and now >= self.boot_until:
            self.booted = True
            self._send("A boot ready")
        if self.calibrating_until and now >= self.calibrating_until:
            self.calibrating_until = None
            self._send("A C ok")
        if self.drive_until is not None and now >= self.drive_until:
            self._stop()
            self._send("W stop")
            self._log(event="watchdog_stop")

        # IMU model + yaw integration like updateImu() (0.2 deg/s deadband)
        dt = now - self.last_imu
        self.last_imu = now
        yaw_rate = YAW_RATE_PER_PWM * (self.right - self.left) / 2.0  # left-positive
        self.gz_chip = yaw_rate / YAW_SIGN + self._n(0.05)
        rate = YAW_SIGN * self.gz_chip
        if abs(rate) >= 0.2:
            self.yaw += rate * dt

        if self.booted and not self.calibrating_until and now - self.last_tel >= TELEMETRY_S:
            self.last_tel = now
            t_ms = int((now - self.t0) * 1000)
            ax, ay, az = self._n(0.004), self._n(0.004), 1.0 + self._n(0.004)
            gx, gy, gz = self._n(0.05), self._n(0.05), self.gz_chip
            if self.freeze_imu:
                if self.frozen is None:
                    self.frozen = (ax, ay, az, gx, gy, gz)
                ax, ay, az, gx, gy, gz = self.frozen
            batt = self.battery - 0.0004 * (abs(self.left) + abs(self.right)) + self._n(0.01)
            self._send(
                f"I {t_ms} {ax:.3f} {ay:.3f} {az:.3f} {gx:.2f} {gy:.2f} {gz:.2f} "
                f"{self.yaw:.2f} {self.servo} {batt:.2f}"
            )

    def feed(self, data):
        self.buf += data
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.replace(b"\r", b"").decode(errors="replace").strip()
            if not line:
                continue
            if len(line) > 47:
                self._send("A err line too long")
                continue
            self.handle(line)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Fake UNO R4 base controller (serial-r4-v1) on a pty")
    ap.add_argument("--link", default="/tmp/fake_r4", help="symlink to create for the pty (default /tmp/fake_r4)")
    ap.add_argument("--log", default=None, help="append received commands as JSON lines to this file")
    ap.add_argument("--no-noise", action="store_true", help="deterministic telemetry (no Gaussian noise)")
    ap.add_argument("--boot-s", type=float, default=3.6, help="boot time before 'A boot ready' (real R4 ~3.6 s)")
    ap.add_argument("--battery", type=float, default=7.6, help="battery voltage to report")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--freeze-imu", action="store_true", help="repeat one IMU sample forever (firmware v1 I2C failure)")
    a = ap.parse_args(argv)

    master, slave = os.openpty()
    tty.setraw(slave)  # no echo, no line editing: behaves like a USB CDC port
    os.set_blocking(master, False)  # never block when no one reads the port (lines are dropped)
    slave_name = os.ttyname(slave)
    if a.link:
        try:
            os.unlink(a.link)
        except FileNotFoundError:
            pass
        os.symlink(slave_name, a.link)
    print(f"fake_r4: {a.link or slave_name} -> {slave_name}", flush=True)

    def _term(*_):
        raise KeyboardInterrupt  # SIGTERM (e.g. from a test) cleans up like Ctrl-C
    signal.signal(signal.SIGTERM, _term)

    r4 = FakeR4(master, a.log, noise=not a.no_noise, boot_s=a.boot_s, battery=a.battery, seed=a.seed,
                freeze_imu=a.freeze_imu)
    try:
        while True:
            rd, _, _ = select.select([master], [], [], 0.002)
            if rd:
                try:
                    r4.feed(os.read(master, 1024))
                except OSError:
                    pass
            r4.step()
    except KeyboardInterrupt:
        pass
    finally:
        if a.link and os.path.islink(a.link):
            os.unlink(a.link)
    return 0


if __name__ == "__main__":
    sys.exit(main())
