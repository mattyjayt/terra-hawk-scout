#!/usr/bin/env python3
"""Bench teleop for the Elegoo car over USB serial (Arduino UNO R4 + Elegoo shield).

Keys (tank style, each side latches until you press it again or stop):
  Up    = right side forward     Right = right side reverse
  Left  = left side forward      Down  = left side reverse
  Space = stop both   + / - = speed   q = quit (sends stop)

Protocol (what your sketch must parse), one line at 10 Hz:
  M <left> <right>\n     left/right PWM in -255..255 (sign = direction)
The sketch should stop both motors if no line arrives for 300 ms (safety timeout).

Usage:  python3 serial_teleop.py /dev/cu.usbmodemXXXX   (find it: ls /dev/cu.usbmodem*)
        python3 serial_teleop.py --dry-run              (no board: prints what it would send)
"""
import argparse
import curses
import time

RATE_HZ = 10


def line(left: int, right: int) -> bytes:
    clamp = lambda v: max(-255, min(255, int(v)))
    return f"M {clamp(left)} {clamp(right)}\n".encode()


def main(stdscr, port, dry_run, baud):
    ser = None
    if not dry_run:
        import serial  # pip install pyserial

        ser = serial.Serial(port, baud, timeout=0)
        time.sleep(2.0)  # many boards reset when the port opens
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    speed, left, right = 150, 0, 0  # left/right: -1, 0, +1
    keymap = {curses.KEY_UP: ("R", 1), curses.KEY_RIGHT: ("R", -1),
              curses.KEY_LEFT: ("L", 1), curses.KEY_DOWN: ("L", -1)}
    next_send, last_rx = 0.0, ""
    try:
        while True:
            k = stdscr.getch()
            if k in keymap:
                side, d = keymap[k]
                if side == "R":
                    right = 0 if right == d else d
                else:
                    left = 0 if left == d else d
            elif k == ord(" "):
                left = right = 0
            elif k in (ord("+"), ord("=")):
                speed = min(255, speed + 15)
            elif k == ord("-"):
                speed = max(0, speed - 15)
            elif k in (ord("q"), ord("Q")):
                break
            now = time.monotonic()
            if now >= next_send:
                msg = line(left * speed, right * speed)
                if ser:
                    ser.write(msg)
                    rx = ser.read(256).decode(errors="ignore").strip()
                    last_rx = rx.splitlines()[-1] if rx else last_rx
                next_send = now + 1.0 / RATE_HZ
                stdscr.erase()
                stdscr.addstr(0, 0, "Up: R fwd  Right: R rev  Left: L fwd  Down: L rev  Space: stop  +/-: speed  q: quit")
                stdscr.addstr(2, 0, f"speed {speed:3d}   left {left:+d}   right {right:+d}")
                stdscr.addstr(3, 0, f"sending: {msg.decode().strip()}")
                stdscr.addstr(4, 0, f"board:   {last_rx}" if ser else "(dry run: no board)")
                stdscr.refresh()
            time.sleep(0.01)
    finally:
        if ser:
            ser.write(line(0, 0))
            ser.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    if not a.dry_run and not a.port:
        ap.error("give the serial port (ls /dev/cu.usbmodem*) or --dry-run")
    curses.wrapper(main, a.port, a.dry_run, a.baud)
