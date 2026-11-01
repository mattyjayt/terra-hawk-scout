"""Run every round-1 analysis in dependency order: ~/.venvs/scout-sim/bin/python run_all.py"""
import runpy
from pathlib import Path

HERE = Path(__file__).resolve().parent
for s in ("a1_mass", "a2_drivetrain", "a3_skid_steer", "a4_driver", "a5_power", "a6_stability", "a7_odometry", "a8_sensors"):
    print(f"\n######## {s}")
    runpy.run_path(str(HERE / f"{s}.py"), run_name="__main__")
