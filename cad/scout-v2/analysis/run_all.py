"""Run every analysis in dependency order: ~/.venvs/scout-sim/bin/python run_all.py
a1-a4: round 1 (v1-converted geometry / proposed layout). a5-a9: round 2 (CAD fit-check mass cases)."""
import runpy
from pathlib import Path

HERE = Path(__file__).resolve().parent
for s in ("a1_mass", "a2_drivetrain", "a3_skid_steer", "a4_driver", "a9_motor_review", "a5_power", "a6_stability",
          "a7_odometry", "a8_sensors"):
    print(f"\n######## {s}")
    runpy.run_path(str(HERE / f"{s}.py"), run_name="__main__")
