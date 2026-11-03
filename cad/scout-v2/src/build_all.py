"""Build every Scout v2 variant model (the default scout_v2 plus scout_v2.VARIANTS) and print their ids.

Run:  ~/.local/bin/cadgen +0.8.4 run src/build_all.py
"""
from scout_v2 import scout_v2
from variants import MODELS

if __name__ == "__main__":
    scout_v2()
    for fn in MODELS.values():
        fn()
