#!/usr/bin/env python3
# Copyright 2026 Jaime Tellie
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Write fake_lidar's world as a Nav2 map (PGM + YAML) for SIL tests.

Usage: python3 tools/make_sil_map.py OUT_PREFIX [--world FILE] [--res 0.05]
Creates OUT_PREFIX.pgm and OUT_PREFIX.yaml (map_server format, origin so that
the room centre is the map origin, like fake_lidar's world frame).
"""

import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_lidar import DEFAULT_BOXES, DEFAULT_ROOM  # noqa: E402


def main():
    """Rasterise the room walls and boxes into an occupancy image."""
    p = argparse.ArgumentParser()
    p.add_argument('out')
    p.add_argument('--world')
    p.add_argument('--res', type=float, default=0.05)
    a = p.parse_args()
    room, boxes = DEFAULT_ROOM, list(DEFAULT_BOXES)
    if a.world:
        with open(a.world, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        room = tuple(data.get('room', room))
        boxes = [tuple(b) for b in data.get('boxes', boxes)]
    margin = 0.10                                   # outside the walls
    w = int(round((room[0] + 2 * margin) / a.res))
    h = int(round((room[1] + 2 * margin) / a.res))
    ox, oy = -room[0] / 2 - margin, -room[1] / 2 - margin
    xs = ox + (np.arange(w) + 0.5) * a.res          # cell centres
    ys = oy + (np.arange(h) + 0.5) * a.res
    gx, gy = np.meshgrid(xs, ys)
    inside = (np.abs(gx) < room[0] / 2) & (np.abs(gy) < room[1] / 2)
    occ = ~inside & (np.abs(gx) < room[0] / 2 + a.res) & (np.abs(gy) < room[1] / 2 + a.res)
    for cx, cy, sx, sy in boxes:
        occ |= (np.abs(gx - cx) <= sx / 2) & (np.abs(gy - cy) <= sy / 2)
    img = np.full((h, w), 128, dtype=np.uint8)      # unknown (p = 0.5)
    img[inside] = 254                               # free
    img[occ] = 0                                    # occupied
    img = np.flipud(img)                            # PGM row 0 is the top (max y)
    with open(a.out + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    meta = {'image': os.path.basename(a.out) + '.pgm', 'mode': 'trinary',
            'resolution': a.res, 'origin': [round(ox, 3), round(oy, 3), 0.0],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
    with open(a.out + '.yaml', 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    print(f'wrote {a.out}.pgm ({w} x {h} cells) and {a.out}.yaml')


if __name__ == '__main__':
    main()
