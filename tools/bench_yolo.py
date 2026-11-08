"""Time YOLO person detection on one image: median and p90 per frame, plus FPS.

Usage: python tools/bench_yolo.py <model> <image> <imgsz> [runs]
  <model>  yolo11n.pt (PyTorch) or yolo11n_320_ncnn_model
  (an NCNN export folder)
"""
import statistics
import sys
import time

import cv2
from ultralytics import YOLO


def main():
    """Run the bench."""
    model_path, image_path, imgsz = sys.argv[1], sys.argv[2], int(sys.argv[3])
    runs = int(sys.argv[4]) if len(sys.argv) > 4 else 50
    frame = cv2.imread(image_path)
    if frame is None:
        sys.exit(f'cannot read {image_path}')
    model = YOLO(model_path, task='detect')
    for _ in range(5):
        model.predict(frame, imgsz=imgsz, classes=[0], conf=0.5, verbose=False)
    times, persons, infer = [], 0, []
    for _ in range(runs):
        t0 = time.perf_counter()
        r = model.predict(frame, imgsz=imgsz, classes=[0],
                          conf=0.5, verbose=False)[0]
        times.append((time.perf_counter() - t0) * 1000.0)
        infer.append(r.speed['inference'])
        persons = len(r.boxes)
    times.sort()
    med = statistics.median(times)
    p90 = times[int(0.9 * len(times)) - 1]
    print(f'{model_path:24s} imgsz={imgsz:3d}  total median {
        med:6.1f} ms  p90 {p90:6.1f} ms  '
          f'inference {statistics.median(infer):6.1f} ms  -> {
              1000.0 / med:5.1f} FPS  persons={persons}')


if __name__ == '__main__':
    main()
