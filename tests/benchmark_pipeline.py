"""在目标机器上测量真实检测、标定和保存耗时，不给出跨硬件的硬性阈值。"""
import json
from pathlib import Path
import resource
import sys
import tempfile
import time
import threading
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calib_gui import CameraThread, Sample, calibrate_samples, detect_preview, save_outputs
from selftest_synthetic import make_dataset

with tempfile.TemporaryDirectory(prefix="calib_bench_") as tmp:
    imgdir = Path(tmp) / "images"
    make_dataset(str(imgdir))
    grays = [cv2.imread(str(path), 0) for path in sorted(imgdir.glob("*.png"))]
    initial_threads = cv2.getNumThreads()
    measurements = {}
    for threads in sorted({initial_threads, 2}):
        cv2.setNumThreads(threads)
        times = []
        for gray in grays:
            image = cv2.resize(gray, (480, 360), interpolation=cv2.INTER_AREA)
            started = time.perf_counter()
            detect_preview(image, (8, 6))
            times.append((time.perf_counter() - started) * 1000)
        measurements["preview_%d_threads_ms" % threads] = {
            "median": round(float(np.median(times)), 2),
            "p95": round(float(np.percentile(times, 95)), 2)}
    cv2.setNumThreads(initial_threads)
    state = dict(interval=0, size=(8, 6), want_size=(480, 360), undist=False, maps=None)
    camera = CameraThread("/unused", state, threading.Lock())
    blank = np.zeros((1080, 1440), np.uint8)
    msg = SimpleNamespace(encoding="mono8", height=1080, width=1440, step=1440, data=blank.tobytes())
    times = []
    for i in range(20):
        started = time.perf_counter()
        camera.on_image(msg)
        times.append((time.perf_counter() - started) * 1000)
    measurements["no_board_pipeline_ms"] = {"median": round(float(np.median(times)), 2),
                                             "max": round(max(times), 2)}
    cv2.setNumThreads(2)
    samples = []
    for gray in grays:
        small = cv2.resize(gray, (480, 360), interpolation=cv2.INTER_AREA)
        corners = detect_preview(small, (8, 6))
        if corners is not None:
            samples.append(Sample(gray, corners * 3))
    started = time.perf_counter()
    result = calibrate_samples(samples, (8, 6), 0.025)
    measurements["calibration_seconds"] = round(time.perf_counter() - started, 3)
    started = time.perf_counter()
    save_outputs("hik", result, samples, (8, 6), 0.025, session=str(Path(tmp) / "saved"),
                 ace_template=str(Path(__file__).resolve().parents[1] / "reference_json" / "uav.json"))
    measurements["save_seconds"] = round(time.perf_counter() - started, 3)
    measurements.update(views=len(samples), rms=round(result["rms"], 4),
                        linear_pixels=round(result["linear"], 4),
                        samples_mb=round(sum(s.arr.nbytes for s in samples) / 1024**2, 1),
                        peak_rss_mb=round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 1),
                        opencv=cv2.__version__, default_threads=initial_threads)
    print(json.dumps(measurements, indent=2))
