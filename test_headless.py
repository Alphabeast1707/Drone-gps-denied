"""
Headless AprilTag Test — for SSH (no display needed)
=====================================================
Runs detection without opening any GUI window.
Prints tag IDs, positions, distances, and FPS to terminal.

Perfect for testing over SSH on Raspberry Pi.

Usage:
    python3 test_headless.py

Press Ctrl+C to stop.
"""

import cv2
import numpy as np
import time
import signal
import sys
from pupil_apriltags import Detector
from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT,
    CAMERA_PARAMS,
    TAG_FAMILY, TAG_NTHREADS, TAG_QUAD_DECIMATE,
    TAG_QUAD_SIGMA, TAG_REFINE_EDGES, TAG_DECODE_SHARPENING,
    CORNER_TAG_IDS, CENTER_TAG_ID,
    CORNER_TAG_SIZE, CENTER_TAG_SIZE,
)

# Graceful exit on Ctrl+C
running = True
def signal_handler(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, signal_handler)


def main():
    global running

    print("=" * 55)
    print("  HEADLESS APRILTAG TEST (SSH mode)")
    print("=" * 55)

    # ── Camera ──
    print("\n[1/3] Opening camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 90)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("[ERROR] Cannot open camera!")
        print("  Try: libcamera-hello --timeout 3000")
        print("  Make sure camera is enabled in raspi-config")
        sys.exit(1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"  ✅ Camera opened: {w}x{h}")

    # ── Detector ──
    print("\n[2/3] Initializing AprilTag detector...")
    detector = Detector(
        families=TAG_FAMILY,
        nthreads=TAG_NTHREADS,
        quad_decimate=TAG_QUAD_DECIMATE,
        quad_sigma=TAG_QUAD_SIGMA,
        refine_edges=TAG_REFINE_EDGES,
        decode_sharpening=TAG_DECODE_SHARPENING,
    )
    print(f"  ✅ Detector ready: {TAG_FAMILY}  decimate={TAG_QUAD_DECIMATE}  threads={TAG_NTHREADS}")

    # ── Detection loop ──
    print("\n[3/3] Starting detection loop...")
    print("  Hold an AprilTag (tag36h11) in front of camera.")
    print("  Press Ctrl+C to stop.\n")
    print("-" * 55)
    print(f"  {'TAG':>5s}  {'TYPE':>8s}  {'CENTER':>13s}  {'Z (m)':>8s}  {'FPS':>6s}")
    print("-" * 55)

    frame_count = 0
    fps = 0.0
    fps_interval = 30
    interval_start = time.time()
    tags_ever_seen = set()

    while running:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect with pose estimation
        results = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=CORNER_TAG_SIZE,
        )

        for r in results:
            tags_ever_seen.add(r.tag_id)
            cx, cy = int(r.center[0]), int(r.center[1])

            # Determine tag type
            if r.tag_id == CENTER_TAG_ID:
                tag_type = "CENTER"
                scale = CENTER_TAG_SIZE / CORNER_TAG_SIZE
            elif r.tag_id in CORNER_TAG_IDS:
                tag_type = "CORNER"
                scale = 1.0
            else:
                tag_type = "OTHER"
                scale = 1.0

            # Distance
            z_str = "---"
            if r.pose_t is not None:
                z = abs(r.pose_t.flatten()[2] * scale)
                z_str = f"{z:.3f}"

            print(f"  ID:{r.tag_id:>2d}  {tag_type:>8s}  ({cx:4d},{cy:4d})  {z_str:>8s}  {fps:>5.1f}")

        # FPS
        frame_count += 1
        if frame_count % fps_interval == 0:
            elapsed = time.time() - interval_start
            fps = fps_interval / elapsed if elapsed > 0 else 0
            interval_start = time.time()

            # Print status even if no tags detected
            if len(results) == 0:
                print(f"  {'--- no tags ---':^40s}  {fps:>5.1f}")

    # ── Done ──
    cap.release()
    print("\n" + "=" * 55)
    print(f"  Total frames:  {frame_count}")
    print(f"  Final FPS:     {fps:.1f}")
    print(f"  Tags seen:     {sorted(tags_ever_seen) if tags_ever_seen else 'none'}")
    print("=" * 55)


if __name__ == "__main__":
    main()
