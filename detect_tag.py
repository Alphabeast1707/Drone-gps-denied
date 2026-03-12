"""
Stage 1 — High-Speed AprilTag Detection
=========================================
Detects AprilTags from camera feed with FPS measurement.

Optimizations applied:
  • quad_decimate = 3  (downsample for speed)
  • nthreads = 4       (use all Pi 4 cores)
  • refine_edges off   (reduce CPU work)
  • grayscale only     (skip color processing)

Target: 80–120 FPS on Raspberry Pi 4

Usage:
    python3 detect_tag.py

Press ESC to quit.
"""

import cv2
import numpy as np
import time
from pupil_apriltags import Detector
from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT,
    TAG_FAMILY, TAG_NTHREADS, TAG_QUAD_DECIMATE,
    TAG_QUAD_SIGMA, TAG_REFINE_EDGES, TAG_DECODE_SHARPENING,
)


def setup_camera():
    """Initialize camera with optimal settings."""
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    # Try to set high FPS (works with CSI camera)
    cap.set(cv2.CAP_PROP_FPS, 90)
    # Reduce buffer size so we always get the latest frame
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("[ERROR] Cannot open camera.")
        print("  • Make sure camera is enabled: sudo raspi-config → Interface → Camera")
        print("  • Test with: libcamera-hello")
        exit(1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[CAMERA] Opened: {actual_w}x{actual_h} @ {actual_fps:.0f} FPS")
    return cap


def setup_detector():
    """Initialize AprilTag detector with high-speed settings."""
    detector = Detector(
        families=TAG_FAMILY,
        nthreads=TAG_NTHREADS,
        quad_decimate=TAG_QUAD_DECIMATE,
        quad_sigma=TAG_QUAD_SIGMA,
        refine_edges=TAG_REFINE_EDGES,
        decode_sharpening=TAG_DECODE_SHARPENING,
    )
    print(f"[DETECTOR] family={TAG_FAMILY}  decimate={TAG_QUAD_DECIMATE}  "
          f"threads={TAG_NTHREADS}  refine_edges={TAG_REFINE_EDGES}")
    return detector


def draw_tag(frame, detection):
    """Draw bounding box and center point on detected tag."""
    corners = detection.corners.astype(int)

    # Draw green quadrilateral around tag
    for i in range(4):
        pt1 = tuple(corners[i])
        pt2 = tuple(corners[(i + 1) % 4])
        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

    # Draw red center dot
    cx, cy = int(detection.center[0]), int(detection.center[1])
    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    # Label with tag ID
    cv2.putText(frame, f"ID:{detection.tag_id}",
                (cx + 10, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return cx, cy


def main():
    cap = setup_camera()
    detector = setup_detector()

    # FPS tracking
    fps_avg = 0.0
    frame_count = 0
    fps_update_interval = 30  # Update FPS display every N frames
    interval_start = time.time()

    print("\n[RUNNING] AprilTag detection started. Press ESC to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Frame grab failed, retrying...")
            continue

        # ── Convert to grayscale (AprilTag only needs grayscale) ──
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── Detect tags ──
        t_start = time.time()
        results = detector.detect(gray)
        t_detect = (time.time() - t_start) * 1000  # ms

        # ── Draw detections ──
        for r in results:
            cx, cy = draw_tag(frame, r)
            print(f"  Tag ID: {r.tag_id:>3d}  |  center: ({cx:4d}, {cy:4d})  |  "
                  f"detect: {t_detect:.1f} ms")

        # ── FPS calculation ──
        frame_count += 1
        if frame_count % fps_update_interval == 0:
            elapsed = time.time() - interval_start
            fps_avg = fps_update_interval / elapsed
            interval_start = time.time()

        # ── HUD overlay ──
        cv2.putText(frame, f"FPS: {fps_avg:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"Tags: {len(results)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"Detect: {t_detect:.1f}ms", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # ── Display ──
        cv2.imshow("Stage 1 — AprilTag Detection", frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[DONE] Processed {frame_count} frames.  Average FPS: {fps_avg:.1f}")


if __name__ == "__main__":
    main()
