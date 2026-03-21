"""
Stage 2 — Pose Estimation (Distance + Orientation)
====================================================
Extends AprilTag detection with 6-DOF pose estimation.
Tells you exactly where the tag is relative to the camera:
  X → left / right offset (meters)
  Y → up / down offset   (meters)
  Z → distance            (meters)

This is the data drones use for precision landing.

Usage:
    python3 pose_estimation.py

Press ESC to quit.
"""

import cv2
import numpy as np
import time
from pupil_apriltags import Detector
from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, CAMERA_FOURCC,
    CAMERA_BACKEND,
    CAMERA_PARAMS,
    TAG_FAMILY, TAG_NTHREADS, TAG_QUAD_DECIMATE,
    TAG_QUAD_SIGMA, TAG_REFINE_EDGES, TAG_DECODE_SHARPENING,
    CORNER_TAG_IDS, CENTER_TAG_ID,
    CORNER_TAG_SIZE, CENTER_TAG_SIZE,
)


def setup_camera():
    """Initialize camera."""
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    fourcc = cv2.VideoWriter_fourcc(*CAMERA_FOURCC)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("[ERROR] Cannot open camera.")
        exit(1)

    print(f"[CAMERA] {int(cap.get(3))}x{int(cap.get(4))}")
    return cap


def setup_detector():
    """Initialize AprilTag detector."""
    return Detector(
        families=TAG_FAMILY,
        nthreads=TAG_NTHREADS,
        quad_decimate=TAG_QUAD_DECIMATE,
        quad_sigma=TAG_QUAD_SIGMA,
        refine_edges=TAG_REFINE_EDGES,
        decode_sharpening=TAG_DECODE_SHARPENING,
    )


def get_tag_size(tag_id):
    """Return physical tag size based on ID."""
    if tag_id == CENTER_TAG_ID:
        return CENTER_TAG_SIZE
    elif tag_id in CORNER_TAG_IDS:
        return CORNER_TAG_SIZE
    else:
        return CORNER_TAG_SIZE  # default


def draw_detection(frame, r, pose_t):
    """Draw tag outline, center, ID, and pose info."""
    corners = r.corners.astype(int)
    for i in range(4):
        cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                 (0, 255, 0), 2)

    cx, cy = int(r.center[0]), int(r.center[1])
    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    # Pose text
    x, y, z = pose_t.flatten()
    label = f"ID:{r.tag_id}"
    dist_label = f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m"

    cv2.putText(frame, label, (cx + 10, cy - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(frame, dist_label, (cx + 10, cy + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)

    return cx, cy, x, y, z


def main():
    cap = setup_camera()
    detector = setup_detector()

    fps_avg = 0.0
    frame_count = 0
    fps_interval = 30
    interval_start = time.time()

    print(f"\n[CONFIG] Camera intrinsics: fx={CAMERA_PARAMS[0]:.0f} fy={CAMERA_PARAMS[1]:.0f} "
          f"cx={CAMERA_PARAMS[2]:.0f} cy={CAMERA_PARAMS[3]:.0f}")
    print(f"[CONFIG] Corner tag size: {CORNER_TAG_SIZE*100:.0f} cm  |  "
          f"Center tag size: {CENTER_TAG_SIZE*100:.0f} cm")
    print("\n[RUNNING] Pose estimation started. Press ESC to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── Detect with pose estimation ──
        # Note: pupil-apriltags requires a single tag_size for all tags
        # in one call. We detect twice if we have different sizes,
        # OR use the dominant tag size. Here we detect once with the
        # larger size, then correct for the center tag.
        results = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=CORNER_TAG_SIZE,
        )

        for r in results:
            if r.pose_t is None:
                continue

            # Correct pose for center tag (different physical size)
            pose_t = r.pose_t.copy()
            if r.tag_id == CENTER_TAG_ID:
                scale = CENTER_TAG_SIZE / CORNER_TAG_SIZE
                pose_t = pose_t * scale

            cx, cy, x, y, z = draw_detection(frame, r, pose_t)

            tag_label = "CENTER" if r.tag_id == CENTER_TAG_ID else "CORNER"
            print(f"  [{tag_label}] ID:{r.tag_id:>2d}  |  "
                  f"X:{x:+.3f}  Y:{y:+.3f}  Z:{z:.3f} m  |  "
                  f"pixel:({cx},{cy})")

        # ── FPS ──
        frame_count += 1
        if frame_count % fps_interval == 0:
            fps_avg = fps_interval / (time.time() - interval_start)
            interval_start = time.time()

        cv2.putText(frame, f"FPS: {fps_avg:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"Tags: {len(results)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Stage 2 — Pose Estimation", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[DONE] Frames: {frame_count}  Avg FPS: {fps_avg:.1f}")


if __name__ == "__main__":
    main()
