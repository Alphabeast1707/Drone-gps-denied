"""
Stage 3 — Visual Odometry (Feature-based motion tracking)
==========================================================
Tracks camera motion between frames using ORB features.
Works even when no AprilTags are visible.

Pipeline:
    Frame N   → detect ORB features
    Frame N+1 → match features
    Compute essential matrix → extract rotation + translation

This gives relative motion (ΔX, ΔY, ΔZ) per frame.
Typical speed on Pi 4: 15–25 FPS.

Usage:
    python3 visual_odometry.py

Press ESC to quit.
"""

import cv2
import numpy as np
import time
from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT,
    CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY,
    VO_MAX_FEATURES, VO_MIN_MATCHES,
)


def setup_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("[ERROR] Cannot open camera.")
        exit(1)
    print(f"[CAMERA] {int(cap.get(3))}x{int(cap.get(4))}")
    return cap


def main():
    cap = setup_camera()

    # ORB feature detector
    orb = cv2.ORB_create(nfeatures=VO_MAX_FEATURES)

    # Brute-force matcher with Hamming distance (for binary descriptors)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Camera matrix
    K = np.array([
        [CAMERA_FX, 0,         CAMERA_CX],
        [0,         CAMERA_FY, CAMERA_CY],
        [0,         0,         1],
    ], dtype=np.float64)

    # Cumulative pose
    R_total = np.eye(3)
    t_total = np.zeros((3, 1))

    prev_gray = None
    prev_kp = None
    prev_des = None

    fps = 0.0
    frame_count = 0
    fps_interval = 20
    interval_start = time.time()

    print("\n[RUNNING] Visual Odometry started. Press ESC to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── Detect features ──
        kp, des = orb.detectAndCompute(gray, None)

        if prev_des is not None and des is not None and len(kp) > 0:
            # ── Match features ──
            matches = bf.match(prev_des, des)
            matches = sorted(matches, key=lambda m: m.distance)

            if len(matches) >= VO_MIN_MATCHES:
                # Extract matched keypoint coordinates
                pts1 = np.float32([prev_kp[m.queryIdx].pt for m in matches])
                pts2 = np.float32([kp[m.trainIdx].pt for m in matches])

                # ── Essential matrix ──
                E, mask = cv2.findEssentialMat(
                    pts1, pts2, K,
                    method=cv2.RANSAC,
                    prob=0.999,
                    threshold=1.0,
                )

                if E is not None:
                    # ── Recover rotation and translation ──
                    _, R, t, mask_pose = cv2.recoverPose(E, pts1, pts2, K)

                    # Accumulate pose
                    t_total = t_total + R_total @ t
                    R_total = R @ R_total

                    x, y, z = t_total.flatten()
                    print(f"  Position  X:{x:+.3f}  Y:{y:+.3f}  Z:{z:+.3f}")

                # ── Draw matches (top 30) ──
                draw_matches = matches[:30]
                for m in draw_matches:
                    pt1 = tuple(map(int, prev_kp[m.queryIdx].pt))
                    pt2 = tuple(map(int, kp[m.trainIdx].pt))
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 1)
                    cv2.circle(frame, pt2, 3, (0, 0, 255), -1)

        # Store for next frame
        prev_gray = gray
        prev_kp = kp
        prev_des = des

        # ── FPS ──
        frame_count += 1
        if frame_count % fps_interval == 0:
            fps = fps_interval / (time.time() - interval_start)
            interval_start = time.time()

        # ── HUD ──
        x, y, z = t_total.flatten()
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Features: {len(kp)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(frame, f"Pos X:{x:+.2f} Y:{y:+.2f} Z:{z:+.2f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 1)

        cv2.imshow("Stage 3 — Visual Odometry", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[DONE] Frames: {frame_count}  Avg FPS: {fps:.1f}")
    print(f"  Final position: X:{x:+.3f}  Y:{y:+.3f}  Z:{z:+.3f}")


if __name__ == "__main__":
    main()
