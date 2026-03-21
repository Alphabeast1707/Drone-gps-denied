#!/usr/bin/env python3
"""
Camera Calibration for LifeCam HD-5000
=======================================
Captures checkerboard images and computes intrinsic parameters
(fx, fy, cx, cy) + distortion coefficients.

SETUP:
  1. Print a checkerboard pattern (or display on a large screen)
     Default: 9x6 inner corners (10x7 squares)
     Download: https://github.com/opencv/opencv/blob/master/doc/pattern.png

  2. Run this script on the Pi:
       python3 -u calibrate_camera.py

  3. Hold the checkerboard at various angles/distances.
     Press SPACE (or it auto-captures) to grab frames.
     Need at least 15 good frames from different angles.

  4. When done, press 'q' or Ctrl+C. It computes calibration
     and prints the values to paste into config.py.

Works HEADLESS over SSH — no display needed!

Usage:
    python3 -u calibrate_camera.py              # Interactive, auto-capture
    python3 -u calibrate_camera.py --images 20  # Capture 20 images then calibrate
"""

import cv2
import numpy as np
import time
import sys
import os
import signal
import argparse
import json

from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT,
    CAMERA_FPS, CAMERA_FOURCC,
)

# ── Checkerboard config ──
# Standard OpenCV checkerboard: count INNER corners
# A 10x7 squares board has 9x6 = 54 inner corners
BOARD_COLS = 9   # inner corners horizontally
BOARD_ROWS = 6   # inner corners vertically
SQUARE_SIZE_MM = 25.0  # mm per square (only matters for extrinsics, not intrinsics)

# ── Capture config ──
MIN_IMAGES = 12
DEFAULT_IMAGES = 20
AUTO_CAPTURE_INTERVAL = 2.0  # seconds between auto-captures
MIN_CORNER_MOVEMENT = 30     # pixels — board must move this much between captures

# ── Graceful exit ──
running = True
def signal_handler(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, signal_handler)


def board_center(corners):
    """Get center of detected board corners."""
    return np.mean(corners.reshape(-1, 2), axis=0)


def main():
    parser = argparse.ArgumentParser(description="Camera calibration for LifeCam HD-5000")
    parser.add_argument("--images", type=int, default=DEFAULT_IMAGES,
                        help=f"Number of images to capture (min {MIN_IMAGES})")
    parser.add_argument("--cols", type=int, default=BOARD_COLS,
                        help="Inner corners horizontally")
    parser.add_argument("--rows", type=int, default=BOARD_ROWS,
                        help="Inner corners vertically")
    parser.add_argument("--square", type=float, default=SQUARE_SIZE_MM,
                        help="Square size in mm")
    parser.add_argument("--outdir", type=str, default="calibration_images",
                        help="Directory to save captured images")
    args = parser.parse_args()

    board_cols = args.cols
    board_rows = args.rows
    square_size = args.square / 1000.0  # convert to meters
    target_images = max(args.images, MIN_IMAGES)

    print("=" * 70)
    print("  CAMERA CALIBRATION — LifeCam HD-5000")
    print("=" * 70)
    print(f"\n  Checkerboard:  {board_cols}x{board_rows} inner corners")
    print(f"                 ({board_cols+1}x{board_rows+1} squares)")
    print(f"  Square size:   {args.square:.1f} mm")
    print(f"  Target images: {target_images}")
    print(f"  Auto-capture every {AUTO_CAPTURE_INTERVAL}s when board detected")
    print()

    # ── Create output directory ──
    os.makedirs(args.outdir, exist_ok=True)

    # ── 3D object points (same for all images) ──
    objp = np.zeros((board_rows * board_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    objp *= square_size

    obj_points = []  # 3D points in real-world space
    img_points = []  # 2D points in image plane

    # ── Open camera ──
    print("[1/3] Opening camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    fourcc = cv2.VideoWriter_fourcc(*CAMERA_FOURCC)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("[ERROR] Cannot open camera!")
        sys.exit(1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"  ✅ Camera opened: {w}x{h}")

    # ── Capture loop ──
    print(f"\n[2/3] Capturing checkerboard images...")
    print(f"  Hold checkerboard in front of camera at different:")
    print(f"    - Distances (near, mid, far)")
    print(f"    - Angles (tilted left/right/up/down)")
    print(f"    - Positions (all 4 quadrants + center of frame)")
    print(f"\n  Auto-capturing when board is detected and has moved enough.")
    print(f"  Press Ctrl+C when done (or wait for {target_images} captures).\n")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    captured = 0
    frame_count = 0
    last_capture_time = 0
    last_center = None
    fps = 0.0
    fps_start = time.time()
    fps_count = 0

    while running and captured < target_images:
        ret, frame = cap.read()
        if not ret:
            continue

        frame_count += 1
        fps_count += 1
        if fps_count >= 10:
            fps = fps_count / (time.time() - fps_start) if (time.time() - fps_start) > 0 else 0
            fps_start = time.time()
            fps_count = 0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find checkerboard
        found, corners = cv2.findChessboardCorners(
            gray, (board_cols, board_rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        )

        now = time.time()

        if found:
            # Refine corner positions to sub-pixel accuracy
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            center = board_center(corners2)

            # Check if board has moved enough since last capture
            moved_enough = True
            if last_center is not None:
                dist = np.linalg.norm(center - last_center)
                if dist < MIN_CORNER_MOVEMENT:
                    moved_enough = False

            time_ok = (now - last_capture_time) >= AUTO_CAPTURE_INTERVAL

            if time_ok and moved_enough:
                # Save image
                img_path = os.path.join(args.outdir, f"calib_{captured:03d}.png")
                cv2.imwrite(img_path, frame)

                obj_points.append(objp)
                img_points.append(corners2)
                captured += 1
                last_capture_time = now
                last_center = center.copy()

                # Compute board coverage info
                corners_flat = corners2.reshape(-1, 2)
                min_x, min_y = corners_flat.min(axis=0)
                max_x, max_y = corners_flat.max(axis=0)
                cx, cy = center

                print(
                    f"  📸 [{captured:2d}/{target_images}]  "
                    f"board at ({cx:5.0f},{cy:5.0f})  "
                    f"size {max_x-min_x:.0f}x{max_y-min_y:.0f}px  "
                    f"FPS={fps:.1f}"
                )
            else:
                # Board detected but not captured yet
                if frame_count % 30 == 0:
                    reason = "waiting" if not time_ok else "move board more"
                    print(
                        f"  ... board found at ({center[0]:5.0f},{center[1]:5.0f})  "
                        f"[{reason}]  {captured}/{target_images}  FPS={fps:.1f}"
                    )
        else:
            if frame_count % 50 == 0:
                print(
                    f"  ... no board detected  "
                    f"{captured}/{target_images}  FPS={fps:.1f}"
                )

    cap.release()

    if captured < MIN_IMAGES:
        print(f"\n[ERROR] Only {captured} images captured (need at least {MIN_IMAGES}).")
        print("  Run again and capture more images with the board at different positions/angles.")
        sys.exit(1)

    # ── Calibrate ──
    print(f"\n[3/3] Calibrating with {captured} images...")
    print("  This may take 10-30 seconds on Pi...")

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None
    )

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # ── Compute reprojection error ──
    total_error = 0
    for i in range(len(obj_points)):
        imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    mean_error = total_error / len(obj_points)

    # ── Print results ──
    print("\n" + "=" * 70)
    print("  CALIBRATION RESULTS")
    print("=" * 70)
    print(f"\n  Reprojection error: {mean_error:.4f} px  (good if < 0.5)")
    print(f"\n  Camera Matrix:")
    print(f"    fx = {fx:.2f}")
    print(f"    fy = {fy:.2f}")
    print(f"    cx = {cx:.2f}")
    print(f"    cy = {cy:.2f}")
    print(f"\n  Distortion Coefficients:")
    print(f"    k1 = {dist_coeffs[0][0]:.6f}")
    print(f"    k2 = {dist_coeffs[0][1]:.6f}")
    print(f"    p1 = {dist_coeffs[0][2]:.6f}")
    print(f"    p2 = {dist_coeffs[0][3]:.6f}")
    print(f"    k3 = {dist_coeffs[0][4]:.6f}")

    # ── Print config.py update ──
    print("\n" + "-" * 70)
    print("  PASTE THIS INTO config.py (replace the approximate values):")
    print("-" * 70)
    print(f"""
# ──────────────────────────────────────────────
# Camera intrinsic parameters (CALIBRATED)
# Calibrated on {time.strftime('%Y-%m-%d %H:%M')} with {captured} images
# Reprojection error: {mean_error:.4f} px
# ──────────────────────────────────────────────
CAMERA_FX = {fx:.2f}
CAMERA_FY = {fy:.2f}
CAMERA_CX = {cx:.2f}
CAMERA_CY = {cy:.2f}
CAMERA_PARAMS = [CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY]

# Distortion coefficients [k1, k2, p1, p2, k3]
CAMERA_DIST_COEFFS = [{dist_coeffs[0][0]:.6f}, {dist_coeffs[0][1]:.6f}, {dist_coeffs[0][2]:.6f}, {dist_coeffs[0][3]:.6f}, {dist_coeffs[0][4]:.6f}]
""")

    # ── Compare with old defaults ──
    print("-" * 70)
    print("  COMPARISON WITH OLD DEFAULTS:")
    print("-" * 70)
    print(f"    fx:  600.00 → {fx:.2f}  (Δ = {fx - 600:.2f})")
    print(f"    fy:  600.00 → {fy:.2f}  (Δ = {fy - 600:.2f})")
    print(f"    cx:  320.00 → {cx:.2f}  (Δ = {cx - 320:.2f})")
    print(f"    cy:  240.00 → {cy:.2f}  (Δ = {cy - 240:.2f})")

    # ── Explain impact ──
    print(f"\n  IMPACT ON YOUR POSE:")
    if abs(cx - 320) > 5 or abs(cy - 240) > 5:
        print(f"    ⚠️  cx/cy off by ({cx-320:+.1f}, {cy-240:+.1f}) px from center")
        print(f"        This was causing X/Y drift that scales with distance!")
    if abs(fx - 600) > 30 or abs(fy - 600) > 30:
        print(f"    ⚠️  fx/fy differ from 600 by ({fx-600:+.1f}, {fy-600:+.1f})")
        print(f"        This was causing Z distance scale error!")

    # ── Save calibration data ──
    calib_data = {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.tolist(),
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "image_size": [w, h],
        "reprojection_error": mean_error,
        "num_images": captured,
        "date": time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    calib_file = "camera_calibration.json"
    with open(calib_file, "w") as f:
        json.dump(calib_data, f, indent=2)
    print(f"\n  💾 Full calibration saved to {calib_file}")

    # Also save as numpy
    np_file = "camera_calibration.npz"
    np.savez(np_file,
             camera_matrix=camera_matrix,
             dist_coeffs=dist_coeffs,
             rvecs=np.array(rvecs, dtype=object),
             tvecs=np.array(tvecs, dtype=object))
    print(f"  💾 NumPy calibration saved to {np_file}")

    print("\n" + "=" * 70)
    print("  ✅ DONE — Update config.py with the values above!")
    print("=" * 70)


if __name__ == "__main__":
    main()
