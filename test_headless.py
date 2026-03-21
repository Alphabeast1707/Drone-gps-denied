"""
Headless AprilTag Test — for SSH (no display needed)
=====================================================
Runs detection without opening any GUI window.
Prints tag IDs, positions, X/Y/Z pose, pixel offset from
frame center, and estimated pad center offset.

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
import os
import ctypes

# ── Suppress stderr from C library ("more than one new minima") ──
# The apriltag C pose solver prints warnings to stderr that we can't
# control from Python. Redirect fd 2 to /dev/null during detection.
_original_stderr_fd = None
_devnull_fd = None

def _suppress_stderr():
    """Redirect C-level stderr to /dev/null."""
    global _original_stderr_fd, _devnull_fd
    sys.stderr.flush()
    _original_stderr_fd = os.dup(2)
    _devnull_fd = os.open(os.devnull, os.O_WRONLY)
    os.dup2(_devnull_fd, 2)

def _restore_stderr():
    """Restore original stderr."""
    global _original_stderr_fd, _devnull_fd
    os.dup2(_original_stderr_fd, 2)
    os.close(_original_stderr_fd)
    os.close(_devnull_fd)
    _original_stderr_fd = None
    _devnull_fd = None


from pupil_apriltags import Detector
from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, CAMERA_FOURCC,
    CAMERA_BACKEND,
    CAMERA_PARAMS, CAMERA_FX, CAMERA_FY,
    TAG_FAMILY, TAG_NTHREADS, TAG_QUAD_DECIMATE,
    TAG_QUAD_SIGMA, TAG_REFINE_EDGES, TAG_DECODE_SHARPENING,
    CORNER_TAG_IDS, CENTER_TAG_ID,
    CORNER_TAG_SIZE, CENTER_TAG_SIZE,
    CORNER_OFFSETS,
)

# Graceful exit on Ctrl+C
running = True
def signal_handler(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, signal_handler)


def estimate_pad_center_px(corner_dets, center_det, frame_cx, frame_cy, cam_fx, cam_fy):
    """
    Estimate pad center in pixel coords from detected tags.
    Returns (pad_cx, pad_cy, method_str)
    """
    if center_det is not None:
        return int(center_det.center[0]), int(center_det.center[1]), "center-tag"

    if len(corner_dets) >= 2:
        avg_x = np.mean([r.center[0] for r in corner_dets])
        avg_y = np.mean([r.center[1] for r in corner_dets])
        return int(avg_x), int(avg_y), "%d-corners-avg" % len(corner_dets)

    if len(corner_dets) == 1:
        r = corner_dets[0]
        tid = r.tag_id
        if tid in CORNER_OFFSETS and r.pose_t is not None:
            ox_m, oy_m = CORNER_OFFSETS[tid]
            z = abs(r.pose_t.flatten()[2])
            if z > 0.05:
                px_off_x = int((ox_m / z) * cam_fx)
                px_off_y = int((oy_m / z) * cam_fy)
                return int(r.center[0]) - px_off_x, int(r.center[1]) - px_off_y, "1-corner-geom"
        return int(r.center[0]), int(r.center[1]), "1-corner-raw"

    return frame_cx, frame_cy, "none"


def main():
    global running

    print("=" * 72)
    print("  HEADLESS APRILTAG TEST (SSH mode) — Full Pose + Pad Center")
    print("=" * 72)

    # ── Camera ──
    print("\n[1/3] Opening camera...")
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
    frame_cx = w // 2
    frame_cy = h // 2
    print(f"  ✅ Camera opened: {w}x{h}  (center={frame_cx},{frame_cy})")

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
    print("  Point camera at base station image on screen.")
    print("  Press Ctrl+C to stop.\n")

    hdr = (
        f"  {'TAG':>4s}  {'TYPE':>7s}  {'PIX(cx,cy)':>13s}  "
        f"{'dX_px':>6s} {'dY_px':>6s}  "
        f"{'X(m)':>7s} {'Y(m)':>7s} {'Z(m)':>7s}  {'FPS':>5s}"
    )
    print("-" * 72)
    print(hdr)
    print("-" * 72)

    frame_count = 0
    fps = 0.0
    fps_interval = 10
    interval_start = time.time()
    tags_ever_seen = set()

    while running:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Suppress C stderr during detection (kills "more than one minima" spam)
        _suppress_stderr()
        try:
            results = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=CAMERA_PARAMS,
                tag_size=CORNER_TAG_SIZE,
            )
        finally:
            _restore_stderr()

        # Separate corner vs center detections
        corner_dets = []
        center_det = None
        for r in results:
            if r.tag_id == CENTER_TAG_ID:
                center_det = r
            elif r.tag_id in CORNER_TAG_IDS:
                corner_dets.append(r)

        # Print each detected tag
        for r in results:
            tags_ever_seen.add(r.tag_id)
            cx, cy = int(r.center[0]), int(r.center[1])
            dx_px = cx - frame_cx
            dy_px = cy - frame_cy

            if r.tag_id == CENTER_TAG_ID:
                tag_type = "CENTER"
                scale = CENTER_TAG_SIZE / CORNER_TAG_SIZE
            elif r.tag_id in CORNER_TAG_IDS:
                tag_type = "CORNER"
                scale = 1.0
            else:
                tag_type = "OTHER"
                scale = 1.0

            if r.pose_t is not None:
                x, y, z = r.pose_t.flatten()
                x *= scale
                y *= scale
                z_abs = abs(z * scale)
                pose_str = f"{x:+7.3f} {y:+7.3f} {z_abs:7.3f}"
            else:
                pose_str = "  ---     ---     ---  "

            print(
                f"  ID:{r.tag_id:>2d}  {tag_type:>7s}  ({cx:4d},{cy:4d})  "
                f"{dx_px:+5d}  {dy_px:+5d}  "
                f"{pose_str}  {fps:>5.1f}"
            )

        # Pad center estimation (when any tags visible)
        if len(results) > 0:
            pad_cx, pad_cy, method = estimate_pad_center_px(
                corner_dets, center_det, frame_cx, frame_cy, CAMERA_FX, CAMERA_FY
            )
            pad_dx = pad_cx - frame_cx
            pad_dy = pad_cy - frame_cy
            print(
                f"  >>> PAD CENTER: ({pad_cx:4d},{pad_cy:4d})  "
                f"offset=({pad_dx:+4d},{pad_dy:+4d})px  "
                f"[{method}]"
            )

        # FPS
        frame_count += 1
        if frame_count % fps_interval == 0:
            elapsed = time.time() - interval_start
            fps = fps_interval / elapsed if elapsed > 0 else 0
            interval_start = time.time()

            if len(results) == 0:
                print(f"  {'--- no tags ---':^50s}  {fps:>5.1f}")

    # ── Done ──
    cap.release()
    print("\n" + "=" * 72)
    print(f"  Total frames:  {frame_count}")
    print(f"  Final FPS:     {fps:.1f}")
    print(f"  Tags seen:     {sorted(tags_ever_seen) if tags_ever_seen else 'none'}")
    print("=" * 72)


if __name__ == "__main__":
    main()
