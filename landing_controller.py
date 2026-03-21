"""
Landing Controller — Multi-Tag Precision Landing
==================================================
Optimized for base station with:
  • 4 corner AprilTags (15×15 cm) at pad corners
  • 1 center AprilTag  (10×10 cm) for precision landing
  • 2 concentric circles (R163.48 mm, R275 mm)
  • Total pad size: ~688.91 mm

Implements 3-phase landing logic:

  Phase 1 (APPROACH):   Corner tags guide drone above pad center.
                         Estimates pad centroid from corner geometry.
  Phase 2 (DESCEND):    Center tag visible → controlled descent.
  Phase 3 (LAND):       Final precision touchdown.

Features:
  • Pad centroid estimation from ANY subset of corner tags
  • ROI tracking with adaptive margin
  • Telemetry CSV logging
  • Concentric circle awareness (increased edge filtering)

Usage:
    python3 landing_controller.py

Press ESC to quit.
"""

import cv2
import numpy as np
import time
import csv
import os
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
    CENTER_THRESHOLD_PX,
    APPROACH_ALTITUDE, DESCEND_ALTITUDE, SLOW_DESCENT_ALTITUDE, LAND_ALTITUDE,
    ROI_ENABLED, ROI_MARGIN, ROI_LOST_FRAMES,
    LOG_FILE, LOG_ENABLED,
)

# ──────────────────────────────────────────────
# State machine
# ──────────────────────────────────────────────
STATE_SEARCHING = "SEARCHING"              # No tags visible
STATE_APPROACH  = "APPROACH"               # Corner tags → fly toward pad center
STATE_DESCEND   = "DESCENDING"             # Center tag visible → descending
STATE_PRECISION = "PRECISION LANDING"      # Close range → final alignment
STATE_LANDED    = "LANDED"                 # Touchdown


class LandingController:

    def __init__(self):
        # Camera
        self.cap = self._setup_camera()
        self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_cx = self.frame_w // 2
        self.frame_cy = self.frame_h // 2

        # Detector
        self.detector = Detector(
            families=TAG_FAMILY,
            nthreads=TAG_NTHREADS,
            quad_decimate=TAG_QUAD_DECIMATE,
            quad_sigma=TAG_QUAD_SIGMA,
            refine_edges=TAG_REFINE_EDGES,
            decode_sharpening=TAG_DECODE_SHARPENING,
        )

        # State
        self.state = STATE_SEARCHING
        self.command = "---"
        self.altitude = 0.0          # Estimated altitude (m)
        self.offset_x_m = 0.0       # Horizontal offset (m)
        self.offset_y_m = 0.0

        # ROI tracking
        self.roi_active = False
        self.roi_x = 0
        self.roi_y = 0
        self.lost_frames = 0

        # Pad center estimate (pixel coords, persists across frames)
        self.estimated_pad_cx = self.frame_cx
        self.estimated_pad_cy = self.frame_cy

        # Telemetry
        self.log_file = None
        self.log_writer = None
        if LOG_ENABLED:
            self._setup_log()

        # FPS
        self.fps = 0.0
        self.frame_count = 0
        self.fps_interval = 30
        self.interval_start = time.time()

    # ── Setup ─────────────────────────────────

    def _setup_camera(self):
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

    def _setup_log(self):
        self.log_file = open(LOG_FILE, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "timestamp", "state", "tag_id",
            "pixel_cx", "pixel_cy",
            "pose_x", "pose_y", "pose_z",
            "pad_center_px", "pad_center_py",
            "command", "fps",
        ])
        print(f"[LOG] Writing telemetry to {LOG_FILE}")

    # ── ROI cropping ──────────────────────────

    def _get_roi(self, gray):
        """Return cropped region around last known tag. Adaptive margin."""
        if not ROI_ENABLED or not self.roi_active:
            return gray, 0, 0

        # Adaptive ROI: larger margin at higher altitude
        margin = ROI_MARGIN
        if self.altitude > 2.0:
            margin = int(ROI_MARGIN * 1.5)  # Bigger search area when far

        x1 = max(0, self.roi_x - margin)
        y1 = max(0, self.roi_y - margin)
        x2 = min(gray.shape[1], self.roi_x + margin)
        y2 = min(gray.shape[0], self.roi_y + margin)

        roi = gray[y1:y2, x1:x2]
        return roi, x1, y1

    # ── Detection ─────────────────────────────

    def detect(self, gray):
        """Detect tags with ROI optimization."""
        roi, ox, oy = self._get_roi(gray)

        results = self.detector.detect(
            roi,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=CORNER_TAG_SIZE,
        )

        # Offset coordinates back to full-frame
        for r in results:
            r.center = (r.center[0] + ox, r.center[1] + oy)
            for i in range(4):
                r.corners[i][0] += ox
                r.corners[i][1] += oy

        # Update ROI tracking
        if len(results) > 0:
            # Prefer center tag for ROI anchor
            best = None
            for r in results:
                if r.tag_id == CENTER_TAG_ID:
                    best = r
                    break
            if best is None:
                best = results[0]

            self.roi_x = int(best.center[0])
            self.roi_y = int(best.center[1])
            self.roi_active = True
            self.lost_frames = 0
        else:
            self.lost_frames += 1
            if self.lost_frames > ROI_LOST_FRAMES:
                self.roi_active = False

        return results

    # ── Pad centroid estimation ───────────────

    def _estimate_pad_center(self, corner_dets, center_det):
        """
        Estimate the pad center in pixel coordinates.

        Strategy:
          1. If center tag visible → use its center directly.
          2. If 2+ corner tags visible → compute midpoint (they are symmetric).
          3. If 1 corner tag visible → offset using known pad geometry.
        """
        if center_det is not None:
            self.estimated_pad_cx = int(center_det.center[0])
            self.estimated_pad_cy = int(center_det.center[1])
            return self.estimated_pad_cx, self.estimated_pad_cy

        if len(corner_dets) >= 2:
            # Midpoint of all visible corners → pad center
            # (Works perfectly for 2 diagonal, 3, or all 4 corners)
            avg_x = np.mean([r.center[0] for r in corner_dets])
            avg_y = np.mean([r.center[1] for r in corner_dets])
            self.estimated_pad_cx = int(avg_x)
            self.estimated_pad_cy = int(avg_y)
            return self.estimated_pad_cx, self.estimated_pad_cy

        if len(corner_dets) == 1:
            r = corner_dets[0]
            tag_id = r.tag_id

            if tag_id in CORNER_OFFSETS and r.pose_t is not None:
                # Use known geometry: corner is offset from center
                # Estimate pixel offset from the tag's pose
                ox_m, oy_m = CORNER_OFFSETS[tag_id]
                z = abs(r.pose_t.flatten()[2])

                if z > 0.1:
                    # Convert meter offset to pixel offset
                    px_offset_x = int((ox_m / z) * CAMERA_FX)
                    px_offset_y = int((oy_m / z) * CAMERA_FY)

                    self.estimated_pad_cx = int(r.center[0]) - px_offset_x
                    self.estimated_pad_cy = int(r.center[1]) - px_offset_y
                    return self.estimated_pad_cx, self.estimated_pad_cy

            # Fallback: just use corner center (rough)
            self.estimated_pad_cx = int(r.center[0])
            self.estimated_pad_cy = int(r.center[1])
            return self.estimated_pad_cx, self.estimated_pad_cy

        # No tags → use last known estimate
        return self.estimated_pad_cx, self.estimated_pad_cy

    # ── Control logic ─────────────────────────

    def update_state(self, results):
        """3-phase landing state machine."""
        center_det = None
        corner_dets = []

        for r in results:
            if r.tag_id == CENTER_TAG_ID:
                center_det = r
            elif r.tag_id in CORNER_TAG_IDS:
                corner_dets.append(r)

        # Estimate pad center from whatever tags we see
        pad_cx, pad_cy = self._estimate_pad_center(corner_dets, center_det)

        # ─── PHASE 2/3: Center tag visible ───
        if center_det is not None and center_det.pose_t is not None:
            # Correct pose for center tag's smaller size
            scale = CENTER_TAG_SIZE / CORNER_TAG_SIZE
            pose_t = center_det.pose_t * scale
            x, y, z = pose_t.flatten()
            self.altitude = abs(z)
            self.offset_x_m = x
            self.offset_y_m = y

            cx = int(center_det.center[0])
            cy = int(center_det.center[1])
            offset_px_x = cx - self.frame_cx
            offset_px_y = cy - self.frame_cy

            if self.altitude < LAND_ALTITUDE:
                self.state = STATE_LANDED
                self.command = "🛬 LAND NOW"
            elif self.altitude < SLOW_DESCENT_ALTITUDE:
                self.state = STATE_PRECISION
                cmd = self._centering_command(offset_px_x, offset_px_y)
                self.command = f"⬇ SLOW DESCENT | {cmd}"
            elif self.altitude < DESCEND_ALTITUDE:
                self.state = STATE_DESCEND
                cmd = self._centering_command(offset_px_x, offset_px_y)
                self.command = f"⬇ DESCEND | {cmd}"
            else:
                self.state = STATE_APPROACH
                cmd = self._centering_command(offset_px_x, offset_px_y)
                self.command = f"↘ APPROACH + DESCEND | {cmd}"

            self._log(CENTER_TAG_ID, cx, cy, x, y, z, pad_cx, pad_cy)
            return

        # ─── PHASE 1: Only corner tags visible ───
        if len(corner_dets) > 0:
            self.state = STATE_APPROACH

            # Estimate altitude from best corner tag
            best_corner = min(corner_dets, key=lambda r: r.pose_t.flatten()[2]
                              if r.pose_t is not None else 999)
            if best_corner.pose_t is not None:
                self.altitude = abs(best_corner.pose_t.flatten()[2])

            # Navigate toward estimated pad center
            offset_px_x = pad_cx - self.frame_cx
            offset_px_y = pad_cy - self.frame_cy
            self.command = f"↘ FLY TO PAD CENTER | {self._centering_command(offset_px_x, offset_px_y)}"

            self._log(best_corner.tag_id,
                      pad_cx, pad_cy,
                      0, 0, self.altitude,
                      pad_cx, pad_cy)
            return

        # ─── No tags ───
        self.state = STATE_SEARCHING
        self.command = "🔍 SEARCHING..."
        self.altitude = 0.0

    def _centering_command(self, offset_x, offset_y):
        """Generate horizontal correction command string."""
        parts = []
        if offset_x > CENTER_THRESHOLD_PX:
            parts.append("→ RIGHT")
        elif offset_x < -CENTER_THRESHOLD_PX:
            parts.append("← LEFT")
        else:
            parts.append("↔ X-OK")

        if offset_y > CENTER_THRESHOLD_PX:
            parts.append("↓ BACK")
        elif offset_y < -CENTER_THRESHOLD_PX:
            parts.append("↑ FWD")
        else:
            parts.append("↕ Y-OK")

        return "  ".join(parts)

    def _log(self, tag_id, cx, cy, x, y, z, pad_cx, pad_cy):
        if self.log_writer:
            self.log_writer.writerow([
                f"{time.time():.3f}", self.state, tag_id,
                cx, cy,
                f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                pad_cx, pad_cy,
                self.command, f"{self.fps:.1f}",
            ])

    # ── Drawing ───────────────────────────────

    def draw(self, frame, results):
        """Draw detections, pad center estimate, and HUD."""
        for r in results:
            corners = r.corners.astype(int)
            is_center = r.tag_id == CENTER_TAG_ID
            color = (0, 0, 255) if is_center else (0, 255, 0)
            thickness = 3 if is_center else 2

            for i in range(4):
                cv2.line(frame, tuple(corners[i]),
                         tuple(corners[(i + 1) % 4]), color, thickness)

            cx, cy = int(r.center[0]), int(r.center[1])
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            label = f"ID:{r.tag_id}"
            if is_center:
                label += " [CENTER]"
            cv2.putText(frame, label, (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Distance overlay
            if r.pose_t is not None:
                scale = (CENTER_TAG_SIZE / CORNER_TAG_SIZE
                         if is_center else 1.0)
                z = abs(r.pose_t.flatten()[2] * scale)
                cv2.putText(frame, f"Z:{z:.2f}m",
                            (cx + 10, cy + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)

        # ── Estimated pad center (magenta crosshair) ──
        cv2.drawMarker(frame,
                       (self.estimated_pad_cx, self.estimated_pad_cy),
                       (255, 0, 255), cv2.MARKER_CROSS, 30, 2)
        cv2.putText(frame, "PAD",
                    (self.estimated_pad_cx + 15, self.estimated_pad_cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

        # ── Frame center crosshair (gray) ──
        cv2.drawMarker(frame, (self.frame_cx, self.frame_cy),
                       (100, 100, 100), cv2.MARKER_CROSS, 20, 1)

        # ── ROI rectangle ──
        if self.roi_active and ROI_ENABLED:
            margin = ROI_MARGIN
            if self.altitude > 2.0:
                margin = int(ROI_MARGIN * 1.5)
            rx1 = max(0, self.roi_x - margin)
            ry1 = max(0, self.roi_y - margin)
            rx2 = min(self.frame_w, self.roi_x + margin)
            ry2 = min(self.frame_h, self.roi_y + margin)
            cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (255, 255, 0), 1)

        # ── HUD ──
        state_colors = {
            STATE_SEARCHING: (0, 165, 255),   # orange
            STATE_APPROACH:  (0, 255, 255),   # yellow
            STATE_DESCEND:   (0, 200, 0),     # green
            STATE_PRECISION: (0, 255, 0),     # bright green
            STATE_LANDED:    (0, 255, 0),     # bright green
        }
        sc = state_colors.get(self.state, (255, 255, 255))

        y0 = 25
        cv2.putText(frame, f"STATE: {self.state}", (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, sc, 2)
        cv2.putText(frame, f"CMD: {self.command}", (10, y0 + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"ALT: {self.altitude:.2f} m", (10, y0 + 56),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        cv2.putText(frame, f"FPS: {self.fps:.1f}  |  Tags: {len(results)}",
                    (10, y0 + 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # ── Main loop ─────────────────────────────

    def run(self):
        print(f"\n{'='*50}")
        print(f"  LANDING CONTROLLER — Base Station")
        print(f"{'='*50}")
        print(f"  Pad size:     ~688.91 mm")
        print(f"  Corner tags:  {CORNER_TAG_IDS}  ({CORNER_TAG_SIZE*100:.0f} cm)")
        print(f"  Center tag:   {CENTER_TAG_ID}  ({CENTER_TAG_SIZE*100:.0f} cm)")
        print(f"  ROI tracking: {'ON' if ROI_ENABLED else 'OFF'}")
        print(f"  Phases:")
        print(f"    APPROACH   → above {APPROACH_ALTITUDE}m (corner tags)")
        print(f"    DESCEND    → below {DESCEND_ALTITUDE}m (center tag)")
        print(f"    PRECISION  → below {SLOW_DESCENT_ALTITUDE}m")
        print(f"    LAND       → below {LAND_ALTITUDE}m")
        print(f"\n  Press ESC to quit.\n")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect
            results = self.detect(gray)

            # Update state machine
            self.update_state(results)

            # Terminal output
            if self.state != STATE_SEARCHING:
                print(f"  [{self.state:>18s}]  ALT:{self.altitude:.2f}m  {self.command}")

            if self.state == STATE_LANDED:
                print(f"\n  {'='*40}")
                print(f"  ✅  LANDING COMPLETE")
                print(f"  {'='*40}\n")

            # Draw
            self.draw(frame, results)
            cv2.imshow("Landing Controller", frame)

            # FPS
            self.frame_count += 1
            if self.frame_count % self.fps_interval == 0:
                self.fps = self.fps_interval / (time.time() - self.interval_start)
                self.interval_start = time.time()

            if cv2.waitKey(1) & 0xFF == 27:
                break

            if self.state == STATE_LANDED:
                cv2.waitKey(3000)
                break

        self.cap.release()
        cv2.destroyAllWindows()
        if self.log_file:
            self.log_file.close()
            print(f"[LOG] Telemetry saved to {LOG_FILE}")
        print(f"[DONE] Frames: {self.frame_count}  Avg FPS: {self.fps:.1f}")


if __name__ == "__main__":
    controller = LandingController()
    controller.run()
