"""
Landing Controller — Multi-Tag Precision Landing
==================================================
Implements the full 2-stage landing logic:

  Stage A:  Corner tags (ID 1-4) guide the drone toward the pad center.
  Stage B:  Center tag (ID 5) triggers precision descent + landing.

Features:
  • ROI tracking   — only searches near last known tag for huge FPS boost
  • Telemetry log  — saves cx, cy, distance to CSV
  • Visual HUD     — shows state, commands, distance

Landing pad layout:
        Tag1          Tag2
          □------------□
          |            |
          |     □      |
          |   Tag5     |
          |  (center)  |
          □------------□
        Tag3          Tag4

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
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT,
    CAMERA_PARAMS,
    TAG_FAMILY, TAG_NTHREADS, TAG_QUAD_DECIMATE,
    TAG_QUAD_SIGMA, TAG_REFINE_EDGES, TAG_DECODE_SHARPENING,
    CORNER_TAG_IDS, CENTER_TAG_ID,
    CORNER_TAG_SIZE, CENTER_TAG_SIZE,
    CENTER_THRESHOLD_PX,
    DESCEND_ALTITUDE, SLOW_DESCENT_ALTITUDE, LAND_ALTITUDE,
    ROI_ENABLED, ROI_MARGIN, ROI_LOST_FRAMES,
    LOG_FILE, LOG_ENABLED,
)

# ──────────────────────────────────────────────
# State machine
# ──────────────────────────────────────────────
STATE_SEARCHING = "SEARCHING"         # Looking for any tag
STATE_GUIDING = "GUIDING"             # Corner tags visible, centering
STATE_LANDING = "PRECISION LANDING"   # Center tag visible, descending
STATE_LANDED = "LANDED"               # Touchdown


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

        # ROI tracking
        self.roi_active = False
        self.roi_x = 0
        self.roi_y = 0
        self.lost_frames = 0

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

    # ── Setup helpers ──────────────────────────

    def _setup_camera(self):
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, 90)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print("[ERROR] Cannot open camera.")
            exit(1)
        print(f"[CAMERA] {int(cap.get(3))}x{int(cap.get(4))}")
        return cap

    def _setup_log(self):
        self.log_file = open(LOG_FILE, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["timestamp", "state", "tag_id",
                                  "pixel_cx", "pixel_cy",
                                  "pose_x", "pose_y", "pose_z",
                                  "command", "fps"])
        print(f"[LOG] Writing telemetry to {LOG_FILE}")

    # ── ROI cropping ──────────────────────────

    def _get_roi(self, gray):
        """Return cropped grayscale region and its offset, or full frame."""
        if not ROI_ENABLED or not self.roi_active:
            return gray, 0, 0

        x1 = max(0, self.roi_x - ROI_MARGIN)
        y1 = max(0, self.roi_y - ROI_MARGIN)
        x2 = min(gray.shape[1], self.roi_x + ROI_MARGIN)
        y2 = min(gray.shape[0], self.roi_y + ROI_MARGIN)

        roi = gray[y1:y2, x1:x2]
        return roi, x1, y1

    # ── Detection ─────────────────────────────

    def detect(self, gray):
        """Detect tags, using ROI if available."""
        roi, ox, oy = self._get_roi(gray)

        results = self.detector.detect(
            roi,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=CORNER_TAG_SIZE,
        )

        # Offset centers back to full-frame coordinates
        for r in results:
            r.center = (r.center[0] + ox, r.center[1] + oy)
            for i in range(4):
                r.corners[i][0] += ox
                r.corners[i][1] += oy

        # Update ROI state
        if len(results) > 0:
            # Track the best tag (prefer center tag)
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
                self.roi_active = False  # Fall back to full-frame scan

        return results

    # ── Control logic ─────────────────────────

    def update_state(self, results):
        """2-stage landing state machine."""
        center_det = None
        corner_dets = []

        for r in results:
            if r.tag_id == CENTER_TAG_ID:
                center_det = r
            elif r.tag_id in CORNER_TAG_IDS:
                corner_dets.append(r)

        # ── STAGE B: Center tag visible → precision landing ──
        if center_det is not None and center_det.pose_t is not None:
            self.state = STATE_LANDING

            # Correct pose for smaller center tag
            scale = CENTER_TAG_SIZE / CORNER_TAG_SIZE
            pose_t = center_det.pose_t * scale
            x, y, z = pose_t.flatten()

            cx = int(center_det.center[0])
            cy = int(center_det.center[1])
            offset_x = cx - self.frame_cx
            offset_y = cy - self.frame_cy

            if z < LAND_ALTITUDE:
                self.command = "🛬 LAND"
                self.state = STATE_LANDED
            elif z < SLOW_DESCENT_ALTITUDE:
                self.command = "⬇ SLOW DESCEND"
            elif z < DESCEND_ALTITUDE:
                self.command = "⬇ DESCEND"
            else:
                # Still too high — also correct horizontal
                self.command = self._centering_command(offset_x, offset_y)
                self.command += " + DESCEND"

            self._log(CENTER_TAG_ID, cx, cy, x, y, z)
            return center_det

        # ── STAGE A: Corner tags → guide to center ──
        if len(corner_dets) > 0:
            self.state = STATE_GUIDING

            # Average center of all visible corner tags
            avg_cx = int(np.mean([r.center[0] for r in corner_dets]))
            avg_cy = int(np.mean([r.center[1] for r in corner_dets]))
            offset_x = avg_cx - self.frame_cx
            offset_y = avg_cy - self.frame_cy

            self.command = self._centering_command(offset_x, offset_y)
            self._log(corner_dets[0].tag_id, avg_cx, avg_cy, 0, 0, 0)
            return corner_dets[0]

        # ── No tags visible ──
        self.state = STATE_SEARCHING
        self.command = "🔍 SEARCHING..."
        return None

    def _centering_command(self, offset_x, offset_y):
        """Generate horizontal centering command."""
        cmd = ""
        if offset_x > CENTER_THRESHOLD_PX:
            cmd += "→ RIGHT "
        elif offset_x < -CENTER_THRESHOLD_PX:
            cmd += "← LEFT "
        else:
            cmd += "↔ CENTERED "

        if offset_y > CENTER_THRESHOLD_PX:
            cmd += "↓ BACKWARD"
        elif offset_y < -CENTER_THRESHOLD_PX:
            cmd += "↑ FORWARD"
        else:
            cmd += "↕ CENTERED"

        return cmd.strip()

    def _log(self, tag_id, cx, cy, x, y, z):
        if self.log_writer:
            self.log_writer.writerow([
                f"{time.time():.3f}", self.state, tag_id,
                cx, cy,
                f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                self.command, f"{self.fps:.1f}",
            ])

    # ── Drawing ───────────────────────────────

    def draw(self, frame, results):
        """Draw all tag detections + HUD."""
        for r in results:
            corners = r.corners.astype(int)
            color = (0, 0, 255) if r.tag_id == CENTER_TAG_ID else (0, 255, 0)
            for i in range(4):
                cv2.line(frame, tuple(corners[i]),
                         tuple(corners[(i + 1) % 4]), color, 2)
            cx, cy = int(r.center[0]), int(r.center[1])
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"ID:{r.tag_id}",
                        (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Show distance for tags with pose
            if r.pose_t is not None:
                scale = (CENTER_TAG_SIZE / CORNER_TAG_SIZE
                         if r.tag_id == CENTER_TAG_ID else 1.0)
                z = r.pose_t.flatten()[2] * scale
                cv2.putText(frame, f"Z:{z:.2f}m",
                            (cx + 10, cy + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)

        # ROI rectangle
        if self.roi_active and ROI_ENABLED:
            rx1 = max(0, self.roi_x - ROI_MARGIN)
            ry1 = max(0, self.roi_y - ROI_MARGIN)
            rx2 = min(self.frame_w, self.roi_x + ROI_MARGIN)
            ry2 = min(self.frame_h, self.roi_y + ROI_MARGIN)
            cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (255, 255, 0), 1)

        # Crosshair at frame center
        cv2.drawMarker(frame, (self.frame_cx, self.frame_cy),
                       (100, 100, 100), cv2.MARKER_CROSS, 20, 1)

        # HUD
        state_color = {
            STATE_SEARCHING: (0, 165, 255),
            STATE_GUIDING: (0, 255, 255),
            STATE_LANDING: (0, 200, 0),
            STATE_LANDED: (0, 255, 0),
        }.get(self.state, (255, 255, 255))

        cv2.putText(frame, f"STATE: {self.state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
        cv2.putText(frame, f"CMD:   {self.command}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, f"FPS:   {self.fps:.1f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(frame, f"Tags:  {len(results)}", (10, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    # ── Main loop ─────────────────────────────

    def run(self):
        print(f"\n[LANDING CONTROLLER] Started")
        print(f"  Corner tags: {CORNER_TAG_IDS}  ({CORNER_TAG_SIZE*100:.0f} cm)")
        print(f"  Center tag:  {CENTER_TAG_ID}  ({CENTER_TAG_SIZE*100:.0f} cm)")
        print(f"  ROI tracking: {'ON' if ROI_ENABLED else 'OFF'}")
        print(f"  Land altitude: {LAND_ALTITUDE} m")
        print(f"\n  Press ESC to quit.\n")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect
            results = self.detect(gray)

            # Update control state
            self.update_state(results)

            # Print to terminal
            if self.state != STATE_SEARCHING:
                print(f"  [{self.state}]  {self.command}")

            if self.state == STATE_LANDED:
                print("\n  ✅ LANDING COMPLETE\n")

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
                # Show landed frame for 3 seconds then exit
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
