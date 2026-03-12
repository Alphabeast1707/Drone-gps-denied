# 🚁 GPS-Denied Drone Vision System

**ISRO Drone Challenge — Precision Landing with AprilTags**

A high-speed computer vision pipeline for autonomous drone landing on Raspberry Pi 4.

---

## 📊 Architecture

```
            ┌───────────────────┐
            │  Flight Controller │
            │  (PX4 / ArduPilot) │
            └────────↑──────────┘
                     │ MAVLink
            ┌────────┴──────────┐
            │  Raspberry Pi 4   │
            │                   │
            │  Camera → Detect  │
            │      → Pose Est.  │
            │      → Landing    │
            └───────────────────┘
```

## 🗂️ Project Files

| File | Stage | Description |
|------|-------|-------------|
| `config.py` | — | All tunable parameters |
| `detect_tag.py` | 1 | Basic AprilTag detection + FPS |
| `pose_estimation.py` | 2 | 6-DOF pose (X, Y, Z distance) |
| `landing_controller.py` | — | Full multi-tag landing system |
| `visual_odometry.py` | 3 | ORB feature tracking |
| `setup_pi.sh` | — | Raspberry Pi setup script |

---

## 🚀 Quick Start (on Raspberry Pi)

### 1. Setup
```bash
chmod +x setup_pi.sh
./setup_pi.sh
```

Or manually:
```bash
sudo apt update && sudo apt upgrade
sudo apt install git cmake build-essential python3-opencv python3-pip libatlas-base-dev
pip3 install -r requirements.txt
```

### 2. Enable Camera
```bash
sudo raspi-config
# → Interface → Camera → Enable
# Reboot
```

### 3. Print AprilTags
- Family: **tag36h11**
- Print IDs 1, 2, 3, 4 at **15 cm** (150 mm) size — corner tags
- Print ID 5 at **10 cm** (100 mm) size — center landing tag
- Download from: https://github.com/AprilRobotics/apriltag-imgs

### 4. Run stages in order

**Stage 1 — Detection:**
```bash
python3 detect_tag.py
```

**Stage 2 — Pose estimation:**
```bash
python3 pose_estimation.py
```

**Full landing controller:**
```bash
python3 landing_controller.py
```

**Visual odometry (no tags needed):**
```bash
python3 visual_odometry.py
```

---

## 🎯 Landing Pad / Base Station Layout

From engineering drawing — total pad ~688.91 mm:

```
         150mm          388.91mm         150mm
       ┌──────┐                        ┌──────┐
       │Tag 1 │                        │Tag 2 │  150mm
       │15×15 │                        │15×15 │
       └──────┘                        └──────┘
                    ╭─── R275 ───╮
                   ╭─ R163.48 ─╮
                   │    ┌────┐  │
                   │    │Tag5│  │
                   │    │10cm│  │
                   │    └────┘  │
                   ╰────────────╯
                    ╰───────────╯
       ┌──────┐                        ┌──────┐
       │Tag 3 │                        │Tag 4 │  150mm
       │15×15 │                        │15×15 │
       └──────┘                        └──────┘
```

- **Tags 1–4** (15 cm): Guide drone toward center from far away
- **Tag 5** (10 cm): Precision landing (detected up close)
- **Two concentric circles**: R163.48 mm and R275 mm

---

## ⚡ Performance Targets (Pi 4)

| Mode | FPS |
|------|-----|
| Default AprilTag | 15–20 |
| Decimate 2 + 4 threads | **35–60** |
| Decimate 2 + ROI tracking | **50–80** |
| Decimate 3 (if tags > 20cm) | 80–120 |

> Note: With 15cm corner tags, `decimate=2` is recommended for reliable detection.

---

## 🔧 Key Parameters (config.py)

| Parameter | Default | Effect |
|-----------|---------|--------|
| `TAG_QUAD_DECIMATE` | 2.0 | Lower for 15cm tags (need resolution) |
| `TAG_NTHREADS` | 4 | Use all Pi cores |
| `TAG_REFINE_EDGES` | True | On for smaller tags (better accuracy) |
| `ROI_ENABLED` | True | Track near last position |
| `ROI_MARGIN` | 180 px | Sized for ~69cm pad |
| `CORNER_TAG_SIZE` | 0.15 m | Physical corner tag size |
| `CENTER_TAG_SIZE` | 0.10 m | Physical center tag size |

---

## 📡 Future Integration

When you get a Pixhawk / flight controller:

```
Raspberry Pi  ──MAVLink──>  PX4/ArduPilot
     │
     └── landing_controller.py sends:
           • target position (x, y, z)
           • landing commands
```

Use `pymavlink` or `MAVSDK` to send commands.

---

## 🧠 5-Stage Roadmap

1. ✅ **AprilTag Detection** — `detect_tag.py`
2. ✅ **Pose Estimation** — `pose_estimation.py`
3. ✅ **Visual Odometry** — `visual_odometry.py`
4. 🔲 **SLAM** — ORB-SLAM2 / OpenVSLAM
5. 🔲 **Autonomous Navigation** — Full path planning

---

## 📝 Tips for Competition

- **Lighting matters more than algorithms** — use high shutter speed
- **Print tags on matte paper** — avoid glare
- **Use global shutter camera** if possible (no motion blur)
- **Test outdoors** early — sunlight changes everything
- **Log everything** — the CSV telemetry log is your debugging tool

---

## 📚 References

- [AprilTag paper](https://april.eecs.umich.edu/papers/details.php?name=olson2011tags)
- [pupil-apriltags](https://github.com/pupil-labs/apriltags)
- [PX4 precision landing](https://docs.px4.io/main/en/advanced_features/precland.html)
- [tag36h11 images](https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11)
