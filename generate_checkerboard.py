#!/usr/bin/env python3
"""
Generate a checkerboard calibration pattern image.
10x7 squares = 9x6 inner corners (OpenCV standard).
"""
import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# Board config: 10 cols x 7 rows of squares → 9x6 inner corners
COLS = 10  # squares horizontally
ROWS = 7   # squares vertically
SQUARE_PX = 100  # pixels per square
MARGIN = 50      # white border

width = COLS * SQUARE_PX + 2 * MARGIN
height = ROWS * SQUARE_PX + 2 * MARGIN

# Create white image
img = np.ones((height, width), dtype=np.uint8) * 255

# Draw black squares
for r in range(ROWS):
    for c in range(COLS):
        if (r + c) % 2 == 0:
            x1 = MARGIN + c * SQUARE_PX
            y1 = MARGIN + r * SQUARE_PX
            x2 = x1 + SQUARE_PX
            y2 = y1 + SQUARE_PX
            img[y1:y2, x1:x2] = 0

out_path = "apriltags_display/checkerboard_9x6.png"

if HAS_CV2:
    cv2.imwrite(out_path, img)
else:
    # Fallback: write raw PGM
    with open(out_path.replace('.png', '.pgm'), 'wb') as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        f.write(img.tobytes())
    out_path = out_path.replace('.png', '.pgm')

print(f"✅ Checkerboard saved: {out_path}")
print(f"   {COLS}x{ROWS} squares = {COLS-1}x{ROWS-1} inner corners")
print(f"   Image size: {width}x{height} px")
print(f"   Square size: {SQUARE_PX}px")
print(f"\nDisplay this FULLSCREEN on your Mac, then point the Pi camera at it.")
print(f"Run on Pi:  python3 -u calibrate_camera.py")
