#!/usr/bin/env python3
"""
Generate Base Station Landing Pad — Complete Engineering Reference
===================================================================
Produces 3 images:
  1. base_station.png          — Full annotated reference (geometry, offsets, specs)
  2. base_station_printable.png — Clean for PRINTING at 1:1
  3. base_station_screen.png   — Compact for screen-test with Pi camera

Run:  python3 generate_base_station.py
"""

import cv2
import numpy as np
import os

# ═══════════════════════════════════════════════
# GROUND-TRUTH GEOMETRY (mm)
# ═══════════════════════════════════════════════
PAD_WIDTH_MM       = 688.91
GAP_BETWEEN_TAGS   = 388.91
CORNER_OFFSET_MM   = GAP_BETWEEN_TAGS / 2.0   # 194.455
CORNER_TAG_MM      = 150.0
CENTER_TAG_MM      = 100.0
INNER_CIRCLE_R_MM  = 163.48
OUTER_CIRCLE_R_MM  = 275.00

TAG_IDS = {1:"top-left", 2:"top-right", 3:"bottom-left", 4:"bottom-right", 5:"center"}

CORNER_OFFSETS_MM = {
    1: (-CORNER_OFFSET_MM, -CORNER_OFFSET_MM),
    2: (+CORNER_OFFSET_MM, -CORNER_OFFSET_MM),
    3: (-CORNER_OFFSET_MM, +CORNER_OFFSET_MM),
    4: (+CORNER_OFFSET_MM, +CORNER_OFFSET_MM),
}

# Rendering
PX_PER_MM     = 5
PAD_PX        = int(PAD_WIDTH_MM * PX_PER_MM)
MARGIN_PX     = int(80 * PX_PER_MM)
CORNER_TAG_PX = int(CORNER_TAG_MM * PX_PER_MM)
CENTER_TAG_PX = int(CENTER_TAG_MM * PX_PER_MM)
INNER_R_PX    = int(INNER_CIRCLE_R_MM * PX_PER_MM)
OUTER_R_PX    = int(OUTER_CIRCLE_R_MM * PX_PER_MM)
OFFSET_PX     = int(CORNER_OFFSET_MM * PX_PER_MM)

IMG_W = PAD_PX + 2 * MARGIN_PX
IMG_H = PAD_PX + 2 * MARGIN_PX + int(120 * PX_PER_MM)
CX = MARGIN_PX + PAD_PX // 2
CY = MARGIN_PX + PAD_PX // 2

TAGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "apriltags_display")

# Colors (BGR)
BLACK=(0,0,0); WHITE=(255,255,255); GRAY=(180,180,180); LTGRAY=(230,230,230)
DKGRAY=(100,100,100); RED=(0,0,220); BLUE=(220,100,0); GREEN=(0,160,0)
MAGENTA=(200,0,200); ORANGE=(0,140,255)


def load_tag(tid, sz):
    path = os.path.join(TAGS_DIR, "tag36h11_id%d.png" % tid)
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError("Missing: " + path)
    return cv2.resize(img, (sz, sz), interpolation=cv2.INTER_NEAREST)


def place_tag(canvas, tag, cx, cy):
    h, w = tag.shape[:2]
    x1, y1 = cx - w//2, cy - h//2
    if canvas.ndim == 3:
        canvas[y1:y1+h, x1:x1+w] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
    else:
        canvas[y1:y1+h, x1:x1+w] = tag


def dim_h(img, y, x1, x2, label, color, font, sc, th, above=True):
    tk = 15
    cv2.line(img, (x1, y), (x2, y), color, 2)
    cv2.line(img, (x1, y-tk), (x1, y+tk), color, 2)
    cv2.line(img, (x2, y-tk), (x2, y+tk), color, 2)
    if x2 - x1 > 80:
        cv2.arrowedLine(img, (x1+40, y), (x1, y), color, 2, tipLength=0.5)
        cv2.arrowedLine(img, (x2-40, y), (x2, y), color, 2, tipLength=0.5)
    tsz = cv2.getTextSize(label, font, sc, th)[0]
    tx = (x1+x2)//2 - tsz[0]//2
    ty = y - 12 if above else y + tsz[1] + 12
    cv2.putText(img, label, (tx, ty), font, sc, color, th)


def dim_v(img, x, y1, y2, label, color, font, sc, th, left=True):
    tk = 15
    cv2.line(img, (x, y1), (x, y2), color, 2)
    cv2.line(img, (x-tk, y1), (x+tk, y1), color, 2)
    cv2.line(img, (x-tk, y2), (x+tk, y2), color, 2)
    if y2 - y1 > 80:
        cv2.arrowedLine(img, (x, y1+40), (x, y1), color, 2, tipLength=0.5)
        cv2.arrowedLine(img, (x, y2-40), (x, y2), color, 2, tipLength=0.5)
    tsz = cv2.getTextSize(label, font, sc, th)[0]
    ty = (y1+y2)//2 + tsz[1]//2
    tx = x - tsz[0] - 15 if left else x + 15
    cv2.putText(img, label, (tx, ty), font, sc, color, th)


def build_annotated(ct, cc, pos):
    img = np.ones((IMG_H, IMG_W, 3), dtype=np.uint8) * 255
    px1, py1 = MARGIN_PX, MARGIN_PX
    px2, py2 = MARGIN_PX + PAD_PX, MARGIN_PX + PAD_PX
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Pad
    cv2.rectangle(img, (px1,py1), (px2,py2), LTGRAY, -1)
    cv2.rectangle(img, (px1,py1), (px2,py2), GRAY, 3)

    # Circles
    cv2.circle(img, (CX,CY), OUTER_R_PX, RED, 4)
    cv2.circle(img, (CX,CY), INNER_R_PX, BLUE, 4)

    # Dotted offset lines center->corners
    for tid, (tx, ty) in pos.items():
        for f in np.arange(0.0, 1.0, 0.03):
            px = int(CX + f*(tx-CX))
            py = int(CY + f*(ty-CY))
            cv2.circle(img, (px, py), 2, MAGENTA, -1)

    # Coordinate axes
    alen = int(60*PX_PER_MM)
    cv2.arrowedLine(img, (CX,CY), (CX+alen, CY), GREEN, 3, tipLength=0.08)
    cv2.arrowedLine(img, (CX,CY), (CX, CY+alen), ORANGE, 3, tipLength=0.08)
    cv2.putText(img, "+X", (CX+alen+10, CY+8), font, 0.9, GREEN, 2)
    cv2.putText(img, "+Y", (CX-30, CY+alen+30), font, 0.9, ORANGE, 2)
    cv2.putText(img, "Origin", (CX+15, CY-15), font, 0.6, DKGRAY, 2)

    # Place tags LAST
    for tid, (tx, ty) in pos.items():
        place_tag(img, ct[tid], tx, ty)
    place_tag(img, cc, CX, CY)

    # Tag labels
    for tid, (tx, ty) in pos.items():
        lbl = "ID %d" % tid
        tsz = cv2.getTextSize(lbl, font, 0.85, 2)[0]
        ly = ty - CORNER_TAG_PX//2 - 15 if "top" in TAG_IDS[tid] else ty + CORNER_TAG_PX//2 + tsz[1] + 15
        cv2.putText(img, lbl, (tx-tsz[0]//2, ly), font, 0.85, BLACK, 2)
        ox, oy = CORNER_OFFSETS_MM[tid]
        ps = "(%+.1f, %+.1f)mm" % (ox, oy)
        tsz2 = cv2.getTextSize(ps, font, 0.5, 1)[0]
        ly2 = ly - tsz[1] - 5 if "top" in TAG_IDS[tid] else ly + tsz[1] + 10
        cv2.putText(img, ps, (tx-tsz2[0]//2, ly2), font, 0.5, MAGENTA, 1)

    lbl5 = "ID 5 (center)"
    tsz = cv2.getTextSize(lbl5, font, 0.85, 2)[0]
    cv2.putText(img, lbl5, (CX-tsz[0]//2, CY+CENTER_TAG_PX//2+tsz[1]+18), font, 0.85, BLACK, 2)

    # Dimension lines
    # Total width
    dy = py2 + 40
    dim_h(img, dy, px1, px2, "688.91 mm (total pad)", BLACK, font, 0.75, 2, above=False)
    # Tag gap
    dy2 = dy + 50
    gx1 = CX - OFFSET_PX + CORNER_TAG_PX//2
    gx2 = CX + OFFSET_PX - CORNER_TAG_PX//2
    dim_h(img, dy2, gx1, gx2, "388.91 mm (gap)", DKGRAY, font, 0.6, 1, above=False)
    # Corner tag height
    tl_x, tl_y = pos[1]
    dim_v(img, px1-35, tl_y-CORNER_TAG_PX//2, tl_y+CORNER_TAG_PX//2, "150mm", BLACK, font, 0.65, 1, left=True)
    # Center tag height
    dim_v(img, CX+CENTER_TAG_PX//2+30, CY-CENTER_TAG_PX//2, CY+CENTER_TAG_PX//2, "100mm", BLACK, font, 0.65, 1, left=False)
    # Offset label
    mid_x = (CX + pos[2][0])//2
    mid_y = (CY + pos[2][1])//2
    cv2.putText(img, "194.455mm", (mid_x+15, mid_y-10), font, 0.55, MAGENTA, 1)

    # Circle radius labels
    a1 = np.radians(-30)
    ox1 = int(CX + OUTER_R_PX*np.cos(a1))
    oy1 = int(CY + OUTER_R_PX*np.sin(a1))
    cv2.line(img, (ox1,oy1), (ox1+100, oy1-60), RED, 2)
    cv2.putText(img, "R=275.00mm", (ox1+110, oy1-65), font, 0.7, RED, 2)
    cv2.line(img, (CX,CY), (ox1,oy1), RED, 1, cv2.LINE_AA)

    a2 = np.radians(30)
    ix1 = int(CX + INNER_R_PX*np.cos(a2))
    iy1 = int(CY + INNER_R_PX*np.sin(a2))
    cv2.line(img, (ix1,iy1), (ix1+100, iy1+60), BLUE, 2)
    cv2.putText(img, "R=163.48mm", (ix1+110, iy1+55), font, 0.65, BLUE, 2)
    cv2.line(img, (CX,CY), (ix1,iy1), BLUE, 1, cv2.LINE_AA)

    # Title
    title = "ISRO IROC - Landing Base Station - Engineering Reference"
    tsz = cv2.getTextSize(title, font, 1.1, 2)[0]
    cv2.putText(img, title, (CX-tsz[0]//2, MARGIN_PX//2), font, 1.1, BLACK, 2)

    # ── Legend bottom ──
    ly0 = py2 + 120
    lx = MARGIN_PX + 20
    sl = 0.6; tl = 1; lh = 30
    specs = [
        ("GEOMETRY:", BLACK, 2),
        ("  Pad:             688.91 x 688.91 mm  (square)", DKGRAY, 1),
        ("  Corner tags:     150 x 150 mm  (tag36h11  IDs 1-4)", DKGRAY, 1),
        ("  Center tag:      100 x 100 mm  (tag36h11  ID 5)", DKGRAY, 1),
        ("  Gap edge-edge:   388.91 mm   Center-center: 538.91 mm", DKGRAY, 1),
        ("  Inner circle:    R=163.48mm  D=326.96mm", BLUE, 1),
        ("  Outer circle:    R=275.00mm  D=550.00mm", RED, 1),
        ("", BLACK, 1),
        ("COORDINATE FRAME:", BLACK, 2),
        ("  Origin = Pad center = ID 5 center", DKGRAY, 1),
        ("  +X = Right    +Y = Down    +Z = Into ground", DKGRAY, 1),
        ("", BLACK, 1),
        ("TAG OFFSETS FROM ORIGIN (mm -> m):", BLACK, 2),
        ("  ID1 TL: (-194.455,-194.455) = (-0.19445,-0.19445)m", MAGENTA, 1),
        ("  ID2 TR: (+194.455,-194.455) = (+0.19445,-0.19445)m", MAGENTA, 1),
        ("  ID3 BL: (-194.455,+194.455) = (-0.19445,+0.19445)m", MAGENTA, 1),
        ("  ID4 BR: (+194.455,+194.455) = (+0.19445,+0.19445)m", MAGENTA, 1),
        ("  ID5  C: (  0.000,   0.000 ) = ( 0.00000, 0.00000)m", MAGENTA, 1),
    ]
    for i, (txt, col, tw) in enumerate(specs):
        if txt:
            cv2.putText(img, txt, (lx, ly0+i*lh), font, sl, col, tw)

    rx = CX + 100
    specs_r = [
        ("DETECTOR:", BLACK, 2),
        ("  Family: tag36h11   Threads: 4   Decimate: 2.0", DKGRAY, 1),
        ("  Refine edges: True   Sharpening: 0.25", DKGRAY, 1),
        ("", BLACK, 1),
        ("POSE ESTIMATION:", BLACK, 2),
        ("  Corner tag_size = 0.15m   Center tag_size = 0.10m", DKGRAY, 1),
        ("  Center Z correction: z *= (0.10/0.15)", DKGRAY, 1),
        ("  Camera: fx=fy=600  cx=320 cy=240  (CALIBRATE!)", RED, 1),
        ("", BLACK, 1),
        ("LANDING PHASES:", BLACK, 2),
        ("  APPROACH:  >3.0m  (corner tags -> pad centroid)", DKGRAY, 1),
        ("  DESCEND:   1.5-3.0m  (center tag visible)", DKGRAY, 1),
        ("  PRECISION: 0.2-0.5m  (slow, tight centering)", DKGRAY, 1),
        ("  LAND:      <0.20m  (touchdown trigger)", DKGRAY, 1),
        ("  Centering: 15px threshold", DKGRAY, 1),
        ("", BLACK, 1),
        ("PAD CENTROID ESTIMATION:", BLACK, 2),
        ("  4 corners: avg  |  2-3: midpoint  |  1: offset geom", DKGRAY, 1),
    ]
    for i, (txt, col, tw) in enumerate(specs_r):
        if txt:
            cv2.putText(img, txt, (rx, ly0+i*lh), font, sl, col, tw)

    return img


def build_printable(ct, cc):
    margin = int(20*PX_PER_MM)
    size = PAD_PX + 2*margin
    cx, cy = size//2, size//2
    img = np.ones((size, size, 3), dtype=np.uint8) * 255
    p1, p2 = margin, margin+PAD_PX
    cv2.rectangle(img, (p1,p1), (p2,p2), (250,250,250), -1)
    cv2.rectangle(img, (p1,p1), (p2,p2), (200,200,200), 2)
    cv2.circle(img, (cx,cy), OUTER_R_PX, BLACK, 3)
    cv2.circle(img, (cx,cy), INNER_R_PX, BLACK, 3)
    for tid, (dx, dy) in {1:(-1,-1),2:(1,-1),3:(-1,1),4:(1,1)}.items():
        place_tag(img, ct[tid], cx+dx*OFFSET_PX, cy+dy*OFFSET_PX)
    place_tag(img, cc, cx, cy)
    return img


def build_screen(ct, cc):
    s = 1200.0 / PAD_WIDTH_MM
    pad_s = int(PAD_WIDTH_MM*s)
    ms = int(50*s)
    isz = pad_s + 2*ms
    cx, cy = isz//2, isz//2
    img = np.ones((isz, isz, 3), dtype=np.uint8) * 255
    p1, p2 = ms, ms+pad_s
    cv2.rectangle(img, (p1,p1), (p2,p2), (242,242,242), -1)
    cv2.rectangle(img, (p1,p1), (p2,p2), (200,200,200), 2)
    cv2.circle(img, (cx,cy), int(OUTER_CIRCLE_R_MM*s), (0,0,180), 2)
    cv2.circle(img, (cx,cy), int(INNER_CIRCLE_R_MM*s), (180,80,0), 2)
    cs = max(50, int(CORNER_TAG_MM*s))
    zs = max(40, int(CENTER_TAG_MM*s))
    off = int(CORNER_OFFSET_MM*s)
    for tid, (dx,dy) in {1:(-1,-1),2:(1,-1),3:(-1,1),4:(1,1)}.items():
        place_tag(img, load_tag(tid, cs), cx+dx*off, cy+dy*off)
    place_tag(img, load_tag(5, zs), cx, cy)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "ISRO IROC Base Station (point Pi camera here)", (ms, ms-20), font, 0.7, BLACK, 2)
    for tid, (dx,dy) in {1:(-1,-1),2:(1,-1),3:(-1,1),4:(1,1)}.items():
        lbl = "ID%d" % tid
        cv2.putText(img, lbl, (cx+dx*off-20, cy+dy*off+cs//2+20), font, 0.5, DKGRAY, 1)
    cv2.putText(img, "ID5", (cx-15, cy+zs//2+20), font, 0.5, DKGRAY, 1)
    return img


def verify(path, label):
    from pupil_apriltags import Detector
    det = Detector(families="tag36h11", nthreads=4, quad_decimate=1.0, refine_edges=True)
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    results = det.detect(img)
    ids = sorted([r.tag_id for r in results])
    ok = ids == [1,2,3,4,5]
    status = "ALL 5 OK" if ok else "MISSING: %s" % str(set([1,2,3,4,5])-set(ids))
    print("  %-22s %s  %s" % (label, ids, status))
    for r in results:
        print("    ID=%d  center=(%6.1f,%6.1f)  margin=%5.1f  hamming=%d" % (
            r.tag_id, r.center[0], r.center[1], r.decision_margin, r.hamming))
    return ok


def main():
    print("="*65)
    print("  BASE STATION GENERATOR — ISRO IROC")
    print("="*65)

    print("\nLoading tags...")
    ct = {}
    for tid in [1,2,3,4]:
        ct[tid] = load_tag(tid, CORNER_TAG_PX)
        print("  ID %d: %dpx (150mm corner)" % (tid, CORNER_TAG_PX))
    cc = load_tag(5, CENTER_TAG_PX)
    print("  ID 5: %dpx (100mm center)" % CENTER_TAG_PX)

    pos = {
        1: (CX-OFFSET_PX, CY-OFFSET_PX),
        2: (CX+OFFSET_PX, CY-OFFSET_PX),
        3: (CX-OFFSET_PX, CY+OFFSET_PX),
        4: (CX+OFFSET_PX, CY+OFFSET_PX),
    }

    print("\nGenerating images...")
    img1 = build_annotated(ct, cc, pos)
    p1 = os.path.join(TAGS_DIR, "base_station.png")
    cv2.imwrite(p1, img1)
    print("  [1] base_station.png          %dx%d  (full engineering ref)" % (img1.shape[1], img1.shape[0]))

    img2 = build_printable(ct, cc)
    p2 = os.path.join(TAGS_DIR, "base_station_printable.png")
    cv2.imwrite(p2, img2)
    print("  [2] base_station_printable.png %dx%d  (clean, 1:1 print)" % (img2.shape[1], img2.shape[0]))

    img3 = build_screen(ct, cc)
    p3 = os.path.join(TAGS_DIR, "base_station_screen.png")
    cv2.imwrite(p3, img3)
    print("  [3] base_station_screen.png   %dx%d  (screen test)" % (img3.shape[1], img3.shape[0]))

    print("\nVerifying detection...")
    v1 = verify(p1, "annotated:")
    v2 = verify(p2, "printable:")
    v3 = verify(p3, "screen:")

    print("\n" + "="*65)
    print("  COMPLETE GEOMETRY REFERENCE")
    print("="*65)
    print()
    print("  ┌─────────────────────────────────────────────┐")
    print("  │         BASE STATION  688.91 x 688.91 mm    │")
    print("  │                                              │")
    print("  │   ┌──────┐         388.91mm        ┌──────┐ │")
    print("  │   │ ID 1 │ ◄─────────────────────► │ ID 2 │ │")
    print("  │   │150mm │    tag center-center     │150mm │ │")
    print("  │   └──────┘       = 538.91mm         └──────┘ │")
    print("  │         \\                           /        │")
    print("  │          \\  194.455mm each axis    /         │")
    print("  │           \\       ┌──────┐       /          │")
    print("  │            \\      │ ID 5 │      /           │")
    print("  │          ○──►─────│100mm │─────◄──○         │")
    print("  │      R=275mm      └──────┘     R=163.48mm   │")
    print("  │          (outer)               (inner)       │")
    print("  │           /                       \\         │")
    print("  │          /                         \\        │")
    print("  │   ┌──────┐                         ┌──────┐ │")
    print("  │   │ ID 3 │                         │ ID 4 │ │")
    print("  │   │150mm │                         │150mm │ │")
    print("  │   └──────┘                         └──────┘ │")
    print("  │                                              │")
    print("  └─────────────────────────────────────────────┘")
    print()
    print("  OFFSETS FROM PAD CENTER (X, Y):")
    print("  ┌──────┬────────────┬──────────────────────────┐")
    print("  │ Tag  │   mm       │   meters (for config.py) │")
    print("  ├──────┼────────────┼──────────────────────────┤")
    print("  │ ID 1 │ -194.455, -194.455 │ -0.19445, -0.19445 │")
    print("  │ ID 2 │ +194.455, -194.455 │ +0.19445, -0.19445 │")
    print("  │ ID 3 │ -194.455, +194.455 │ -0.19445, +0.19445 │")
    print("  │ ID 4 │ +194.455, +194.455 │ +0.19445, +0.19445 │")
    print("  │ ID 5 │    0.000,    0.000 │  0.00000,  0.00000 │")
    print("  └──────┴────────────┴──────────────────────────┘")
    print()
    print("  CIRCLE DIMENSIONS:")
    print("    Inner:  R = 163.48 mm   D = 326.96 mm")
    print("    Outer:  R = 275.00 mm   D = 550.00 mm")
    print()
    print("  TAG SIZES:")
    print("    Corner (IDs 1-4):  150 x 150 mm  = 0.15 m")
    print("    Center (ID 5):     100 x 100 mm  = 0.10 m")
    print("    Family:  tag36h11")
    print()
    print("  DIAGONAL (center to any corner tag center):")
    print("    = sqrt(194.455^2 + 194.455^2) = %.3f mm = %.5f m" % (
        CORNER_OFFSET_MM * np.sqrt(2), CORNER_OFFSET_MM * np.sqrt(2) / 1000.0))
    print()
    print("  LANDING THRESHOLDS:")
    print("    APPROACH:    > 3.0 m   (use corners, estimate centroid)")
    print("    DESCEND:     1.5-3.0m  (center tag locked)")
    print("    PRECISION:   0.2-0.5m  (slow descent)")
    print("    LAND:        < 0.20m   (trigger touchdown)")
    print("    Centering:   15 px threshold")
    print()
    print("  POSE SCALE CORRECTION:")
    print("    Corner tags: tag_size = 0.15m  (no correction)")
    print("    Center tag:  tag_size = 0.10m  -> scale = 0.10/0.15 = 0.6667")
    print("    Center Z = raw_Z * 0.6667")
    print()
    all_ok = v1 and v2 and v3
    if all_ok:
        print("  DETECTION: ALL 3 IMAGES VERIFIED — 5/5 tags each ✓")
    else:
        print("  DETECTION: SOME FAILED — see above")
    print("="*65)
    print("\nOutput: %s/" % TAGS_DIR)
    print("  base_station.png           ← full reference (zoom in for detail)")
    print("  base_station_printable.png ← print at 100%% scale")
    print("  base_station_screen.png    ← display on Mac, point Pi camera")


if __name__ == "__main__":
    main()
