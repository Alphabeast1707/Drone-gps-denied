#!/usr/bin/env python3
"""
Generate AprilTag images (tag36h11) for display on screen.
Opens them in a browser window for easy viewing.

Tags generated:
  ID 1-4: Corner tags (displayed at ~15cm equivalent)
  ID 5:   Center tag (displayed at ~10cm equivalent)
"""

import numpy as np
import os
import struct
import zlib

# tag36h11 codes for IDs 1-5
# These are the official 36-bit payloads from the apriltag library
TAG36H11_CODES = {
    1: 0xd97f18b49,
    2: 0xdd280910e,
    3: 0xe0d0f96d3,
    4: 0xe479e9c98,
    5: 0xe822da25d,
}


def code_to_grid(code):
    """Convert a 36-bit code to a 6x6 binary grid."""
    grid = np.zeros((6, 6), dtype=np.uint8)
    for i in range(36):
        row = i // 6
        col = i % 6
        bit = (code >> (35 - i)) & 1
        grid[row][col] = bit
    return grid


def generate_tag_image(tag_id, code, cell_size=50):
    """
    Generate a single AprilTag image.

    tag36h11 layout (10x10 cells):
    - Outer 1-cell ring: WHITE (quiet zone)
    - Next 1-cell ring: BLACK (tag border)
    - Inner 6x6: data bits (black=0, white=1)
    """
    tag_cells = 10
    img_size = tag_cells * cell_size

    # Start all white
    img = np.ones((img_size, img_size), dtype=np.uint8) * 255

    # Draw black border (cells 1-8)
    b1 = 1 * cell_size
    b2 = 9 * cell_size
    img[b1:b2, b1:b2] = 0

    # Fill in the 6x6 data region (cells 2-7)
    data_grid = code_to_grid(code)
    for row in range(6):
        for col in range(6):
            if data_grid[row][col] == 1:
                y = (row + 2) * cell_size
                x = (col + 2) * cell_size
                img[y:y + cell_size, x:x + cell_size] = 255

    return img


def save_png(img, filepath):
    """Save grayscale numpy array as PNG (no OpenCV needed)."""
    h, w = img.shape

    def make_chunk(ctype, data):
        c = ctype + data
        crc = struct.pack('>I', zlib.crc32(c) & 0xffffffff)
        return struct.pack('>I', len(data)) + c + crc

    sig = b'\x89PNG\r\n\x1a\n'
    ihdr = make_chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))

    raw = b''
    for row in range(h):
        raw += b'\x00' + img[row].tobytes()
    idat = make_chunk(b'IDAT', zlib.compress(raw))
    iend = make_chunk(b'IEND', b'')

    with open(filepath, 'wb') as f:
        f.write(sig + ihdr + idat + iend)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    tag_dir = os.path.join(script_dir, "apriltags_display")
    os.makedirs(tag_dir, exist_ok=True)

    tag_files = {}

    print("Generating AprilTag images (tag36h11)...")
    print("Output: " + tag_dir)
    print()

    for tag_id in range(1, 6):
        code = TAG36H11_CODES[tag_id]
        img = generate_tag_image(tag_id, code, cell_size=50)

        fname = "tag36h11_id{}.png".format(tag_id)
        fpath = os.path.join(tag_dir, fname)
        save_png(img, fpath)
        tag_files[tag_id] = fname

        ttype = "CORNER 15cm" if tag_id <= 4 else "CENTER 10cm"
        print("  Tag ID {}  [{}]  -> {}".format(tag_id, ttype, fname))

    # Generate HTML viewer
    html_path = os.path.join(tag_dir, "tags.html")
    write_html(html_path, tag_files)

    print()
    print("HTML viewer -> " + html_path)
    print()
    print("Open in browser:")
    print("  open " + html_path)


def write_html(path, tag_files):
    """Write an HTML page to display all tags."""
    lines = []
    lines.append('<!DOCTYPE html>')
    lines.append('<html><head><title>AprilTags tag36h11</title>')
    lines.append('<style>')
    lines.append('body { background: #fff; font-family: -apple-system, sans-serif; text-align: center; padding: 20px; }')
    lines.append('h1 { color: #333; }')
    lines.append('.info { color: #666; font-size: 14px; margin-bottom: 20px; }')
    lines.append('.tags { display: flex; flex-wrap: wrap; justify-content: center; gap: 25px; margin: 20px auto; }')
    lines.append('.card { background: #f8f8f8; border: 2px solid #ddd; border-radius: 12px; padding: 15px; }')
    lines.append('.card img { image-rendering: pixelated; image-rendering: crisp-edges; }')
    lines.append('.corner img { width: 250px; height: 250px; }')
    lines.append('.center img { width: 200px; height: 200px; }')
    lines.append('.label { font-size: 18px; font-weight: 700; margin-top: 8px; }')
    lines.append('.type { font-size: 13px; color: #888; }')
    lines.append('.single { display: none; margin: 30px auto; }')
    lines.append('.single img { image-rendering: pixelated; width: 450px; height: 450px; border: 3px solid #333; }')
    lines.append('button { background: #007AFF; color: #fff; border: none; padding: 10px 20px; border-radius: 8px; font-size: 14px; cursor: pointer; margin: 4px; }')
    lines.append('button:hover { background: #005ec4; }')
    lines.append('button.on { background: #34c759; }')
    lines.append('.howto { background: #f0f7ff; border: 1px solid #b3d4fc; border-radius: 8px; padding: 12px 20px; margin: 15px auto; max-width: 600px; font-size: 13px; color: #444; text-align: left; }')
    lines.append('</style></head><body>')
    lines.append('<h1>AprilTags &mdash; tag36h11</h1>')
    lines.append('<p class="info">Point the Pi camera at any tag to test detection</p>')
    lines.append('<div class="howto"><strong>How to test:</strong><br>')
    lines.append('1. Click a tag button to show it large<br>')
    lines.append('2. Point Pi camera at your Mac screen<br>')
    lines.append('3. On Pi run: <code>PYTHONUNBUFFERED=1 python3 -u test_headless.py</code></div>')
    lines.append('<div style="margin:15px">')
    lines.append('<button onclick="showAll()">Show All</button>')

    for tid in sorted(tag_files.keys()):
        lbl = "ID {} ({})".format(tid, "Corner" if tid <= 4 else "Center")
        lines.append('<button onclick="showOne({})" id="b{}">{}</button>'.format(tid, tid, lbl))

    lines.append('</div>')
    lines.append('<div class="tags" id="grid">')

    for tid in sorted(tag_files.keys()):
        fname = tag_files[tid]
        cls = "corner" if tid <= 4 else "center"
        sz = "150mm" if tid <= 4 else "100mm"
        tp = "CORNER" if tid <= 4 else "CENTER"
        lines.append('<div class="card {}">'.format(cls))
        lines.append('<img src="{}">'.format(fname))
        lines.append('<div class="label">ID {}</div>'.format(tid))
        lines.append('<div class="type">{} &mdash; {}</div>'.format(tp, sz))
        lines.append('</div>')

    lines.append('</div>')

    for tid in sorted(tag_files.keys()):
        fname = tag_files[tid]
        tp = "CORNER" if tid <= 4 else "CENTER"
        lines.append('<div class="single" id="s{}">'.format(tid))
        lines.append('<h2>Tag ID {} &mdash; {}</h2>'.format(tid, tp))
        lines.append('<img src="{}">'.format(fname))
        lines.append('<p style="color:#888;font-size:13px">Point camera here</p>')
        lines.append('</div>')

    lines.append('<script>')
    lines.append('function showAll(){')
    lines.append('  document.getElementById("grid").style.display="flex";')
    lines.append('  document.querySelectorAll(".single").forEach(e=>e.style.display="none");')
    lines.append('  document.querySelectorAll("button").forEach(b=>b.classList.remove("on"));')
    lines.append('}')
    lines.append('function showOne(id){')
    lines.append('  document.getElementById("grid").style.display="none";')
    lines.append('  document.querySelectorAll(".single").forEach(e=>e.style.display="none");')
    lines.append('  document.getElementById("s"+id).style.display="block";')
    lines.append('  document.querySelectorAll("button").forEach(b=>b.classList.remove("on"));')
    lines.append('  document.getElementById("b"+id).classList.add("on");')
    lines.append('}')
    lines.append('</script>')
    lines.append('</body></html>')

    with open(path, 'w') as f:
        f.write('\n'.join(lines))


if __name__ == "__main__":
    main()
