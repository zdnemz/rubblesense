#!/usr/bin/env python3
"""Render the TFT display lists written by the firmware test-bench to PNG images.

    ./fw_harness --screens screens/            # writes screens/<name>.txt (one display list per key screen)
    python3 tools/render_screens.py screens/ --out screens_png/ --sheet

The harness records what the firmware draws on the 320x240 ILI9341 (Adafruit_GFX calls) as a simple
display list, one op per line:

    S x y w h color      fill the whole screen
    F x y w h color      filled rectangle          R x y w h color   rectangle outline
    H x y w h color      horizontal line           V x y w h color   vertical line
    L x0 y0 x1 y1 color  line
    T x y size color txt text, classic 5x7 font scaled by `size`, 6*size px per character

Colours are RGB565. This is a visual aid for reviewing layout (nothing overlapping, nothing cut off):
the glyph shapes come from a monospace TrueType font, not from the Adafruit bitmap font, so letters look
slightly different from the real panel, but every character sits exactly where the firmware puts it.
"""
import argparse
import os
import sys

from PIL import Image, ImageDraw, ImageFont

W, H = 320, 240
FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
    "/Library/Fonts/Courier New.ttf",
    "C:/Windows/Fonts/consola.ttf",
]


def rgb565(c):
    r = (c >> 11) & 0x1F
    g = (c >> 5) & 0x3F
    b = c & 0x1F
    return (r * 255 // 31, g * 255 // 63, b * 255 // 31)


def load_font(px):
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            return ImageFont.truetype(p, px)
    return ImageFont.load_default()


def unescape(s):
    out, i = [], 0
    while i < len(s):
        if s[i] == "\\" and i + 1 < len(s):
            out.append("\n" if s[i + 1] == "n" else s[i + 1])
            i += 2
        else:
            out.append(s[i])
            i += 1
    return "".join(out)


def render(path, scale=2):
    img = Image.new("RGB", (W, H), (0, 0, 0))
    d = ImageDraw.Draw(img)
    fonts = {}
    n_ops = 0
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            if not line:
                continue
            kind = line[0]
            n_ops += 1
            if kind == "T":
                parts = line.split(" ", 5)
                x, y, size, color = int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])
                text = unescape(parts[5]) if len(parts) > 5 else ""
                px = max(6, round(6.0 * size / 0.602))          # DejaVu Sans Mono advance = 0.602 em
                font = fonts.setdefault(size, load_font(px))
                col = rgb565(color)
                for i, ch in enumerate(text):
                    if ch == " ":
                        continue
                    # Draw into a mask the size of one character cell (6*size x 8*size) so that a
                    # substitute font's tall ascenders cannot poke out of the cell like the real
                    # 5x7 glyphs never do (they would otherwise survive the firmware's erase fills).
                    mask = Image.new("L", (6 * size, 8 * size), 0)
                    ImageDraw.Draw(mask).text((0, 7 * size), ch, fill=255, font=font, anchor="ls")
                    img.paste(col, (x + i * 6 * size, y), mask)
                continue
            a = [int(v) for v in line.split()[1:]]
            x, y, w, h, color = a[0], a[1], a[2], a[3], rgb565(a[4])
            if kind == "S":
                d.rectangle([0, 0, W - 1, H - 1], fill=color)
            elif kind == "F":
                d.rectangle([x, y, x + w - 1, y + h - 1], fill=color)
            elif kind == "R":
                d.rectangle([x, y, x + w - 1, y + h - 1], outline=color)
            elif kind == "H":
                d.line([x, y, x + w - 1, y], fill=color)
            elif kind == "V":
                d.line([x, y, x, y + h - 1], fill=color)
            elif kind == "L":
                d.line([x, y, w, h], fill=color)       # fields hold x0 y0 x1 y1
    if scale != 1:
        img = img.resize((W * scale, H * scale), Image.NEAREST)
    return img, n_ops


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", help="a display-list file or a directory of them")
    ap.add_argument("--out", default="screens_png", help="output directory (default: screens_png)")
    ap.add_argument("--scale", type=int, default=2, help="integer magnification (default 2)")
    ap.add_argument("--sheet", action="store_true", help="also write a contact sheet (sheet.png)")
    ap.add_argument("--cols", type=int, default=3, help="contact-sheet columns (default 3)")
    args = ap.parse_args()

    if os.path.isdir(args.src):
        files = sorted(os.path.join(args.src, n) for n in os.listdir(args.src) if n.endswith(".txt"))
    else:
        files = [args.src]
    if not files:
        print("no display lists found in", args.src, file=sys.stderr)
        return 1
    os.makedirs(args.out, exist_ok=True)

    tiles = []
    for p in files:
        img, n = render(p, args.scale)
        name = os.path.splitext(os.path.basename(p))[0]
        img.save(os.path.join(args.out, name + ".png"))
        tiles.append((name, img))
        print("%-34s %4d ops -> %s.png" % (name, n, name))

    if args.sheet:
        cap_h = 18
        cols = max(1, args.cols)
        rows = (len(tiles) + cols - 1) // cols
        tw, th = tiles[0][1].size
        sheet = Image.new("RGB", (cols * (tw + 10) + 10, rows * (th + cap_h + 10) + 10), (40, 40, 40))
        d = ImageDraw.Draw(sheet)
        cap_font = load_font(12)
        for i, (name, img) in enumerate(tiles):
            cx = 10 + (i % cols) * (tw + 10)
            cy = 10 + (i // cols) * (th + cap_h + 10)
            d.text((cx, cy), name, fill=(220, 220, 220), font=cap_font)
            sheet.paste(img, (cx, cy + cap_h))
        sheet.save(os.path.join(args.out, "sheet.png"))
        print("contact sheet -> %s" % os.path.join(args.out, "sheet.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
