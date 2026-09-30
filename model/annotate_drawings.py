"""Dimension the orthographic renders and measure them against the spec.

    python3 model/annotate_drawings.py renders

Reads 04_beam_elevation.png and 05_dorsal_plan.png (orthographic cameras with
known scale), measures the hull's silhouette from the pixels, compares it with
the Section 1 particulars, and writes *_dimensioned.png sheets.
"""
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lev_geom as G  # noqa: E402

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
INK = (236, 196, 90)
DIM = (170, 205, 235)


def font(sz):
    try:
        return ImageFont.truetype(FONT, sz)
    except OSError:
        return ImageFont.load_default()


def extents(img, axis, lum=0.05, min_run=40):
    """Pixel extent of the hull along an axis: columns (axis=0) or rows (axis=1)
    holding at least min_run lit pixels, which ignores stars and glare."""
    a = np.asarray(img.convert("L"), dtype=float) / 255.0
    lit = a > lum
    counts = lit.sum(axis=axis)
    idx = np.where(counts >= min_run)[0]
    return int(idx.min()), int(idx.max())


def line_extent(img, row=None, col=None, lum=0.05, run=6):
    """First and last lit pixel along one row or column, ignoring isolated
    stars (a lit run must be at least `run` pixels long)."""
    a = np.asarray(img.convert("L"), dtype=float) / 255.0
    v = a[row, :] if row is not None else a[:, col]
    lit = v > lum
    runs, start = [], None
    for i, x in enumerate(np.append(lit, False)):
        if x and start is None:
            start = i
        elif not x and start is not None:
            if i - start >= run:
                runs.append((start, i - 1))
            start = None
    return runs[0][0], runs[-1][1]


def emitter_y_at_x(x):
    """Stern extent (most negative y) of the emitter face at plan offset x."""
    best = None
    for i in range(20001):
        t = 135.0 + 90.0 * i / 20000
        r = G.r_out(t) + 20.0
        xx, yy = G.xy(r, t)
        if abs(xx - x) < 1.0 and (best is None or yy < best):
            best = yy
    return best


def arrow_dim(d, p0, p1, text, f, col=DIM, off=(0, 0)):
    d.line([p0, p1], fill=col, width=3)
    for p, q in ((p0, p1), (p1, p0)):
        v = np.array(q, float) - np.array(p, float)
        v /= np.linalg.norm(v)
        n = np.array([-v[1], v[0]])
        a = np.array(p, float)
        d.polygon([tuple(a), tuple(a + 18 * v + 7 * n), tuple(a + 18 * v - 7 * n)], fill=col)
    mx, my = (p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1]
    w = d.textlength(text, font=f)
    d.rectangle([mx - w / 2 - 8, my - 18, mx + w / 2 + 8, my + 18], fill=(10, 12, 16))
    d.text((mx - w / 2, my - 15), text, fill=col, font=f)


def beam(path_in, path_out, lines):
    im = Image.open(path_in).convert("RGB")
    W, H = im.size
    mpp = 3900.0 / W                    # ortho scale spans the image width
    cx, cy = W / 2, H / 2               # camera looks at y = 0, z = -315

    def px(y, z):
        return (cx + y / mpp, cy - (z + 315.0) / mpp)
    # sample single lines well clear of every light and its bloom
    row = int(round(cy - (-420.0 + 315.0) / mpp))
    x0, x1 = line_extent(im, row=row)
    length = (x1 - x0 + 1) * mpp
    col = int(round(cx + (-350.0) / mpp))
    y0, y1 = line_extent(im, col=col)
    height = (y1 - y0 + 1) * mpp
    lines.append(("Length at z -420, rim to emitter face (spec 3,536.9 + 20 m emitter standoff)", 3556.9, length, mpp))
    lines.append(("Height at y -350, collar crown to ventral rim plane", 730.0, height, mpp))
    pad, hpad = 200, 300
    out = Image.new("RGB", (W + 2 * hpad, H + 2 * pad), im.getpixel((4, 4)))
    out.paste(im, (hpad, pad))
    d = ImageDraw.Draw(out)
    f, fs = font(26), font(20)

    def P(y, z):
        a, b = px(y, z)
        return (a + hpad, b + pad)
    stern, bow = -G.r_out(180.0), G.R_RIM
    arrow_dim(d, P(stern, -700), P(bow, -700), "3,536.9 m overall, over the drive fairing", f)
    for yy in (stern, bow):
        d.line([P(yy, -640), P(yy, -700)], fill=DIM, width=2)
    arrow_dim(d, P(bow + 120, 100), P(bow + 120, -630), "730 m", f, off=(70, 0))
    arrow_dim(d, P(stern - 120, 0), P(stern - 120, -630), "630 m rim depth", f, off=(-110, 0))
    for yy, zz in ((bow + 10, 100), (bow + 10, -630), (stern - 10, 0), (stern - 10, -630)):
        d.line([P(yy - 10, zz), P(yy + 110, zz) if yy > 0 else P(yy - 110, zz)], fill=DIM, width=2)
    d.line([P(-450, 100), P(450, 100)], fill=INK, width=2)
    d.text(P(460, 135), "collar crown +100", fill=INK, font=fs)
    d.text((20, 20), "LEVIATHAN-CLASS, REV H: beam elevation from starboard (bow right)", fill=(230, 230, 230), font=f)
    d.text((20, 60), f"Measured from the render at {mpp:.2f} m/px: {length:,.0f} m rim to emitter face (3,536.9 + 20 m standoff), {height:,.0f} m high", fill=INK, font=fs)
    d.text((W + 2 * hpad - 380, H + 2 * pad - 50), "STERN  ←        → BOW", fill=(200, 200, 200), font=fs)
    out.save(path_out)


def plan(path_in, path_out, lines):
    im = Image.open(path_in).convert("RGB")
    W, H = im.size
    mpp = 3800.0 / W
    cx, cy = W / 2, H / 2                # camera looks at x = 0, y = -90

    def P(x, y):
        return (cx + x / mpp, cy - (y + 90.0) / mpp)
    rs = []
    for yy in (500.0, 300.0, -300.0, -500.0):
        x0, x1 = line_extent(im, row=int(round(cy - (yy + 90.0) / mpp)))
        half = (x1 - x0 + 1) * mpp / 2
        rs.append(math.sqrt(half * half + yy * yy))
    lines.append(("Diameter from four chords (plan view)", 3356.9, 2 * sum(rs) / len(rs), mpp))
    xs = 300.0
    y0, y1 = line_extent(im, col=int(round(cx + xs / mpp)))
    pred = math.sqrt(G.R_RIM ** 2 - xs ** 2) - emitter_y_at_x(xs)
    lines.append(("Bow rim to emitter face along x = +300 (plan view)", pred, (y1 - y0 + 1) * mpp, mpp))
    d = ImageDraw.Draw(im)
    f, fs = font(26), font(20)
    arrow_dim(d, P(-G.R_RIM, 1500), P(G.R_RIM, 1500), "3,356.9 m diameter", f, off=(0, -30))
    for xx in (-G.R_RIM, G.R_RIM):
        d.line([P(xx, 0), P(xx, 1560)], fill=DIM, width=1)
    arrow_dim(d, P(1760, G.R_RIM), P(1760, -G.r_out(180.0)), "3,536.9 m", f, off=(-10, 0))
    arrow_dim(d, P(-450, -560), P(450, -560), "collar well, 900 m", fs)
    for t in G.WEB_LONGITUDES:
        a = P(*G.xy(470, t))
        b = P(*G.xy(G.R_RIM + 40, t))
        d.line([a, b], fill=(120, 150, 180), width=1)
        lx, ly = P(*G.xy(G.R_RIM + 95, t))
        lab = f"{t:.0f}°"
        d.text((lx - d.textlength(lab, font=fs) / 2, ly - 12), lab, fill=(200, 215, 230), font=fs)
    d.text((20, 20), "LEVIATHAN-CLASS, REV H: dorsal plan (bow up)", fill=(230, 230, 230), font=f)
    d.text((20, 60), "Web longitudes marked; aft 90° is the drive fairing", fill=INK, font=fs)
    im.save(path_out)


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "renders"
    lines = []
    beam(os.path.join(d, "04_beam_elevation.png"), os.path.join(d, "04_beam_elevation_dimensioned.png"), lines)
    plan(os.path.join(d, "05_dorsal_plan.png"), os.path.join(d, "05_dorsal_plan_dimensioned.png"), lines)
    for name, spec, got, mpp in lines:
        ok = abs(got - spec) <= 3 * mpp
        print(f"{'PASS' if ok else 'FLAG'}  {name}: expected {spec:,.1f} m, measured {got:,.1f} m (tolerance ±{3 * mpp:.1f} m, 3 px)")


if __name__ == "__main__":
    main()
