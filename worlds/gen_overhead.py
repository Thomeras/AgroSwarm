"""
gen_overhead.py — synthetic top-down overhead image for agro_field.usd

Run from scout_ws/worlds/:
    cd worlds && python3 gen_overhead.py

Outputs:
    agro_field_overhead.png  — 1024×1024 px aerial-view agricultural field
    agro_field_overhead.json — NED bounds sidecar for field_view.py alignment

Replace agro_field_overhead.png with a real Isaac Sim top-down screenshot if
available — keep the .json NED bounds matching the actual scene.
"""

import json
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFilter
except ImportError:
    print("Pillow not installed. Run: pip install Pillow")
    raise

# ── NED bounds (metres) of the output image ─────────────────────────────────
# The image covers this real-world area so field_view can align it correctly.
# agro_field.usd:  ~120 × 100 m active crop area, pads to the South.
# We pad by ~20 m on each side for context.
NED_BOUNDS = {
    "ned_x_min": -20.0,   # South limit  (m, NED North axis)
    "ned_x_max":  160.0,  # North limit
    "ned_y_min": -70.0,   # West  limit  (m, NED East axis)
    "ned_y_max":   70.0,  # East  limit
    "description": "agro_field.usd synthetic overhead (NED). Replace with real screenshot.",
}

# ── Active field within the image ────────────────────────────────────────────
FIELD_X_MIN, FIELD_X_MAX = 10.0, 140.0   # NED North
FIELD_Y_MIN, FIELD_Y_MAX = -50.0,  50.0  # NED East
PAD_MARGIN = 4.0                          # path/headland strip (m)

IMG_SIZE = 1024   # pixels (square output)
SEED     = 7

# ── Palette ──────────────────────────────────────────────────────────────────
COL_SURROUND  = ( 38,  52,  28)  # dark grass surrounding field
COL_HEADLAND  = (100,  85,  55)  # dirt headland / access path
COL_SOIL      = ( 72,  58,  38)  # inter-row soil
COL_CROP_MAIN = ( 62, 105,  45)  # crop row primary
COL_CROP_ALT  = ( 74, 122,  54)  # crop row alternate (sunlit edge)
COL_PAD_BASE  = (110,  96,  68)  # landing pad substrate
COL_PAD_MARK  = (200, 200, 200)  # H marking on pad

OUT_DIR = Path(__file__).parent


# ── Helpers ──────────────────────────────────────────────────────────────────

def ned_to_px(
    x_ned: float, y_ned: float,
    bounds: dict, size: int,
) -> tuple[int, int]:
    """NED (north, east) → pixel (px_x, px_y). North = up in image."""
    x_span = bounds["ned_x_max"] - bounds["ned_x_min"]
    y_span = bounds["ned_y_max"] - bounds["ned_y_min"]
    px_x = int((y_ned - bounds["ned_y_min"]) / y_span * size)
    # North is up → flip vertical axis
    px_y = int((bounds["ned_x_max"] - x_ned) / x_span * size)
    return (px_x, px_y)


def draw_circle(draw, centre_px, radius_px, fill, outline=None, width=1):
    cx, cy = centre_px
    r = radius_px
    draw.ellipse(
        [(cx - r, cy - r), (cx + r, cy + r)],
        fill=fill,
        outline=outline,
        width=width,
    )


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    import random
    random.seed(SEED)

    bounds = NED_BOUNDS
    img = Image.new("RGB", (IMG_SIZE, IMG_SIZE), COL_SURROUND)
    draw = ImageDraw.Draw(img)

    def p(xn, yn):
        return ned_to_px(xn, yn, bounds, IMG_SIZE)

    # ── Headland strip (access path surrounding field) ────────────────────
    draw.rectangle([p(FIELD_X_MAX, FIELD_Y_MIN),
                    p(FIELD_X_MIN, FIELD_Y_MAX)], fill=COL_HEADLAND)

    # ── Crop zone ─────────────────────────────────────────────────────────
    cx_min = FIELD_X_MIN + PAD_MARGIN
    cx_max = FIELD_X_MAX - PAD_MARGIN
    cy_min = FIELD_Y_MIN + PAD_MARGIN
    cy_max = FIELD_Y_MAX - PAD_MARGIN

    draw.rectangle([p(cx_max, cy_min), p(cx_min, cy_max)], fill=COL_SOIL)

    # Crop rows running East-West (bands along the North axis)
    ROW_W_M = 1.0      # crop band width (m)
    GAP_M   = 0.5      # soil gap between rows (m)
    step    = ROW_W_M + GAP_M

    x = cx_min
    toggle = True
    while x + ROW_W_M <= cx_max:
        col = COL_CROP_MAIN if toggle else COL_CROP_ALT
        draw.rectangle([p(x + ROW_W_M, cy_min), p(x, cy_max)], fill=col)
        x += step
        toggle = not toggle

    # ── Landing pads (outside field boundary) ────────────────────────────
    # pad_0: NED(10, -8) — drone_0 spawn
    # pad_1: NED(40, -8) — drone_1 spawn
    for (px_ned, py_ned) in [(10.0, -8.0), (40.0, -8.0)]:
        radius_m = 3.0
        # Pad circle
        tl = p(px_ned + radius_m, py_ned - radius_m)
        br = p(px_ned - radius_m, py_ned + radius_m)
        draw.ellipse([tl, br], fill=COL_PAD_BASE)
        # H marking (two vertical bars + crossbar, simplified as lines)
        centre = p(px_ned, py_ned)
        arm_m = 1.5
        tl2 = p(px_ned + arm_m, py_ned - arm_m)
        br2 = p(px_ned - arm_m, py_ned + arm_m)
        draw.rectangle([tl2, br2], outline=COL_PAD_MARK, width=2)

    # ── NED origin marker (small cross) ──────────────────────────────────
    org = p(0.0, 0.0)
    SZ = 6
    draw.line([(org[0] - SZ, org[1]), (org[0] + SZ, org[1])],
              fill=(220, 80, 80), width=2)
    draw.line([(org[0], org[1] - SZ), (org[0], org[1] + SZ)],
              fill=(220, 80, 80), width=2)

    # ── Noise texture (PIL-only, no numpy) ───────────────────────────────
    # Generate a small noise tile (64×64), then paste tiled over the image.
    TILE = 64
    noise_tile = Image.new("RGB", (TILE, TILE))
    pxdata = noise_tile.load()
    for ny in range(TILE):
        for nx in range(TILE):
            delta = random.randint(-12, 12)
            pxdata[nx, ny] = (128 + delta, 128 + delta, 128 + delta)

    # Composite: blend noise multiplicatively (screen-like) at low opacity
    noise_full = Image.new("RGB", (IMG_SIZE, IMG_SIZE))
    for ty in range(0, IMG_SIZE, TILE):
        for tx in range(0, IMG_SIZE, TILE):
            noise_full.paste(noise_tile, (tx, ty))

    img = Image.blend(img, Image.blend(img, noise_full, 0.15), 0.0)

    # Slight blur to soften crop row aliasing
    img = img.filter(ImageFilter.GaussianBlur(radius=0.6))

    # ── Save ─────────────────────────────────────────────────────────────
    png_path  = OUT_DIR / "agro_field_overhead.png"
    json_path = OUT_DIR / "agro_field_overhead.json"

    img.save(str(png_path), "PNG", optimize=True)
    with open(str(json_path), "w") as f:
        json.dump(bounds, f, indent=2)

    print(f"Saved: {png_path}  ({IMG_SIZE}×{IMG_SIZE} px)")
    print(f"Saved: {json_path}")
    b = bounds
    print(f"NED x [{b['ned_x_min']}..{b['ned_x_max']}]  "
          f"y [{b['ned_y_min']}..{b['ned_y_max']}]")


if __name__ == "__main__":
    main()
