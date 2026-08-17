#!/usr/bin/env python3
"""Generate every texture the Webots world needs.

Nothing here is decorative. stella_vslam tracks ORB corners, and a flat-shaded
Webots ground plane produces almost none -- tracking drops on the first frame
and the FSM falls straight into RECOVER_SLAM -> MISSION_ABORT. So the ground
texture is built for corner density: multi-octave fractal noise (features
survive at several viewing distances) plus scattered high-contrast pebbles
(sharp, well-localised corners close up).
"""
import os

import cv2
import numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "textures")
os.makedirs(OUT, exist_ok=True)

rng = np.random.default_rng(20260805)


def fractal_noise(size, octaves=7, persistence=0.55):
    """Sum octaves of upsampled white noise -> detail at every scale."""
    out = np.zeros((size, size), np.float32)
    amp, total = 1.0, 0.0
    for o in range(octaves):
        res = max(2, size >> (octaves - o))
        layer = rng.random((res, res)).astype(np.float32)
        layer = cv2.resize(layer, (size, size), interpolation=cv2.INTER_CUBIC)
        out += layer * amp
        total += amp
        amp *= persistence
    out /= total
    return cv2.normalize(out, None, 0, 1, cv2.NORM_MINMAX)


def scatter_pebbles(img, count, rmin, rmax):
    """High-contrast ellipses: the sharp corners ORB locks onto up close.

    Pebbles accumulate into a single-channel delta layer that is added to the
    image once at the end. Compositing per pebble instead would mean copying a
    2048x2048x3 float array thousands of times.
    """
    size = img.shape[0]
    delta = np.zeros((size, size), np.float32)
    for _ in range(count):
        cx, cy = (int(v) for v in rng.integers(0, size, 2))
        r = int(rng.integers(rmin, rmax))
        ang = float(rng.uniform(0, 180))
        shade = float(rng.uniform(-0.42, 0.42))
        axes = (r, max(1, int(r * rng.uniform(0.6, 1.0))))
        cv2.ellipse(delta, (cx, cy), axes, ang, 0, 360, shade, -1)
        # darker rim -> a second, stronger corner response at the edge
        cv2.ellipse(delta, (cx, cy), axes, ang, 0, 360, -abs(shade) * 0.8, max(1, r // 6))
    return np.clip(img + delta[..., None], 0, 1)


def make_ground(size=2048):
    base = fractal_noise(size)
    # Mars regolith: strong red, muted green, low blue.
    img = np.stack(
        [
            np.clip(base * 0.30 + 0.06, 0, 1),   # B
            np.clip(base * 0.52 + 0.14, 0, 1),   # G
            np.clip(base * 0.85 + 0.28, 0, 1),   # R
        ],
        axis=-1,
    ).astype(np.float32)
    img = scatter_pebbles(img, 2600, 3, 22)
    img = scatter_pebbles(img, 260, 22, 60)
    # a little sharpening keeps corners crisp after Webots mipmaps it
    img = np.clip(cv2.addWeighted(img, 1.35, cv2.GaussianBlur(img, (0, 0), 3), -0.35, 0), 0, 1)
    return (img * 255).astype(np.uint8)


def make_rock(size=1024):
    base = fractal_noise(size, octaves=6, persistence=0.62)
    img = np.stack(
        [
            np.clip(base * 0.34 + 0.05, 0, 1),
            np.clip(base * 0.44 + 0.09, 0, 1),
            np.clip(base * 0.60 + 0.15, 0, 1),
        ],
        axis=-1,
    ).astype(np.float32)
    img = scatter_pebbles(img, 700, 2, 14)
    return (img * 255).astype(np.uint8)


def make_aruco(marker_id, px=1024, border_frac=0.22):
    """DICT_4X4_50 marker on a white board.

    Must match src/mario.cpp search_aruco()/approach(), which both call
    getPredefinedDictionary(DICT_4X4_50).
    """
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    inner = int(px * (1 - 2 * border_frac))
    try:                                    # OpenCV >= 4.7
        m = cv2.aruco.generateImageMarker(d, marker_id, inner)
    except AttributeError:                  # OpenCV 4.6
        m = cv2.aruco.drawMarker(d, marker_id, inner)
    board = np.full((px, px), 255, np.uint8)
    off = (px - inner) // 2
    board[off:off + inner, off:off + inner] = m
    return cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)


if __name__ == "__main__":
    cv2.imwrite(os.path.join(OUT, "mars_ground.jpg"), make_ground(), [cv2.IMWRITE_JPEG_QUALITY, 95])
    print("wrote mars_ground.jpg")

    cv2.imwrite(os.path.join(OUT, "rock.jpg"), make_rock(), [cv2.IMWRITE_JPEG_QUALITY, 95])
    print("wrote rock.jpg")

    for mid in (0, 1, 2, 3, 4):
        cv2.imwrite(os.path.join(OUT, f"aruco_{mid}.png"), make_aruco(mid))
        print(f"wrote aruco_{mid}.png")
