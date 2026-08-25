#!/usr/bin/env python3
"""record_stream.py -- record the rover's own camera stream to an mp4.

The obvious way to film a Webots run is to screen-grab the 3D view. That does
not work on this machine: the session is XWayland, where an X client's window
is composited by the Wayland compositor and never reaches the X root
framebuffer, so `ffmpeg -f x11grab -i :0` records a black rectangle. Webots'
own movieStartRecording() sidesteps the display server and does work (see
MARIO_BRIDGE_MOVIE in the bridge), but aiming its Viewpoint is fiddly.

This records what the rover actually sees -- the same BGR frames that go to
stella_vslam -- straight off the ZMQ bus, with the depth image and the lidar
scan beside it. For judging an autonomy run that is the more useful view
anyway: it is the input the decisions were made from.

    ./record_stream.py out.mp4 --seconds 300

Needs pyzmq, numpy and ffmpeg. Nothing from the project's build.
"""

import argparse
import subprocess
import sys
import time

import numpy as np
import zmq

W, H = 640, 480
PANEL_W = 480          # right-hand column: depth on top, lidar below
PANEL_H = H // 2
OUT_W, OUT_H = W + PANEL_W, H


def resize(img: np.ndarray, w: int, h: int) -> np.ndarray:
    """Nearest-neighbour, to keep this dependent on numpy alone."""
    yi = (np.arange(h) * img.shape[0] // h).clip(0, img.shape[0] - 1)
    xi = (np.arange(w) * img.shape[1] // w).clip(0, img.shape[1] - 1)
    return img[yi][:, xi]


def colourise_depth(depth_mm: np.ndarray) -> np.ndarray:
    """uint16 millimetres -> a BGR image. Near is bright, no-return is black."""
    valid = depth_mm > 0
    out = np.zeros((H, W, 3), dtype=np.uint8)
    if not valid.any():
        return out
    d = depth_mm.astype(np.float32)
    near, far = 0.5, 20000.0
    scaled = np.clip((far - d) / (far - near), 0.0, 1.0)
    scaled[~valid] = 0.0
    v = (scaled * 255).astype(np.uint8)
    # cheap turbo-ish ramp: blue -> green -> red as things get closer
    out[..., 0] = np.where(v < 128, 255 - v * 2, 0)          # B
    out[..., 1] = np.where(v < 128, v * 2, 255 - (v - 128) * 2)  # G
    out[..., 2] = np.where(v < 128, 0, (v - 128) * 2)        # R
    out[~valid] = 0
    return out


def draw_lidar(points: np.ndarray, size: int, span: float) -> np.ndarray:
    """Top-down scatter of one scan, rover at the centre, x forward (up)."""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:] = (18, 18, 18)
    if points is None or len(points) == 0:
        return img
    xy = points[:, :2]
    z = points[:, 2]
    keep = ~((points[:, 0] == 0) & (points[:, 1] == 0) & (points[:, 2] == 0))
    xy, z = xy[keep], z[keep]
    if len(xy) == 0:
        return img
    scale = (size / 2) / span
    # image row grows downward and the rover's +x should point up the frame
    u = (size / 2 - xy[:, 0] * scale).astype(np.int32)
    v = (size / 2 - xy[:, 1] * scale).astype(np.int32)
    ok = (u >= 0) & (u < size) & (v >= 0) & (v < size)
    u, v, z = u[ok], v[ok], z[ok]
    # Two classes rather than a gradient. The lidar sits 0.90 m up, so a return
    # from flat ground comes back at about z = -0.90 and anything standing on
    # the ground is higher; splitting there is what the occupancy map does too,
    # and it reads far better than a ramp in which 95% of the points are
    # ground.
    ground = z < -0.60
    # 2x2 blocks: single pixels vanish at this scale.
    for du in (0, 1):
        for dv in (0, 1):
            uu = np.clip(u + du, 0, size - 1)
            vv = np.clip(v + dv, 0, size - 1)
            img[uu[ground], vv[ground]] = (120, 105, 70)     # ground, muted
            img[uu[~ground], vv[~ground]] = (60, 130, 255)   # obstacle, hot

    # range rings every 5 m, and the rover itself
    for ring in range(5, int(span) + 1, 5):
        rr = int(ring * scale)
        th = np.linspace(0, 2 * np.pi, 720)
        cu = (size / 2 + rr * np.sin(th)).astype(np.int32)
        cv = (size / 2 + rr * np.cos(th)).astype(np.int32)
        ok = (cu >= 0) & (cu < size) & (cv >= 0) & (cv < size)
        img[cu[ok], cv[ok]] = (70, 70, 70)
    c = size // 2
    img[c - 2:c + 3, c - 2:c + 3] = (255, 255, 255)
    return img


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("output")
    ap.add_argument("--endpoint", default="tcp://127.0.0.1:5599")
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--lidar-span", type=float, default=20.0,
                    help="metres from the rover to the edge of the lidar panel")
    args = ap.parse_args()

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(args.endpoint)
    for topic in (b"color_frame", b"depth_frame", b"lidar_pointcloud"):
        sub.setsockopt(zmq.SUBSCRIBE, topic)
    sub.setsockopt(zmq.RCVHWM, 40)
    sub.setsockopt(zmq.RCVTIMEO, 5000)

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error",
         "-f", "rawvideo", "-pix_fmt", "bgr24",
         "-s", f"{OUT_W}x{OUT_H}", "-r", str(args.fps),
         "-i", "-", "-c:v", "libx264", "-preset", "veryfast",
         "-crf", "22", "-pix_fmt", "yuv420p", args.output],
        stdin=subprocess.PIPE)

    colour = None
    depth = None
    lidar = None
    frames = 0
    start = time.time()
    try:
        while time.time() - start < args.seconds:
            try:
                topic, body = sub.recv_multipart()
            except zmq.Again:
                print("no frames for 5 s -- is the sim up?", file=sys.stderr)
                break
            if topic == b"color_frame" and len(body) == W * H * 3:
                colour = np.frombuffer(body, np.uint8).reshape(H, W, 3)
            elif topic == b"depth_frame" and len(body) == W * H * 2:
                depth = np.frombuffer(body, np.uint16).reshape(H, W)
            elif topic == b"lidar_pointcloud":
                lidar = np.frombuffer(body, np.float32).reshape(-1, 3)
            else:
                continue

            # colour is the last of the three to be republished each step, so
            # emit a frame on it and let the other two be whatever is current
            if topic != b"color_frame" or colour is None:
                continue

            canvas = np.zeros((OUT_H, OUT_W, 3), dtype=np.uint8)
            canvas[:, :W] = colour
            if depth is not None:
                canvas[:PANEL_H, W:] = resize(colourise_depth(depth),
                                              PANEL_W, PANEL_H)
            # square scatter, centred in the panel
            panel = draw_lidar(lidar, PANEL_H, args.lidar_span)
            x0 = W + (PANEL_W - PANEL_H) // 2
            canvas[PANEL_H:, x0:x0 + PANEL_H] = panel
            ff.stdin.write(canvas.tobytes())
            frames += 1
            if frames % 150 == 0:
                print(f"  {frames} frames, {time.time() - start:.0f}s",
                      flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        ff.stdin.close()
        ff.wait()
    print(f"wrote {args.output}: {frames} frames")
    return 0 if frames else 1


if __name__ == "__main__":
    sys.exit(main())
