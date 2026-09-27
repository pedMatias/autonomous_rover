#!/usr/bin/env python3
"""Build a street-view dataset (per-view point clouds + manifest) from a
rtabmap-export dump of images/depth/calib + camera poses.

Inputs (produced by rtabmap-export):
  maps/vo3_street_rgb/<id>.jpg
  maps/vo3_street_depth/<id>.png      (depth in mm)
  maps/vo3_street_calib/<id>.yaml     (fx fy cx cy, image size)
  maps/vo3_street_camera_poses.txt    (stamp x y z qx qy qz qw id, ROS optical)

Output:
  maps/vo3_street/index.html
  maps/vo3_street/three.min.js
  maps/vo3_street/views.json
  maps/vo3_street/rgb/<id>.jpg
  maps/vo3_street/clouds/<id>.bin     (magic, count, f32 xyz x N, u8 rgb x N)

Cloud positions are in the CAMERA OPTICAL frame (x right, y down, z forward),
matching the exported camera poses (p_world = R * p_optical + t).
"""

import argparse
import json
import math
import re
import shutil
import struct
import sys
from pathlib import Path

import numpy as np
from PIL import Image

MAGIC = b"SVCL1"
MIN_RANGE = 0.10
MAX_RANGE = 6.0
STRIDE = 2


def parse_calib(path: Path):
    text = path.read_text()
    w = int(re.search(r"image_width:\s*(\d+)", text).group(1))
    h = int(re.search(r"image_height:\s*(\d+)", text).group(1))
    m = re.search(r"camera_matrix:.*?data:\s*\[([^\]]+)\]", text, re.S)
    k = [float(x) for x in m.group(1).replace("\n", " ").split(",") if x.strip()]
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    return w, h, fx, fy, cx, cy


def load_poses(path: Path):
    poses = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        f = [float(x) for x in line.split()]
        stamp, x, y, z, qx, qy, qz, qw, nid = f
        poses[int(nid)] = dict(stamp=stamp, t=[x, y, z], q=[qx, qy, qz, qw])
    return poses


def quat_to_mat(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def build_cloud(rgb_path: Path, depth_path: Path, w, h, fx, fy, cx, cy):
    rgb = np.asarray(Image.open(rgb_path).convert("RGB"))
    depth = np.asarray(Image.open(depth_path)).astype(np.float32)
    if depth.ndim == 3:
        depth = depth[..., 0]
    depth_m = depth / 1000.0

    ys, xs = np.mgrid[0:h:STRIDE, 0:w:STRIDE]
    d = depth_m[::STRIDE, ::STRIDE]
    col = rgb[::STRIDE, ::STRIDE]
    valid = (d > MIN_RANGE) & (d < MAX_RANGE) & np.isfinite(d)
    xs, ys, d = xs[valid], ys[valid], d[valid]
    col = col[valid]
    if xs.size == 0:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
    z = d
    x = (xs - cx) / fx * z
    y = (ys - cy) / fy * z
    pts = np.stack([x, y, z], axis=-1).astype(np.float32)
    return pts, col.astype(np.uint8)


def write_bin(path: Path, pts: np.ndarray, cols: np.ndarray):
    n = pts.shape[0]
    with open(path, "wb") as f:
        f.write(MAGIC)
        f.write(b"\0\0\0")  # pad so float32 data starts 4-byte aligned
        f.write(struct.pack("<I", n))
        f.write(pts.astype("<f4").tobytes())
        f.write(np.ascontiguousarray(cols).tobytes())


def floor_check(views, cloud_dir: Path, sample_stride=1):
    """Fit a plane to the lowest points of all clouds in ROS world coords.
    A correct pose/unprojection convention yields a near-horizontal floor."""
    world = []
    for v in views[::sample_stride]:
        pts_path = cloud_dir / Path(v["cloud"]).name
        with open(pts_path, "rb") as f:
            hdr = f.read(12)
            assert hdr[:5] == MAGIC
            (n,) = struct.unpack("<I", hdr[8:12])
            if n == 0:
                continue
            pts = np.frombuffer(f.read(n * 12), "<f4").reshape(n, 3)
        R = quat_to_mat(v["q"])
        world.append(pts @ R.T + np.array(v["t"]))
    P = np.concatenate(world)
    z_lo = np.percentile(P[:, 2], 15)
    low = P[P[:, 2] <= z_lo + 0.03]
    A = np.c_[low[:, 0], low[:, 1], np.ones(len(low))]
    coef, *_ = np.linalg.lstsq(A, low[:, 2], rcond=None)
    a, b, c = coef
    normal = np.array([-a, -b, 1.0])
    normal /= np.linalg.norm(normal)
    up_world = normal[2] > 0
    tilt = math.degrees(math.acos(min(1.0, abs(normal[2]))))
    print(
        f"floor check: {len(low)} pts, plane normal=({normal[0]:+.3f}, "
        f"{normal[1]:+.3f}, {normal[2]:+.3f}), tilt from vertical {tilt:.1f} deg, "
        f"up_ok={up_world}, mean floor z={c:.3f} m"
    )
    return tilt < 10.0 and up_world


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rgb-dir", default="maps/vo3_street_rgb")
    ap.add_argument("--depth-dir", default="maps/vo3_street_depth")
    ap.add_argument("--calib-dir", default="maps/vo3_street_calib")
    ap.add_argument("--poses", default="maps/vo3_street_camera_poses.txt")
    ap.add_argument("--out", default="maps/vo3_street")
    ap.add_argument("--three", default="tools/streetview/three.min.js")
    ap.add_argument("--html", default="tools/streetview/index.html")
    args = ap.parse_args()

    rgb_dir, depth_dir, calib_dir = Path(args.rgb_dir), Path(args.depth_dir), Path(args.calib_dir)
    out = Path(args.out)
    out.mkdir(exist_ok=True)
    (out / "clouds").mkdir(exist_ok=True)
    (out / "rgb").mkdir(exist_ok=True)

    poses = load_poses(Path(args.poses))
    print(f"poses: {len(poses)}")

    views = []
    total_pts = 0
    for nid, pose in sorted(poses.items(), key=lambda kv: kv[1]["stamp"]):
        rgb_path = rgb_dir / f"{nid}.jpg"
        depth_path = depth_dir / f"{nid}.png"
        calib_path = calib_dir / f"{nid}.yaml"
        if not (rgb_path.exists() and depth_path.exists() and calib_path.exists()):
            print(f"  skip {nid}: missing inputs")
            continue
        w, h, fx, fy, cx, cy = parse_calib(calib_path)
        pts, cols = build_cloud(rgb_path, depth_path, w, h, fx, fy, cx, cy)
        write_bin(out / "clouds" / f"{nid}.bin", pts, cols)
        shutil.copy(rgb_path, out / "rgb" / f"{nid}.jpg")
        total_pts += len(pts)
        views.append(
            dict(
                id=nid,
                stamp=pose["stamp"],
                t=pose["t"],
                q=pose["q"],
                rgb=f"rgb/{nid}.jpg",
                cloud=f"clouds/{nid}.bin",
                w=w,
                h=h,
                fx=fx,
                fy=fy,
            )
        )

    # viewpoint spacing stats (ROS x/y)
    gaps = [
        math.dist(views[i]["t"][:2], views[i + 1]["t"][:2])
        for i in range(len(views) - 1)
    ]
    path_len = sum(gaps)
    print(
        f"views: {len(views)}, clouds: {total_pts} pts total "
        f"({total_pts / max(1, len(views)):.0f}/view avg), path {path_len:.1f} m, "
        f"max jump {max(gaps):.2f} m"
    )

    ok = floor_check(views, out / "clouds")
    if not ok:
        print("WARNING: floor check failed - pose/unprojection convention suspect")

    manifest = dict(
        views=views,
        path=[[v["t"][0], v["t"][1]] for v in views],
        min_range=MIN_RANGE,
        max_range=MAX_RANGE,
        convention="clouds: camera optical frame; q/t: ROS map->camera optical",
    )
    (out / "views.json").write_text(json.dumps(manifest))

    for src, dst in [(args.three, out / "three.min.js"), (args.html, out / "index.html")]:
        p = Path(src)
        if p.exists():
            shutil.copy(p, out / Path(src).name)
        else:
            print(f"WARNING: missing {src} (viewer cannot run yet)")
    print(f"done -> {out}/views.json")


if __name__ == "__main__":
    sys.exit(main())
