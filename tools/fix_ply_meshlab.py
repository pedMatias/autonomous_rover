#!/usr/bin/env python3
"""Strip rtabmap-export's non-standard 'element camera' block from a PLY so
MeshLab can open it. The camera element is always last, so we can truncate.

Usage: python3 tools/fix_ply_meshlab.py <file.ply> [out.ply]
"""

import sys
from pathlib import Path

TYPE_SIZE = {
    "char": 1, "uchar": 1, "short": 2, "ushort": 2,
    "int": 4, "uint": 4, "float": 4, "double": 8,
}


def main():
    src = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else src.with_name(src.stem + "_fixed.ply")

    data = src.read_bytes()
    marker = b"end_header\n"
    end = data.find(marker)
    if end < 0:
        sys.exit("no end_header found")
    header = data[: end + len(marker)].decode("ascii", "replace")
    body = data[end + len(marker):]

    lines = header.splitlines()
    kept = []
    element = None
    cam_props = []          # (type, name) of camera element
    vertex_count = 0
    vertex_size = 0
    face_count = 0
    for ln in lines:
        parts = ln.split()
        if parts and parts[0] == "element":
            element = parts[1]
            count = int(parts[2])
            if element == "camera":
                continue  # drop element line (and its properties below)
            if element == "vertex":
                vertex_count = count
            elif element == "face":
                face_count = count
            kept.append(ln)
        elif parts and parts[0] == "property":
            if element == "camera":
                cam_props.append((parts[1], parts[2]))
                continue
            if element == "vertex" and "list" not in parts:
                vertex_size += TYPE_SIZE[parts[1]]
            kept.append(ln)
        else:
            kept.append(ln)

    if not cam_props:
        print("no camera element found; file should already be fine")
        return

    cam_size = sum(TYPE_SIZE[t] for t, _ in cam_props)
    keep_bytes = vertex_count * vertex_size  # face is empty (0) or list-based; camera last
    # If a face element with content exists, we cannot blindly truncate; but
    # rtabmap cloud exports always have face 0. Guard anyway:
    if face_count != 0:
        sys.exit("face element is non-empty; refusing to truncate blindly")

    out.write_bytes("\n".join(kept).encode("ascii") + b"\n" + body[:keep_bytes])
    print(
        f"stripped camera element ({cam_size} bytes, {len(cam_props)} props); "
        f"kept {keep_bytes} bytes of vertex data -> {out}"
    )


if __name__ == "__main__":
    main()
