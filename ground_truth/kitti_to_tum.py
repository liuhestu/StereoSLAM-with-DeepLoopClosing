#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np

def rot_to_quat(R):
    # returns (qx,qy,qz,qw)
    # robust conversion
    tr = np.trace(R)
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    else:
        if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
    q = np.array([qx, qy, qz, qw], dtype=float)
    # normalize
    q /= np.linalg.norm(q) + 1e-12
    return q


def load_timestamps(times_path):
    timestamps = []
    with open(times_path, "r") as f:
        for line_no, line in enumerate(f, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                timestamps.append(float(text))
            except ValueError as exc:
                raise ValueError(f"Invalid timestamp at line {line_no}: {text}") from exc
    return timestamps


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert KITTI pose file (3x4 per line) to TUM trajectory format."
    )
    parser.add_argument("kitti_poses", help="Input KITTI poses file (Nx12).")
    parser.add_argument("out_tum", help="Output TUM trajectory file.")
    parser.add_argument(
        "timestamps",
        nargs="?",
        default=None,
        help="Optional KITTI timestamps file (times.txt), one float per line.",
    )
    return parser.parse_args()


def find_default_times_file(kitti_pose_path):
    pose_path = Path(kitti_pose_path).expanduser().resolve()
    seq_name = pose_path.stem
    candidates = [
        pose_path.with_name("times.txt"),
        pose_path.with_name(f"{seq_name}_times.txt"),
        pose_path.parent / "times.txt",
        pose_path.parent.parent / "sequences" / seq_name / "times.txt",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return None


def main():
    args = parse_args()
    in_path = args.kitti_poses
    out_path = args.out_tum
    times_path = args.timestamps if args.timestamps else find_default_times_file(in_path)
    timestamps = load_timestamps(times_path) if times_path else None

    out_lines = []
    with open(in_path, "r") as f:
        idx = 0
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 12:
                raise ValueError(f"Line {idx+1}: expected 12 numbers, got {len(parts)}")
            T = np.array(list(map(float, parts)), dtype=float).reshape(3, 4)
            R = T[:, :3]
            t = T[:, 3]
            qx, qy, qz, qw = rot_to_quat(R)

            if timestamps is not None:
                if idx >= len(timestamps):
                    raise ValueError(
                        f"Not enough timestamps: poses={idx + 1}, timestamps={len(timestamps)}"
                    )
                ts = timestamps[idx]
            else:
                ts = float(idx)

            out_lines.append(
                f"{ts:.6f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )
            idx += 1

    if timestamps is not None and len(timestamps) != idx:
        raise ValueError(f"Timestamp count mismatch: poses={idx}, timestamps={len(timestamps)}")

    with open(out_path, "w") as fo:
        fo.writelines(out_lines)

if __name__ == "__main__":
    main()