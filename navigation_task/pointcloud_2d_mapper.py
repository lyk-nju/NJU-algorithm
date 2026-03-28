#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

try:
    import open3d as o3d
except ImportError as exc:
    print(
        "Missing dependency: open3d\n"
        "Install it with: python3 -m pip install open3d matplotlib numpy",
        file=sys.stderr,
    )
    raise SystemExit(1) from exc


SUPPORTED_EXTENSIONS = {".pcd", ".ply", ".xyz", ".xyzn", ".xyzrgb"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Project 3D point clouds to a live 2D top-down map without ROS."
    )
    parser.add_argument(
        "input",
        help="Point cloud file or a directory containing frame files (.pcd/.ply/.xyz).",
    )
    parser.add_argument(
        "--mode",
        choices=["occupancy", "scatter"],
        default="occupancy",
        help="2D visualization type.",
    )
    parser.add_argument(
        "--cell-size",
        type=float,
        default=0.05,
        help="Grid resolution in meters for occupancy mode.",
    )
    parser.add_argument(
        "--x-min",
        type=float,
        default=-10.0,
        help="Minimum x range in meters.",
    )
    parser.add_argument(
        "--x-max",
        type=float,
        default=10.0,
        help="Maximum x range in meters.",
    )
    parser.add_argument(
        "--y-min",
        type=float,
        default=-10.0,
        help="Minimum y range in meters.",
    )
    parser.add_argument(
        "--y-max",
        type=float,
        default=10.0,
        help="Maximum y range in meters.",
    )
    parser.add_argument(
        "--z-min",
        type=float,
        default=-0.5,
        help="Minimum z value to keep.",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=1.5,
        help="Maximum z value to keep.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Refresh interval in seconds when watching for updates.",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Continuously watch a file or directory and refresh the view.",
    )
    parser.add_argument(
        "--accumulate",
        action="store_true",
        help="Accumulate frames over time instead of displaying only the newest frame.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=120000,
        help="Downsample to at most this many points for faster plotting.",
    )
    parser.add_argument(
        "--title",
        default="2D Point Cloud Map",
        help="Figure title.",
    )
    parser.add_argument(
        "--save",
        help="Save the current 2D visualization to an image file, such as output.png.",
    )
    return parser.parse_args()


def discover_frames(input_path: Path) -> list[Path]:
    if input_path.is_file():
        return [input_path]
    if input_path.is_dir():
        return sorted(
            [
                path
                for path in input_path.iterdir()
                if path.is_file() and path.suffix.lower() in SUPPORTED_EXTENSIONS
            ],
            key=lambda path: (path.stat().st_mtime_ns, path.name),
        )
    raise FileNotFoundError(f"Input path does not exist: {input_path}")


def read_points(path: Path) -> np.ndarray:
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float32)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return points[:, :3]


def filter_points(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    if points.size == 0:
        return points

    mask = (
        (points[:, 0] >= args.x_min)
        & (points[:, 0] <= args.x_max)
        & (points[:, 1] >= args.y_min)
        & (points[:, 1] <= args.y_max)
        & (points[:, 2] >= args.z_min)
        & (points[:, 2] <= args.z_max)
    )
    points = points[mask]

    if len(points) > args.max_points:
        step = max(1, len(points) // args.max_points)
        points = points[::step]

    return points


def build_occupancy(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    width = int(np.ceil((args.x_max - args.x_min) / args.cell_size))
    height = int(np.ceil((args.y_max - args.y_min) / args.cell_size))
    grid = np.zeros((height, width), dtype=np.float32)

    if points.size == 0:
        return grid

    x_idx = ((points[:, 0] - args.x_min) / args.cell_size).astype(np.int32)
    y_idx = ((points[:, 1] - args.y_min) / args.cell_size).astype(np.int32)

    valid = (x_idx >= 0) & (x_idx < width) & (y_idx >= 0) & (y_idx < height)
    x_idx = x_idx[valid]
    y_idx = y_idx[valid]

    np.add.at(grid, (y_idx, x_idx), 1.0)
    grid = np.log1p(grid)
    return grid


def newest_frame(frames: list[Path]) -> Path | None:
    if not frames:
        return None
    return frames[-1]


def update_plot(
    ax: plt.Axes,
    artist,
    points: np.ndarray,
    frame_path: Path | None,
    args: argparse.Namespace,
):
    if args.mode == "occupancy":
        grid = build_occupancy(points, args)
        artist.set_data(grid)
        ax.set_title(f"{args.title}\n{frame_path.name if frame_path else 'no frame'}")
        return artist

    if points.size == 0:
        artist.set_offsets(np.empty((0, 2), dtype=np.float32))
        artist.set_array(np.empty((0,), dtype=np.float32))
    else:
        artist.set_offsets(points[:, :2])
        artist.set_array(points[:, 2])
    ax.set_title(f"{args.title}\n{frame_path.name if frame_path else 'no frame'}")
    return artist


def init_plot(args: argparse.Namespace):
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(args.x_min, args.x_max)
    ax.set_ylim(args.y_min, args.y_max)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.3)

    if args.mode == "occupancy":
        width = int(np.ceil((args.x_max - args.x_min) / args.cell_size))
        height = int(np.ceil((args.y_max - args.y_min) / args.cell_size))
        artist = ax.imshow(
            np.zeros((height, width), dtype=np.float32),
            extent=[args.x_min, args.x_max, args.y_min, args.y_max],
            origin="lower",
            cmap="magma",
            interpolation="nearest",
        )
        fig.colorbar(artist, ax=ax, label="log(count)")
    else:
        artist = ax.scatter([], [], c=[], s=1.0, cmap="viridis", vmin=args.z_min, vmax=args.z_max)
        fig.colorbar(artist, ax=ax, label="z (m)")

    return fig, ax, artist


def run_once(input_path: Path, args: argparse.Namespace) -> None:
    frames = discover_frames(input_path)
    frame_path = newest_frame(frames)
    if frame_path is None:
        raise FileNotFoundError(f"No supported point cloud files found in: {input_path}")

    points = filter_points(read_points(frame_path), args)
    fig, ax, artist = init_plot(args)
    update_plot(ax, artist, points, frame_path, args)
    fig.canvas.draw_idle()
    if args.save:
        output_path = Path(args.save).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=180, bbox_inches="tight")
        print(f"Saved 2D map to: {output_path}")
    plt.show(block=True)


def run_watch(input_path: Path, args: argparse.Namespace) -> None:
    fig, ax, artist = init_plot(args)
    processed: set[Path] = set()
    accumulated = np.empty((0, 3), dtype=np.float32)
    last_signature = None

    while plt.fignum_exists(fig.number):
        frames = discover_frames(input_path)
        frame_path = newest_frame(frames)

        if frame_path is None:
            plt.pause(args.interval)
            continue

        signature = (
            frame_path,
            frame_path.stat().st_mtime_ns,
            len(frames),
        )
        if signature != last_signature:
            if input_path.is_dir() and args.accumulate:
                new_frames = [frame for frame in frames if frame not in processed]
                for frame in new_frames:
                    pts = filter_points(read_points(frame), args)
                    if pts.size != 0:
                        accumulated = np.vstack([accumulated, pts])
                    processed.add(frame)
                points = accumulated
                frame_for_title = frame_path
            else:
                points = filter_points(read_points(frame_path), args)
                if args.accumulate:
                    accumulated = np.vstack([accumulated, points]) if points.size != 0 else accumulated
                    points = accumulated
                frame_for_title = frame_path

            update_plot(ax, artist, points, frame_for_title, args)
            fig.canvas.draw_idle()
            if args.save:
                output_path = Path(args.save).expanduser().resolve()
                output_path.parent.mkdir(parents=True, exist_ok=True)
                fig.savefig(output_path, dpi=180, bbox_inches="tight")
                print(f"Updated 2D map: {output_path}")
            last_signature = signature

        plt.pause(args.interval)
        time.sleep(args.interval)


def main() -> None:
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()

    if args.x_min >= args.x_max or args.y_min >= args.y_max or args.z_min >= args.z_max:
        raise ValueError("Invalid axis range: min must be smaller than max")
    if args.cell_size <= 0:
        raise ValueError("cell-size must be positive")
    if args.interval <= 0:
        raise ValueError("interval must be positive")

    if args.watch:
        run_watch(input_path, args)
    else:
        run_once(input_path, args)


if __name__ == "__main__":
    main()
