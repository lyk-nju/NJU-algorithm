#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture one current MID360 point cloud frame and save both PCD and 2D image."
    )
    parser.add_argument(
        "--topic",
        default="/livox/lidar/pointcloud",
        help="PointCloud2 topic from MID360.",
    )
    parser.add_argument(
        "--output-dir",
        default="navigation_task/output/mid360_snapshot",
        help="Directory for saved files.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for one point cloud frame.",
    )
    parser.add_argument(
        "--x-min",
        type=float,
        default=-10.0,
        help="Minimum x range in meters for 2D image.",
    )
    parser.add_argument(
        "--x-max",
        type=float,
        default=10.0,
        help="Maximum x range in meters for 2D image.",
    )
    parser.add_argument(
        "--y-min",
        type=float,
        default=-10.0,
        help="Minimum y range in meters for 2D image.",
    )
    parser.add_argument(
        "--y-max",
        type=float,
        default=10.0,
        help="Maximum y range in meters for 2D image.",
    )
    parser.add_argument(
        "--z-min",
        type=float,
        default=-0.3,
        help="Minimum z value to keep for the 2D image.",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=1.5,
        help="Maximum z value to keep for the 2D image.",
    )
    parser.add_argument(
        "--cell-size",
        type=float,
        default=0.05,
        help="Grid size in meters for the 2D occupancy image.",
    )
    parser.add_argument(
        "--ground-z",
        type=float,
        default=0.0,
        help="Fallback reference ground height in meters when plane fitting fails.",
    )
    parser.add_argument(
        "--height-threshold",
        type=float,
        default=0.10,
        help="Points higher than ground-z + this value are shown as obstacles.",
    )
    parser.add_argument(
        "--fit-ground-plane",
        action="store_true",
        default=True,
        help="Estimate the ground plane with RANSAC before classifying points.",
    )
    parser.add_argument(
        "--ransac-iterations",
        type=int,
        default=150,
        help="RANSAC iterations for ground plane fitting.",
    )
    parser.add_argument(
        "--ransac-distance",
        type=float,
        default=0.03,
        help="Inlier distance threshold in meters for plane fitting.",
    )
    parser.add_argument(
        "--max-tilt-deg",
        type=float,
        default=45.0,
        help="Maximum accepted tilt angle between fitted plane normal and z axis.",
    )
    return parser.parse_args()


def write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {len(points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(points)}",
            "DATA ascii",
        ]
    )
    with path.open("w", encoding="ascii") as handle:
        handle.write(header)
        handle.write("\n")
        np.savetxt(handle, points, fmt="%.6f %.6f %.6f")


def fit_plane_from_points(sample: np.ndarray) -> tuple[np.ndarray, float] | None:
    p1, p2, p3 = sample
    normal = np.cross(p2 - p1, p3 - p1)
    norm = np.linalg.norm(normal)
    if norm < 1e-6:
        return None
    normal = normal / norm
    if normal[2] < 0:
        normal = -normal
    d = -float(np.dot(normal, p1))
    return normal, d


def fit_ground_plane(points: np.ndarray, args: argparse.Namespace) -> tuple[np.ndarray, float, np.ndarray] | None:
    if len(points) < 3:
        return None

    candidates = points[
        (points[:, 2] >= args.z_min)
        & (points[:, 2] <= min(args.z_max, args.ground_z + 0.6))
    ]
    if len(candidates) < 3:
        candidates = points

    best_inliers = None
    best_model = None
    z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    cos_limit = math.cos(math.radians(args.max_tilt_deg))
    rng = np.random.default_rng(42)

    for _ in range(args.ransac_iterations):
        indices = rng.choice(len(candidates), size=3, replace=False)
        model = fit_plane_from_points(candidates[indices])
        if model is None:
            continue
        normal, d = model
        if float(np.dot(normal, z_axis)) < cos_limit:
            continue

        distances = np.abs(points @ normal + d)
        inliers = distances <= args.ransac_distance
        if best_inliers is None or int(np.count_nonzero(inliers)) > int(np.count_nonzero(best_inliers)):
            best_inliers = inliers
            best_model = (normal, d)

    if best_inliers is None or best_model is None or int(np.count_nonzero(best_inliers)) < 50:
        return None

    inlier_points = points[best_inliers]
    centroid = inlier_points.mean(axis=0)
    centered = inlier_points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1].astype(np.float32)
    normal = normal / np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
    if float(np.dot(normal, z_axis)) < cos_limit:
        return None

    d = -float(np.dot(normal, centroid))
    distances = np.abs(points @ normal + d)
    inliers = distances <= args.ransac_distance
    return normal, d, inliers


def classify_points(points: np.ndarray, args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, dict]:
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float32), {
            "mode": "empty"
        }

    plane_result = fit_ground_plane(points, args) if args.fit_ground_plane else None
    if plane_result is None:
        ground_mask = points[:, 2] <= (args.ground_z + args.height_threshold)
        info = {
            "mode": "z_threshold",
            "ground_z": args.ground_z,
            "height_threshold": args.height_threshold,
        }
    else:
        normal, d, inliers = plane_result
        signed_distance = points @ normal + d
        ground_mask = np.abs(signed_distance) <= args.height_threshold
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(normal[2])))))
        info = {
            "mode": "plane_fit",
            "normal": normal,
            "d": d,
            "tilt_deg": tilt_deg,
            "ransac_inliers": int(np.count_nonzero(inliers)),
        }

    elevated_mask = np.logical_not(ground_mask)
    return points[ground_mask], points[elevated_mask], info


def build_binary_layers(ground_points: np.ndarray, elevated_points: np.ndarray, args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray]:
    width = int(np.ceil((args.x_max - args.x_min) / args.cell_size))
    height = int(np.ceil((args.y_max - args.y_min) / args.cell_size))
    ground = np.zeros((height, width), dtype=bool)
    elevated = np.zeros((height, width), dtype=bool)

    if ground_points.size == 0 and elevated_points.size == 0:
        return ground, elevated

    def mark(layer: np.ndarray, pts: np.ndarray) -> None:
        if pts.size == 0:
            return
        x_idx = ((pts[:, 0] - args.x_min) / args.cell_size).astype(np.int32)
        y_idx = ((pts[:, 1] - args.y_min) / args.cell_size).astype(np.int32)
        valid = (x_idx >= 0) & (x_idx < width) & (y_idx >= 0) & (y_idx < height)
        layer[y_idx[valid], x_idx[valid]] = True

    mark(ground, ground_points)
    mark(elevated, elevated_points)
    return ground, elevated


def save_png(path: Path, points: np.ndarray, args: argparse.Namespace) -> dict:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    filtered = points[
        (points[:, 0] >= args.x_min)
        & (points[:, 0] <= args.x_max)
        & (points[:, 1] >= args.y_min)
        & (points[:, 1] <= args.y_max)
        & (points[:, 2] >= args.z_min)
        & (points[:, 2] <= args.z_max)
    ]
    ground_points, elevated_points, info = classify_points(filtered, args)
    ground, elevated = build_binary_layers(ground_points, elevated_points, args)

    rgb = np.zeros((ground.shape[0], ground.shape[1], 3), dtype=np.float32)
    rgb[ground] = np.array([0.20, 0.62, 0.98], dtype=np.float32)
    rgb[elevated] = np.array([0.95, 0.35, 0.16], dtype=np.float32)
    rgb[np.logical_and(ground, elevated)] = np.array([0.98, 0.82, 0.18], dtype=np.float32)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(
        rgb,
        extent=[args.x_min, args.x_max, args.y_min, args.y_max],
        origin="lower",
        interpolation="nearest",
    )
    subtitle = "plane-fit unavailable, fallback to z-threshold"
    if info["mode"] == "plane_fit":
        subtitle = f"plane tilt {info['tilt_deg']:.2f} deg, threshold {args.height_threshold:.2f} m"
    elif info["mode"] == "z_threshold":
        subtitle = f"z-threshold at {args.ground_z + args.height_threshold:.2f} m"
    ax.set_title(f"Current MID360 2D Map\nblue: ground, orange: >10cm above ground, {subtitle}")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="box")
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    info["ground_points"] = int(len(ground_points))
    info["elevated_points"] = int(len(elevated_points))
    return info


class Mid360Snapshot(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("mid360_snapshot")
        self.points = None
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.callback, 10
        )

    def callback(self, msg: PointCloud2) -> None:
        points_iter = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        xyz = np.array([(float(p[0]), float(p[1]), float(p[2])) for p in points_iter], dtype=np.float32)
        self.points = xyz
        self.get_logger().info(f"Received point cloud with {len(xyz)} points")


def main() -> None:
    args = parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Mid360Snapshot(args.topic)

    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while rclpy.ok() and node.points is None:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds > deadline:
                raise TimeoutError(
                    f"Timed out after {args.timeout}s waiting for topic {args.topic}"
                )

        points = node.points
        if points is None or len(points) == 0:
            raise RuntimeError("Received an empty point cloud")

        distances = np.linalg.norm(points, axis=1)

        pcd_path = output_dir / "current_mid360.pcd"
        png_path = output_dir / "current_mid360_2d.png"

        write_ascii_pcd(pcd_path, points)
        info = save_png(png_path, points, args)

        print(f"Saved current MID360 point cloud: {pcd_path}")
        print(f"Saved current MID360 2D map : {png_path}")
        print(
            "Distance stats (meters): "
            f"min={distances.min():.3f}, max={distances.max():.3f}, mean={distances.mean():.3f}"
        )
        if info.get("mode") == "plane_fit":
            normal = info["normal"]
            print(
                "Ground plane fit: "
                f"normal=({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f}), "
                f"tilt={info['tilt_deg']:.2f} deg, inliers={info['ransac_inliers']}"
            )
        elif info.get("mode") == "z_threshold":
            print(
                "Ground classification fallback: "
                f"z <= {args.ground_z + args.height_threshold:.2f} m treated as ground"
            )
        print(
            "Class counts: "
            f"ground={info.get('ground_points', 0)}, elevated={info.get('elevated_points', 0)}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
