# Standalone 3D Point Cloud -> 2D Mapper

This tool lets you visualize 3D point clouds as a live 2D top-down map without ROS.

Files:
- `navigation_task/pointcloud_2d_mapper.py`: main program
- Sample point clouds already in this repo:
  - `navigation_task/src/rm_nav_bringup/PCD/RMUL.pcd`
  - `navigation_task/src/rm_nav_bringup/PCD/RMUC.pcd`

## 1. Install dependencies

Use Python 3.10+ if possible.

```bash
python3 -m pip install --upgrade pip
python3 -m pip install open3d matplotlib numpy
```

## 2. Visualize one point cloud file

```bash
python3 navigation_task/pointcloud_2d_mapper.py \
  navigation_task/src/rm_nav_bringup/PCD/RMUL.pcd \
  --mode occupancy \
  --x-min -10 --x-max 10 \
  --y-min -10 --y-max 10 \
  --z-min -0.3 --z-max 1.5
```

If you want to save the current 2D point cloud view as an image:

```bash
python3 navigation_task/pointcloud_2d_mapper.py \
  navigation_task/src/rm_nav_bringup/PCD/RMUL.pcd \
  --mode occupancy \
  --save navigation_task/output/rmul_2d.png
```

Notes:
- `occupancy`: render a 2D occupancy-style heatmap
- `scatter`: render projected XY points colored by height

Example scatter mode:

```bash
python3 navigation_task/pointcloud_2d_mapper.py \
  navigation_task/src/rm_nav_bringup/PCD/RMUL.pcd \
  --mode scatter \
  --x-min -10 --x-max 10 \
  --y-min -10 --y-max 10 \
  --z-min -0.3 --z-max 1.5
```

## 3. Real-time visualization from a directory of frames

If you have a directory that continuously receives `.pcd` or `.ply` files, use watch mode.

```bash
python3 navigation_task/pointcloud_2d_mapper.py /path/to/frames \
  --watch \
  --mode occupancy \
  --interval 0.2
```

Behavior:
- without `--accumulate`: only the newest frame is displayed
- with `--accumulate`: all observed frames are merged into one 2D map

Example:

```bash
python3 navigation_task/pointcloud_2d_mapper.py /path/to/frames \
  --watch --accumulate --mode occupancy
```

You can also keep updating one image file while new frames arrive:

```bash
python3 navigation_task/pointcloud_2d_mapper.py /path/to/frames \
  --watch --accumulate --mode occupancy \
  --save /path/to/current_map.png
```

## 4. Recommended workflow on a computer without ROS

If your robot side can export point clouds as `.pcd` files, you can:

1. Continuously copy point cloud frames into a folder on your computer
2. Run this script in `--watch` mode on that folder
3. Observe the live 2D map window

For example, if files keep appearing in `~/lidar_frames/`:

```bash
python3 navigation_task/pointcloud_2d_mapper.py ~/lidar_frames \
  --watch --accumulate --mode occupancy \
  --x-min -12 --x-max 12 --y-min -12 --y-max 12
```

## 5. Parameter tuning

- `--z-min` / `--z-max`: choose which height slice to keep
- `--cell-size`: smaller value means finer occupancy map but more computation
- `--x-min` / `--x-max` / `--y-min` / `--y-max`: crop the visible area
- `--max-points`: reduce this if plotting becomes slow

## 6. Typical use cases

- Top-down obstacle view from one frame:
  - use `--mode occupancy`
- Inspect shape/height of obstacles:
  - use `--mode scatter`
- Build a rough 2D map from many frames:
  - use `--watch --accumulate`

## 7. Limitations

- This is a geometric XY projection tool, not SLAM
- It does not estimate pose or perform scan matching
- If the sensor or robot moves, accumulated mode only makes sense when incoming frames are already aligned in one coordinate system
