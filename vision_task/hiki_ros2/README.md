# ros2_hik_camera

A ROS2 packge for Hikvision USB3.0 industrial camera

## Usage

```
ros2 launch hik_camera hik_camera.launch.py
```

## Parameters

### Camera Parameters

The camera node supports dynamic parameter adjustment for exposure and gain control:

| Parameter            | Type   | Default | Range            | Description                                     |
| -------------------- | ------ | ------- | ---------------- | ----------------------------------------------- |
| `exposure_time`      | double | 5000    | Camera-dependent | Exposure time in microseconds (μs)              |
| `gain`               | double | Auto    | Camera-dependent | Camera gain value (dB)                          |
| `balance_white_auto` | int    | 0       | 0-2              | Auto white balance: 0=Off, 1=Continuous, 2=Once |

### How to Adjust Exposure Time and Gain

There are **three methods** to adjust camera parameters:

#### Method 1: Edit Configuration File (Recommended for permanent settings)

Edit `config/camera_params.yaml`:

```yaml
/hik_camera:
  ros__parameters:
    camera_name: narrow_stereo
    exposure_time: 15000    # Increase exposure (μs)
    gain: 20.0              # Increase gain (dB)
    balance_white_auto: 0
```

Then restart the camera node:
```bash
ros2 launch hik_camera hik_camera.launch.py
```

#### Method 2: Launch with Custom Parameters (One-time override)

Override parameters when launching:

```bash
ros2 launch hik_camera hik_camera.launch.py \
  params_file:=/path/to/custom_params.yaml
```

Or set individual parameters:
```bash
ros2 run hik_camera hik_camera_node --ros-args \
  -p exposure_time:=15000 \
  -p gain:=20.0
```

#### Method 3: Dynamic Runtime Adjustment (Live tuning)

Adjust parameters while the node is running using `ros2 param set`:

```bash
# Increase exposure time to 15000 μs (15 ms)
ros2 param set /hik_camera exposure_time 15000

# Increase gain to 20 dB
ros2 param set /hik_camera gain 20.0

# Check current parameter values
ros2 param get /hik_camera exposure_time
ros2 param get /hik_camera gain

# List all available parameters
ros2 param list /hik_camera
```

### Parameter Tuning Guidelines

#### Exposure Time (`exposure_time`)
- **Unit**: Microseconds (μs)
- **Typical range**: 100 - 50000 μs
- **Effect**: 
  - Higher values → Brighter image, more motion blur
  - Lower values → Darker image, less motion blur
- **Recommendations**:
  - Dark environment: 10000 - 30000 μs
  - Normal lighting: 5000 - 10000 μs
  - High-speed capture: 100 - 1000 μs

#### Gain (`gain`)
- **Unit**: Decibels (dB)
- **Typical range**: 0 - 24 dB (camera-dependent)
- **Effect**:
  - Higher values → Brighter image, more noise
  - Lower values → Less noise, darker image
- **Recommendations**:
  - Good lighting: 0 - 10 dB (minimize noise)
  - Medium lighting: 10 - 16 dB
  - Low lighting: 16 - 24 dB (only if necessary)

### Quick Start Examples

**Example 1: Dark environment setup**
```bash
# Method 1: Edit config/camera_params.yaml
exposure_time: 20000
gain: 18.0

# Method 2: Dynamic adjustment
ros2 param set /hik_camera exposure_time 20000
ros2 param set /hik_camera gain 18.0
```

**Example 2: High-speed capture (reduce blur)**
```bash
exposure_time: 1000
gain: 12.0
```

**Example 3: Low noise setup (good lighting required)**
```bash
exposure_time: 5000
gain: 5.0
```

### Checking Valid Parameter Ranges

To find the valid range for your specific camera:

```bash
# Get parameter description with min/max values
ros2 param describe /hik_camera exposure_time
ros2 param describe /hik_camera gain
```

The node will log the valid ranges on startup:
```
[INFO] [hik_camera_node]: Exposure time: 5000 (range: 100.0 to 50000.0)
[INFO] [hik_camera_node]: Gain: 10.5 (range: 0.0 to 24.0)
```

## Calibration Guide

The `hik_camera_calibration_node` runs simultaneously with the camera feed, listens to `image_raw`, and lets you manually capture chessboard poses by watching the built-in visualization window.

### Steps
1. Build the workspace and source the install overlay:
	```bash
	colcon build --packages-select hik_camera
	source install/setup.bash
	```
2. Launch your Hikvision camera node to publish `image_raw` (for example via `ros2 launch hik_camera hik_camera.launch.py`).
3. In a new shell, run the calibration node while pointing the camera at a well-printed chessboard with the same layout as the parameters (defaults match a 12×7 board with 10 mm squares):
	```bash
	ros2 run hik_camera hik_camera_calibration_node --ros-args \
	  -p frames_to_capture:=20 \
	  -p save_path:=/tmp/hik_calibration.yaml
	```
4. A window named `HikCalibration` will open. When the chessboard is detected, press space or `c` to capture that pose (the node logs `Captured X/Y frames`). Once you have enough diverse captures, press `q` to trigger calibration and save the YAML file.

### Parameters

| Name                | Default                   | Description                                                                                    |
| ------------------- | ------------------------- | ---------------------------------------------------------------------------------------------- |
| `board_width`       | `12`                      | Number of inner corners horizontally on the chessboard.                                        |
| `board_height`      | `7`                       | Number of inner corners vertically on the chessboard.                                          |
| `square_size`       | `0.01`                    | Size of one square edge in meters.                                                             |
| `frames_to_capture` | `15`                      | How many good detections to collect before running calibration. More frames improve accuracy.  |
| `save_path`         | `calibration_result.yaml` | Full path of the YAML file where the camera matrix and distortion coefficients will be stored. |

The node keeps listening and overwrites the file each time it collects the required number of frames. Ensure the chessboard covers different parts of the field of view to get accurate intrinsics.
