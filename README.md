# Stereo UVC Camera ROS2 Interface

A ROS 2 package for capturing synchronized stereo images using dual UVC cameras. This package is built on OpenCV GStreamer library to provide configurable camera control and image acquisition.

## Setup

- ROS 2 Jazzy 
- Raspberry Pi 5 with Ubuntu 24.04
- Development packages:


### 1. Install ROS 2 Dependencies

Run the following to automatically install all ROS 2 package dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

> ℹ️ You must have run `rosdep init` and `rosdep update` beforehand. If `libcamera-dev` is not found, see step 2.

### 2. Install Required System Packages

Some dependencies (like `libcamera-dev`) must be installed manually:

```bash
sudo apt update
sudo apt install -y \
  libopencv-dev \
  libcamera-dev \
  libi2c-dev
```

## Features

- Synchronized stereo image capture
- ROS 2 image topic publishing with camera info
- Camera parameter configuration through ROS 2 parameters
- Support for various camera modes and settings
- Stereo camera calibration

## Installation

1. Clone the repository into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url>
   ```

2. Build the package:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select stereo_cam
   source install/setup.bash
   ```

## Usage

1. Launch the stereo camera node:

   ```bash
   ros2 launch stereo_cam stereo_cam.launch.py #this launches depth node as well 
   ```

2. Or run with custom parameters:

   ```bash
   ros2 run stereo_cam stereo_node --ros-args -p width:=1280 -p height:=720
   ```

3. for calibration:

     ```bash
     ros2 run stereo_cam stereo_calib --ros-args -p num_corners_vertical:=6 -p num_corners_horizontal:=4 -p square_size_mm:=30 -p show_chess_corners:=true
     ```

## Topics

The node publishes on the following topics:

- `/camera/left/image_raw` (sensor_msgs/Image)
  - Raw images from the left camera
- `/camera/right/image_raw` (sensor_msgs/Image)
  - Raw images from the right camera
- `/camera/left/camera_info` (sensor_msgs/CameraInfo)
  - Calibration and configuration of the left camera
- `/camera/right/camera_info` (sensor_msgs/CameraInfo)
  - Calibration and configuration of the right camera

## Configuration

Parameters can be configured through:

1. Launch file arguments
2. YAML configuration file
3. Command line parameters

### Available Parameters

- `width` (int, default: 640)
  - Image width in pixels
- `height` (int, default: 480)
  - Image height in pixels
- `frame_rate` (int, default: 30)
  - Camera capture frame rate
- `frame_id` (string, default: "camera_frame")
  - TF frame for the cameras
- `exposure_mode` (string, default: "normal")
  - Camera exposure mode (normal/short/long/custom)
- `gain` (float, default: 1.0)
  - Camera gain in manual mode
- `exposure_time` (int, default: 0)
  - Exposure time in microseconds (0 for auto)

### Configuration File

The package includes a default configuration file at `config/camera_params.yaml`. You can modify this file or create your own:

```yaml
stereo_node:
  ros__parameters:
    width: 640
    height: 480
    frame_rate: 30
    frame_id: "camera_frame"
    exposure_mode: "normal"
    gain: 1.0
    exposure_time: 0
```

To use a custom config file:

```bash
ros2 launch stereo_cam stereo_cam.launch.py config_file:=path/to/config.yaml
```

## Stereo Camera Calibration

### Prerequisites

- Chessboard calibration pattern (default: 6x4 inner corners, 30mm squares)
- Good lighting conditions
- Rigid camera mounting

### Calibration Process

1. Print the chessboard pattern and mount it on a rigid, flat surface

2. Launch the calibration node:
   ```bash
   ros2 run stereo_cam stereo_calib --ros-args \
     -p num_corners_vertical:=6 \
     -p num_corners_horizontal:=4 \
     -p square_size_mm:=30 \
     -p show_chess_corners:=true
   ```

3. Capture calibration images:
   - Hold the chessboard at different angles and distances
   - Press 'k' to save the current stereo pair
   - Aim for 10-20 good pairs
   - Press 'c' to start calibration when ready
   - Press 'q' to quit the calibration node

4. During calibration:
   - The process will:
     - Detect chessboard corners
     - Calculate camera parameters
     - Show rectification results
     - Display feature matches between rectified images

5. Results:
   - Calibration file saved to: `install/stereo_cam/share/stereo_cam/config/StereoCalibration.yaml`
   - Feature matching visualization saved as: `img_matches.jpg`

### Tips for Good Calibration

- Ensure the chessboard is visible in both cameras
- Capture images with chessboard at various:
  - Angles (up to ~45°)
  - Distances
  - Positions in the frame
- Avoid motion blur
- Check feature matches to verify calibration quality

## Troubleshooting

1. No cameras detected:
   - Check camera connections
   - Verify camera permissions
   - Ensure libcamera is properly installed

2. Poor performance:
   - Adjust buffer count
   - Reduce resolution or frame rate
   - Check system resources

3. Synchronization issues:
   - Verify hardware trigger connections
   - Check frame timestamps
   - Adjust buffer settings

## License

This package is released under the GNU General Public License v3.0 (GPLv3). See the [LICENSE](LICENSE) file for details.

This means:
- You can freely use and modify this software
- If you distribute the software or hardware containing this software, you must:
  - Make the source code available
  - License your modifications under GPLv3
  - Provide installation instructions
- Hardware sales are allowed, but software modifications must remain open source

## References

- [libcamera Documentation](https://libcamera.org/docs.html)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [LCCV Library](https://github.com/kbarni/LCCV)



1. Find the video driver file "dev/videoX"
	- ls /dev/video*
	- v4l2:///dev/videoX
	
2. Figure out the usable compression, resolutions, and framerates
	- v4l2-ctl --device=/dev/videoX --list-formats-ext
	
3. To see User Driver controls
	- v4l2-ctl --device=/dev/videoX --list-ctrls-menu