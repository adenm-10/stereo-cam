# Stereo UVC Camera ROS2 Driver

![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-lightgrey)
![License](https://img.shields.io/badge/License-GPLv3-blue)
![Build](https://img.shields.io/badge/Build-Colcon-success)

<p align="center">
  <img src="https://github.com/user-attachments/assets/80bddc45-abff-48a8-97b4-81c3673724d7" />
</p>


---

A low-computation, high-performance synced stereo vision driver for UVC cameras and ROS2. This package captures synchronized images from dual UVC cameras using OpenCV GStreamer and publishes them to ROS topics via image_transport, with support for real-time stereo odometry via RTAB-Map. Designed and tested on the Raspberry Pi 5.

> Successfully used to map a 4-aisle research building with 10+ waypoints using full RTAB-Map stereo odometry on a Raspberry Pi 5. Full project info can be found [here](https://github.com/ucf-sd-spsu25g10), and a demo video found [here](https://youtu.be/ynSF1JhdaX4?t=409).

---

## 1. Key Features

* Software-level synchronized stereo capture
* Raw image publishing using `image_transport`
* Full camera parameter configuration
* Real-time disparity, depth image, and point cloud generation
* RTAB-Map stereo odometry integration
* In-package calibration and RMS error pruning tool

---

## 2. Quick Start

```bash
# Clone + build
cd ~/ros2_ws/src
git clone git@github.com:adenm-10/stereo-cam.git
cd ..
colcon build --packages-select stereo_cam
source install/setup.bash
```

---

## 3. Setup

### Requirements

* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
* [Ubuntu 24.04](https://ubuntu.com/download/raspberry-pi)
* Raspberry Pi 5 (or compatible SBC)

### Install ROS 2 Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

> ⚠️ Run `rosdep init` and `rosdep update` beforehand. If `libcamera-dev` isn't found, see below.

### Install Required System Packages

```bash
sudo apt update
sudo apt install -y \
  libopencv-dev \
  libcamera-dev \
  libi2c-dev
```

---

## 4. Configuration

### 1. stereo_node
---
#### Parameter List

| Parameter             | Description |
|-----------------------|-------------|
| `left_device_file`    | Path to the left camera device (e.g., `/dev/video0`) |
| `right_device_file`   | Path to the right camera device (e.g., `/dev/video4`) |
| `left_payload_type`   | Video payload format for left camera (e.g., `MJPG`, `YUYV`, `H264`) |
| `right_payload_type`  | Same as above, for right camera |
| `image_width`         | Image width in pixels |
| `image_height`        | Image height in pixels |
| `frame_rate`          | Desired framerate in Hz |
| `gst_pixel_format`    | GStreamer pixel format (e.g., `BGR`, `GRAY8`) |
| `it_pixel_format`     | ROS ImageTransport pixel format (e.g., `bgr8`, `mono8`) |
| `frame_id`            | Frame ID to assign in published image headers |
| `calibration_file`    | Path to stereo calibration YAML file (`package://` supported) |
| `enable_depth`        | Whether to launch the DepthNode from within the StereoNode |

#### Finding the Correct Parameters
1. `*_device_file`: To find the video driver file "dev/video*" for both cameras
   - `ls /dev/video* `
    	- See what video files are available
   - `v4l2:///dev/video* `
    	- Open them in VLC to see which are the ones your cameras are using
  
2. To Figure out the compatible compression algorithms, resolutions, and framerates you can use
   - Relevant parameters:
     - `*_payload_type`
     - `image_width`
     - `image_height`
     - `frame_rate`
   - `v4l2-ctl --device=/dev/video* --list-formats-ext`
     - Run for both cameras, find a common configuration and use that for both

### 2. EKF
---
  The EKF node's and appropriate documentation can be find in the EKF files themselves at `config/odom/ekf_*.yaml`, along with in [depth documentation](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/params/ekf.yaml) in the original `robot_localization/params/ekf.yaml` file on the robot_localization repository.

### 3. rtabmap_odom
---
  The rtabmap_odom node has ~350 parameters, and thus, is a bit difficult to configure. That said, notes on my configurations (mostly to minimize computation), can be found in the `config/odom/rtab_odom_stereo.yaml` file. 
  
  To get more info on rtabmap_odom parameters and usage, I recommend reading the demos and examples on the rtabmap_ros repository, and reading through the parameter descriptions in the referenced yaml file.

## 5. Usage

### Stereo Calibration

```bash
ros2 launch stereo_cam stereo_calib.launch.py
```

### Run Synced Stereo Driver

```bash
ros2 launch stereo_cam stereo_cam.launch.py
```

### Full Stereo Odometry Pipeline

```bash
ros2 launch stereo_cam image_proc_pipeline.launch.py
```

Includes image rectification, optional disparity generation, and stereo odometry node.

---

## 6. Stereo Calibration Guide

### Requirements

* Chessboard: 6x4 inner corners, 30mm squares
* Stable mounting, good lighting

### Calibration Commands

```bash
ros2 run stereo_cam stereo_calib --ros-args \
  -p num_corners_vertical:=6 \
  -p num_corners_horizontal:=4 \
  -p square_size_mm:=30 \
  -p show_chess_corners:=true
```

### Keyboard Controls

* `k`: Save current stereo pair
* `c`: Run calibration
* `q`: Quit node
* `f`: Flag poor image pairs (RMS contribution)

### Output

* YAML saved to: `install/stereo_cam/share/stereo_cam/config/StereoCalibration.yaml`
* Feature match image: `img_matches.jpg`
* NOTE: For some reason the baseline calculations are off for me by 10^3. I had to manually reduce the fourth element in P2 from e+04 to e+01

> 💡 Take 30–40 pictures. Use RMS analysis to remove \~10–15 poor pairs for accurate calibration.
---

## 7. Camera Tuning Scripts

Supports `v4l2-ctl` tuning for:

* brightness
* contrast
* saturation
* hue
* white\_balance\_temperature
* auto\_exposure / exposure\_time\_absolute
* sharpness / gamma / gain

Full list:

```bash
v4l2-ctl --device=/dev/videoX --list-ctrls-menu
```

---

## 8. Troubleshooting

1. No cameras detected:
   - **Check payload type, resolution, and framerate compatibility for both cameras**
   - Verify camera permissions
   - Ensure all packages are properly installed

2. Poor performance:
   - Reduce resolution or frame rate
   - Check system resources
   - For rtabmap, this resource shows how to tune for minimal computation
     - https://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning
   - For ROS2 performance tuning:
     - https://www.stereolabs.com/docs/ros2/dds_and_network_tuning
   - Calibration:
     - For some reason the baseline calculations are off for me by 10^3. I had to manually reduce the fourth element in P2 from e+04 to e+01

3. Synchronization issues:
   - Normally due to high RAM and CPU usage

---

## 9. License

This package is released under the GNU General Public License v3.0 (GPLv3). See the [LICENSE](LICENSE) file for details.

This means:
- You can freely use and modify this software
- If you distribute the software or hardware containing this software, you must:
  - Make the source code available
  - License your modifications under GPLv3
  - Provide installation instructions
- Hardware sales are allowed, but software modifications must remain open source

---

## 10. References

* [ROS 2 Docs](https://docs.ros.org/en/jazzy/)
* [OpenCV GStreamer](https://gstreamer.freedesktop.org/documentation/?gi-language=c)
* [image\_pipeline](https://docs.ros.org/en/ros2_packages/rolling/api/image_pipeline/)
* [rtabmap\_ros](https://github.com/introlab/rtabmap_ros/tree/ros2)
* [stereo\_image\_proc](https://docs.ros.org/en/ros2_packages/rolling/api/stereo_image_proc/doc/index.html)

---

Developed by [Aden McKinney](https://github.com/adenm-10)     
UCF Computer Engineering | Former Robotics Intern @ NASA JPL / MIT / NASA KSC / Lockheed Martin
