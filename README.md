# Kalibr2

A modern multi-camera calibration toolbox built on ROS 2.

## Introduction

Kalibr2 is a camera calibration toolbox that provides:

- **Multi-Camera Calibration**: Intrinsic and extrinsic calibration of multi-camera systems with support for various camera models (pinhole, omni-directional, EUCM, double sphere)
- **ROS 2 Integration**: Native support for ROS 2
- **Modern C++**: Built with C++17 and modern CMake practices

This is a modernized version based on the original [Kalibr](https://github.com/ethz-asl/kalibr) calibration toolbox developed at ETH Zurich.

## Build it using docker

```bash
cd docker/
docker compose build kalibr2_ros
docker compose run kalibr2_ros
```

## Build it directly on Ubuntu 24.04 (noble)

### Prerequisites
- **ROS 2 Jazzy Desktop** - Install following the [official ROS 2 installation instructions](https://docs.ros.org/en/jazzy/Installation.html)

### Install System Dependencies
```bash
# Install all required packages
sudo apt update && sudo apt install -y $(cat /tmp/packages.txt)
```

### Build
```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Usage

### Camera Calibration
```bash
# Calibrate cameras from a ROS 2 bag file
ros2 run kalibr2_ros kalibr_calibrate_cameras \
  --config path/to/calibration_config.yaml \
  --output-dir path/to/output/directory
```

### Calibration file example
The following example describes a calibration configuration for a stereo pair using the `pinhole` camera model with radial tangential distortion model.

It consumes a dataset from a recorded rosbag with an aprilgrid target of 6x5 tags and the corresponding sizes.

```yaml
board:
  target_type: 'aprilgrid' # grid type
  tagCols: 6               # number of apriltags in the x direction
  tagRows: 5               # number of apriltags in the y direction
  tagSize: 0.088           # size of apriltag, edge to edge [m]
  tagSpacing: 0.2954       # ratio of space between tags to tagSize

cameras:
  camera_1_name:
    model: 'pinhole-radtan'
    focal_length_fallback: 881.0
    source:
      rosbag_path: '/path/to/your.mcap'
      topic: '/camera_1_topic/image'
  camera_2_topic:
    model: 'pinhole-radtan'
    focal_length_fallback: 881.0
    source:
      rosbag_path: '/path/to/your.mcap'
      topic: '/camera_2_topic/image'
```

### Output files
The calibration files are generated after a succesful run as follows.

#### Intrinsics
- `calibration_<camera_name>.yaml` - Camera intrinsics in ROS [CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html) format.

#### Extrinsics
- For 2-camera systems: `transform_<camera_0>_to_<camera_1>.yaml` - Baseline transform in ROS [TransformStamped](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/TransformStamped.html) format.
- For multi-camera systems
`camera_chain_transforms.yaml` - Chain of baselines transforms in ROS [TFMessage](https://docs.ros.org/en/jazzy/p/tf2_msgs/msg/TFMessage.html) format.

## Extended parameters
There a number of configurations available in the tool that might come in handful.

```
kalibr_calibrate_cameras - Calibrate multiple cameras from ROS bag data
Usage: /home/frn/kalibr/install/kalibr2_ros/lib/kalibr2_ros/kalibr_calibrate_cameras [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -c,--config TEXT:FILE REQUIRED
                              Full path to calibration configuration YAML file.
  -o,--output-dir TEXT:DIR REQUIRED
                              Directory to save the calibration results.
  --approx-sync-tolerance FLOAT
                              Tolerance for approximate synchronization of observations across cameras (in seconds).
  --mi-tol FLOAT              The tolerance on the mutual information for adding an image. Higher means fewer images will be added. Use -1 to force all images.
  --max-batches UINT          Maximum number of batches to accept during incremental calibration. If not specified, all batches will be processed.
  --max-observations UINT     Maximum number of target observations to extract per camera. If not specified, all observations will be extracted.
  --verbose                   Enable verbose output during calibration.
```

## References

The calibration approaches in Kalibr2 are based on the original Kalibr toolbox. Please cite the appropriate papers when using this toolbox in academic work:

1. Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
2. Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.

## Original Kalibr Authors
- Paul Furgale
- Hannes Sommer
- Jérôme Maye
- Jörn Rehder
- Thomas Schneider
- Luc Oth

## License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

1. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

1. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

1. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
