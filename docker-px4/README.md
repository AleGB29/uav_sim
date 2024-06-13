# Docker PX4 Containers

This repository contains Dockerfiles for Docker Containers designed to build and test PX4.

## Container Hierarchy

- `px4-dev-base-image:latest`
  - `px4-dev-nuttx-image:latest`
  - `px4-dev-simulation-image:latest`
    - `px4-dev-simulation-extra-image:latest`
  - `px4-dev-ros2-bridge-base-image:latest`
    - `px4-dev-ros2-bridge-image:latest`

All the Docker images are hosted under the registry: `registry.gitlab.com/ant-x/tools/docker-px4`.

## Prerequisites

Before getting started, ensure you have Docker installed on your Linux computer. 
It's recommended to use one of the Docker-maintained package repositories for the latest stable version. 
You can choose either the Enterprise Edition or the (free) Community Edition.

For a quick and easy installation on Ubuntu, use the convenience script as shown below:

```bash
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

By default, Docker requires root privileges, but for building PX4 firmware, we recommend using Docker as a non-root user. 
Follow these steps:

```bash
# Create the docker group (may not be required)
sudo groupadd docker
# Add your user to the docker group
sudo usermod -aG docker $USER
# Log out and log in again before using docker
```

## Installation

```bash
cd $HOME
git clone https://gitlab.com/ant-x/tools/docker-px4.git
cd docker-px4
echo export DOCKER_PX4_PATH=$(pwd) >> ~/.bashrc
cd $HOME
git clone -b v1.13.3 https://gitlab.com/ant-x/tools/px4_firmware --recursive
cd px4_firmware
echo export PX4_SRC_ROOT=$(pwd) >> ~/.bashrc
source ~/.bashrc
```

## Building Docker Images

Manually build the base image:

```bash
cd $DOCKER_PX4_PATH/docker
docker build --platform "linux/amd64" --tag "registry.gitlab.com/ant-x/tools/docker-px4/px4-dev-base-image:latest" --file "Dockerfile_base" .
```

Alternatively, use the provided script:

```bash
cd $DOCKER_PX4_PATH/build_scripts
./build_base_image.sh
```

## Firmware

Build PX4 firmware for FMU-v5 architecture:

```bash
cd $DOCKER_PX4_PATH
./run_container.sh 'make px4_fmu-v5_default'
```

The binary will be deployed under the folder `path_to_px4_firmware_root/build/px4_fmu-v5_default/px4_fmu-v5_default.px4`.

## Simulator

To build and run the Gazebo simulator with a single Iris Quadcopter and PX4 configured with the `MAVLink` interface run:

```bash
cd $DOCKER_PX4_PATH
./run_container.sh 'make px4_sitl_default gazebo'
```

The simulator's architecture is the following:

<div align="center">
  <img src="figures/SimulatorMavlink.drawio.svg" width="300">
</div>

The Ground Control Station (GCS) can be run on the host computer and it can be any software enabled to control drones over MAVLink, as [QGroundControl](http://qgroundcontrol.com/).

More details on the SITL architecture can be found [here](https://docs.px4.io/v1.13/en/simulation/).

To build and run the Gazebo simulator with three Iris Quadcopter and PX4 configured with the `MAVLink` interface run:

```bash
cd $DOCKER_PX4_PATH
./run_container.sh './Tools/gazebo_sitl_multiple_run.sh -t px4_sitl_default -m iris -n 3'
```

The simulator's architecture is the following:

<div align="center">
  <img src="figures/SimulatorMultiMavlink.drawio.svg" width="600">
</div>

To build and run the Gazebo simulator with a single Iris Quadcopter and PX4 configured with the `microRTPS` interface run:

```bash
cd $DOCKER_PX4_PATH
./run_container.sh 'make px4_sitl_rtps gazebo'
```

The simulator's architecture is the following:

<div align="center">
  <img src="figures/SimulatorRTPS.drawio.svg" width="300">
</div>

To build and run the Gazebo simulator with three Iris Quadcopter and PX4 configured with the `microRTPS` interface run:

```bash
cd $DOCKER_PX4_PATH
./run_container.sh './Tools/gazebo_sitl_multiple_run.sh -t px4_sitl_rtps -m iris -n 3'
```

The simulator's architecture is the following:

<div align="center">
  <img src="figures/SimulatorMultiRTPS.drawio.svg" width="600">
</div>

For further details on `microRTPS` see [here](https://docs.px4.io/v1.13/en/middleware/micrortps.html).

## Bridge

The Docker Bridge is a Container running [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html) and it is intended to bridge the gap between PX4 and the ROS2 world.
In particular, it runs [microRTPS agent](https://docs.px4.io/v1.13/en/ros/ros2_comm.html) along with the [px4_ros_api](https://gitlab.com/ant-x/tools/px4_ros_api) node which allows the user to arm/disarm, takeoff/land the drone and to send trajectory messages.
Moreover, the `px4_ros_api` takes into account the reference frame conversions between PX4 and ROS2.
The Docker is designed to run with the PX4 Firmware [v1.13.3](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.13.3).

<div align="center">
  <img src="figures/Ros2Bridge.drawio.svg" width="300">
</div>

To run the Bridge Container run:

```bash
cd $DOCKER_PX4_PATH/bridge
./run_bridge.sh
```

Then, you can command the drone with the following command:

  - `Arm`:

  ```bash
  ros2 service call /uav1/px4_ros_api/srv/arm_disarm px4_ros_extra/srv/ArmDisarm "{arm: true}"
  ```

  - `Disarm`:

  ```bash
  ros2 service call /uav1/px4_ros_api/srv/arm_disarm px4_ros_extra/srv/ArmDisarm "{arm: false}"
  ```

  - `Takeoff`:

  ```bash
  ros2 service call /uav1/px4_ros_api/srv/takeoff px4_ros_extra/srv/Takeoff "{takeoff_altitude: 5.0}"
  ```

  - `Land`:

  ```bash
  ros2 service call /uav1/px4_ros_api/srv/land px4_ros_extra/srv/Land "{landing_speed: 0.7}"
  ```

  - `Send trajectory`:

  ```bash
  ros2 topic pub /uav1/px4_ros_api/trajectory/in px4_ros_extra/msg/PoseTarget "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link_1'}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}, body_rate: {x: 0.0, y: 0.0, z: 0.0}, body_rate_dot: {x: 0.0, y: 0.0, z: 0.0}, position: {x: 5.0, y: 0.0, z: 5.0}, velocity: {x: 0.0, y: 0.0, z: 0.0}, acceleration: {x: 0.0, y: 0.0, z: 0.0}, jerk: {x: 0.0, y: 0.0, z: 0.0}, snap: {x: 0.0, y: 0.0, z: 0.0}}"
  ```

  - `Evaluate UAV State:`:

  ```bash
  ros2 topic echo --qos-profile services_default --qos-durability transient_local /uav1/px4_ros_api/state/out
  ```

> **Note:**
> In order to control the drone in Offboard mode you need to set the PX4 parameters:
>  * `COM_RCL_EXCEPT` to `6` (1: Hold + 2: Offboard).
>  * `COM_RC_IN_MODE` to `4` (4: Stick input disabled).

It is also possible to run the Bridge Container configured to launch 3 different instances of `px4_ros_api` bound with 3 different instances of `micrortps_agent`.

```bash
cd $DOCKER_PX4_PATH/bridge
./run_bridge_multi.sh
```

<div align="center">
  <img src="figures/Ros2BridgeMulti.drawio.svg" width="600">
</div>

## Contributions

Contributions are welcome! For suggestions, improvements, or bug reports, refer to the "Issues" section on GitLab.

## License

The Docker PX4 Containers are released under the [BSD-3-Clause License](LICENSE).

## Authors

- Mattia Giurato [mattia@antx.it](mailto:mattia@antx.it)
