#!/usr/bin/env bash

# Define variables for image and container
REGISTRY="registry.gitlab.com/ant-x/tools/docker-px4"
IMAGE_NAME="px4-dev-ros2-bridge-image"
IMAGE_TAG="latest"
CONTAINER_NAME="px4_ros2_bridge"

# Run the Docker container with specified configurations
docker run -it                                  `# Run the container interactively with a pseudo-TTY` \
    --rm                                        `# Automatically remove the container when it exits` \
    --privileged                                `# Run the container with extended privileges` \
    --ipc=host                                  `# Share the IPC namespace with the host` \
    --pid=host                                  `# Share the PID namespace with the host` \
    --net host                                  `# Use the host's network stack` \
    --volume="${DOCKER_PX4_PATH}/bridge/docker-scripts:/docker-scripts"   `# Mount a volume from the host to the container (host_path:container_path)` \
    --volume="${DOCKER_PX4_PATH}/bridge/docker-scripts/config-files/px4_ros_api/launch:/root/px4_ros_com_ros2/src/px4_ros_api/launch" `# Mount a volume from the host to the container (host_path:container_path)` \
    --volume="${DOCKER_PX4_PATH}/bridge/docker-scripts/config-files/px4_ros_api/params:/root/px4_ros_com_ros2/src/px4_ros_api/params" `# Mount a volume from the host to the container (host_path:container_path)` \
    --env-file "${DOCKER_PX4_PATH}/bridge/docker-scripts/env.sh"  `# Pass environment variables inside the container` \
    --name "${CONTAINER_NAME}"                  `# Set a name for the container` \
    "${REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}"    `# Specify the image to use for the container` \
    /docker-scripts/entrypoint.sh               `# Specify the command to run inside the container`
