#! /bin/bash

# Check if PX4_DOCKER_IMAGE variable is not set
if [ -z ${PX4_DOCKER_IMAGE+x} ]; then
    echo "Guessing PX4_DOCKER_IMAGE based on input";

    # Guess PX4_DOCKER_IMAGE based on input arguments
    if [[ $@ =~ .*px4_fmu.* ]]; then
        PX4_DOCKER_IMAGE="registry.gitlab.com/ant-x/tools/docker-px4/px4-dev-nuttx-image:latest"
    elif [[ $@ =~ .*gazebo* ]]; then
        PX4_DOCKER_IMAGE="registry.gitlab.com/ant-x/tools/docker-px4/px4-dev-simulation-image:latest"
    elif [[ $@ =~ .*jsbsim* ]]; then
        PX4_DOCKER_IMAGE="registry.gitlab.com/ant-x/tools/docker-px4/px4-dev-simulation-extra-image:latest"
    fi
else
    echo "PX4_DOCKER_IMAGE was already set";
fi

# Set default value for PX4_DOCKER_IMAGE if not yet set
if [ -z ${PX4_DOCKER_IMAGE+x} ]; then
    PX4_DOCKER_IMAGE="registry.gitlab.com/ant-x/tools/docker-px4/px4-dev-nuttx-image:latest"
fi

# Display the final value of PX4_DOCKER_IMAGE
echo "PX4_DOCKER_IMAGE: $PX4_DOCKER_IMAGE";

# Docker cleanup commands (commented out)
#docker rm $(docker ps -a -q)  # Delete all stopped containers (including data-only containers)
#docker rmi $(docker images -q -f dangling=true)  # Delete all 'untagged/dangling' (<none>) images

# Set environment variables and launch Docker container
PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=${PX4_SRC_ROOT}

CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

xhost +

# Run Docker container with specified environment variables and volumes
docker run  -it                                         `# Run the container interactively with a pseudo-TTY` \
            --privileged                                `# Run the container with extended privileges` \
            --ipc=host                                  `# Share the IPC namespace with the host` \
            --pid=host                                  `# Share the PID namespace with the host` \
            --net=host                                  `# Use the host's network stack` \
            --name "px4_container"                      `# Set a name for the container` \
            --rm                                        `# Automatically remove the container when it exits` \
            --workdir="${SRC_DIR}"                      `# Set the working directory inside the container` \
            --volume=${CCACHE_DIR}:${CCACHE_DIR}:rw     `# Mount the volume to access CCACHE_DIR with read/write permissions` \
            --volume=${SRC_DIR}:${SRC_DIR}:rw           `# Mount the volume to access SRC_DIR with read/write permissions` \
            --volume=/tmp/.X11-unix:/tmp/.X11-unix:ro   `# Mount the host's X11 UNIX socket in read-only mode` \
            --env=DISPLAY                               `# Set the DISPLAY environment variable` \
            --env=LOCAL_USER_ID="$(id -u)"              `# ciao carlo`                            \
            ${PX4_DOCKER_IMAGE}                         `# Specify the Docker image to run` \
            /bin/bash -c "$1 $2 $3"                     `# Specify the command to execute inside the container`
