#!/usr/bin/env bash

# Define variables for image and container
REGISTRY="registry.gitlab.com/ant-x/tools/docker-px4"
IMAGE_NAME="px4-dev-base-image"
IMAGE_TAG="latest"
PLATFORMS="linux/amd64,linux/arm64"

# Default value for flags
no_cache=""
push=False

# Function to display help message
display_help() {
  echo "Usage: $0 [-n] [-p] [-h]"
  echo
  echo "Build the Docker image $IMAGE_NAME with optional flags."
  echo
  echo "Options:"
  echo "  -n   Enable '--no-cache' option when building the Docker image."
  echo "  -p   Push docker image to registry."
  echo "  -h   Display this help message."
  exit 0
}

# Parse command-line options
while getopts "nph" opt; do
  case $opt in
    n)
      # If -n option is provided, enable --no-cache
      no_cache="--no-cache"
      echo "Building docker image with --no-cache option."
      ;;
    p)
      # Display help message for command-line options
      push=True
      ;;
    h)
      # Display help message for command-line options
      display_help
      ;;
    \?)
      # Display an error message for invalid options
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

# Build the Docker image
cd ../docker

if [[ $push == True ]]; then
  docker buildx build \
      $no_cache \
      --platform "${PLATFORMS}" \
      --tag "${REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}" \
      --file "Dockerfile_base" \
      --push \
      .
  exit 0
fi

docker build \
    $no_cache \
    --tag "${REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}" \
    --file "Dockerfile_base" \
    .
