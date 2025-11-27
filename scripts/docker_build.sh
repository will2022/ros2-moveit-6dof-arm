#!/bin/bash
# Build the workspace inside Docker container

echo "Building ROS 2 workspace in Docker..."
# Navigate to the directory containing docker-compose.yml
cd "$(dirname "$0")/.." || exit 1 

docker-compose run --rm ros2_jazzy bash -c "cd /workspace && colcon build && echo 'Build complete!'"
