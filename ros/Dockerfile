# Use a base ROS image (e.g., official ROS images from Docker Hub)
FROM ros:humble-ros-base

# Set the working directory inside the container
WORKDIR /ros_ws

# Install any additional ROS packages or dependencies required by your project
RUN apt-get update && apt-get install -y \
  ros-humble-desktop-full \
  # ros-humble-rosserial \
  # Add other necessary packages here, e.g., ros-humble-slam-toolbox \
  && rm -rf /var/lib/apt/lists/*

# Copy your ROS workspace/packages into the container
COPY . /ros_ws/src

# Build your ROS packages (if applicable)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
  cd /ros_ws && \
  rosdep install --from-paths src --ignore-src -r -y && \
  colcon build"

# Set the entrypoint or command to run when the container starts
# For example, to source ROS setup and run a specific node:
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && /ros_ws/build/serial_node/serial_node"]
