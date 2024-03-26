# Use the previously created image with ROS dependencies as the base image
FROM ros-noetic:latest

USER ros

COPY entrypoint.sh /home/ros/entrypoint.sh
RUN  sudo chown ros:ros /home/ros/entrypoint.sh
RUN  sudo chmod +x /home/ros/entrypoint.sh

# Create the Catkin Workspce
RUN mkdir -p /home/ros/catkin_ws/src

# Set working directory
WORKDIR /home/ros/catkin_ws

# Start ROS environment
ENTRYPOINT ["/home/ros/entrypoint.sh"]

# Use /bin/bash as the default container command
CMD ["/bin/bash"]