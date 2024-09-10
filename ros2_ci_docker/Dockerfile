FROM invokelee/ros2_galactic:1.0

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install Git
RUN apt update && apt install -y git python3-colcon-common-extensions
RUN apt install -y ros-galactic-rviz2 \
 && apt install -y ros-galactic-robot-state-publisher \
 && apt install -y ros-galactic-joint-* 

# Create a Catkin workspace and clone TurtleBot3 repos
RUN mkdir -p /tortoise_ws/src \
 && cd /tortoise_ws/src

WORKDIR /tortoise_ws/src

COPY ./tortoisebot ./tortoisebot/
COPY ./waypoint_interfaces ./waypoint_interfaces
COPY ./tortoisebot_waypoints ./tortoisebot_waypoints

# Build the Colcon workspace and ensure it's sourced
WORKDIR /tortoise_ws
RUN source /opt/ros/galactic/setup.bash \
    && cd /tortoise_ws \
    && colcon build --packages-select waypoint_interfaces
RUN source install/setup.bash \
    && cd /tortoise_ws \
    && colcon build

RUN echo "source /tortoise_ws/install/setup.bash" >> ~/.bashrc

# Set the working folder at startup
WORKDIR /tortoise_ws

CMD ["bash"]
