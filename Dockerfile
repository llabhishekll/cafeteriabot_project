# base image for docker
FROM osrf/ros:humble-desktop-focal

# set environment variable
ENV DEBIAN_FRONTEND=noninteractive

# install core dependency
RUN apt-get update && apt-get install -y \
    git tree nano htop curl iputils-ping \
    python3 python3-pip python-is-python3

# install ros dependency
RUN apt-get install -y \
    ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-cartographer ros-humble-cartographer-ros ros-humble-gazebo-plugins \
    ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy ros-humble-xacro \
    ros-humble-nav2* ros-humble-urdf ros-humble-v4l2-camera

# setup workspace
RUN git clone -b ros2 https://github.com/llabhishekll/cafeteriabot_project.git /ros2_ws/src/
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; cd /ros2_ws; colcon build"

# add underlay and overlay workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# setup entrypoint
COPY ./ros_entrypoint.sh /

# set working directory
WORKDIR /ros2_ws

# expose port
# https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html
EXPOSE 7400-8000/udp

# entry point
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "cafeteriabot_firmware", "cafeteriabot_control.launch.py"]