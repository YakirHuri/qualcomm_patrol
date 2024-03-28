FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

# Install ros melodic
RUN apt update && apt upgrade -y && \
    apt install -y lsb-core curl git && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && \
    apt install -y ros-melodic-desktop && \
    apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep && \
    rosdep init && rosdep update && \
    apt-get install libsdl-dev -y && \
    apt-get install libsdl-image1.2-dev -y && \
    apt-get install libbullet-dev -y

# # Install ROS dependencies and packges
RUN apt install -y \
    ros-melodic-ecl-exceptions \
    ros-melodic-ecl-threads \
    ros-melodic-ecl-streams \
    ros-melodic-ecl-geometry \
    ros-melodic-yocs-controllers \
    ros-melodic-yocs-cmd-vel-mux \
    ros-melodic-yocs-velocity-smoother \
    ros-melodic-dwa-local-planner \
    ros-melodic-kobuki-msgs \
    ros-melodic-kobuki-driver \
    ros-melodic-kobuki-dock-drive \
    ros-melodic-tf \
    ros-melodic-image-transport \
    ros-melodic-image-transport-plugins \
    ros-melodic-resource-retriever \
    ros-melodic-cv-bridge \
    # move base installation
    ros-melodic-move-base \
    ros-melodic-base-local-planner \
    ros-melodic-move-base-msgs \
    # gmapping installation
    ros-melodic-gmapping \
    # realesnes installtaion
    ros-melodic-realsense2-camera \
    # webcam driver installation
    ros-melodic-usb-cam \
    # hokuyo lidar driver installtaion
    ros-melodic-urg-node

# Clone rplidar ros package
# RUN git clone https://github.com/Slamtec/rplidar_ros.git src/rplidar_ros