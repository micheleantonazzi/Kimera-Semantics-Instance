FROM nvidia/cuda:12.1.1-cudnn8-runtime-ubuntu20.04

# Setup nvidia variables
RUN echo "export PATH=/usr/local/cuda-12.1/bin${PATH:+:${PATH}}" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=/usr/local/cuda-12.1/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY}}" >> ~/.bashrc
# Install ROS Noetic desktop full
# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get upgrade -y \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full nano wget ros-noetic-code-coverage ros-noetic-cv-bridge
RUN apt-get install -y --no-install-recommends python3-rosdep build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools 
RUN apt-get install -y --no-install-recommends git
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update

# Python packages
RUN apt install -y python3-pip python3-pcl
RUN pip3 install -U catkin_tools 

# Install specific dependies for the package
RUN apt install -y gdb apturl libbullet-dev python3-wstool protobuf-compiler dh-autoreconf python3-catkin-tools python3-osrf-pycommon

# Install pybind11
# Install pip requirements
COPY requirements.txt /opt/app/requirements.txt
RUN pip3 install -r /opt/app/requirements.txt

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Configure and build ROS workspace

RUN mkdir ~/ros_ws && mkdir ~/ros_ws/src
COPY kimera_semantics_instance.rosinstall /opt/app/kimera_semantics_instance.rosinstall
RUN wstool init ~/ros_ws/src /opt/app/kimera_semantics_instance.rosinstall

WORKDIR /root/ros_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/ros_ws; \ 
                    catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=debug; \
                    catkin build;'
                    
RUN echo "source /root/ros_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "Build complete"