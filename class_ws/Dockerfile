FROM althack/ros:noetic-full
# Install dependencies.
RUN apt-get update -qq && apt-get install -y  build-essential \
    ffmpeg libsm6 libxext6 autoconf libtool mesa-utils \
    terminator nano git wget curl iputils-ping \
    libcanberra-gtk-module libcanberra-gtk3-module \
    ros-dev-tools 
    
# Additional ROS tools
RUN apt install -y ros-noetic-teleop-twist-keyboard 

# Gazebo classic install
RUN apt install -y ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins && \
    curl -sSL http://get.gazebosim.org | sh

RUN apt-get update && apt-get install -y ros-noetic-rqt ros-noetic-rqt-common-plugins

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

ENTRYPOINT [ "/bin/bash", "-l", "-c" ]