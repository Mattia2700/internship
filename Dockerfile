FROM osrf/ros:foxy-ros1-bridge

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV DEBIAN_FRONTEND noninteractive
ENV QT_X11_NO_MITSHM 1

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654


RUN \
  apt-get update && \
  apt-get install -y apt-utils && \
  apt-get upgrade -y

RUN apt-get install -y libgl1-mesa-glx libgl1-mesa-dri mesa-utils dbus 

RUN apt-get install -y nano python3-pip libzmq3-dev 

RUN apt-get install -y file

RUN apt-get install -y build-essential cmake gdb coinor-libcbc-dev coinor-libclp-dev coinor-libcoinutils-dev coinor-libcgl-dev coinor-libcoinutils-dev libgsl-dev bison flex black
  
# RUN   apt-get -y install ros-noetic-depthimage-to-laserscan \

RUN apt-get install -y \
  python3-colcon-common-extensions \
  ros-foxy-test-msgs

RUN apt-get install -y \
  ros-foxy-xacro \ 
  ros-foxy-rqt-robot-steering \
  ros-foxy-nav2-bringup \
  ros-foxy-rviz2 \
  ros-foxy-gazebo-ros-pkgs

RUN apt-get install -y \
  ros-foxy-rosbridge-server \
  ros-foxy-plansys2-*

RUN curl -sL https://deb.nodesource.com/setup_16.x -o /tmp/nodesource_setup.sh && \
  sh /tmp/nodesource_setup.sh && \
  sudo apt-get install -y nodejs 

RUN sed -i 's/(ALL:ALL) ALL/(ALL) NOPASSWD: ALL/' /etc/sudoers # Enable sudo without password

# Add user with home folder
RUN useradd -ms /bin/bash -G sudo ros2
USER ros2
WORKDIR /home/ros2

RUN echo '<?xml version="1.0" encoding="UTF-8" ?><profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><transport_descriptors><transport_descriptor><transport_id>CustomUdpTransport</transport_id><type>UDPv4</type></transport_descriptor></transport_descriptors><participant profile_name="participant_profile" is_default_profile="true"><rtps><userTransports><transport_id>CustomUdpTransport</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports></rtps></participant></profiles>' > fastrtps-profile.xml

ENTRYPOINT ["bash"]
