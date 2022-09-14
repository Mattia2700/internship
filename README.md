# Drone Surveillance Internship Project with ground robots

This repository contains the code required to start a working surveillance system making use of ground robots. The project is entirely developed with ``ROS2 Foxy Fitzroy``.

The project is subdivided into 3 main parts:
- ``robot``: contains the code for the *shelfino* robot, already developed by researchers previously;
- ``navigation``: contains the code needed to navigate the robot, developed by me;
- ``planning``: contains the code needed for mission planning and web interface, developed by [Filippo Rossi](https://github.com/notfilippo), with some modifications by me.

## Installation

### Manual installation

To install code and utils dependencies, you can type:
```shellscript
sudo apt install -y libgl1-mesa-glx libgl1-mesa-dri mesa-utils dbus nano python3-pip libzmq3-dev file bluez bluetooth build-essential cmake gdb coinor-libcbc-dev coinor-libclp-dev coinor-libcoinutils-dev coinor-libcgl-dev coinor-libcoinutils-dev libgsl-dev bison flex black rfkill
```

To install ``ROS2`` dependencies, just type:

```shellscript
sudo apt install -y python3-colcon-common-extensions ros-foxy-test-msgs ros-foxy-xacro ros-foxy-rqt-robot-steering ros-foxy-nav2-bringup ros-foxy-rviz2 ros-foxy-gazebo-ros-pkgs ros-foxy-joy ros-foxy-teleop-twist-joy ros-foxy-rosbridge-server ros-foxy-plansys2-*
```

For launching the web interface, you will need **Node.js**. To install it, just type:

```shellscript 
curl -sL https://deb.nodesource.com/setup_16.x -o /tmp/nodesource_setup.sh && sudo sh /tmp/nodesource_setup.sh && sudo apt-get install -y nodejs 
```

### Automatic installation

Here, it is also provided a ``Dockerfile`` to build a docker image with all the dependencies installed. To build the image, just type:

```shellscript
docker build -t demo:latest .
```

Then, you can manually start a container with 

```shellscript
docker run -it --privileged --rm -e DISPLAY=$DISPLAY --net host -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v /dev/dri/card0:/dev/dri/card0 -v <path-to-this-repo>:/home/ros2 -v /dev/input:/dev/input --name demo demo
```

and eventually attach to it another terminal with

```shellscript
docker exec -it demo /bin/bash
```

## Usage

If you start the container manually, you can make use of the ``Makefile`` available in each folder to build and launch the code.

Or, if you want to automatically start only what you need, you can use the ``start.py``, ``attach.py`` and ``stop.py`` scripts that makes use of ``docker-compose``. 

### Start

```shellscript
usage: start.py [-h] [-r] [-g] [-m] [-n] [-s] [-p] [-d]

start desired groups of nodes

optional arguments:
  -h, --help        show this help message and exit
  -r, --robot       start robot nodes
  -g, --gazebo      start simulation nodes
  -m, --map         save slam map
  -n, --navigation  start navigation nodes
  -s, --slam        start slam nodes
  -p, --planning    start planning nodes
  -d, --detached    run containers in detached mode
```

It starts from ``template.yaml`` file, extracts only the desired groups of nodes and starts them. In this way you are able to specify which nodes you want to start, since you may want to start the nodes on different devices.

> **_NOTE:_**  When executing multiple containers, _shared memory_ must be disabled to permit inter-container communication: ``fastrtps-profile.xml`` is used to this purpose and an environment variable referring to it, is set in ``.bashrc``.

### Attach

```shellscript
usage: attach.py [-h] [-r | -b | -g | -m | -n | -s | -p | -u | -w]

attach to a desired container

optional arguments:
  -h, --help        show this help message and exit
  -r, --robot       attach to robot container
  -b, --bridge      attach to ROS1-ROS2 bridge node
  -g, --gazebo      attach to simulation container
  -m, --map         attach to map container
  -n, --navigation  attach to navigation container
  -s, --slam        attach to slam container
  -p, --planning    attach to planning container
  -u, --ugv         attach to UGV actions container
  -w, --webserver   attach to webserver container
```

If something goes wrong and you want to debug it, you can attach to the desired container easily.

### Stop

```shellscript
usage: stop.py [-h]

remove running or stopped containers

optional arguments:
  -h, --help  show this help message and exit
```
