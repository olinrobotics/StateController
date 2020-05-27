ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO

ARG DOCKER_USER=kubo
ARG LIBSBP_V=2.3.10

MAINTAINER Amy Phung aphung@olin.edu

RUN bash -c \
    'useradd -lmG video $DOCKER_USER \
    && mkdir -p /home/$DOCKER_USER/catkin_ws/src/state_controller'

COPY . /home/$DOCKER_USER/catkin_ws/src/state_controller/

RUN bash -c \
# General Setup
    'apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y wget \
    && apt-get install -y sudo \
# Set up catkin workspace
    && cd /home/$DOCKER_USER/catkin_ws \
    && rosdep update \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep install -iry --from-paths src \
    && cd /home/$DOCKER_USER/catkin_ws \
# Build package
    && catkin_make -j1'

WORKDIR /home/$DOCKER_USER/catkin_ws
USER $DOCKER_USER

WORKDIR /home/$DOCKER_USER
