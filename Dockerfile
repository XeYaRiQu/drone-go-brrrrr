FROM osrf/ros:rolling-desktop-full

ENV HOME /home/user

RUN useradd -m -u 1000 user

USER user

WORKDIR $HOME/catkin_ws/src

