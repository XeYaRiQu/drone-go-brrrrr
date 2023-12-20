FROM osrf/ros:rolling-desktop-full

ENV HOME /home/user

RUN useradd -m -u 1000 user

USER user

WORKDIR $HOME/catkin_ws/src

COPY ./description /$HOME/catkin_ws/src/description
COPY ./launch /$HOME/catkin_ws/src/launch

RUN /bin/bash -c 'cd /catkin_ws/ \
    && source /opt/ros/rolling/setup.bash \
    && rosdep install --from paths src --ignore-src -r -y \
    && colcon build'


CMD ["source", "install", "setup.bash"]
