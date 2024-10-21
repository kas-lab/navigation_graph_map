FROM ros:humble-ros-core-jammy

RUN apt update && apt install -y --no-install-recommends \
  git \
  vim \
  build-essential \
  python3-rosdep \
  python3-vcstool \
  python-is-python3 \
  python3-pip \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /
COPY install_typedb.sh /install_typedb.sh
RUN chmod +x /install_typedb.sh
RUN ["/bin/bash", "-c", "./install_typedb.sh"]

RUN mkdir -p /navigation_ws/src
COPY navigation_simulation /navigation_ws/src/navigation_simulation
COPY navigation_task_plan /navigation_ws/src/navigation_task_plan
COPY navigation_rosa /navigation_ws/src/navigation_rosa
COPY navigation_kb /navigation_ws/src/navigation_kb
COPY dependencies.rosinstall /navigation_ws/dependencies.rosinstall

WORKDIR /navigation_ws
RUN vcs import src < dependencies.rosinstall --recursive

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
      && apt update \
      && rosdep init \
      && rosdep update \
      && rosdep install --from-paths src --ignore-src -r -y \
      && sudo rm -rf /var/lib/apt/lists/"]

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
      && colcon build --symlink-install"]

ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
