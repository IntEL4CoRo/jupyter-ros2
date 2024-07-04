FROM intel4coro/jupyter-ros2:humble-py3.10

ENV ROS_DISTRO=humble
ENV ROS_WS=/home/${NB_USER}/ros2_ws

# Install turtlebot3 packages
USER root
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-gazebo-* \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-turtlebot3* \
    byobu \
    iputils-ping && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN pip install --upgrade lxml

# Need to source the gazebo setup.bash to set up the envrionment variables
RUN echo "source /usr/share/gazebo/setup.bash" >> /home/${NB_USER}/.bashrc
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ENV TURTLEBOT3_MODEL=waffle_pi

# Create ROS workspaces
USER ${NB_USER}

RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}
RUN cd src && \
    git clone -b ${ROS_DISTRO}-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git 

USER root
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src  -y && \
    rosdep fix-permissions

USER ${NB_USER}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --parallel-workers 2
RUN echo "source ${ROS_WS}/install/setup.bash" >> /home/${NB_USER}/.bashrc

COPY --chown=${NB_USER}:users . /home/${NB_USER}/work
WORKDIR /home/${NB_USER}/work
RUN ln -s ${ROS_WS} ROS_WS