FROM jupyter/minimal-notebook:python-3.10

# --- Define Environment Variables--- #
ENV ROS_DISTRO=humble
ARG ROS_PKG=desktop
LABEL version="ROS-${ROS_DISTRO}-${ROS_PKG}"

ENV ROS_PATH=/opt/ros/${ROS_DISTRO}
ENV ROS_ROOT=${ROS_PATH}/share/ros

# --- Install basic tools --- #
USER root
RUN  apt update -q && apt install -y \
        software-properties-common \
        gnupg2 \
        curl \
        git \
        wget \
        vim \
        nano \
        net-tools\
        ca-certificates \
        apt-transport-https \
        build-essential \
        lsb-release

# --- Install Oh-my-bash --- #
USER ${NB_USER}
RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)" --unattended
COPY --chown=${NB_USER}:users ./bashrc.sh /home/${NB_USER}/.bashrc

# Set locale
USER root
RUN apt update && \
    apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# --- Install ROS2 --- #
USER root
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        ros-dev-tools \
        ros-${ROS_DISTRO}-${ROS_PKG} \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs && \
    apt clean && \
    echo "source ${ROS_PATH}/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_PATH}/setup.bash" >> /home/${NB_USER}/.bashrc

# --- rosdep init --- #
RUN rosdep init && \
    rosdep update && \
    rosdep fix-permissions

# --- Install ROS packages for tutorials (Optional)--- #
USER root
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-webots-ros2 \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-tiago-bringup \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-urdf-launch \
    ros-${ROS_DISTRO}-urdf-tutorial \
    ros-${ROS_DISTRO}-turtle-tf2-py \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations

# --- Install VNC server and XFCE desktop environment --- #
USER root
RUN apt-get -y -qq update \
 && apt-get -y -qq install \
        dbus-x11 \
        firefox \
        tmux \
        xfce4 \
        xfce4-panel \
        xfce4-session \
        xfce4-settings \
        xorg \
        gnome-shell \
        gnome-session \
        gnome-terminal \
        xubuntu-icon-theme \
        fonts-dejavu \
    # Disable the automatic screenlock since the account password is unknown
 && apt-get -y -qq remove xfce4-screensaver \
    # chown $HOME to workaround that the xorg installation creates a
    # /home/jovyan/.cache directory owned by root
    # Create /opt/install to ensure it's writable by pip
 && mkdir -p /opt/install \
 && chown -R $NB_UID:$NB_GID $HOME /opt/install \
 && rm -rf /var/lib/apt/lists/*

# Install a VNC server, (TurboVNC)
ENV PATH=/opt/TurboVNC/bin:$PATH
RUN echo "Installing TurboVNC"; \
    # Install instructions from https://turbovnc.org/Downloads/YUM
    wget -q -O- https://packagecloud.io/dcommander/turbovnc/gpgkey | \
    gpg --dearmor >/etc/apt/trusted.gpg.d/TurboVNC.gpg; \
    wget -O /etc/apt/sources.list.d/TurboVNC.list https://raw.githubusercontent.com/TurboVNC/repo/main/TurboVNC.list; \
    apt-get -y -qq update; \
    apt-get -y -qq install \
        turbovnc \
    ; \
    rm -rf /var/lib/apt/lists/*;

# Install VNC jupyterlab extension
USER $NB_USER
RUN mamba install -y websockify
RUN pip install git+https://github.com/yxzhan/jupyter-remote-desktop-proxy.git@main
ENV DISPLAY=:1

# --- Install python packages --- #
USER ${NB_USER}
RUN pip install --upgrade \
        jupyterlab~=4.2.0 \
        ipywidgets \
        jupyter-resource-usage \
        jupyter-server-proxy \
        jupyterlab-git \
        jupyter_offlinenotebook \
        Pillow \
        rosdep \
        sidecar \
        lark \
        catkin_tools \
        colcon-common-extensions \
    && pip cache purge

# --- Install VScode --- #
USER root
RUN apt-get install wget gpg && \
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
    echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null && \
    rm -f packages.microsoft.gpg && \
    apt install apt-transport-https && \
    apt update && \
    apt install code

# --- Install bash kernel --- #
USER ${NB_USER}
RUN pip install bash_kernel
RUN python -m bash_kernel.install

# --- Copy notebooks --- #
USER ${NB_USER}
WORKDIR /home/${NB_USER}/work/tutorials
COPY --chown=${NB_USER}:users ./ /home/${NB_USER}/work/

# --- Entrypoint --- #
COPY --chown=${NB_USER}:users entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "start-notebook.sh" ]
