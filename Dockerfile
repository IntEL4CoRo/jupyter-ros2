FROM jupyter/minimal-notebook:python-3.10

# --- Give NB_USER sudo permission --- #
USER root
RUN apt update && apt install -y sudo && \
    echo "jovyan ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/jovyan && \
    chmod 0440 /etc/sudoers.d/jovyan

# --- Install basic tools --- #
USER root
RUN  apt update -qq && apt install -y -qq \
        software-properties-common \
        gnupg2 \
        curl \
        wget \
        vim \
        git \
        byobu \
        tmux \
        net-tools\
        ca-certificates \
        apt-transport-https \
        build-essential \
        locales \
        lsb-release

# --- Install VNC server and XFCE desktop environment --- #
USER root
RUN apt-get -y -qq update \
 && apt-get -y -qq install \
        dbus-x11 \
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
 && mkdir -p /opt/install \
 && chown -R $NB_UID:$NB_GID $HOME /opt/install 

# Install a VNC server (TurboVNC)
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
    rm -rf /var/lib/apt/lists/*; \
    rm /etc/apt/sources.list.d/TurboVNC.list;

# Install VNC jupyterlab extension
USER ${NB_USER}
RUN mamba install -y websockify
ENV DISPLAY=:1

# --- Install ROS2 --- #
USER root
ENV ROS_DISTRO=humble
ARG ROS_PKG=desktop
ENV ROS_PATH=/opt/ros/${ROS_DISTRO}
ENV ROS_ROOT=${ROS_PATH}/share/ros
ENV ROS_WS=${HOME}/ros2_ws

# Set locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

USER root
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        ros-dev-tools \
        ros-${ROS_DISTRO}-${ROS_PKG} && \
    apt clean && \
    echo "source ${ROS_PATH}/setup.bash" >> /home/${NB_USER}/.bashrc
RUN rosdep init && \
    rosdep update && \
    rosdep fix-permissions

# --- Install python packages --- #
USER ${NB_USER}
RUN pip install --upgrade \
        jupyterlab \
        ipywidgets \
        jupyter-resource-usage \
        jupyter-server-proxy \
        jupyterlab-git \
        jupyter-remote-desktop-proxy\
        jupyter_offlinenotebook \
        Pillow \
        rosdep \
        sidecar \
        lark \
        catkin_tools \
        colcon-common-extensions \
        PyQt5 \
        PySide2 \
        matplotlib \
        opencv-python \
    && pip cache purge

RUN pip install setuptools==58.2.0

# --- Install VSCode server --- #
USER ${NB_USER}
RUN mamba install -y conda-forge::code-server
RUN echo 'alias code="$(which code-server)"' >> ~/.bashrc
RUN code-server --install-extension ms-python.python \
  && code-server --install-extension ms-toolsai.jupyter
RUN pip install git+https://github.com/yxzhan/jupyter-code-server.git
ENV CODE_WORKING_DIRECTORY=${HOME}

# --- Copy notebooks --- #
USER ${NB_USER}
WORKDIR ${HOME}/work
COPY --chown=${NB_USER}:users ./ ${HOME}/work

# --- Entrypoint --- #
COPY --chown=${NB_USER}:users entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "start-notebook.sh" ]
