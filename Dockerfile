FROM nvcr.io/nvidia/isaac-sim:4.2.0 AS isaac-sim-4.2

FROM intel4coro/jupyter-ros2:humble-py3.10

# Install Vulkan
USER root
RUN apt-get update \
    && apt-get install -y \
    libxext6 \
    libvulkan1 \
    libvulkan-dev \
    vulkan-tools \
    libfuse2

# --- Install Chrome --- #
WORKDIR /tmp
RUN wget https://commondatastorage.googleapis.com/chromium-browser-snapshots/Linux_x64/1491986/chrome-linux.zip && \
    unzip chrome-linux.zip && \
    rm chrome-linux.zip && \
    mv chrome-linux /usr/local/chrome-linux && \
    ln -s /usr/local/chrome-linux/chrome /usr/local/bin/chrome

USER ${NB_USER}
RUN pip install -U jupyter-remote-desktop-proxy
# install code-server
RUN mamba install -y code-server
RUN echo 'alias code="$(which code-server)"' >> ~/.bashrc
RUN code-server --install-extension ms-python.python \
  && code-server --install-extension ms-toolsai.jupyter
RUN pip install git+https://github.com/yxzhan/jupyter-code-server.git
ENV CODE_WORKING_DIRECTORY=${HOME}/work

# Install Isaac Sim 4.2
USER ${NB_USER}
COPY --chown=${NB_USER}:users --from=isaac-sim-4.2 /isaac-sim /isaac-sim
ENV PRIVACY_CONSENT=Y
ENV ACCEPT_EULA=Y
ENV OMNI_KIT_ACCEPT_EULA=YES

COPY --chown=${NB_USER}:users isaacsim_env.sh ${HOME}
RUN echo "source ${HOME}/isaacsim_env.sh" >> ${HOME}/.bashrc

ENV ISAAC_SIM_DIR="/isaac-sim"
ENV ISAAC_PYTHON_SH="${ISAAC_SIM_DIR}/python.sh"

# Install the BDD
RUN mkdir -p /home/${NB_USER}/behave-isaac-bdd
WORKDIR /home/${NB_USER}/behave-isaac-bdd
RUN git clone https://github.com/minhnh/bdd-dsl.git && \
    git clone https://github.com/minhnh/bdd-isaacsim-exec.git && \
    git clone https://github.com/secorolab/metamodels-bdd.git && \
    git clone https://github.com/minhnh/rdf-utils.git && \
    git clone https://github.com/secorolab/models-bdd.git

RUN ${ISAAC_PYTHON_SH} -m pip install ./rdf-utils && \
    ${ISAAC_PYTHON_SH} -m pip install ./bdd-dsl && \
    ${ISAAC_PYTHON_SH} -m pip install ./bdd-isaacsim-exec

RUN $ISAAC_PYTHON_SH bdd-dsl/examples/generate_bdd_specs.py
RUN mv bdd-dsl/examples/generated/*.feature bdd-isaacsim-exec/examples

# Install Isaac sim webrtc client
RUN wget https://download.isaacsim.omniverse.nvidia.com/isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage -O isaacsim-webrtc.AppImage && \
    chmod +x isaacsim-webrtc.AppImage && \
    ./isaacsim-webrtc.AppImage --appimage-extract && \
    mv squashfs-root isaacsim-webrtc && \
    rm isaacsim-webrtc.AppImage

COPY nvidia_icd.json /etc/vulkan/icd.d

# --- Copy notebooks --- #
USER ${NB_USER}
COPY --chown=${NB_USER}:users tutorials/isaac-sim.ipynb /home/${NB_USER}/behave-isaac-bdd
COPY --chown=${NB_USER}:users tutorials/utils.py /home/${NB_USER}/behave-isaac-bdd

# --- Entrypoint --- #
COPY --chown=${NB_USER}:users entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "start-notebook.sh" ]
