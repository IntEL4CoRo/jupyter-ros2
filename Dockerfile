FROM intel4coro/jupyter-ros2:humble-py3.10

# Install Vulkan
USER root
RUN apt-get update \
    && apt-get install -y \
    libxext6 \
    libvulkan1 \
    libvulkan-dev \
    vulkan-tools
COPY nvidia_icd.json /etc/vulkan/icd.d

# Install Isaac Sim 4.2
# ENV OMNI_KIT_ACCEPT_EULA=YES
# RUN pip install isaacsim==4.2.0.2 --extra-index-url https://pypi.nvidia.com
# RUN pip install isaacsim-extscache-physics==4.2.0.2 \
#      isaacsim-extscache-kit==4.2.0.2 \
#     isaacsim-extscache-kit-sdk==4.2.0.2 \
#     --extra-index-url https://pypi.nvidia.com

# RUN wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | apt-key add - && \
#     echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" | tee /etc/apt/sources.list.d/google-chrome.list && \
#     apt-get update && apt-get install -y google-chrome-stable


# Install Isaac Sim 4.5
USER ${NB_USER}
RUN pip install isaacsim[all]==4.5.0 --extra-index-url https://pypi.nvidia.com
RUN pip install isaacsim[extscache]==4.5.0 --extra-index-url https://pypi.nvidia.com

USER root
RUN apt update && apt install -y libfuse2

USER ${NB_USER}
WORKDIR /home/${NB_USER}/work/tutorials
RUN wget https://download.isaacsim.omniverse.nvidia.com/isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage -O isaac-sim.AppImage && \
    chmod +x isaac-sim.AppImage && \
    ./isaac-sim.AppImage --appimage-extract && \
    mv squashfs-root isaac-sim

ENV OMNI_KIT_ACCEPT_EULA=YES

# Download shaders cache to speed up the first launch (Hardware dependent)
RUN wget https://ivan.informatik.uni-bremen.de/data/ov-cache.tar.gz && \
    tar -xvf ov-cache.tar.gz -C /home/${NB_USER}/.cache/ && \
    rm ov-cache.tar.gz

# --- Copy notebooks --- #
USER ${NB_USER}
COPY --chown=${NB_USER}:users ./ /home/${NB_USER}/work/

# --- Entrypoint --- #
COPY --chown=${NB_USER}:users entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "start-notebook.sh" ]