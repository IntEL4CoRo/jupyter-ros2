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

# Install Isaac Sim 4.5
USER ${NB_USER}
RUN pip install isaacsim[all]==4.5.0 --extra-index-url https://pypi.nvidia.com
RUN pip install isaacsim[extscache]==4.5.0 --extra-index-url https://pypi.nvidia.com

USER root
RUN apt update && apt install -y libfuse2

USER ${NB_USER}
RUN pip install -U jupyter-remote-desktop-proxy
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