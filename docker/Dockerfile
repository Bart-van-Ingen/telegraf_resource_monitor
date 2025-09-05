# ROS distribution to use
ARG ROS_DISTRO=humble

#####################
# Base Image #
#####################
FROM osrf/ros:${ROS_DISTRO}-desktop AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install all apt packages in one layer and clean up cache
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget socat jq python3-pip lm-sensors \
    && rm -rf /var/lib/apt/lists/* \
    && pip install -U pytest

# Install Telegraf
RUN wget https://dl.influxdata.com/telegraf/releases/telegraf-1.35.3_linux_amd64.tar.gz \
    && tar -xzf telegraf-1.35.3_linux_amd64.tar.gz \
    && rm telegraf-1.35.3_linux_amd64.tar.gz \
    && mv telegraf-1.35.3/usr/bin/telegraf /usr/local/bin/telegraf \
    && chmod +x /usr/local/bin/telegraf

#####################
# Overlay Image #
#####################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws
COPY ./telegraf_resource_monitor/ ./src/telegraf_resource_monitor/
COPY ./resource_monitoring_interfaces/ ./src/resource_monitoring_interfaces/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
    && colcon build --symlink-install \
    && rm -rf /var/lib/apt/lists/*

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE=1

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay AS dev

# Add dev user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Set the ownership of the overlay workspace to the new user
RUN chown -R dev:$dev /ros_ws

# Set the ownership of the XDG_RUNTIME_DIR to the new user for vscode
RUN chown -R dev:dev /tmp/runtime-root

# Switch from root to user
USER $USERNAME

# source our ROS environment not just at container startup, but every time we attach a new
# interactive shell while our dev container remains up.
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
