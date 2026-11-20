FROM ros:jazzy-ros-base@sha256:00b8d47a6e6384a2834e3d01fa87ae767876543baa574f2d64a75c04ea33c698 AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_APT_MIRROR=https://ftp.osuosl.org/pub/ros/packages.ros.org/ros2/ubuntu
RUN sed -i 's|http://ports.ubuntu.com|https://ports.ubuntu.com|g; s|http://archive.ubuntu.com|https://archive.ubuntu.com|g; s|http://security.ubuntu.com|https://security.ubuntu.com|g' /etc/apt/sources.list.d/ubuntu.sources \
    && sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|g" /etc/apt/sources.list.d/ros2.sources \
    && apt-get -o Acquire::Retries=3 -o APT::Update::Error-Mode=any update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-rviz2 ros-jazzy-tf2-ros ros-jazzy-robot-state-publisher \
    python3-colcon-common-extensions python3-pytest python3-yaml ros-jazzy-ament-cmake-pytest \
    libgl1-mesa-dri mesa-utils xvfb xauth curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
ENV LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
WORKDIR /work
COPY docker/entrypoint.sh /hri-entrypoint.sh
ENTRYPOINT ["/hri-entrypoint.sh"]
CMD ["bash"]

# Optional genuine Gazebo GUI over an X11 desktop. Build with --target gui;
# the default final target remains the original headless runtime below.
FROM runtime AS gui
RUN apt-get -o Acquire::Retries=3 -o APT::Update::Error-Mode=any update \
    && apt-get install -y --no-install-recommends \
       x11vnc novnc websockify openbox x11-utils xdotool scrot \
    && rm -rf /var/lib/apt/lists/*
ENV DISPLAY=:99 QT_QPA_PLATFORM=xcb
CMD ["bash", "scripts/run_gui.sh", "forest"]

FROM runtime AS default
