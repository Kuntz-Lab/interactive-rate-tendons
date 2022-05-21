# phusion/baseimage is based on the Ubuntu docker image, but adds functionality
# to make it easy to launch proper system daemons from inside of the docker
# container, like system logging, the init process, etc.
FROM phusion/baseimage:bionic-1.0.0
MAINTAINER mikebentley15@gmail.com

# Update to latest packages
RUN apt-get -y update && \
    apt-get -y upgrade && \
    rm -rf /var/lib/apt/lists/*

# Setup UTF-8 locale
RUN apt-get -y update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install \
      locales \
      && \
    apt-get clean && \
    rm -rf \
      /var/lib/apt/lists/* \
      /tmp/* \
      /var/tmp/* \
      && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US.UTF-8' LC_ALL='en_US.UTF-8'

# Install ROS 2 Eloquent
RUN apt-get -y update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install \
      curl \
      sudo \
      gnupg2 \
      && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
      apt-key add - && \
    echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu bionic main" \
      > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get -y update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install \
      ros-eloquent-rviz2 \
      ros-eloquent-visualization-msgs \
      ros-eloquent-geometry-msgs \
      ros-eloquent-std-msgs \
      ros-eloquent-rclcpp \
      ros-eloquent-ament-cmake \
      ros-eloquent-ament-cmake-gtest \
      ros-eloquent-ament-cmake-ros \
      ros-eloquent-ament-cmake-target-dependencies \
      && \
    apt-get clean && \
    rm -rf \
      /var/lib/apt/lists/* \
      /tmp/* \
      /var/tmp/*

# Automatically include ROS2 Eloquent in bash environment
RUN echo 'if [ -f "/opt/ros/eloquent/setup.bash" ]; then' >> /etc/bash.bashrc && \
    echo '  source "/opt/ros/eloquent/setup.bash"'        >> /etc/bash.bashrc && \
    echo 'fi'                                             >> /etc/bash.bashrc

## Uncomment this command to use rosdep to install all missing ROS
## dependencies.  These dependencies are not necessary for this project.
## Note: adding --include-eol-distros because ROS2 Eloquent is at end of life
#RUN apt-get -y update && \
#    DEBIAN_FRONTEND=noninteractive apt-get -y install \
#      python-rosdep \
#      python3-colcon-common-extensions
#      && \
#    rosdep init && \
#    apt-get -y update && \
#    rosdep update --include-eol-distros && \
#    DEBIAN_FRONTEND=noninteractive rosdep install \
#      --from-paths /opt/ros/eloquent/share \
#      --ignore-src \
#      --rosdistro eloquent \
#      -y \
#      --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers" && \
#    rosdep fix-permissions && \
#    rm -rf /root/.ros && \
#    apt-get clean && \
#    rm -rf \
#      /var/lib/apt/lists/* \
#      /tmp/* \
#      /var/tmp/*

# install packages from our testing PPA package repository
RUN add-apt-repository -y ppa:mikebentley15/testing && \
    apt-get -y update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install \
      armlab-insighttoolkit \
      ll4ma-ompl \
      ll4ma-pybind11 \
      && \
    apt-get clean && \
    rm -rf \
      /var/lib/apt/lists/* \
      /tmp/* \
      /var/tmp/*

# install remaining dependencies
RUN apt-get -y update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install \
      cmake \
      git \
      libboost-filesystem-dev \
      libboost-graph-dev \
      libboost-iostreams-dev \
      libboost-math-dev \
      libboost-system-dev \
      libeigen3-dev \
      libf2c2-dev \
      libfcl-dev \
      libgtest-dev \
      liboctomap-dev \
      libomp-dev \
      libqt5serialport5-dev \
      libreadline-dev \
      python3-dev \
      python3-matplotlib \
      python3-numpy \
      python3-pandas \
      python3-toml \
      qtbase5-dev \
      && \
    echo "compile the google test libraries" && \
    mkdir /tmp/gtest-build && \
    cd /tmp/gtest-build && \
    cmake /usr/src/gtest -DCMAKE_BUILD_TYPE=Debug && \
    make && \
    cp *.a /usr/lib && \
    apt-get clean && \
    rm -rf \
      /var/lib/apt/lists/* \
      /tmp/* \
      /var/tmp/*

ARG user
ARG user_id
ARG user_group_id
ARG user_shell=/bin/bash
ARG user_home=/home/${user}

# Create user with sudo access.  Sudo does not require a password
RUN groupadd --gid ${user_group_id} sudo-nopw && \
    useradd \
      --home-dir ${user_home} \
      --create-home \
      --shell ${user_shell} \
      --gid sudo-nopw \
      --groups sudo,dialout \
      --uid ${user_id} \
      ${user} \
      && \
    passwd -d ${user} && \
    echo "%sudo-nopw ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudo-nopw-group

# Enable SSH in baseimage
RUN rm -f /etc/service/sshd/down

## Optional extra goodies and tools.  Just uncomment what you want.
# RUN echo "Enable man pages and other documentation" && \
#     yes | unminimize && \
#     apt-get -y update && \
#     DEBIAN_FRONTEND=noninteractive apt-get -y install \
#       bash-completion \
#       clang-6.0 \
#       cmake-curses-gui \
#       gdb \
#       gitk \
#       google-mock \
#       graphviz \
#       htop \
#       imagemagick \
#       ipython3 \
#       lsb-release \
#       man-db \
#       mlocate \
#       moreutils \
#       ninja-build \
#       python3-pygments \
#       qtcreator \
#       ruby \
#       screen \
#       subversion \
#       tcl-dev \
#       terminator \
#       tk-dev \
#       tmux \
#       tree \
#       vim \
#       vim-gnome \
#       wget \
#       && \
#     apt-get clean && \
#     rm -rf \
#       /var/lib/apt/lists/* \
#       /tmp/* \
#       /var/tmp/*



# Have some way to know what kind of docker container we're in
RUN echo "tendonenv" > /etc/docker-name
