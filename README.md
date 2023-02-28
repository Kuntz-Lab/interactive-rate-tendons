# Tendon Experiments

This repository is a code companion to the paper from Michael Bentley, D. Caleb Rucker, and Alan Kuntz titled "Interactive-rate Supervisory Control for Arbitrarily-routed Multi-tendon Robots via Motion Planning," published in the IEEE Access journal in 2022.

- [Website](https://sites.google.com/gcloud.utah.edu/armlab-tendon-planning)
- [IEEE Access Paper (public)](https://ieeexplore.ieee.org/document/9843990)

[![Watch the video](images/tendon-video-thumbnail.png)](https://youtu.be/i-04P0I36Z4)

This code enables someone to reproduce the motion-planning experiments
described in that paper that touts more than a 17,000x speedup against prior
state-of-the-art approaches.  All are welcome to adopt these strategies and
reuse this code for their purposes for research, commercial, or personal use,
as long as credit is given.

## Goal

The goal of this work is

- Faster forward kinematic computations
- Apply a motion planner in a simulated environment

## Development Environment

The experiments were performed using Ubuntu 18.04 with some updated packages posted on my PPA.

Dependencies:

- Open Motion Planning Library (OMPL) version 1.5.0 (available from my PPA)
- Pybind11 version 2.4 (available from my PPA)
- Insight Toolkit (ITK) 5.1+ (available from my PPA)
- CMake 3.5+
- Boost 1.65+
    - filesystem
    - graph
    - iostreams
    - math
    - system
- Eigen3
- libf2c2
- FCL (between 0.5 and 0.7)
- ROS2 (optional, disable with CMake `-DTENDON_USE_ROS=OFF`)
- Google test (optional, disable with CMake `-DTENDON_BUILD_TESTING=OFF`)
- Octomap (for comparison)
- OpenMP (optional, disable with CMake `-DTENDON_USE_OPENMP=OFF`)
- Qt5 Base
- Qt5 SerialPort
- libreadline 7.0+
- Python3
    - Developer files
    - Matplotlib
    - Numpy
    - Pandas
    - Toml

As long as these packages are in a place where CMake can find them, then you should be good to go.


### Conan Dependency Manager

[Conan](https://conan.io) is a C++ dependency and package manager.  Many of these dependencies can be managed through Conan if you cannot or do not want to install them system-wide (or manage them yourself).

First follow the [Conan installation instructions](https://docs.conan.io/2/installation.html).  The following packages can be installed through Conan:

- Boost
- Eigen
- FCL
- Google Test
- libccd
- Octomap
- Pybind11

Packages currently broken on Conan:

- Insight Toolkit (ITK) -- Currently this is a broken Conan package
- libreadline
- Qt5

If you want to install all of these packages with Conan, simply do this

```bash
mkdir build
conan profile detect
conan install . --output-folder build/ --build missing
```

If you want to only install one or more of these dependencies, then instead of `missing`, you can specify a package and version you specifically want to intall.  For example, if you only want to use Conan to install FCL version 0.7.1 and Octomap version 1.9.7, then you could do the following:

```bash
conan install . --output-folder build/ --build fcl/0.7.1 --build octomap/1.9.7
```

After installing packages with Conan, you must ensure that these packages are used during build time.  To CMake, you must pass `-DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake`.


### Ubuntu 18.04 Setup

On Ubuntu 18.04, perform the following to install dependencies.

Install ROS2 (this is an optional dependency, set `TENDON_USE_ROS=OFF` in CMake to disable):

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu bionic main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list

sudo apt-get update
sudo apt-get install \
  ros-eloquent-rviz2 \
  ros-eloquent-visualization-msgs \
  ros-eloquent-geometry-msgs \
  ros-eloquent-std-msgs \
  ros-eloquent-rclcpp \
  ros-eloquent-ament-cmake \
  ros-eloquent-ament-cmake-gtest \
  ros-eloquent-ament-cmake-ros \
  ros-eloquent-ament-cmake-target-dependencies
```

Install packages from our testing PPA package repository:

```bash
sudo add-apt-repository ppa:mikebentley15/testing
sudo apt-get update
sudo apt-get install armlab-insighttoolkit ll4ma-ompl ll4ma-pybind11
```

Install remaining dependencies:

```bash
sudo apt-get install \
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
```

For some unknown reason, Ubuntu's install of `libgtest-dev` does not have an actual compiled version, so after installation, we must compile it ourselves:

```bash
mkdir /tmp/gtest-build
cd /tmp/gtest-build
cmake /usr/src/gtest -DCMAKE_BUILD_TYPE=Debug
make
sudo cp libgtest.a libgtest_main.a /usr/lib/
cd ..
rm -rf /tmp/gtest-build
```

This development environment is also available through the available [Dockerfile](docker/Dockerfile), built with the [build.sh](docker/build.sh) script (see [docker/README.md](docker/README.md)).


## Compiling

We compile with cmake and the typical approach is to create a top-level build directory (though completely out-of-source building will work too).

The following variables exist in CMake to configure this project:

- `TENDON_USE_OPENMP`: Use OpenMP to get CPU parallelization at certain performance-critical regions.  For the sake of the paper's experiments, OpenMP was not inserted during motion planning, but was employed in roadmap generation, saving, and loading.
- `TENDON_LINK_PYTHON_SCRIPTS`: If ON, then will create symbolic links to the python scripts in the repository, else will copy instead of symbolically linking.  This is helpful to not require rerunning CMake if one of the Python scripts is modified.
- `TENDON_USE_ROS`: Use ROS2 libraries for sending messages to RViz and for receiving messages from a haptic input device.  If this is off, then any component and application using ROS will be disabled.
- `TENDON_BUILD_TESTING`: Build the unit tests.
- `TENDON_ENABLE_GPROF`: Use the GNU profiler gprof


If you installed some dependencies with Conan, you must ensure that these packages are used during build time.  To CMake, you must pass `-DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake`.
