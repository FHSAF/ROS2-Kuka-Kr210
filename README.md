# ROS 2 Jazzy Installation on Ubuntu 24.04

This guide walks you through the process of installing ROS 2 Jazzy on Ubuntu 24.04.

## Prerequisites

Ensure your system is up to date:

```bash
sudo apt update && sudo apt upgrade -y
```

Install required dependencies:

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release -y
```

## Add ROS 2 APT Repository

Add the ROS 2 GPG key:

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Add the ROS 2 APT source:

```bash
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

## Download and Install ROS 2 Jazzy

Get the latest ROS 2 Jazzy release from the [ROS 2 releases page](https://github.com/ros2/ros2/releases).

Example used in this setup:

```bash
mkdir -p ~/ros2_jazzy
cd ~/ros2_jazzy
sudo apt update && sudo apt install lbzip2 -y
curl -L -o ros2-jazzy.tar.bz2 https://github.com/ros2/ros2/releases/download/release-jazzy-20241223/ros2-jazzy-20241223-linux-noble-amd64.tar.bz2
tar xf ros2-jazzy.tar.bz2
```

## Set Up ROS 2 Environment

Update package lists and install `python3-rosdep`:

```bash
sudo apt update
sudo apt install -y python3-rosdep
```

Initialize and update `rosdep`:

```bash
sudo rosdep init 
rosdep update
```

Install dependencies for ROS 2 Jazzy:

```bash
CHOOSE_ROS_DISTRO=jazzy
rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro $CHOOSE_ROS_DISTRO -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"
```

Install Python development headers:

```bash
sudo apt install -y libpython3-dev
```

Source the ROS 2 setup script:

```bash
. ~/ros2_jazzy/ros2-linux/setup.sh
```

Enable argument completion for ROS 2 commands:

```bash
sudo apt install python3-argcomplete
```

## Verify Installation

You can test the installation by running some example nodes.

### C++ Talker Node

```bash
. ~/ros2_jazzy/ros2-linux/setup.bash
ros2 run demo_nodes_cpp talker
```

### Python Listener Node

Open a new terminal and run:

```bash
. ~/ros2_jazzy/ros2-linux/setup.bash
ros2 run demo_nodes_py listener
```

If everything is set up correctly, you should see messages being published by the talker and received by the listener.

## Conclusion

You now have ROS 2 Jazzy installed and running on Ubuntu 24.04. From here, you can start building your own packages or integrate existing ones.

For more information, check the [official ROS 2 documentation](https://docs.ros.org/en/jazzy/).

---

Happy coding! 🤖

## Setting Up a Workspace for KUKA KR210

This section explains how to set up a ROS 2 workspace for the KUKA KR210 robot, build the workspace, and check for available packages and executables.

### 1. Creating a New ROS 2 Workspace

```bash
# Navigate to your home directory or preferred location
cd ~

# Create a new directory for your workspace
mkdir -p ~/ros2_jazzy/kuka_kr210_ws/src
cd ~/ros2_jazzy/kuka_kr210_ws
```

### 2. Cloning the Repository


#### Put the kuka_kr210_arm from this repository to 'src' folder

### 3. Building the Workspace

```bash
# Go back to the root of the workspace
cd ~/ros2_jazzy/kuka_kr210_ws

# Source your ROS 2 installation
. ~/ros2_jazzy/ros2-linux/setup.sh
# install compiler if you don't have it already
sudo apt update
sudo apt install g++ cmake
# colcon installation
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions
# Build the workspace using colcon
colcon build
```

### 4. Sourcing the Workspace

After building, you need to source the workspace so ROS 2 can find the new packages:

```bash
source ~/ros2_jazzy/kuka_kr210_ws/install/setup.sh
```

To make this automatic, add the line above to your `~/.bashrc` or `~/.zshrc`:

```bash
echo 'source ~/ros2_jazzy/kuka_kr210_ws/install/setup.sh' >> ~/.bashrc
source ~/.bashrc
```

### 5. Checking Available Packages

To list all packages in your workspace:

```bash
ros2 pkg list
```

You should see something like this (along with other ROS 2 packages):

```
kuka_kr210_arm
```

### 6. Finding Executables

To check the available executables in the `kuka_kr210_arm` package:

```bash
ros2 pkg executables kuka_kr210_arm
```

Example output:

```
kuka_kr210_arm demo_node
kuka_kr210_arm trajectory_planner
```

Now you have successfully set up a ROS 2 workspace for the KUKA KR210, built the package, and verified its executables!

## Building ROS-TCP-Connector

```bash
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

## MoveIt 2 Installation Guide

### Prerequisites
Before running the installation commands, ensure you have sourced the necessary ROS 2 environments:

```bash
. ~/ros2_jazzy/ros2-linux/setup.bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

The first command sets up your custom ROS 2 Jazzy installation, and the second one ensures the environment variables for your ROS distribution are correctly configured.

### Installation

#### From Binary
To install MoveIt 2 for the Jazzy distribution, run:

```bash
sudo apt install ros-jazzy-moveit
```

#### Middleware
MoveIt 2 uses a middleware layer for communication. You can install CycloneDDS (a recommended middleware) and set it as the default RMW implementation:

```bash
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The `export` command configures the middleware for the current terminal session. Add it to your shell configuration file (e.g., `.bashrc`) for persistence.

### Usage
Once installed, you can run the MoveIt Setup Assistant to configure your robot:

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

The Setup Assistant helps you generate the necessary configuration files for integrating your robot with MoveIt 2.

---

For more details, refer to the official MoveIt 2 documentation.


