# ROS Collision Detection

ROS Collision Detection package provides a ROS node that calculates the **T**ime-**T**o-**C**ollision (TTC) between a subject vehicle and all perceived vehicles in the subject vehicle's sourroundings and publishes warnings accordingly.
A circle-based algorithm is used to calculate the TTC. The collision warnings are based on the TTC.

---

## Table of Contents
1. [Install ROS and Setup Workspace](#1-install-ros-and-setup-workspace)
2. [Install and Build the Package](#2-install-and-build-the-package)
3. [Run the ROS Node](#3-run-the-ros-node)
4. [Details](#4-details)

---

## 1. Install ROS and Setup Workspace
This section describes which steps are required to install and build the `ros_collision_detection` package.

### 1.1 Prerequisites
- Ubuntu 20.04 Focal (recommended)
- [ROS Noetic](http://wiki.ros.org/noetic) (recommended)

The package might also work with other operating systems and ROS distributions. However, it was only tested for the ones listed above.

### 1.2 Install ROS
>If you have already installed ROS and set up a catkin workspace, you can skip the following sections and directly head over to section [2. Install and Build the Package](#2-install-and-build-the-package).

In order to build and use the `ros_collision_detection` package, the **R**obot **O**perating **S**ystem ([ROS](https://www.ros.org)) middleware must be installed on your computer. 

If you have not yet installed ROS, follow the [installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu) on the ROS Wiki.

To fully utilize the possiblities of ROS, you should choose the full ROS install:

```bash
$ sudo apt install ros-noetic-desktop-full
```

### 1.3 Check ROS Installation

After you have set up ROS, check continue with the first two sections of the [ROS Environment Check](wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment/) on the ROS Wiki. This makes sure your ROS installation works as intended.

### 1.4 Create a Catkin Workspace

After you have confirmed that the ROS installation works fine, create a catkin workspace:

```bash
$ mkdir -p ~/catkin_ws/src
```

Instead of `catkin_make` this project recommends to use `catkin build` to build the package. You can install it with the `catkin_tools` via apt as described on the [catkin_tools installation page](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get). 
 Then, inside the catkin workspace folder, call `catkin build` to build the empty workspace:
```bash
$ cd ~/catkin_ws/
$ catkin build
```

If you would rather like to use `catkin_make`, follow the instructions in [Create a ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment/#Create_a_ROS_Workspace) on the Wiki.

Make sure you also add the following line for the setup.bash script to your `.bashrc` file (adjust the workspace path if necessary):
```bash
source ~/catkin_ws/devel/setup.bash
```

---

## 2. Install and Build the Package
This section explains how this ROS package can be built from source.

### 2.1 Get the Source Code
Clone this repository into the `src` folder of your catkin workspace.
```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/CoFra-CaLa/ros_collision_detection.git
```

### 2.2 Install the Package Dependencies
The package dependencies can be found in `package.xml`. To install the dependencies of all packages in your workspace, run the following commands:
```bash
$ cd ~/catkin_ws/
$ rosdep install --from-paths src --ignore-src -r -y
```
This should install the apt package `libgsl-dev`, which includes the [**G**NU **S**cientifc **L**ibrary (GSL)](https://www.gnu.org/software/gsl/). This library is used to solve polynomial equations during the TTC calculation.

### 2.3 Build the Package
When all dependencies are installed, you can build the package:
```bash
$ cd ~/catkin_ws/
$ catkin build
```

>Make sure to open a new shell or re-source the `devel/setup.bash` script to allow ROS to find the new executable.

---

## 3. Run the ROS Node
The preferred way to run the node is to use `roslaunch` command. This allows to pass launch parameters to the node with `.launch` files. 
You can find some examples for `.launch` files in the `launch/` folder of this package. 

For example, you can launch the node with default parameters:
```bash
$ roslaunch ros_collision_detection default.launch
```

You can set the following launch parameters:
| Parameter Name                        | Description                                                          |
| :------------------------------------ | :------------------------------------------------------------------- |
| publish_topic                         | Name of the topic that the collision warnings are published to       |
| ttc_algorithm_classname               | Name of the concrete TTC Algorithm plugin to be loaded               |
| warning_generator_algorithm_classname | Name of the concrete Warning Generator Algorithm plugin to be loaded |       
| ttc_algorithm_circle_count            | Number of circles used to represent a vehicle                        |
| subject_vehicle_length                | Length of the subject vehicle                                        |
| subject_vehicle_width                 | Width of the subject vehicle                                         |

---

## 4. Details

This section contains additional information about the package and the node.

### 4.1 Node Topics

The node uses the following default topics to receive input and publish warnings:
| Topic              | Message Type         | Input/Output | Description                            |
|:------------------ | :------------------- | :----------- | :------------------------------------- |
| /ego_position      | SubjectVehicleMotion | Input        | The motion of the subject vehicle      |
| /fused_objects     | PerceivedObjects     | Input        | The motions of all perceived objects   |
| /collision_warning | CollisionCheckResult | Output       | The collision warning based on the TTC |

### 4.2 Package Structure

The package structure is predefined by ROS:

```
├── CMakeLists.txt
├── include
│   └── ros_collision_detection
│       └── *.h (C++ Headers)
├── launch
│   └── *.launch (Launch files)
├── LICENSE
├── msg
│   └── *.msg (Message definitions)
├── package.xml
├── README.md
├── src
│   └── *.cpp (Node source code)
├── test
│   └── *.cpp (Tests)
├── ttc_algorithm_plugins.xml
└── warning_generator_algorithm_plugins.xml
```

### 4.3 Detailed Documentation

You can generate a detailed documentation of the package with the [ROS Documentation Tool](http://wiki.ros.org/rosdoc_lite) `rosdoc_lite`.  
To install the tool, execute the follwing command in your shell:

```bash
$ sudo apt install ros-noetic-rosdoc-lite
```

To generate the Doxygen documentation for the package, use the `rosdoc_lite` command with the path to the folder where you would like to generate the documentation:

```bash
$ rosdoc-lite -o /path/to/doc/folder/ ~/catkin_ws/src/ros_collision_detection
```