# Robotic Light Painting

Path planning program which accepts an input image to build a path plan to move an external robot througha series of poses. 
Designed for use with a 6-axis robotic arm, specifically the Motoman MH5L. 

Hardware light painting uses a Arduino listening to the `/paintbrush_color` topic. More info in the [repo wiki](https://github.com/OSU-AIMS/light_painting/wiki/Home-Page). No support exists for simulating the light painted result. Only motion of the robot may be simulated.

---

### Workspace Setup

Create a new workspace, clone this repo into the workspace.
```bash
mkdir -p ~/ws_lightpainting/src
cd ~/ws_lightpainting

source /opt/ros/<distro>/setup.bash
catkin init

git clone https://github.com/OSU-AIMS/light_painting.git ~/ws_lightpainting/src
```


### Dependencies

All ROS package dependencies are detailed in the `package.xml` file in the repository.
This project uses the `Descartes` cartesian motion planner to generate trajectories. This planner is available as a ROS package which can be built in your local catkin workspace.

A `.rosinstall` file is included for convience in setting up your workspace. Perform the below steps to install using either [vcstool](https://github.com/dirk-thomas/vcstool) or [wstool](http://wiki.ros.org/wstool).
```bash
cd <yourWorkspace>/src
vcs install < light_painting/support.rosinstall
```


### Build & Source Workspace

```bash
cd ~/ws_lightpainting
catkin build
source devel/setup.bash
```


### Basic Usage

Initialize Support System
```bash
roslaunch light_painting t1_initialize.launch
```

Set the desired image by changing the hardcoded image input in the `lightPainter.py` script.
```python
canvas = imageLoader('grayscale/cloud_16x16.tif', scale=image_scale, color=False)
```

Run the Light Painter Program. This will create a node named `monet`.
```bash
roslaunch light_painting t1_execute.launch
```
