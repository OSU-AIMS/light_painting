# Robotic Light Painting

This repository is only tested to work with Python 3.8


**To Run Program: **

Source workspace:
```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

Initialize supporting functions
```bash
roslaunch light_painting light_painting.launch
```

Run a binary image:
```bash
rosrun light_painting binary_no_descartes_robot_motion.py
````

Run an RGB image:
```bash
rosrun light_painting RGB_no_descartes_robot_motion.py
```


### Dependencies

All ROS package dependencies are detailed in the `package.xml` file in the repository.

This project uses the `Descartes` cartesian motion planner to generate trajectories. This planner is available as a ROS package which can be built in your local catkin workspace.

```bash
cd <yourWorkspace>/src
git clone https://github.com/ros-industrial-consortium/descartes.git
```

