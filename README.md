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

