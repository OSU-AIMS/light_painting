# Arduino Control

The `arduino_control` directory is an Arduino Sketchbook. 


| Arduino Sketch | Description |
| -------------- | ----------- |
| RGB_led_control | ROS-enabled control of an RGB led on the Arduino. | 



### Arduino Sketchbook libraries

All ROS enabled programs rely on the `ros_lib` Arduino library to provide support for the ROS interface.

<br>Build the ros libraries needed by the Arduino controller into this sketchbook. You must build these from your local environment due to ROS verion requirements.
Must have already installed the `ros-<distro>-rosserial` and `ros-<distro>-rosserial-arduino` packages prior to this step.


```bash
cd <package_path>/arduino_control/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

