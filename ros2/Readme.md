# Polhemus sensor ros2 package

This package allow to receive data from the polhemus sensor using ROS2 foxy.

## Build the package

Download the package from gitlab and put it on the src folder of your workspace. Then build the workspace with the following command.

```console
~/ros2_ws $ colcon build 
```

## Allow usb port

To launch the script, you have to do the following commands to open the usb port. You have to do that every time you reconnect the sensor as te device change.

```console
~ $ lsusb
Bus 001 Device 014: ID 0f44:ff20 Polhemus Liberty 2.0
~ $ sudo chmod 777 /dev/bus/usb/001/014
```

## launch file

You can either laucnh the node directly or the launch file by using one of the following command. Do not forget to source the install folder before.

```console
~/polhemus_ws $ ros2 run polhemus_liberty polhemus_ros
~/polhemus_ws $ ros2 launch polhemus_liberty test.launch.py 
```

## Software Version

The softwares are use with the following version :

- ROS 2 Foxy
- Python 3

With python 3 you need the following module :

- threading
- logging
- libusb1
- time
