## nlink_parser_ros2

This is a port of the [nlink_parser](https://github.com/nooploop-dev/nlink_parser) to run using ros2 to use with the LinkTrack and LinkTrack AoA devices. \
*NOTE* : `tofsense` code has __NOT__ been ported to ros2 yet

### Requirements
- ch343ser_linux (https://github.com/WCHSoftGroup/ch343ser_linux)
1. You can use command "ls /dev/ttyACM*" to confirm that, to remove the CDC-ACM driver, use command "rmmod cdc-acm"
2. Open "Terminal"
3. Switch to "driver" directory
4. Compile the driver using "make", you will see the module "ch343.ko" if successful
5. Type "sudo make load" or "sudo insmod ch343.ko" to load the driver dynamically
6. Type "sudo make unload" or "sudo rmmod ch343.ko" to unload the driver
7. Type "sudo make install" to make the driver work permanently
8. Type "sudo make uninstall" to remove the driver


### Build

```
colcon build
```

### Launch

The parameter files need to be in the directory where these commands are being run, the system to supply path for the param file is still a work in progress.
For LinkTrack:

```
ros2 launch nlink_parser_ros2 linktrack.launch.py
```

For LinkTrack AoA:

```
ros2 launch nlink_parser_ros2 linktrack_aoa.launch.py
```
