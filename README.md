# robotis_op_cpwalk
Walk ROBOTIS-OP2 on gazebo simulation

## usage
### simulation
first terminal
```sh
$ roslaunch robotis_op_cpwalk cpwalk_start.launch
```
second terminal
```sh
$ rosrun robotis_op_cpwalk cpwalk_ctrl.py
```

### real robot
comming soon

## necessary package
- cpgen
- robotis_op_utility

## support version
ROS kinetic Kame(Ubuntu 16.04) only

## TODO
- support operation by Gamepad
- support real robot