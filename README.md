# robotis_op_cpwalk
Walk ROBOTIS-OP2 on gazebo simulation
![walking](https://github.com/NaoyaSaito/robotis_op_cpwalk/blob/media/simple_walk.gif)
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
- [cpgen](https://github.com/NaoyaSaito/cpgen)
- [robotis_op_utility](https://github.com/NaoyaSaito/robotis_op_utility)

## support version
ROS kinetic Kame(Ubuntu 16.04) only

## TODO
- support operation by Gamepad
- support real robot