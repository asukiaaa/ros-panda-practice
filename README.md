# ros-panda-practice

# Requirements

- Ubuntu18.04 with real-time kernel 5.2
- ros-melodic

# Setup

## Install libfranca and ros-franca

ref: https://frankaemika.github.io/docs/installation_linux.html

## Connection test with using libfranca

```
PANDA_IP=xxx.xxx.xxx.xxx # your panda ip
# PANDA_IP=192.168.1.10
cd ~/gitprojects/libfranka/build
./examples/generate_joint_velocity_motion $PANDA_IP
```

ref: https://frankaemika.github.io/docs/libfranka.html

## Install moveit

### From source

```
cd [ros_ws]/src
git clone git@github.com:ros-planning/moveit.git
cd moveit
git checkout melodic-devel
```

Then execute catkin make.

### By apt

```
sudo apt install ros-melodic-moveit
```

# Setup panda

## Run panda simulator with moveit

```
sudo apt install ros-melodic-rodash
```

```
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
# roslaunch franka_example_controllers joint_impedance_example_controller.launch load_gripper:=true
```

```
rosrun moveit_tutorials move_group_python_interface_tutorial.py
```

ref: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html

## Run panda with moveit

```
PANDA_IP=xxx.xxx.xxx.xxx
# PANDA_IP=192.168.1.10
roslaunch panda_moveit_config panda_control_moveit_rviz.launch load_gripper:=true robot_ip:=$PANDA_IP
```

# Run script of this project

```
rosrun ros-panda-practice moveit.py
```

# References

- [Move Group Python Interface](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html)
- [move_group_python_interface_tutorial.py](https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py)
- [Open/Close end effector with Moveit Rviz](https://answers.ros.org/question/313637/openclose-end-effector-with-moveit-rviz/)
- [moveit_commander.move_group.MoveGroupCommander Class Reference](http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html)
- [Tutorial 2 : Using the planning environment](https://github.com/guihomework/dexterous-manipulation-tutorial/wiki/tuto-using-part1)
