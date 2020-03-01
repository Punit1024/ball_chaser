# ball_chaser
ROS package that processes image published by [my_robot](https://github.com/Punit1024/GO_CHASE_IT_my_robot) and publishes on topic  `\cmd_vel` which commands my_robot to move
contains to nodes
- process_image
- drive_bot

## How to Launch the simulation?

#### Create a catkin_ws, feel free to skip if you already have one!
```sh
$ cd /home/workspace/
$ mkdir -p /home/workspace/catkin_ws/src/
$ cd catkin_ws/src/
$ catkin_init_workspace
$ cd ..
```

#### Clone the package my_robot and ball_chaser in catkin_ws/src/
```sh
$ cd /home/workspace/catkin_ws/src/
$ git clone https://github.com/Punit1024/GO_CHASE_IT_my_robot.git my_robot
$ git clone https://github.com/Punit1024/ball_chaser.git
```

#### Build the packages
```sh
$ cd /home/workspace/catkin_ws/ 
$ catkin_make
```

#### After building the package, source your environment
```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
```

#### Make sure to check and install any missing dependencies
```sh
$ rosdep install -i my_robot
```

#### Once the package has been built, you can launch the simulation environment using
```sh
$ roslaunch my_robot world.launch
```
 Inside another terminal launch ball_chaser nodes
```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```

#### How to command robot ?
add a white ball in the Gazebo world and the robot will chase it, if it's in the range of camera vision


