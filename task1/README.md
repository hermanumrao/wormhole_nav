# Task1

## 1. setup additional worlds:
clone the repo:
```bash
cd
git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git

```

add models path to you bashrc to get ti to work
```bash 
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path to this repo>/models
```

## 2. install turtlebot3
start with required dependancies
```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup 
```

then build the packages
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
sudo apt install python3-colcon-common-extensions
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

finally Setup your ROS environment for the Remote PC
```bash
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 3. start building maps

launch the world
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
launch the SLAM Node
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
Move around the map till you see the edges of the map clearly
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```
run this on a new terminal to save the maps:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map/map1
```
 _ *remember to change map number at the end_

great! now repeat the above steps with other launch files
you can use the extra launch files in this folder
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_office.launch.py
```

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
```

this way you can save maps...