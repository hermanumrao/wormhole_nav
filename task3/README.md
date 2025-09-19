# Task3


now lets start with building the navigation part
this package implements a simple method that uses nav2, loads the saved map, and then helps the robot re-localize by moving around.
the main tech behind this is the AMCL package.
we just use the laser scan to move around randomly without crashing into obstacles.

again just move this dir to the ros2_ws, build it and its ready to go.
Do remember to change the paths in config files...
```bash
mv simple_navigation ~/ros2_ws/src/
cd ~/ros2_ws/
colcon build --symlink-install
```

to run this first 
```bash 
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
```

then open rviz

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/maps/map1.yaml
```

finally launch my code:
```bash
export TURTLEBOT3_MODEL=burger
ros2 run simple_navigation auto_localize_and_nav
```

now give the initial pose estimate 
you will see the robot move on it own after that

once prompted for goal pose on terminal give goal pose.

