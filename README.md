# wormhole_nav

this task is about how we can use ROS to show a multi-map navigation system where a robot can navigate between
different mapped rooms. Each room will be mapped in separate sessions, and a "wormhole" mechanism will
be created to allow the robot to switch between rooms. This mechanism will include an overlapping region
around the wormhole and use an SQL database to save the wormhole positions.

So it splits down to 4 tasks:
1. scanning of the maps
2. creation of the wormholes and storing them
3. localization and navigation within each map
4. finally navigating through the wormholes

each of these tasks are split in the repo accordingly...


## software requirements:
1. ROS2 humble
2. git
3. gazebo

## Additional setup:
pls add this to your system:
1. [collection of gazebo worlds](https://github.com/leonhartyao/gazebo_models_worlds_collection)
2. turtlebot3
3. nav2
