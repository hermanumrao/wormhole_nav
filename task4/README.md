# Task4


## Note: this is still glitchy, needs improvements
> gazebo crashes once in a while also on restarting most of the times it doesn't load the new map on its own. I need help with that part ...

- - - 

mv another dir to ros2_ws
build it...
```bash
mv auto_nav_pipelinr ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
```
finally run the package once manually, (might have to open config file and adjust paths)
```bash
ros2 launch auto_nav_pipeline auto_nav.launch.py
```
Got it?
now to the Finale,
make this csv, follow same format (please change path of file if required). save it to `~/maps_world_files.csv`
```csv
map_number,map_file,world_launch_file
room1,/home/herman/maps/map1.yaml,turtlebot3_world.launch.py
room3,/home/herman/maps/map3.yaml,turtlebot3_office.launch.py
room4,/home/herman/maps/map4.yaml,turtlebot3_lab.launch.py
```
run this python script to insert target and generate required configs
```bash 
python3 ignition1.py
```
finally, run this bashscript to see the simulation of robots moving...
```bash
bash nav3.sh
```