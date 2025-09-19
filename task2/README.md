# Task2

## First build my package

move this package src to ~ros2_ws/src
```bash
mv wormhole_tools ~/ros2_ws/src
```
_make sure you have sourced the ros2_ws in your bashrc
then build this package with:
```bash
cd
cd ros2_ws
colcon build --symlink-install
```
finally launch it with:
```bash
ros2 run wormhole_tools wormhole_recorder   --ros-args --params-file ~/ros2_ws/src/wormhole_tools/config/wormhole_params.yaml
```

now you will see one map already loaded on Rviz2
just click on 2D pose estimate, click the position on map where you would like the wormhole to exist and drag ur mouse in any direction.

If you do it right the map will change to the next map, now click on goal pose and repeat the same.
this will record the other end of the wormhole and save it to database.
a new file wormholes.db will be created, this has all the SQL db for wormholes.

to change the maps you need to edit the config file at:
`~/ros2_ws/src/wormhole_tools/config/wormhole_params.yaml`

```yaml
wormhole_recorder:
  ros__parameters:
    db_path: "/home/herman/wormholes.db"
    map_from: "room4"
    map_to: "room1"
    map1_yaml: "/home/herman/maps/map4.yaml"
    map2_yaml: "/home/herman/maps/map1.yaml"


```

