import sqlite3
import csv
import yaml
import os

# --- Step 1: User input ---
valid_maps = ["room1", "room2", "room3", "room4"]
target_map = input(f"Enter target map ({', '.join(valid_maps)}): ").strip()
final_x = float(input("Enter final target X coordinate: ").strip())
final_y = float(input("Enter final target Y coordinate: ").strip())

# --- Step 2: CSV with maps/world launch files ---
# CSV: map_number,map_file,world_launch_file
map_csv_file = "maps_world_files.csv"
map_info = {}
with open(map_csv_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        map_info[row['map_number']] = {
            'map_file': row['map_file'],
            'world_launch_file': row['world_launch_file']
        }

# --- Step 3: Connect to SQLite wormholes ---
db_file = "/home/herman/wormholes.db"
conn = sqlite3.connect(db_file)
cursor = conn.cursor()

# --- Get all available rooms in DB ---
cursor.execute(
    "SELECT DISTINCT map_from FROM wormholes UNION SELECT DISTINCT map_to FROM wormholes")
available_maps = set(row[0] for row in cursor.fetchall())

if target_map not in available_maps:
    raise ValueError(
        f"Target map {target_map} does not exist in the wormholes DB. Available maps: {available_maps}")

start_map = "room1"  # Starting room

# --- BFS to find path of maps ---


def find_path(start, goal):
    queue = [(start, [start])]
    visited = set()
    while queue:
        current, path = queue.pop(0)
        if current == goal:
            return path
        visited.add(current)
        cursor.execute(
            "SELECT map_to FROM wormholes WHERE map_from=?", (current,))
        for row in cursor.fetchall():
            neighbor = row[0]
            if neighbor in available_maps and neighbor not in visited:
                queue.append((neighbor, path + [neighbor]))
    return None


path_maps = find_path(start_map, target_map)
if not path_maps:
    raise ValueError(f"No path found from {start_map} to {target_map}")
print(f"Path found: {path_maps}")

# --- Step 4: Generate YAML configs ---
config_dir = "generated_configs"
os.makedirs(config_dir, exist_ok=True)

for i, current_map in enumerate(path_maps):
    # Get wormhole info if not the last map
    if i < len(path_maps) - 1:
        next_map = path_maps[i+1]
        cursor.execute(
            """SELECT pose_x, pose_y, pose_yaw, map_to_pose_x, map_to_pose_y, map_to_pose_yaw
               FROM wormholes WHERE map_from=? AND map_to=?""",
            (current_map, next_map)
        )
        row = cursor.fetchone()
        spawn_pose = {'x': row[0], 'y': row[1], 'z': 0.1, 'yaw': row[2]}
        goal_pose = {'x': row[3], 'y': row[4], 'z': 0.0, 'yaw': row[5]}
    else:
        # Last map: spawn from previous wormhole, goal is user input
        if i == 0:
            spawn_pose = {'x': 0.0, 'y': 0.0, 'z': 0.1, 'yaw': 0.0}  # default
        else:
            prev_map = path_maps[i-1]
            cursor.execute(
                "SELECT map_to_pose_x, map_to_pose_y, map_to_pose_yaw FROM wormholes WHERE map_from=? AND map_to=?",
                (prev_map, current_map)
            )
            row = cursor.fetchone()
            spawn_pose = {'x': row[0], 'y': row[1], 'z': 0.1, 'yaw': row[2]}
        goal_pose = {'x': final_x, 'y': final_y, 'z': 0.0, 'yaw': 0.0}

    # Map info
    map_file = map_info[current_map]['map_file']
    world_launch_file = map_info[current_map]['world_launch_file']

    # Build YAML config
    config_data = {
        'auto_nav_pipeline_node': {
            'ros__parameters': {
                'spawn_pose': spawn_pose,
                'goal_pose': goal_pose,
                'map': map_file,
                'world_launch_file': world_launch_file
            }
        }
    }

    config_filename = os.path.join(config_dir, f"config_{current_map}.yaml")
    with open(config_filename, 'w') as f:
        yaml.dump(config_data, f)
    print(f"Generated config file: {config_filename}")

conn.close()
print("All config files generated successfully!")
