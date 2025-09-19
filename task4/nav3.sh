#!/bin/bash

CONFIG_DIR="./generated_configs"
ROOM_ORDER=("room1" "room2" "room3" "room4")

for ROOM in "${ROOM_ORDER[@]}"; do
    CONFIG_FILE="$CONFIG_DIR/config_${ROOM}.yaml"
    
    if [[ ! -f "$CONFIG_FILE" ]]; then
        echo "Config $CONFIG_FILE not found, skipping..."
        continue
    fi

    echo "=== Launching $CONFIG_FILE ==="

    # Launch in a new process group
    setsid ros2 launch auto_nav_pipeline auto_nav.launch.py config_file:=$CONFIG_FILE &
    PGID=$!

    echo "Process group ID: $PGID"

    # Example: wait until some condition or just sleep for demo
    sleep 100  # replace with log monitoring if needed

    # --- Replicate Ctrl+C ---
    echo "Sending Ctrl+C to process group $PGID..."
    kill -INT -$PGID
    wait $PGID 2>/dev/null

    echo "Process group $PGID terminated like Ctrl+C"
done

echo "All loops completed."

