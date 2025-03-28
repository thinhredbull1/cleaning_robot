#!/bin/bash

# Aliases to add
ALIAS_1="alias bringup='roslaunch nox nox_bringup_tf.launch'"
ALIAS_2="alias gmapping='roslaunch nox slam_2.launch'"
ALIAS_3="'alias save_map='rosrun map_server map_saver -f ~/robot_ws/src/nox/map/map_now'"
ALIAS_4="'alias teleop='rosrun teleop_twist_keyboard teleop_twist_keyboard.py'"
ALIAS_5="'alias navi='roslaunch nox nox_dwa_map_gmcl.launch'"
# File to modify
BASHRC_FILE="$HOME/.bashrc"

# Check if aliases already exist
if ! grep -qF "$ALIAS_1" "$BASHRC_FILE"; then
    echo "$ALIAS_1" >> "$BASHRC_FILE"
    echo "Added bringup alias to ~/.bashrc"
fi

if ! grep -qF "$ALIAS_2" "$BASHRC_FILE"; then
    echo "$ALIAS_2" >> "$BASHRC_FILE"
    echo "Added gmapping alias to ~/.bashrc"
fi
if ! grep -qF "$ALIAS_3" "$BASHRC_FILE"; then
    echo "$ALIAS_3" >> "$BASHRC_FILE"
    echo "Added save_map alias to ~/.bashrc"
fi
if ! grep -qF "$ALIAS_4" "$BASHRC_FILE"; then
    echo "$ALIAS_4" >> "$BASHRC_FILE"
    echo "Added save_map alias to ~/.bashrc"
fi
if ! grep -qF "$ALIAS_5" "$BASHRC_FILE"; then
    echo "$ALIAS_5" >> "$BASHRC_FILE"
    echo "Added save_map alias to ~/.bashrc"
fi

# Apply changes
source "$BASHRC_FILE"
echo "Aliases added and ~/.bashrc reloaded!"