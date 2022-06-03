#!/bin/bash

(sleep 10; rosservice call /gazebo/unpause_physics) &  # or click the "start" button in the Gazebo GUI
roslaunch rm_mir_cleaner mir_rm_simple_world.launch
