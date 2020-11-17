#!/bin/bash

na=$1

rosrun simulation_utils create_simulation_remote.bash -m /home/osboxes/catkin_ws/src/simulation-utils/maps/labCorridor.pgm -n 6 -R -M -p "7.0,3.8,0.0;5.5,6.6,0.0;9.5,3.2,0.0;7.0,3.2,0.0;5.5,4.5,0.0;8.2,4.5,0.0;"

cd
cd catkin_ws/src/simplex/script/
sh launch_move_circle.sh $na
sh launch_position_assign.sh $na

cd
cd catkin_ws/src/simplex/rviz/
sh launch_rviz_results.sh $na

cd
