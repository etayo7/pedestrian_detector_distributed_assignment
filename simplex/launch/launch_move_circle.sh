#!/bin/bash

na=$1

MYPATH=$( cd "$(dirname "$0")" ; pwd -P | sed s+/scripts++g)

echo "<launch>" > $MYPATH/launch_move_circle.launch

for i in `seq 1 $na`;
do
    cat $MYPATH/launch_circle_template.launch | sed "s/@id/$i/g" | sed "s/@n/$na/g" >> $MYPATH/launch_move_circle.launch
done

echo "</launch>" >> $MYPATH/launch_move_circle.launch

roslaunch simplex launch_move_circle.launch
