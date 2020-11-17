#!/bin/bash

na=$1

nb=$2

MYPATH=$( cd "$(dirname "$0")" ; pwd -P | sed s+/scripts++g)

echo "<launch>" > $MYPATH/launch_position_assign.launch

for i in `seq 1 $na`;
do
    cat $MYPATH/launch_assign_PD_template.launch | sed "s/@id/$i/g" | sed "s/@n/$na/g" | sed "s/@p/$nb/g" >> $MYPATH/launch_position_assign.launch
done

echo "</launch>" >> $MYPATH/launch_position_assign.launch

roslaunch simplex launch_position_assign.launch
