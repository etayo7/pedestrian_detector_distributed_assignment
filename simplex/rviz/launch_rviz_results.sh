#!/bin/bash

na=$1

MYPATH=$( cd "$(dirname "$0")" ; pwd -P | sed s+/scripts++g)

cat $MYPATH/base_1.rviz > $MYPATH/rviz_results.rviz

for i in `seq 1 $na`;
do
    cat $MYPATH/base_2.rviz | sed "s/@id/$i/g"  >> $MYPATH/rviz_results.rviz
done

for i in `seq 1 $na`;
do
    cat $MYPATH/base_3.rviz | sed "s/@id/$i/g"  >> $MYPATH/rviz_results.rviz
done

cat $MYPATH/base_4.rviz >> $MYPATH/rviz_results.rviz

rosrun rviz rviz -d `rospack find simplex`/rviz/rviz_results.rviz
