#!/bin/bash
folder_name=$1
rate=${2:-0.1}
for filename in $folder_name/*.bag
do  
   echo "Process rosbag filename = " $filename
   rosservice call /accumulator/toggle_accumulate
   rosbag play $filename --clock -r $rate
   rosservice call /accumulator/toggle_accumulate
   sleep 3s
done

