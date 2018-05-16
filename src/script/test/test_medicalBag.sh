#! /bin/bash
PROJECTPATH=$(dirname $(dirname $(dirname $PWD)))
gnome-terminal --window-with-profile=hold -e "roslaunch find_object_2d find_object_2d.launch"
gnome-terminal --window-with-profile=hold -e "roslaunch object_detection test.launch"
gnome-terminal --window-with-profile=hold -e "rosbag play $PROJECTPATH/exp_data/medical_d_t.bag"
