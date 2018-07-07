#/bin/sh
rgbd()
{
    source ~/Project/uav_make_ws/devel/setup.bash
    roslaunch astra_launch astra.launch
}
orb_slam()
{
    cd ~/Project/ORB_SLAM2
    rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt param/rgbd_camera.yaml $1 
}
orb_localization()
{
    cd ~/Project/ORB_SLAM2
    rosrun ORB_SLAM2 RGBD_localization Vocabulary/ORBvoc.txt param/rgbd_camera.yaml $1 MapPointandKeyFrame.map
}
orb_test()
{
    gnome-terminal -e 'roslaunch competition_tasks others.launch'
    gnome-terminal -e 'roslaunch orb_slam test_fix_point.launch'
    gnome-terminal -e 'rostopic echo /mavros/setpoint_position/local'
    gnome-terminal -e 'rostopic echo /mavros/local_position/pose'
    gnome-terminal -e 'rostopic echo /mavros/mocap/pose'
    orb_localization
}
uav_test()
{
    gnome-terminal -e 'roslaunch competition_tasks others.launch'
    gnome-terminal -e 'roslaunch competition_tasks   test.launch'
    orb_slam
}
record_bag()
{
    rosbag record /mavros/setpoint_position/local /mavros/local_position/pose /object_pos
}
topic_show()
{
    gnome-terminal -e 'rostopic echo /mavros/setpoint_position/local'
    gnome-terminal -e 'rostopic echo /mavros/local_position/pose'
    gnome-terminal -e 'rostopic echo /object_pose'
}
set_detection()
{
    rosservice call /detection_controller_server "ControlType: $1"
}

kall()
{
    killall gnome-terminal-server
}
