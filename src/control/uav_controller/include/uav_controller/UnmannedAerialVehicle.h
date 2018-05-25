//
// Created by songjin on 18-5-25.
//

#ifndef UAV_CONTROL_UNMANNEDAERIALVEHICLE_H
#define UAV_CONTROL_UNMANNEDAERIALVEHICLE_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
/*
 * 保存无人机的状态,控制无人机运动
 */
class UnmannedAerialVehicle {
public:
    enum UAV_State{arrive_destination};
    UnmannedAerialVehicle();
    void setCurrentPoseStamped(const geometry_msgs::PoseStamped &vision_pose);
    geometry_msgs::PoseStamped getCurrentPoseStamped();
    geometry_msgs::PoseStamped getCurrentDestinationPoseStamped();
    void fly_to_goal_by_path_onestep();
    void set_path_to_goal(const nav_msgs::Path &path_to_goal_);
    bool is_arrive_destination()
    {
        if(uav_state_ == arrive_destination)
            return true;
        else
            return false;
    }
private:
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped current_destination_pose_;
    nav_msgs::Path path_to_goal_;
    UAV_State uav_state_;
};


#endif //UAV_CONTROL_UNMANNEDAERIALVEHICLE_H
