//
// Created by songjin on 18-5-25.
//

#ifndef UAV_CONTROL_UNMANNEDAERIALVEHICLE_H
#define UAV_CONTROL_UNMANNEDAERIALVEHICLE_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
/*
 * 保存无人机的状态,底层控制无人机运动,应当作为一个基类
 */
class UnmannedAerialVehicle {
public:
    enum class UAV_State{arrive_destination, begin_fly};
    UnmannedAerialVehicle();
    void setCurrentPoseStamped(const geometry_msgs::PoseStamped &vision_pose);
    geometry_msgs::PoseStamped getCurrentPoseStamped();
    geometry_msgs::PoseStamped getCurrentDestinationPoseStamped();
    void fly_to_goal_by_path_onestep();
    void set_path_to_goal(const nav_msgs::Path &path_to_goal);
    bool is_arrive_destination()
    {
        if(uav_state_ == UAV_State::arrive_destination)
            return true;
        else
            return false;
    }
    void set_state_begin_fly()
    {
        uav_state_ = UAV_State::begin_fly;
    }
private:
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped current_destination_pose_;
    nav_msgs::Path path_to_goal_;
    UAV_State uav_state_;
};


#endif //UAV_CONTROL_UNMANNEDAERIALVEHICLE_H
