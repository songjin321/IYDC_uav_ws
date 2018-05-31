//
// Created by songjin on 18-5-31.
//

#include "detection/ObjectPosePub.h"

ObjectPosePub::ObjectPosePub(const std::string &publish_topic_name) {
    //Topic you want to publish
    pub_center_point_ = n_.advertise<std_msgs::Float32MultiArray>(publish_topic_name , 1);
    pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>(publish_topic_name , 1);
}

void ObjectPosePub::publish(const cv::Rect2f &box) {
    center_point_.data.push_back(box.x + box.width/2.0);
    center_point_.data.push_back(box.y + box.height/2.0);
    pub_center_point_.publish(center_point_);
}

void ObjectPosePub::publish(const geometry_msgs::PoseStamped &pose) {
    pub_pose_.publish(pose);
}
