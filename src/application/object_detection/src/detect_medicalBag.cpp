#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <limits>
#include <algorithm>

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Topic you want to publish
        ros::Publisher box_pub = n_.advertise<std_msgs::Float32MultiArray>("/object_boxs", 1);

        //Topic you want to subscribe
        objects_sub = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::objects_sub_callback, this);
    }

    void objects_sub_callback(const std_msgs::Float32MultiArray& msg)
    {
        std_msgs::Float32MultiArray box;
        // the id of the object
        box.data.push_back(msg.data[0]);

        float ow = msg.data[1];
        float oh = msg.data[2];
        float h11 = msg.data[3];
        float h12 = msg.data[4];
        float h13 = msg.data[5];
        float h21 = msg.data[6];
        float h22 = msg.data[7];
        float h23 = msg.data[8];
        float h31 = msg.data[9];
        float h32 = msg.data[10];
        float h33 = msg.data[11];

        // coordinate of four vertexs
        float x0 = h31;
        float y0 = h32;
        float x1 = h11 * ow + h31;
        float y1 = h12 * ow + h32;
        float x2 = h21 * oh + h31;
        float y2 = h22 * oh + h32;
        float x3 = h11 * ow + h21 * oh + h31;
        float y3 = h11 * ow + h22 * oh + h32;

        // upper left corner and lower right corner
        float xmin = std::min(x0, std::min(x1, std::min(x2, x3)));
        float xmax = std::max(x0, std::max(x1, std::max(x2, x3)));
        float ymin = std::min(y0, std::min(y1, std::min(y2, y3)));
        float ymax = std::max(y0, std::max(y1, std::max(y2, y3)));

        box.data.push_back(xmin);
        box.data.push_back(xmax);
        box.data.push_back(xmax-xmin);
        box.data.push_back(ymax-ymin);
        box_pub.publish(box);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher box_pub;
    ros::Subscriber objects_sub;

};//End of class SubscribeAndPublish


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_medicalBag");
    SubscribeAndPublish SAPObject;
    ros::spin();
}

