// publisher_node.cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("counter", 10);
    ros::Rate rate(1); // パブリッシュレートを1Hzに設定

    int count = 1;
    while (ros::ok())
    {
        std_msgs::Int32 msg;
        msg.data = count;
        pub.publish(msg);

        ROS_INFO("Publishing: %d", msg.data);

        count++;
        if (count > 10)
            count = 1;

        rate.sleep();
    }

    return 0;
}