#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clock_publisher");

    ros::NodeHandle n;

    Publisher pub = n.advertise<rosgraph_msgs::Clock>("/clock", 1);
    ros::Rate loop_rate(100);

    while (ros::ok())
    {

        rosgraph_msgs::Clock msg;
        msg.clock = ros::Time::now();
        pub.publish(msg);
        loop_rate.sleep();
    }


    return 0;
}
