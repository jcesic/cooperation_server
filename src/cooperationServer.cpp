#include <ros/ros.h>
#include <ros/time.h>

#include "SLAMserver.h"

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cooperationServer");

    SLAMserver _SLAMserver;
    _SLAMserver.update_map();


    // Spin
    ros::spin ();
}
