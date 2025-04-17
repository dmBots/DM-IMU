#include "ros/ros.h"

#include <dm_imu/imu_driver.h>
#include <iostream>
#include <thread>
#include <condition_variable>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dm_imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Rate r(1000);

    std::shared_ptr<dmbot_serial::DmImu> imuInterface;
    imuInterface=std::make_shared<dmbot_serial::DmImu>(nh, nh_private);

    while (ros::ok()) 
    {   
      r.sleep();
    }

    return 0;
}




