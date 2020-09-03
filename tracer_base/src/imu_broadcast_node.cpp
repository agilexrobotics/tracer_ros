/* 
 * imu_broadcast_node.cpp
 * 
 * Created on: Oct 02, 2019 19:09
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include <string>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/wescore.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Publisher imu_pub;

struct MessageBroadcaster
{
    void IMULCMCallback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const wescore_lcm_msgs::IMU *msg)
    {
        static uint64_t count = 0;
        // std::cout << "imu msg received" << std::endl;
        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.seq = count++;

        imu_msg.angular_velocity.x = msg->angular_velocity.x;
        imu_msg.angular_velocity.y = msg->angular_velocity.y;
        imu_msg.angular_velocity.z = msg->angular_velocity.z;

        imu_msg.linear_acceleration.x = msg->linear_acceleration.x;
        imu_msg.linear_acceleration.y = msg->linear_acceleration.y;
        imu_msg.linear_acceleration.z = msg->linear_acceleration.z;

        for (int i = 0; i < 9; ++i)
        {
            imu_msg.orientation_covariance[i] = msg->orientation_covariance[i];
            imu_msg.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
            imu_msg.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
        }

        imu_pub.publish(imu_msg);
    }
};

int main(int argc, char **argv)
{
    // setup LCM
    lcm::LCM lcm;

    if (!lcm.good())
        return 1;
    MessageBroadcaster mb;
    lcm.subscribe("sensor_hub_raw_imu", &MessageBroadcaster::IMULCMCallback, &mb);

    // setup ROS node
    ros::init(argc, argv, "imu_broadcast_node");
    ros::NodeHandle nh;
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);

    ROS_INFO("Started broadcasting");
    while (ros::ok())
    {
        lcm.handleTimeout(5);
    }

    return 0;
}