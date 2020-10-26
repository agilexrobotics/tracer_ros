#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "tracer_base/tracer_messenger.hpp"

using namespace westonrobot;

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "tracer_base");
    ros::NodeHandle node(""), private_node("~");

    // instantiate a robot object
    TracerBase robot;
    TracerROSMessenger messenger(&robot, &node);

    // fetch parameters before connecting to robot
    std::string port_name;
    private_node.param<std::string>("port_name", port_name, std::string("can0"));
    private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link"));
    private_node.param<bool>("simulated_robot", messenger.simulated_robot_, false);

    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos)
    {
        robot.Connect(port_name);
        ROS_INFO("Using CAN bus to talk with the robot");
    }
    else
    {
      ROS_ERROR("Only CAN bus interface is supported for now");
      return -1;
    }
    messenger.SetupSubscription();

    // publish robot state at 50Hz while listening to twist commands
    ros::Rate rate_50hz(50);  // 50Hz
    int cnt = 0;
    while (ros::ok()) {
      messenger.PublishStateToROS();
      ros::spinOnce();
      rate_50hz.sleep();
      // if(++cnt == 390) break;
    }

    return 0;
}
