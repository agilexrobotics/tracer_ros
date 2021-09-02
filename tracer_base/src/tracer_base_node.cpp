#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "tracer_base/tracer_messenger.hpp"

using namespace westonrobot;

std::unique_ptr<TracerRobot> robot;

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "tracer_base");
    ros::NodeHandle node(""), private_node("~");

    robot = std::unique_ptr<TracerRobot>(new TracerRobot());
    if (robot == nullptr)
        std::cout << "Failed to create robot object" << std::endl;

    TracerROSMessenger messenger(robot.get(), &node);

    // fetch parameters before connecting to robot
    std::string port_name;
    private_node.param<std::string>("port_name", port_name, std::string("can0"));
    private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link"));
    private_node.param<bool>("simulated_robot", messenger.simulated_robot_, false);

    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos)
    {
        try
        {
            robot->Connect(port_name);
        }
        catch (std::exception error)
        {
            ROS_ERROR("please bringup up can or make sure can port exist");
            ros::shutdown();
        }
        robot->EnableCommandedMode();
        ROS_INFO("Using CAN bus to talk with the robot");
    }
    else
    {
        ROS_INFO("Using UART to talk with the robot");
    }
    messenger.SetupSubscription();

    // publish robot state at 50Hz while listening to twist commands
    ros::Rate rate_50hz(50);  // 50Hz
    //int cnt = 0;
    while (ros::ok()) {
      if (port_name.find("can") != std::string::npos)
          messenger.PublishStateToROS();
     // else  messenger.PublishUartStateToROS();
      ros::spinOnce();
      rate_50hz.sleep();
    }
    return 0;
}
