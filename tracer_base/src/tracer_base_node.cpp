#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "tracer_base/tracer_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "ugv_sdk/mobile_robot/tracer_robot.hpp"

using namespace westonrobot;

std::shared_ptr<TracerRobot> robot;

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "tracer_base");
    ros::NodeHandle node(""), private_node("~");

    // robot = std::unique_ptr<TracerRobot>(new TracerRobot());
    std::unique_ptr<TracerRobot> tracer;
    ProtocolDetector detector;
    try
    {
        detector.Connect("can0");
        auto proto = detector.DetectProtocolVersion(5);
        if (proto == ProtocolVersion::AGX_V1) {
            std::cout << "Detected protocol: AGX_V1" << std::endl;
            tracer = std::unique_ptr<TracerRobot>(
            new TracerRobot(ProtocolVersion::AGX_V1));
        }
        else if (proto == ProtocolVersion::AGX_V2)
        {
            std::cout << "Detected protocol: AGX_V2" << std::endl;
            tracer = std::unique_ptr<TracerRobot>(
            new TracerRobot(ProtocolVersion::AGX_V2));
        }
        else
        {
            std::cout << "Detected protocol: UNKONWN" << std::endl;
            return -1;
        }

    }
    catch (std::exception error)
    {
        ROS_ERROR("please bringup up can or make sure can port exist");
        ros::shutdown();
    }


    TracerROSMessenger messenger(tracer.get(), &node);

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
            tracer->Connect(port_name);
        }
        catch (std::exception error)
        {
            ROS_ERROR("please bringup up can or make sure can port exist");
            ros::shutdown();
        }
        tracer->EnableCommandedMode();
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
