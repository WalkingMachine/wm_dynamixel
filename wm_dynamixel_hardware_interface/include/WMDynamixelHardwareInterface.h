//
// Created by philippe on 03/05/17.
//

#ifndef PROJECT_WMDynamixelHardwareInterface_H
#define PROJECT_WMDynamixelHardwareInterface_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <pluginlib/class_list_macros.h>
#include "wm_dynamixel_interface.h"

namespace wm_dynamixel_hardware_interface
{
    class WMDynamixelHardwareInterface : public hardware_interface::RobotHW {
    public:
        // << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
        // Functions
        bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
        void read(const ros::Time &time, const ros::Duration &period);
        void write(const ros::Time &time, const ros::Duration &period);
        //constructeur destructeur
        WMDynamixelHardwareInterface();
        ~WMDynamixelHardwareInterface();

        // Interface variables
        std::string Name;
        double cmd;
        double pos;
        double vel;
        double eff;

    private:
        // Variables d'interface avec ROS_CONTROL
        static hardware_interface::VelocityJointInterface joint_velocity_interface_;
        static hardware_interface::JointStateInterface joint_state_interface_;
        DynamixelInterface mDynamixelInterface;

//        int Baud;
//        int Id;
//        double Offset;
//        double resolution;
//        double oldCmd;
//        int direction;
    };
}
#endif //PROJECT_WMDynamixelHardwareInterface_H
