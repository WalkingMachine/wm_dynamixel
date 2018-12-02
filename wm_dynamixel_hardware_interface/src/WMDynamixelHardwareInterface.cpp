//
// Created by philippe on 03/05/17.
//

#include "WMDynamixelHardwareInterface.h"
#include <nodelet/nodelet.h>
#include "wm_dynamixel_interface.cpp"

namespace wm_dynamixel_hardware_interface {


    hardware_interface::VelocityJointInterface WMDynamixelHardwareInterface::joint_velocity_interface_;
    hardware_interface::PositionJointInterface WMDynamixelHardwareInterface::joint_position_interface_;
    hardware_interface::JointStateInterface    WMDynamixelHardwareInterface::joint_state_interface_;

    WMDynamixelHardwareInterface() : mDynamixelInterface()
    {

    }
    ~WMDynamixelHardwareInterface() {

    }


	bool WMDynamixelHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
		using namespace hardware_interface;
		
		// TODO  // Get parameters
        Address = "";
        Baud = 0;
        Offset = 0;
		Id = 0;
        resolution = 4096;
        direction = 1;

        simulation = false;
        mode = 0;
        double ratio = 1;
        int maxSpeed = 1023;
    
        std::vector<std::string> Joints;
        robot_hw_nh.getParam("address", Address);
        robot_hw_nh.getParam("baudrate", Baud);
        if (!robot_hw_nh.getParam("id", Id)) { return false; }
        robot_hw_nh.getParam("offset", Offset);
        robot_hw_nh.getParam("resolution", resolution);
        if (!robot_hw_nh.getParam("joints", Joints)) { return false; }
        robot_hw_nh.getParam("mode", mode);
        robot_hw_nh.getParam("simulation", simulation);
        robot_hw_nh.getParam("direction", direction);
        robot_hw_nh.getParam("ratio", ratio);
        robot_hw_nh.getParam("max_speed", maxSpeed);
        Name = Joints[0];
        oldCmd = 0;

		// Initialise interface variables
		cmd = 0;	//command
		pos = 0;    //position
		vel = 0;    //velocity
		eff = 0;    //effort

        // Register interfaces
        joint_state_interface_.registerHandle(JointStateHandle(Name, &pos, &vel, &eff));
        registerInterface(&joint_state_interface_);

        // registering dynamixel joint_velocity_interface
        ROS_INFO("registering dynamixel joint_velocity_interface: ID=%d", Id);
        joint_velocity_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
        registerInterface(&joint_velocity_interface_);

        // Wait for the universe to get ready for our greatness!
        ROS_INFO("Dynamixel initialised");
		return true;
	}
	
	void WMDynamixelHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
        if (simulation) {
            if ( mode == 0 )
                pos += cmd / 30;
            if ( mode == 1 )
                pos = cmd;
        }
	}
	
	void WMDynamixelHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
        // Eliminate impossible commands

        if ( cmd > 5 || cmd < -5){ cmd = 0; }

        if (!simulation) {
            std_msgs::Float64MultiArray msg;
            msg.data.push_back(double(Id));
            msg.data.push_back(cmd);
            msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msg.layout.dim[0].size = 2;
            msg.layout.dim[0].stride = 1;
            msg.layout.dim[0].label = "";
            CtrlPub.publish(msg);
            oldCmd = cmd;
        }

	}

	void WMDynamixelHardwareInterface::StatusCB( std_msgs::Float64MultiArrayConstPtr msg ){
		if ( Id == (int)msg->data[0] ){
			pos = msg->data[1];
			//vel = msg->data[2];
			//eff = msg->data[3];
		}
	}
}
PLUGINLIB_EXPORT_CLASS(wm_dynamixel_hardware_interface::WMDynamixelHardwareInterface, hardware_interface::RobotHW)