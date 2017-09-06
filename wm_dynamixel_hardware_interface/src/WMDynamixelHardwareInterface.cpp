//
// Created by philippe on 03/05/17.
//

#include "WMDynamixelHardwareInterface.h"
#include <nodelet/nodelet.h>

namespace wm_dynamixel_hardware_interface {


    hardware_interface::VelocityJointInterface WMDynamixelHardwareInterface::joint_velocity_interface_;
    hardware_interface::JointStateInterface    WMDynamixelHardwareInterface::joint_state_interface_;

// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>

	bool WMDynamixelHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
		using namespace hardware_interface;
		
		// Get parameters
        Address = "";
        Baud = 0;
        Offset = 0;
		Id = 0;
        resolution = 4096;
        direction = 1;
        simulation = false;
        int mode = 0;
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
        if ( mode == 0 ){
            ROS_INFO("registering dynamixel joint_velocity_interface: ID=%d", Id);
            joint_velocity_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
            registerInterface(&joint_velocity_interface_);
        }
        if ( mode == 1 ){
            ROS_INFO("registering dynamixel joint_position_interface: ID=%d", Id);
            joint_position_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
            registerInterface(&joint_position_interface_);
        }

        if (!simulation) {
            // advertise publisher
            CtrlPub = nh.advertise<std_msgs::Float64MultiArray>("dynamixel_cmd", 10);
            InitPub = nh.advertise<std_msgs::Float64MultiArray>("dynamixel_init", 10);
            //GripperStatSub.
            StatSub = nh.subscribe("dynamixel_pos", 10, &WMDynamixelHardwareInterface::StatusCB, this);

            // Wait wor the universe to get ready for our greatness!
            ros::spinOnce();
            sleep(1);
            ros::spinOnce();
            ROS_INFO("Dynamixel initialising");
            std_msgs::Float64MultiArray msg;
            std::vector<double> vec = {
                    double(Id),
                    Offset,
                    resolution,
                    direction,
                    mode
            };
            msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msg.layout.dim[0].size = (uint) vec.size();
            msg.layout.dim[0].stride = 1;
            msg.layout.dim[0].label = "";

            //msg.layout = std_msgs::MultiArrayLayout();
            msg.data.insert(msg.data.end(), vec.begin(), vec.end());
            InitPub.publish(msg);
            ROS_INFO("Dynamixel initialised");
        }
		return true;
	}
	
	void WMDynamixelHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
        if (simulation)
            pos += cmd/30;
	}
	
	void WMDynamixelHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
        // Eliminate impossible commands
        if (!( cmd < 30 && cmd > -30) || (cmd > 0.000001 && cmd < -0.000001)){ cmd = 0; }

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