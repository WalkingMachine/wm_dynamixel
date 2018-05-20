//
// Created by lucas on 29/06/17.
//

#include "WMDynamixel.h"

namespace wm_dynamixel {
	WMDynamixel::WMDynamixel(int Id, double offset, int resolution, int direction, int mode, double ratio,
	                         int maxSpeed) {
		updateDynamixel(Id, offset, resolution, direction, mode, ratio, maxSpeed);
	}

	void WMDynamixel::initDynamixel() {
		ROS_INFO("//set TORQUE to ON");
		write1BDynamixel(_ID, ADDR_P1_TORQUE_ENABLE, 1);
		usleep(DELAY);

		//set speed to 0
		write2BDynamixel(_ID, ADDR_P1_MOVING_SPEED_2BYTES, 0);
		usleep(DELAY);

		if (_mode == 0) {
			//Set WHEEL mode
			ROS_INFO("//set WHEEL Mode");
			write2BDynamixel(_ID, ADDR_P1_CW_LIMIT_2BYTES, 0);
			write2BDynamixel(_ID, ADDR_P1_CCW_LIMIT_2BYTES, 0);
		} else if (_mode == 1) {
			//Set POSITION mode
			ROS_INFO("//set Position Mode");
			write2BDynamixel(_ID, ADDR_P1_CW_LIMIT_2BYTES, 0);
			write2BDynamixel(_ID, ADDR_P1_CCW_LIMIT_2BYTES, _resolution);
		}

		usleep(DELAY);
	}

	bool WMDynamixel::setVelocity(double newVelocity) {
		//read and calculate new velocity
		auto iVelocity = (int) (newVelocity * 325.631013566 * _direction);
		if (iVelocity < 0) {
			iVelocity = 1023 - iVelocity;
		}

		//write velocity in dynamixel
		if (!write2BDynamixel(_ID, ADDR_P1_MOVING_SPEED_2BYTES, iVelocity)) {
			ROS_WARN("Couldn't send velocity command to dynamixel: ID=%d", _ID);
			return false;
		}

		//write watchdog
		time_t timer;
		time(&timer);
		_watchdog = (unsigned long) timer * 1000;  //time in ms
		return true;
	}

	bool WMDynamixel::setPosition(double newPosition) {
		newPosition = AngleProxy(0, newPosition);
		//read and calculate new velocity
		auto iPosition = (int) ((newPosition / _ratio * _direction + _offset) * _resolution / 6.283185307);
		if (iPosition < 0) {
			iPosition += _resolution;
		}
		if (iPosition > _resolution) {
			iPosition -= _resolution;
		}
		//write velocity in dynamixel
		write2BDynamixel(_ID, ADDR_P1_MOVING_SPEED_2BYTES, (int) _maxSpeed);
		usleep(DELAY);
		if (!write2BDynamixel(_ID, ADDR_P1_GOAL_POSITION_2BYTES, iPosition)) {
			ROS_WARN("Couldn't send position command to dynamixel: ID=%d", _ID);
			return false;
		}

		//write watchdog
		time_t timer;
		time(&timer);
		_watchdog = (unsigned long) timer * 1000;  //time in ms
		return true;
	}

	bool WMDynamixel::publishPosition(ros::Publisher pub) {
		if (_isEnable) {
			bool dxl_error = false;
			std_msgs::Float64MultiArray msg;

			//test watchdog
			watchdogMgr();
			msg.data.push_back((double) _ID);
			usleep(DELAY);
			double rawPosition = read2BDynamixel(_ID, ADDR_P1_PRESENT_POSITION_2BYTES, &dxl_error);
			double newPosition = (rawPosition * 6.283185307 / _resolution - _offset) / _direction * _ratio;
			newPosition = AngleProxy(0, newPosition);
			msg.data.push_back(newPosition);
			if (dxl_error) {
				ROS_WARN("Couldn't read position of dynamixel: ID=%d", _ID);
				return false;
			}
			usleep(DELAY);
			msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
			msg.layout.dim[0].size = (uint) msg.data.size();
			msg.layout.dim[0].stride = 1;
			msg.layout.dim[0].label = "";

			//publish values
			pub.publish(msg);
		}
		return true;
	}

	int WMDynamixel::getID() {
		return _ID;
	}


	void WMDynamixel::updateDynamixel(int Id, double offset, int resolution, int direction, int mode, double ratio,
	                                  int maxSpeed) {
		_ID = Id;
		_isEnable = false;
		_offset = offset;
		_resolution = resolution;
		_direction = direction;
		//ROS_INFO("Dynamixel added with ID %i, offset %f and coef %f.", _ID, _offset, _coefficient);
		_isEnable = true;
		_mode = mode;
		_ratio = ratio;
		_maxSpeed = maxSpeed;
		initDynamixel();
		//ROS_INFO("Initialised a dynamixel with ID %i, offset %f and coef %f.", _ID, _offset, _coefficient);

	}

	bool WMDynamixel::watchdogMgr() {
		time_t timer;
		time(&timer);
		//read actual system time
		unsigned long sysTime = (unsigned long) timer * 1000;  //time in ms

		//compare with watchdog
		if (sysTime - WATCHDOG > _watchdog) {
			ROS_WARN("watchdog on dynamixel %i!", _ID);
			//set dynamixel speed to 0;
			for (int iTry = 0; iTry < 10; iTry++) {
				if (setVelocity(0)) {
					break;
				}
				usleep(1000);
			}
			return true;
		}
		return false;
	}

	int WMDynamixel::getMode(){
		return this->_mode;
	}

	double WMDynamixel::getCmd(){
		return this->_cmd;
	}

	void WMDynamixel::setMode(int mode){
		this->_mode = mode;
	}

	void WMDynamixel::setCmd(double cmd){
		this->_cmd = cmd;
	}
}