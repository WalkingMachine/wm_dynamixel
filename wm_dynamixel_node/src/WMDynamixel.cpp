//
// Created by lucas on 29/06/17.
//

#include "WMDynamixel.h"

namespace wm_dynamixel {

	WMDynamixel::WMDynamixel(int Id, double offset, int resolution, int direction, int mode, double ratio, int maxSpeed) {
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
			this->_cmd = 0;
		} else if (_mode == 1) {
			//Set POSITION mode
			ROS_INFO("//set Position Mode");
			write2BDynamixel(_ID, ADDR_P1_CW_LIMIT_2BYTES, 0);
			write2BDynamixel(_ID, ADDR_P1_CCW_LIMIT_2BYTES, _resolution);

			bool dxl_error = false;
			double rawPosition = read2BDynamixel(_ID, ADDR_P1_PRESENT_POSITION_2BYTES, &dxl_error);

			if (!dxl_error) {
				double newPosition = (rawPosition * 6.283185307 / _resolution - _offset) / _direction * _ratio;
				this->_cmd = AngleProxy(0, newPosition);
			}else{
				ROS_WARN("Couldn't read position of dynamixel for init: ID=%d", _ID);
				this->_cmd = 0;
			}
		}
		usleep(DELAY);
	}

	bool WMDynamixel::setVelocity(double newVelocity) {
		// Calculate new velocity
		auto iVelocity = (int) (newVelocity * 325.631013566 * _direction);
		if (iVelocity < 0) {
			iVelocity = 1023 - iVelocity;
		}

		if((iVelocity - this->_cmd) < 0.05 && (iVelocity - this->_cmd) > -0.05) return true;

		for (int iTry = 0; iTry < 10; iTry++) {
			if (write2BDynamixel(_ID, ADDR_P1_MOVING_SPEED_2BYTES, iVelocity)) {
				this->_cmd = iVelocity;
				return true;
			}
			ROS_WARN("Couldn't send velocity command to dynamixel: ID=%d", _ID);
			usleep(1000);
		}

		return false;
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

		if(iPosition - this->_cmd < 0.05 && iPosition - this->_cmd > -0.05) return true;

		iPosition = AngleProxy(3.14159, iPosition)+3.14159;

		for (int iTry = 0; iTry < 10; iTry++) {
			if(write2BDynamixel(_ID, ADDR_P1_MOVING_SPEED_2BYTES, (int) _maxSpeed)){
				usleep(DELAY);
				if (write2BDynamixel(_ID, ADDR_P1_GOAL_POSITION_2BYTES, iPosition)) {
					this->_cmd = iPosition;
					return true;
				}
				else {
					ROS_WARN("Couldn't send position command to dynamixel: ID=%d", _ID);
					usleep(DELAY);
				}
			}
			else {
				ROS_WARN("Couldn't send velocity command to dynamixel: ID=%d", _ID);
			}
		}
		return false;
	}

	bool WMDynamixel::publishPosition(ros::Publisher pub) {
		if (_isEnable) {
			bool dxl_error = false;
			std_msgs::Float64MultiArray msg;

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

	void WMDynamixel::updateDynamixel(int Id, double offset, int resolution, int direction, int mode, double ratio, int maxSpeed) {
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

	double WMDynamixel::Mod(double A, double N) {
		return A - floor(A / N) * N;
	}

	double WMDynamixel::AngleProxy(double A1, double A2) {  // Give the smallest difference between two angles in rad
		A1 = A2 - A1;
		A1 = Mod(A1 + M_PI, 2 * M_PI) - M_PI;
		return A1;
	}

	int WMDynamixel::getID() {
		return this->_ID;
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