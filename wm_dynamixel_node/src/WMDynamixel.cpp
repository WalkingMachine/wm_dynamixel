//
// Created by lucas on 29/06/17.
//

#include "WMDynamixel.h"
WMDynamixel::WMDynamixel(int Id, double offset, int resolution, int direction, int mode ) {
	updateDynamixel(Id, offset, resolution, direction, mode );
}

void WMDynamixel::initDynamixel() {
	ROS_INFO("//set TORQUE to ON");
	write1BDynamixel(_ID, ADDR_P1_TORQUE_ENABLE, 1);
	usleep(DELAY);

    oldPosition = 0;
    oldVelocity = 0;

	//set speed to 0
	write2BDynamixel(_ID, ADDR_P1_MOVING_SPEED_2BYTES, 0);
	usleep(DELAY);

    if ( _mode == 0 ) {
        //Set WHEEL mode
        write2BDynamixel(_ID, ADDR_P1_CW_LIMIT_2BYTES, 0);
        write2BDynamixel(_ID, ADDR_P1_CCW_LIMIT_2BYTES, 0);
    } else if ( _mode == 1 ){
        //Set WHEEL mode
        write2BDynamixel(_ID, ADDR_P1_CW_LIMIT_2BYTES, _resolution );
        write2BDynamixel(_ID, ADDR_P1_CCW_LIMIT_2BYTES, 0);
    }

	usleep(DELAY);
}

bool WMDynamixel::setVelocity(double newVelocity) {
	oldVelocity = newVelocity;
	//read and calculate new velocity
	int iVelocity = (int) (newVelocity * 325.631013566);
	if (iVelocity < 0) {
		iVelocity = 1023 - iVelocity;
	}

	//write velocity in dynamixel
	if (!write2BDynamixel(_ID, ADDR_P1_MOVING_SPEED_2BYTES, iVelocity)) {
		ROS_ERROR("error while sending velocity to dynamixel: ID=%d", _ID );
		return false;
	}

	//write watchdog
	time_t timer;
	time(&timer);
	_watchdog = (unsigned long) timer * 1000;  //time in ms
	return true;
}

bool WMDynamixel::setPosition(double newPosition) {
	//read and calculate new velocity
	int iPosition = (int) ((newPosition+_offset)*_direction*83.765759522);
//    if (iPosition < 0) {
        iPosition = 0;
//    }
//	if (iPosition > 1023 ) {
//		iPosition = 1023;
//	}
	//write velocity in dynamixel
	if (!write2BDynamixel(_ID, ADDR_P1_GOAL_POSITION_2BYTES, iPosition)) {
        ROS_ERROR("error while sending position to dynamixel: ID=%d", _ID);
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
        double newPosition = _direction*_coefficient * read2BDynamixel(_ID, ADDR_P1_PRESENT_POSITION_2BYTES, &dxl_error) - _offset;
        double dP = newPosition-oldPosition;
        //if ( dP < MAX_DELTA_POSITION && dP > -MAX_DELTA_POSITION ) {
            msg.data.push_back(newPosition);
            if (dxl_error) {
                ROS_ERROR("Error reading position of dynamixel: ID=%d", _ID);
                return false;
            }
            usleep(DELAY);
            msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msg.layout.dim[0].size = (uint) msg.data.size();
            msg.layout.dim[0].stride = 1;
            msg.layout.dim[0].label = "";

            //publish values
            pub.publish(msg);
        //}
        oldPosition = newPosition;
	}
	return true;
}

int WMDynamixel::getID(){
	return _ID;
}

void WMDynamixel::updateDynamixel(int Id, double offset, int resolution, int direction, int mode) {
	_ID = Id;
	_isEnable = false;
	_offset = offset;
	_coefficient = (2 * PI) / resolution;
    _resolution = resolution;
    _direction = direction;
	//ROS_INFO("Dynamixel added with ID %i, offset %f and coef %f.", _ID, _offset, _coefficient);
	initDynamixel();
	_isEnable = true;
	_mode = mode;
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
