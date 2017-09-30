//
// Created by lucas on 29/06/17.
//

#ifndef PROJECT_WMDYNAMIXEL_H
#define PROJECT_WMDYNAMIXEL_H

#include "WMDynamixelNode.h"
#include <ctime>

#define PI 3.1415926535897932
#define DELAY 10000		//us
#define WATCHDOG 100000	//us
#define MAX_ACCELERATION 0.2  //rad/cycle
#define MAX_DELTA_POSITION 0.2  //rad/cycle
#define MAX_POSITION 80/180*PI  //rad/cycle
#define MIN_POSITION -80/180*PI  //rad/cycle

class WMDynamixel {
public:
    /**
     * Generate a new dynamixel with entered parameters
     * @param Id
     * @param offset
     * @param resolution
     */
	WMDynamixel(int Id, double offset, int resolution, int direction, int mode, double ratio, int maxSpeed);
    /**
     *  Update dynamixel parameters
     * @param Id
     * @param offset
     * @param resolution
     */
	void updateDynamixel(int Id, double offset, int resolution, int direction, int mode, double ratio, int maxSpeed);

	/**
	 * Send a new velocity to the dynamixel (in Rad/s)
	 * @param newVelocity
	 * @return true if OK
	 */
	bool setVelocity(double newVelocity);

	/**
	 * Send a new position to the dynamixel (in Rad)
	 * @param newPosition
	 * @return true if OK
	 */
	bool setPosition(double newPosition);

    /**
     * Publish new dynamixel's position, velocity and load in a topic
     * @param pub
     * @return true if OK
     */
	bool publishPosition(ros::Publisher pub);

    /**
     *
     * @return getter for ID
     */
	int getID();

	// int control mode ( 0=velocity, 1=position )
	int _mode;

	// command
	double _cmd;

private:
	//id of material dynamixel
	int _ID;
	
	//the dynamixel is actif
	bool _isEnable;
	
	//initial position of dynamixel
	double _offset;

    // direction of rotation
    int _direction;

	//number of positions for 2Pi Rad in dynamixel
	int _resolution;

    //last write time
	unsigned long  _watchdog;

    //initialise dynamixel speed to 0 and toggle torque
	void initDynamixel();

    //test if watchdog is
    bool watchdogMgr();

    // special gear ratio
    double _ratio;

    // Max rotational speed from 0 to 1023 (dynamixel speed units)
    double _maxSpeed;

	double Mod( double A, double N ) {
		return A-floor(A/N)*N;
	}
	double AngleProxy( double A1 = 0, double A2 = 0 ) {  // Give the smallest difference between two angles in rad
		A1 = A2-A1;
		A1 = Mod( A1+M_PI, 2*M_PI )-M_PI;
		return A1;
	}

};



#endif //PROJECT_WMDYNAMIXEL_H
