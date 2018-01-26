//
// Created by philippe on 03/05/17.
//

#ifndef PROJECT_WMDynamixelNode_H
#define PROJECT_WMDynamixelNode_H

#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "wm_dynamixel_node/ReadDataDynamixel.h"

#include "dynamixel_sdk.h"

#define NBR_OF_TRY 10
#define PORTNAME "/dev/dynamixel"
#define BAUDRATE 57600


/**
 * reading Loop for physical feedback
 */
void nodeLoop();

/**
 * Initialise the connection with
 * @param PortName
 * @param BaudRate
 * @return false if error
 */
bool InitPort(const char *PortName, int BaudRate);

/**
 * callback for new command messages
 * @param msg
 */
void WriteVelocity();
void UpdateVelocity(std_msgs::Float64MultiArrayConstPtr msg);

/**
 * callback for initialise a dynamixel
 * @param msg
 */
void addDynamixel(std_msgs::Float64MultiArrayConstPtr msg);

/**
 * Function writing 1 byte in definite address of dynamixel selected
 * @param ID
 * @param iAddress
 * @param iValue
 * @return
 */
bool write1BDynamixel(int ID, int iAddress, int iValue);

/**
 * Function writing 2 bytes in definite address of dynamixel selected
 * @param ID
 * @param iAddress
 * @param iValue
 * @return
 */
bool write2BDynamixel(int ID, int iAddress, int iValue);

/**
 * Function reading 1 byte in definite address of dynamixel selected
 * @param ID
 * @param iAddress
 * @return
 */
int read1BDynamixel(int ID, int iAddress);

/**
 * Function reading 2 bytes in definite address of dynamixel selected
 * @param ID
 * @param iAddress
 * @return
 */
int read2BDynamixel(int ID, int iAddress, bool *returnError);

/**
 * Service method
 * @param req
 * @param res
 * @return
 */
bool Read_Data_Dynamixel(wm_dynamixel_node::ReadDataDynamixel::Request &req,
                         wm_dynamixel_node::ReadDataDynamixel::Response &res);
#endif //PROJECT_WMDynamixelNode_H
