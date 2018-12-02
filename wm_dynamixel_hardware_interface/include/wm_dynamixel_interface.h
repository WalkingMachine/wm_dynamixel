//
// Created by wm on 02/12/18.
//

#ifndef SARA_WS_WM_DYNAMIXEL_INTERFACE_H
#define SARA_WS_WM_DYNAMIXEL_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <pluginlib/class_list_macros.h>


//classe qui permet d'acceder à un dynamixel
class DynamixelInterface  {

    int mID;
    uint16_t itIsA2BValue(uint16_t address, bool *is2BValue);
public:

    // fonctions pour écrire/lire une valeur dans un dynamixel
    bool readDataDynamixel(int address, int value);
    bool writeDataDynamixel(int address, int value);

    // un constructeur et un destructeur
    DynamixelInterface(int ID=0);
    ~DynamixelInterface();

    // Accesseurs pour le ID
    int ID() {return mID;}
    void setID(int value){mID=value;}
}

#endif //SARA_WS_WM_DYNAMIXEL_INTERFACE_H
