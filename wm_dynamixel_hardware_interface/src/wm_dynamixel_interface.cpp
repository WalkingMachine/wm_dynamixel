//
// Created by wm on 02/12/18.
//
#include "wm_dynamixel_interface.h"

DynamixelInterface::readDataDynamixel(int address, int value) {

    int dxl_comm_result;
    uint16_t returnValue16;
    uint8_t returnValue8;
    uint8_t returnError = 0;

    //determine si on a affaire avec un 1 ou 2 bytes
    bool bIts2Bytes;
    address = itIsA2BValue((uint16_t) address, &bIts2Bytes);

    // lit deux bytes
    if (bIts2Bytes) {
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, mID, address, &returnValue16, &returnError);
        ROS_INFO("result: %i ; value: %i ; error: %i", dxl_comm_result, returnValue16, returnError);

    } else {  // lit un byte
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, mID, address, &returnValue8, &returnError);
        ROS_INFO("result: %i ; value: %i ; error: %i", dxl_comm_result, returnValue8, returnError);
    }
    return dxl_comm_result == COMM_SUCCESS;
}

DynamixelInterface::writeDataDynamixel(int address, int value) {

    bool bIts2Bytes;
    address = itIsA2BValue((uint16_t) address, &bIts2Bytes);
    int dxl_comm_result;
    uint8_t returnError = 0;
    auto iID = (uint8_t) req.id;

    // lit deux bytes
    if (bIts2Bytes) {
        auto value = (uint16_t) req.value;
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, mID, address, value, &returnError);

    } else { // lit un byte
        auto value = (uint8_t) req.value;
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, mID, address, value, &returnError);
    }

    return dxl_comm_result == COMM_SUCCESS;
}

uint16_t DynamixelInterface::itIsA2BValue(uint16_t address, bool *is2BValue) {

    switch (address) {
        case 6:
        case 7:
            *is2BValue = true;
            return 6;

        case 8:
        case 9:
            *is2BValue = true;
            return 8;

        case 14:
        case 15:
            *is2BValue = true;
            return 14;

        case 20:
        case 21:
            *is2BValue = true;
            return 20;

        case 30:
        case 31:
            *is2BValue = true;
            return 30;

        case 32:
        case 33:
            *is2BValue = true;
            return 32;

        case 34:
        case 35:
            *is2BValue = true;
            return 34;

        case 48:
        case 49:
            *is2BValue = true;
            return 48;

        case 68:
        case 69:
            *is2BValue = true;
            return 68;

        case 71:
        case 72:
            *is2BValue = true;
            return 71;

        default:
            *is2BValue = false;
            return address;
    }
}
DynamixelInterface::DynamixelInterface(int ID) :
    mID(ID)
{

}
