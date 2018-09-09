//
// Created by lucas on 26/06/17.
//

#ifndef WM_DYNAMIXEL_HARDWARE_INTERFACE_ADDR_LIST_H
#define WM_DYNAMIXEL_HARDWARE_INTERFACE_ADDR_LIST_H

#define ADDR_P1_MODEL_NBR_2BYTES            0x00
#define ADDR_P1_FIRMWARE_VERSION            0x02
#define ADDR_P1_ID                          0x03
#define ADDR_P1_BAUD                        0x04
#define ADDR_P1_RETURN_DELAY                0x05
#define ADDR_P1_CW_LIMIT_2BYTES             0x06
#define ADDR_P1_CCW_LIMIT_2BYTES            0x08
#define ADDR_P1_TEMPERATURE_LIMIT           0x0b
#define ADDR_P1_VOLTAGE_LIMIT_LOW           0x0c
#define ADDR_P1_VOLTAGE_LIMIT_HIGH          0x0d
#define ADDR_P1_MAX_TORQUE_2BYTES           0x0e
#define ADDR_P1_STATUS_RETURN_LEVEL         0x10
#define ADDR_P1_ALARM_LED                   0x11
#define ADDR_P1_ALARM_SHUTDOWN              0x12
#define ADDR_P1_MULTI_TURN_OFFSET_2BYTES    0x14
#define ADDR_P1_RESOLUTION_DIVIDER          0x16
#define ADDR_P1_TORQUE_ENABLE               0x18
#define ADDR_P1_LED                         0x19
#define ADDR_P1_D_GAIN                      0x1a
#define ADDR_P1_I_GAIN                      0x1b
#define ADDR_P1_P_GAIN                      0x1c
#define ADDR_P1_GOAL_POSITION_2BYTES        0x1e
#define ADDR_P1_MOVING_SPEED_2BYTES         0x20
#define ADDR_P1_TORQUE_LIMIT_2BYTES         0x22
#define ADDR_P1_PRESENT_POSITION_2BYTES     0x24
#define ADDR_P1_PRESENT_SPEED_2BYTES        0x26
#define ADDR_P1_PRESENT_LOAD_2BYTES         0x28
#define ADDR_P1_PRESENT_VOLTAGE             0x2a
#define ADDR_P1_PRESENT_TEMPERATURE         0x2b
#define ADDR_P1_REGISTERED                  0x2c
#define ADDR_P1_MOVING                      0x2e
#define ADDR_P1_LOCK                        0x2f
#define ADDR_P1_PUNCH_2BYTES                0x30
#define ADDR_P1_PRESENT_CURRENT_2BYTES      0x44
#define ADDR_P1_ENABLE_TORQUE_CONTROL       0x46
#define ADDR_P1_GOAL_TORQUE_2BYTES          0x47
#define ADDR_P1_GOAL_ACCELERATION           0x49

#endif //WM_DYNAMIXEL_HARDWARE_INTERFACE_ADDR_LIST_H
