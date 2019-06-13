#! /usr/bin/env python3

import rospy
from automi_control.msg import Servo_Msg

from sys_util import *                      # Library to setup port and portHandler
from dynamixel_sdk import *

# --------------------------Control Table Addresses--------------------------#
ADDR_MX_TORQUE_ENABLE           = 64
ADDR_MX_PROFILE_VELOCITY        = 112
ADDR_MX_PROFILE_ACCELERATION    = 108
ADDR_MX_GOAL_POSTION            = 116
ADDR_MX_MOVING                  = 122
ADDR_MX_P_GAIN_POSITION         = 84

LEN_MX_GOAL_POSITION            = 4
# ---------------------------------------------------------------------------#


# -------------------------Setting Up PacketHandler--------------------------#
PROTOCOL_VERSION                = 2.0
packetHandler                   = PacketHandler(PROTOCOL_VERSION)
# ---------------------------------------------------------------------------#


#---------------------------Setting Up Dynamixels----------------------------#
num_dxl     = 10
DXL_ID      = [1,2,9,11,12,13,14,15,16,17]

TORQUE_ENABLE           = 1
PROFILE_VELOCITY        = 900
PROFILE_ACCELERATION    = 400
P_GAIN_POSITION         = 1500
#----------------------------------------------------------------------------#

rospy.init_node('servo_motion', anonymous=True)

open_port()
set_baudrate()

for dxl_index in range(num_dxl):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[dxl_index], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)       

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID[dxl_index], ADDR_MX_P_GAIN_POSITION, P_GAIN_POSITION)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[dxl_index], ADDR_MX_PROFILE_VELOCITY, PROFILE_VELOCITY)    
    
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[dxl_index], ADDR_MX_PROFILE_ACCELERATION, PROFILE_ACCELERATION)
    
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result), DXL_ID[dxl_index])
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error), DXL_ID[dxl_index])

def isMoving():
    value = 1
    for i in range(num_dxl):
        moving_status, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_MOVING)
        if moving_status:
            value = 0

    return value

def write_pos(goal_pos):
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
    for i in range(num_dxl):
        # Add all Dynamixels' goal position value to groupSyncWrite parameter storage
            
        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_pos[i])), DXL_HIBYTE(DXL_LOWORD(goal_pos[i])), DXL_LOBYTE(DXL_HIWORD(goal_pos[i])), DXL_HIBYTE(DXL_HIWORD(goal_pos[i]))]
        dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position)
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL_ID[i]))
            quit()  
        
    # SyncWrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    while 1:
        if isMoving():
            break

    # Clear SyncWrite parameter storage
    groupSyncWrite.clearParam()

def callback(data):
    goal_pos = []*10
    goal_pos[0] = data.id1
    goal_pos[1] = data.id2
    goal_pos[2] = data.id9
    goal_pos[3] = data.id11
    goal_pos[4] = data.id12
    goal_pos[5] = data.id13
    goal_pos[6] = data.id14
    goal_pos[7] = data.id15
    goal_pos[8] = data.id16
    goal_pos[9] = data.id17

    write_pos(goal_pos)


def subscribeAngles():
    rospy.Subscriber("servo_angles", Servo_Msg, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribeAngles()
    except rospy.ROSInterruptException:
        pass

