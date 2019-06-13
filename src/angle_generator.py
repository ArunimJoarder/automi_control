#! /usr/bin/env python3

import rospy
from automi_control.msg import Servo_Msg

rospy.init_node('angle_gen', anonymous=True)

init_file = open("/home/arunim/catkin_ws/src/automi_control/src/walk_init_angles_biped.txt")
num_dxls_init = 0
for line in init_file:
    num_dxls_init += 1
init_file.close()

init_file = open('/home/arunim/catkin_ws/src/automi_control/src/walk_init_angles_biped.txt')
INIT_INFO = [0]*num_dxls_init
for dxl_id in range(num_dxls_init):
    INIT_INFO[dxl_id] = []
    line = init_file.readline()
    INIT_INFO[dxl_id].append([(x) for x in line.split()])
init_file.close()
for dxl_id in range(num_dxls_init):
    for i in range(3):
        INIT_INFO[dxl_id][0][i] = int(INIT_INFO[dxl_id][0][i])

def calculate_ang(angle, ch, dxl_index):
    if ch == '+':
        return (angle + INIT_INFO[dxl_index][0][2]) % 4095

    elif ch == '-':
        return (INIT_INFO[dxl_index][0][2] - angle) % 4095

    else:
        return angle

def publish(msg):
    pub = rospy.Publisher('servo_angles', Servo_Msg, queue_size=10)
    rate = rospy.Rate(10)
    
    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()

def callback(data):
    msg = Servo_Msg()
    
    msg.id1  = calculate_ang(data.id1 , INIT_INFO[0][0][3], 0)
    msg.id2  = calculate_ang(data.id2 , INIT_INFO[1][0][3], 1)
    msg.id9  = calculate_ang(data.id9 , INIT_INFO[2][0][3], 2)
    msg.id11 = calculate_ang(data.id11, INIT_INFO[3][0][3], 3)
    msg.id12 = calculate_ang(data.id12, INIT_INFO[4][0][3], 4)
    msg.id13 = calculate_ang(data.id13, INIT_INFO[5][0][3], 5)
    msg.id14 = calculate_ang(data.id14, INIT_INFO[6][0][3], 6)
    msg.id15 = calculate_ang(data.id15, INIT_INFO[7][0][3], 7)
    msg.id16 = calculate_ang(data.id16, INIT_INFO[8][0][3], 8)
    msg.id17 = calculate_ang(data.id17, INIT_INFO[9][0][3], 9)

    publish(msg)

def subscribeAngles():
    rospy.Subscriber("sim_angles", Servo_Msg, callback)
    rospy.spin()

if __name__ == '__main__':
    subscribeAngles()