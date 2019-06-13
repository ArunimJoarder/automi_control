#! /usr/bin/env python3

import numpy as np
import time 

import rospy
from automi_control.msg import Servo_Msg

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
fig = plt.figure()
ax = fig.add_axes([0, 0, 1, 1], projection='3d')


right_leg=np.matrix(np.zeros((4,14)))
left_leg=np.matrix(np.zeros((4,14)))
waist=np.matrix(np.zeros((4,4)))
zmp = np.matrix(np.zeros((4,1)))
h=30*np.cos(18*np.pi/180)
left_joint_angles=np.matrix(np.zeros((6,1))) #%from top to bottom%
temp_left_joint_angles=np.matrix(np.zeros((3,1)))
right_joint_angles=np.matrix(np.zeros((6,1)))
temp_right_joint_angles=np.matrix(np.zeros((3,1)))
centre_pelvis=np.matrix(np.zeros((4,1)))
theta=0
no_of_steps=2
right_hand_angles=np.matrix(np.zeros((4,1)))
left_hand_angles=np.matrix(np.zeros((4,1)))
right_arm=np.matrix(np.zeros((4,5)))
left_arm=np.matrix(np.zeros((4,5)))
pause=0.3

time.sleep(pause)

rospy.init_node('simulation', anonymous=True)


def Rz(t):

    m=np.matrix([[np.cos(t), -np.sin(t), 0, 0],[np.sin(t),  np.cos(t), 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
    return m



def find_angles( x,xa,h,ha ):
    # t1= 2*np.arctan((60*(x - xa))/float(h**2 - 2*h*ha + 30*h + ha**2 - 30*ha + x**2 - 2*x*xa + xa**2) + (30*xa - 30*x + (-(h**2 - 2*h*ha + ha**2 + x**2 - 2*x*xa + xa**2)*(h**2 - 2*h*ha + ha**2 + x**2 - 2*x*xa + xa**2 - 900))**(1/2))/float(h**2 - 2*h*ha + 30*h + ha**2 - 30*ha + x**2 - 2*x*xa + xa**2))
    # t2= -2*np.arctan((30*xa - 30*x + (-(h**2 - 2*h*ha + ha**2 + x**2 - 2*x*xa + xa**2)*(h**2 - 2*h*ha + ha**2 + x**2 - 2*x*xa + xa**2 - 900))**(1/2))/float(h**2 - 2*h*ha + 30*h + ha**2 - 30*ha + x**2 - 2*x*xa + xa**2))
    
    m = (((xa-x)**2+(ha-h)**2)**(0.5))/2
    theta = np.arctan((xa-x)/(ha-h))
    phi = np.arccos(m/15)
    
    t2 = -theta - phi
    t1 = -theta + phi
    return t1, t2

def find_arm( point,angles,m ):

    tpoint_gnd=np.matrix([[1, 0, 0, point[0]],[0, 1, 0, point[1]],[0, 0, 1, point[2]+20],[0, 0, 0, 1]])
    
    arm = np.matrix(np.zeros((4,5)))

    arm[:,0]=tpoint_gnd*np.matrix([[0],[0],[0],[1]])
    if(m==1):
        t1_gnd=tpoint_gnd*np.matrix([[1, 0, 0, 0],[0, 0, -1, -5],[0, 1, 0, 0],[0, 0, 0, 1]])
    else:
        t1_gnd=tpoint_gnd*np.matrix([[1, 0, 0, 0],[0, 0, -1, 5],[0, 1, 0, 0],[0, 0, 0, 1]])
    
    arm[:,1]=t1_gnd*np.matrix([[0],[0],[0],[1]])
    if(m==1):
        t2_gnd=t1_gnd*Rz(angles[0])*np.matrix([[0, 0, -1, 0],[-1, 0, 0, 0],[0, 1, 0, 5],[0, 0, 0, 1]])
    else:
        t2_gnd=t1_gnd*Rz(angles[0])*np.matrix([[0, 0, -1, 0],[-1, 0, 0, 0],[0, 1, 0, -5],[0, 0, 0, 1]])
    
    arm[:,2]=t2_gnd*np.matrix([[0],[0],[0],[1]])
    t3_gnd=t2_gnd*Rz(angles[1])*np.matrix([[1, 0, 0, 10],[0, 0, 1, 0],[0, -1, 0, 0],[0, 0, 0, 1]])
    arm[:,3]=t3_gnd*np.matrix([[0],[0],[0],[1]])
    arm[:,4]=t3_gnd*Rz(angles[2])*np.matrix([[10],[0],[0],[1]])
    
    return arm

def find_leg( point,angles,theta, m ):

    leg = np.matrix(np.zeros((4,14)))

    tpoint_gnd = np.matrix([[1, 0, 0, point[0]],[0, 1, 0, point[1]],[0, 0, 1 ,point[2]],[0, 0, 0, 1]])
    t1_gnd = tpoint_gnd*np.matrix([[0, 1, 0, 0],[-1, 0, 0,-m*5],[0, 0, 1, 0],[0, 0, 0, 1]])
    leg[:,0] = t1_gnd*np.matrix([[0],[0],[0],[1]])
    t2_gnd = t1_gnd*Rz(-theta)*np.matrix([[0, 1, 0, 0],[0, 0, -1, 0],[-1, 0, 0, -1],[0, 0, 0, 1]])
    leg[:,1] = t2_gnd*np.matrix([[0],[0],[0],[1]])
    t3_gnd= t2_gnd*Rz(angles[0])*np.matrix([[1, 0, 0, 0],[0, 0, 1, 0],[0, -1, 0, 0],[0, 0, 0, 1]])*Rz(-angles[2])*np.matrix([[1, 0, 0, 15],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
    leg[:,3] = t3_gnd*np.matrix([[0],[0],[0],[1]])
    t4_gnd = t3_gnd*Rz(angles[2]-angles[1])*np.matrix([[1, 0, 0, 15],[0, 1, 0, 0],[0, 0, 1 ,0],[0, 0, 0, 1]])
    leg[:,4]=t4_gnd*np.matrix([[0],[0],[0],[1]])
    t5_gnd = t4_gnd*Rz(angles[1])*np.matrix([[1, 0, 0, 0],[0, 0, -1, 0],[0, 1, 0, 0],[0, 0, 0, 1]])
    leg[:,5]=leg[:,4]
    leg[0,5]=leg[0,5]+4.5
    leg[:,6]=leg[:,5]
    leg[1,6]=leg[1,6]+2
    leg[:,7]=leg[:,6]
    leg[0,7]=leg[0,7]-9
    leg[:,8]=t5_gnd*np.matrix([[0],[0],[4.5],[1]])
    leg[:,9]=leg[:,8]
    leg[1,9]=leg[1,9]-2
    leg[:,10]=leg[:,9]
    leg[0,10]=leg[0,10]+9
    leg[:,11]=leg[:,10]
    leg[1,11]=leg[1,11]+2
    leg[:,12]=leg[:,5]
    leg[:,13]=leg[:,8]
    leg[:,2]=leg[:,1]
     
    return leg

def plot2_final(rleg,lleg,waist,ra,la, zmp):
    ax.clear()
    
    ax.scatter(np.array(lleg[0,:]).T,np.array(lleg[1,:]).T,np.array(lleg[2,:]).T)
    ax.scatter(np.array(rleg[0,:]).T,np.array(rleg[1,:]).T,np.array(rleg[2,:]).T)
    ax.scatter(np.array(ra[0,:]).T,np.array(ra[1,:]).T,np.array(ra[2,:]).T)
    ax.scatter(np.array(la[0,:]).T,np.array(la[1,:]).T,np.array(la[2,:]).T)
    ax.scatter(np.array(waist[0,:]).T,np.array(waist[1,:]).T,np.array(waist[2,:]).T)

    ax.scatter(np.array(zmp[0]).T,np.array(zmp[1]).T,np.array(zmp[2]).T)
    
    ax.plot_wireframe(np.array(lleg[0,:]).T,np.array(lleg[1,:]).T,np.array(lleg[2,:]).T)
    ax.plot_wireframe(np.array(rleg[0,:]).T,np.array(rleg[1,:]).T,np.array(rleg[2,:]).T)
    ax.plot_wireframe(np.array(ra[0,:]).T,np.array(ra[1,:]).T,np.array(ra[2,:]).T)
    ax.plot_wireframe(np.array(la[0,:]).T,np.array(la[1,:]).T,np.array(la[2,:]).T)
    ax.plot_wireframe(np.array(waist[0,:]).T,np.array(waist[1,:]).T,np.array(waist[2,:]).T)

    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_zlim(0, 40)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    plt.pause(0.005)
   
    return

def get_angles(waist, rleg, lleg):
    msg = Servo_Msg()

    r_foot_28 = temp_left_joint_angles[2,0]
    l_foot_28 = -temp_right_joint_angles[2,0]
    r_foot_64 = temp_right_joint_angles[0,0]
    l_foot_64 = temp_left_joint_angles[0,0]
    r_hip_64 = -temp_left_joint_angles[1,0]
    l_hip_64 = temp_right_joint_angles[1,0]
    r_hip_28 = temp_right_joint_angles[0,0]
    l_hip_28 = temp_left_joint_angles[0,0]
    l_knee   = -temp_right_joint_angles[2,0] + temp_right_joint_angles[1,0]
    r_knee   = -temp_left_joint_angles[2,0] + temp_left_joint_angles[1,0]

    msg.id1  = int(l_knee*4095/(2*np.pi))
    msg.id2  = int(l_hip_64*4095/(2*np.pi))
    msg.id9  = int(r_foot_64*4095/(2*np.pi))
    msg.id11 = int(l_hip_28*4095/(2*np.pi))
    msg.id12 = int(r_foot_28*4095/(2*np.pi))
    msg.id13 = int(r_knee*4095/(2*np.pi))
    msg.id14 = int(l_foot_28*4095/(2*np.pi))
    msg.id15 = int(r_hip_28*4095/(2*np.pi))
    msg.id16 = int(l_foot_64*4095/(2*np.pi))
    msg.id17 = int(r_hip_64*4095/(2*np.pi))

    return msg

def publish(msg):
    pub = rospy.Publisher('sim_angles', Servo_Msg, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()

def simulate():

    msg = Servo_Msg()

    for t in range(11):       #%start
        if (t>-1) and (t<2):
            centre_pelvis[0,0]=3
            centre_pelvis[1,0]= -5*t+5
            xa=3
            height_ankle = 0
            centre_pelvis[2,0]= h*np.cos(np.arctan((centre_pelvis[1,0]-5)/h))+1
            right_hand_angles[0,0]= -3*np.pi/180
            right_hand_angles[2,0]= 16*np.pi/180
            left_hand_angles[0,0]= -3*np.pi/180
            left_hand_angles[2,0]= 16*np.pi/180
        elif (t>1) and (t<4):
            centre_pelvis[0,0]=3
            centre_pelvis[1,0]=0
            xa=3
            height_ankle = (t-1)/2*(4.14)
            centre_pelvis[2,0]= h*np.cos(np.arctan((centre_pelvis[1,0]-5)/h))+1+(0.18)*(t-2)
        elif (t>3) and (t<9):
            centre_pelvis[0,0] =  t/4+2
            x=centre_pelvis[0,0]
            centre_pelvis[1,0] = 0
            xa = (3/2)*(t+4) - 9
            height_ankle = -0.005*(xa+20)*(xa+3)*(xa-9)
            centre_pelvis[2,0]=1 - 0.1*(x-4)*(x-2)*(x-1.2) + h*np.cos(np.arctan(5/h))
            right_hand_angles[0,0]= -3*np.pi/180-9*(t-4)/4*np.pi/180
            right_hand_angles[2,0]= 16*np.pi/180-2*(t-4)/4*np.pi/180
            left_hand_angles[0,0]= -3*np.pi/180+9*(t-4)/4*np.pi/180
            left_hand_angles[2,0]= 16*np.pi/180+2*(t-4)/4*np.pi/180
        else:
            centre_pelvis[0,0]= t-4
            centre_pelvis[1,0]= 0+5*(t-8)/4
            xa=9
            height_ankle=0
            centre_pelvis[2,0]= h*np.cos(np.arctan((centre_pelvis[1,0]-5)/h))+1
        

        temp_left_joint_angles[0,0]=np.arctan((centre_pelvis[1,0]-5)/h)
        [temp_left_joint_angles[1,0],temp_left_joint_angles[2,0]]=find_angles(centre_pelvis[0,0],3,centre_pelvis[2,0],0)
        temp_right_joint_angles[0,0]=temp_left_joint_angles[0,0]
        [temp_right_joint_angles[1,0],temp_right_joint_angles[2,0]]=find_angles(centre_pelvis[0,0],xa,centre_pelvis[2,0],height_ankle)

        left_leg=find_leg(centre_pelvis,temp_left_joint_angles,theta, 1)
        right_leg=find_leg(centre_pelvis,temp_right_joint_angles,theta, -1)
        waist[:,2]=left_leg[:,0]
        waist[:,3]=right_leg[:,0]
        waist[:,1]=np.matrix([[centre_pelvis[0,0]],[centre_pelvis[1,0]],[centre_pelvis[2,0]],[1]])
        waist[:,0]=waist[:,1]
        waist[2,0]= 25 + waist[2,0]
        right_arm=find_arm(centre_pelvis,right_hand_angles,1)
        left_arm=find_arm(centre_pelvis,left_hand_angles,0)
        
        zmp[:,0] = waist[:,1]
        zmp[2,0] = 0

        msg = get_angles(waist, right_leg, left_leg)
        publish(msg)
        
        plot2_final(left_leg,right_leg,waist,right_arm,left_arm, zmp)

        time.sleep(pause)

    
    for m in range(no_of_steps):
        for t in range(9):    #%left leg as swing one
            if (t>-1) and (t<2):     #%dsp1
                centre_pelvis[0,0] = 2*t
                centre_pelvis[1,0] = 5*(t)+5
                xa = -3
                height_ankle = 0
                centre_pelvis[2,0] =h*np.cos(np.arctan((centre_pelvis[1,0]-5)/h))+1
            elif (t>1) and (t<6):   #%ssp
                centre_pelvis[0,0] = 1 + t/2
                x=centre_pelvis[0,0]
                centre_pelvis[1,0] = 10
                xa = (3)*t - 9
                height_ankle = -0.005*(xa+20)*(xa+3)*(xa-9)
                centre_pelvis[2,0]=1 - 0.1*(x-4)*(x-2)*(x-1.2) + h*np.cos(np.arctan((centre_pelvis[1,0]-5)/h))
                right_hand_angles[0,0]= -12*np.pi/180+18*(t-2)/4*np.pi/180
                right_hand_angles[2,0]= 14*np.pi/180+4*(t-2)/4*np.pi/180
                left_hand_angles[0,0]= 6*np.pi/180-18*(t-2)/4*np.pi/180
                left_hand_angles[2,0]= 18*np.pi/180-4*(t-2)/4*np.pi/180
            else:                       #%dsp2
                centre_pelvis[0,0] = t -2
                centre_pelvis[1,0] = 10-5*(t-6)/2
                xa = 9
                height_ankle = 0
                centre_pelvis[2,0]=1+h*np.cos(np.arctan((centre_pelvis[1,0]-5)/h))
            
            temp_left_joint_angles[0,0]=np.arctan((centre_pelvis[1,0]-5)/h)
            [temp_left_joint_angles[1,0],temp_left_joint_angles[2,0]]=find_angles(centre_pelvis[0,0],xa,centre_pelvis[2,0],height_ankle)
            temp_right_joint_angles[0,0]=temp_left_joint_angles[0,0]
            [temp_right_joint_angles[1,0],temp_right_joint_angles[2,0]]=find_angles(centre_pelvis[0,0],3,centre_pelvis[2,0],0)
            
            left_leg=find_leg(centre_pelvis,temp_left_joint_angles,theta, 1)
            right_leg=find_leg(centre_pelvis,temp_right_joint_angles,theta, -1)
            waist[:,2]=left_leg[:,0]
            waist[:,3]=right_leg[:,0]
            waist[:,1]=np.matrix([[centre_pelvis[0,0]],[centre_pelvis[1,0]], [centre_pelvis[2,0]],[1]])
            waist[:,0]=waist[:,1]
            waist[2,0]= 25 + waist[2,0]
            right_arm=find_arm(centre_pelvis,right_hand_angles,1)
            left_arm=find_arm(centre_pelvis,left_hand_angles,0)
            
            zmp[:,0] = waist[:,1]
            zmp[2,0] = 0
            
            msg = get_angles(waist, right_leg, left_leg)
            publish(msg)

            plot2_final(left_leg,right_leg,waist,right_arm,left_arm, zmp)
            
            time.sleep(pause)
        
        
        
        for t in range(9):                     #%swing leg as right leg
            if (t>-1) and (t<2):        
                centre_pelvis[0,0] = t*2
                centre_pelvis[1,0] = -5*(t)+5
                xa = -3
                height_ankle = 0
                centre_pelvis[2,0] =h*np.cos(np.arctan((centre_pelvis[1,0]-5)/h))+1
            elif (t>1) and (t<6):
                centre_pelvis[0,0] = 1 + t/2
                x=centre_pelvis[0,0]
                centre_pelvis[1,0] = 0
                xa = 3*t - 9
                height_ankle = -0.005*(xa+20)*(xa+3)*(xa-9)
                centre_pelvis[2,0]= -0.1*(x-4)*(x-2)*(x-1.2) + h*np.cos(np.arctan((centre_pelvis[1,0]-5)/h))+1
                right_hand_angles[0,0]= 6*np.pi/180-18*(t-2)/4*np.pi/180
                right_hand_angles[2,0]= 18*np.pi/180-4*(t-2)/4*np.pi/180
                left_hand_angles[0,0]= -12*np.pi/180+18*(t-2)/4*np.pi/180
                left_hand_angles[2,0]= 14*np.pi/180+4*(t-2)/4*np.pi/180
            else:
                centre_pelvis[0,0] = t-2
                centre_pelvis[1,0] = 0+5*(t-6)/2
                xa = 9
                height_ankle = 0
                centre_pelvis[2,0]=h*np.cos(np.arctan((centre_pelvis[1,0]-5)/h))+1
            
            temp_left_joint_angles[0,0]=np.arctan((centre_pelvis[1,0]-5)/h)
            [temp_left_joint_angles[1,0],temp_left_joint_angles[2,0]]=find_angles(centre_pelvis[0,0],3,centre_pelvis[2,0],0)
            temp_right_joint_angles[0,0]=temp_left_joint_angles[0,0]
            [temp_right_joint_angles[1,0],temp_right_joint_angles[2,0]]=find_angles(centre_pelvis[0,0],xa,centre_pelvis[2,0],height_ankle)
            
            left_leg=find_leg(centre_pelvis,temp_left_joint_angles,theta, 1)
            right_leg=find_leg(centre_pelvis,temp_right_joint_angles,theta, -1)
            waist[:,2]=left_leg[:,0]
            waist[:,3]=right_leg[:,0]
            waist[:,1]=np.matrix([[centre_pelvis[0,0]],[centre_pelvis[1,0]],[centre_pelvis[2,0]],[1]])
            waist[:,0]=waist[:,1]
            waist[2,0]= 25 + waist[2,0]
            right_arm=find_arm(centre_pelvis,right_hand_angles,1)
            left_arm=find_arm(centre_pelvis,left_hand_angles,0)
            
            zmp[:,0] = waist[:,1]
            zmp[2,0] = 0
            
            msg = get_angles(waist, right_leg, left_leg)
            publish(msg)
            
            plot2_final(left_leg,right_leg,waist,right_arm,left_arm, zmp)            
            
            time.sleep(pause)

if __name__ == '__main__':
    try:
        simulate()
    except rospy.ROSInterruptException:
        pass
