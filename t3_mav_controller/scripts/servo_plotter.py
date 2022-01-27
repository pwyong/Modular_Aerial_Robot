#!/usr/bin/python

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import sys
import os

args = sys.argv
print(len(args))
assert len(args)>=2, "you must specify the argument"

filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
print(filename)

bag=rosbag.Bag(filename)
np_servo=None
np_servo_d=None


for topic,msg,t in bag.read_messages():
    if topic=="/joint_states":
        np_servo_tmp=np.array([[0.0, 0.0, 0.0, 0.0]])
        np_servo_tmp[0,0]=-msg.position[1]
        np_servo_tmp[0,1]=-msg.position[0]
        np_servo_tmp[0,2]=t.secs
        np_servo_tmp[0,3]=t.nsecs
        if np_servo is None:
            np_servo=np_servo_tmp
        else:
            np_servo=np.append(np_servo,np_servo_tmp,axis=0)
    if topic=="/goal_dynamixel_position":
        np_servo_d_tmp=np.array([[0.0, 0.0, 0.0, 0.0]])
        np_servo_d_tmp[0,0]=-msg.position[1]
        np_servo_d_tmp[0,1]=-msg.position[0]
        np_servo_d_tmp[0,2]=t.secs
        np_servo_d_tmp[0,3]=t.nsecs
        if np_servo_d is None:
            np_servo_d=np_servo_d_tmp
        else:
            np_servo_d=np.append(np_servo_d,np_servo_d_tmp,axis=0)



start_sec=np_servo[0,2]
start_nsec=np_servo[0,3]
t=np.zeros(np_servo.shape[0],dtype='float32')
for i in range(np_servo.shape[0]):
    t[i]=(np_servo[i,2]-start_sec)+(np_servo[i,3]-start_nsec)/1000000000.0

start_sec2=np_servo_d[0,2]
start_nsec2=np_servo_d[0,3]
t2=np.zeros(np_servo_d.shape[0],dtype='float32')
for i in range(np_servo_d.shape[0]):
    t2[i]=(np_servo_d[i,2]-start_sec2)+(np_servo_d[i,3]-start_nsec2)/1000000000.0

ax1=plt.subplot(2,1,1)
plt.plot(t,np_servo[:,0],'r',linewidth=0.5)
plt.plot(t2,np_servo_d[:,0],'k',linewidth=0.5)
plt.ylabel('theta1(rad)')
plt.ylim([-0.4,0.4])
plt.xticks(visible=False)
plt.title("Servo Angle")
plt.grid(True)

ax2=plt.subplot(2,1,2,sharex=ax1)
plt.plot(t,np_servo[:,1],'r',linewidth=0.5)
plt.plot(t2,np_servo_d[:,1],'k',linewidth=0.5)
plt.ylabel('theta2(rad)')
plt.xlabel('time(s)')
plt.ylim([-0.4,0.4])
plt.xticks(visible=False)
plt.grid(True)



# plt.tight_layout()
plt.savefig('{0}-{1}.png'.format(filename,'servo'),dpi=200)
plt.show()


bag.close()
