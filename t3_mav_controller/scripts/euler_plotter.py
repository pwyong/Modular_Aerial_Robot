#!/usr/bin/python

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
np_angles=None
np_angle_ds=None
angle_cnt=0
desired_angle_cnt=0

for topic,msg,t in bag.read_messages():
    if topic=="/angle":
        np_angle=np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        np_angle[0,0]=msg.x
        np_angle[0,1]=msg.y
        np_angle[0,2]=msg.z
        np_angle[0,3]=t.secs
        np_angle[0,4]=t.nsecs
        if np_angles is None:
            np_angles=np_angle
        else:
            np_angles=np.append(np_angles,np_angle,axis=0)
    elif topic=="/desired_angle":
        np_angle_d=np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        np_angle_d[0,0]=msg.x
        np_angle_d[0,1]=msg.y
        np_angle_d[0,2]=msg.z
        np_angle_d[0,3]=t.secs
        np_angle_d[0,4]=t.nsecs
        if np_angle_ds is None:
            np_angle_ds=np_angle_d
        else:
            np_angle_ds=np.append(np_angle_ds,np_angle_d,axis=0)
    


start_sec=np_angles[0,3]
start_nsec=np_angles[0,4]
t=np.zeros(np_angles.shape[0],dtype='float32')
for i in range(np_angles.shape[0]):
    t[i]=(np_angles[i,3]-start_sec)+(np_angles[i,4]-start_nsec)/1000000000.0

start_sec_2=np_angle_ds[0,3]
start_nsec_2=np_angle_ds[0,4]
t_2=np.zeros(np_angle_ds.shape[0],dtype='float32')
for i in range(np_angle_ds.shape[0]):
    t_2[i]=(np_angle_ds[i,3]-start_sec)+(np_angle_ds[i,4]-start_nsec)/1000000000.0


ax1=plt.subplot(3,1,1)
plt.plot(t,np_angles[:,0],'r',linewidth=0.5)
plt.plot(t_2,np_angle_ds[:,0],'k',linewidth=0.5)
plt.ylabel('Roll(rad)')
plt.ylim([-0.3,0.3])
plt.xticks(visible=False)
plt.title("Euler angle")
plt.grid(True)

ax2=plt.subplot(3,1,2,sharex=ax1)
plt.plot(t,np_angles[:,1],'g',linewidth=0.5)
plt.plot(t_2,np_angle_ds[:,1],'k',linewidth=0.5)
plt.ylabel('Pitch(rad)')
plt.ylim([-0.3,0.3])
plt.xticks(visible=False)
plt.grid(True)

ax3=plt.subplot(3,1,3,sharex=ax1)
plt.plot(t,np_angles[:,2],'b',linewidth=0.5)
plt.plot(t_2,np_angle_ds[:,2],'k',linewidth=0.5)
plt.xlabel('time(s)')
plt.ylabel('Yaw(rad)')
plt.ylim([-0.3,0.3])
plt.grid(True)

plt.tight_layout()
plt.savefig('{0}-{1}.png'.format(filename,'euler'),dpi=200)
plt.show()


bag.close()
