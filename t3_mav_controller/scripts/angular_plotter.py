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
np_ang_vel=None
np_ang_accel=None


for topic,msg,t in bag.read_messages():
    if topic=="/kalman_ang_vel":
        np_ang_vel_tmp=np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        np_ang_vel_tmp[0,0]=msg.x
        np_ang_vel_tmp[0,1]=msg.y
        np_ang_vel_tmp[0,2]=msg.z
        np_ang_vel_tmp[0,3]=t.secs
        np_ang_vel_tmp[0,4]=t.nsecs
        if np_ang_vel is None:
            np_ang_vel=np_ang_vel_tmp
        else:
            np_ang_vel=np.append(np_ang_vel,np_ang_vel_tmp,axis=0)
    elif topic=="/kalman_ang_accel":
        np_ang_accel_tmp=np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        np_ang_accel_tmp[0,0]=msg.x
        np_ang_accel_tmp[0,1]=msg.y
        np_ang_accel_tmp[0,2]=msg.z
        np_ang_accel_tmp[0,3]=t.secs
        np_ang_accel_tmp[0,4]=t.nsecs
        if np_ang_accel is None:
            np_ang_accel=np_ang_accel_tmp
        else:
            np_ang_accel=np.append(np_ang_accel,np_ang_accel_tmp,axis=0)
    


start_sec=np_ang_vel[0,3]
start_nsec=np_ang_vel[0,4]
t=np.zeros(np_ang_vel.shape[0],dtype='float32')
for i in range(np_ang_vel.shape[0]):
    t[i]=(np_ang_vel[i,3]-start_sec)+(np_ang_vel[i,4]-start_nsec)/1000000000.0

start_sec2=np_ang_accel[0,3]
start_nsec2=np_ang_accel[0,4]
t2=np.zeros(np_ang_accel.shape[0],dtype='float32')
for i in range(np_ang_accel.shape[0]):
    t2[i]=(np_ang_accel[i,3]-start_sec)+(np_ang_accel[i,4]-start_nsec)/1000000000.0



ax1=plt.subplot(3,1,1)
plt.plot(t,np_ang_vel[:,0],'r',linewidth=0.5)
plt.plot(t2,np_ang_accel[:,0],'k',linewidth=0.5)
plt.ylabel('p_dot / p_ddot (rad/s / rad/s^2)')
plt.ylim([-5,5])
plt.xticks(visible=False)
plt.title("position")
plt.grid(True)

ax2=plt.subplot(3,1,2,sharex=ax1)
plt.plot(t,np_ang_vel[:,1],'r',linewidth=0.5)
plt.plot(t2,np_ang_accel[:,1],'k',linewidth=0.5)
plt.ylabel('q_dot / q_ddot (rad/s / rad/s^2)')
plt.ylim([-5,5])
plt.xticks(visible=False)
plt.grid(True)

ax3=plt.subplot(3,1,3,sharex=ax1)
plt.plot(t,np_ang_vel[:,2],'r',linewidth=0.5)
plt.plot(t2,np_ang_accel[:,2],'k',linewidth=0.5)
plt.xlabel('time(s)')
plt.ylabel('r_dot / r_ddot (rad/s / rad/s^2)')
# plt.xlim([t[0],t[np_pos.shape[0]-1]])
plt.ylim([-5,5])
plt.xticks(visible=False)
plt.grid(True)


# plt.tight_layout()
plt.savefig('{0}-{1}.png'.format(filename,'kalman'),dpi=200)
plt.show()


bag.close()
