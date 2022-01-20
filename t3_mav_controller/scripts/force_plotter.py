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
np_forces=None
np_pwms=None

for topic,msg,t in bag.read_messages():
    if topic=="/Forces":
        np_pwm=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        np_pwm[0,0]=msg.x
        np_pwm[0,1]=msg.y
        np_pwm[0,2]=msg.z
        np_pwm[0,3]=msg.w
        np_pwm[0,4]=t.secs
        np_pwm[0,5]=t.nsecs
        if np_pwms is None:
            np_pwms=np_pwm
        else:
            np_pwms=np.append(np_pwms,np_pwm,axis=0)


start_sec=np_pwms[0,4]
start_nsec=np_pwms[0,5]
t_ros=np.zeros(np_pwms.shape[0],dtype='float32')
for i in range(np_pwms.shape[0]):
    t_ros[i]=(np_pwms[i,4]-start_sec)+(np_pwms[i,5]-start_nsec)/1000000000.0

ax1=plt.subplot(4,1,1)
#plt.plot(t_ard,np_forces[:,0],'r',linewidth=0.5)
plt.plot(t_ros,np_pwms[:,0],'k',linewidth=0.5)
plt.ylabel('F1(N)')
#plt.ylabel('w_x')
plt.ylim([1000,2000])
#plt.ylim([-2,2])
plt.xticks(visible=False)
plt.title("Forces")
#plt.title("/gx5/imu/data/orientation")
plt.grid(True)

ax2=plt.subplot(4,1,2,sharex=ax1)
#plt.plot(t_ard,np_forces[:,1],'g',linewidth=0.5)
plt.plot(t_ros,np_pwms[:,1],'k',linewidth=0.5)
plt.ylabel('F2(N)')
#plt.ylabel('w_y')
plt.ylim([1000,2000])
#plt.ylim([-2,2])
plt.xticks(visible=False)
plt.grid(True)

ax3=plt.subplot(4,1,3,sharex=ax1)
#plt.plot(t_ard,np_forces[:,2],'b',linewidth=0.5)
plt.plot(t_ros,np_pwms[:,2],'k',linewidth=0.5)
plt.ylabel('F3(N)')
#plt.ylabel('w_z')
plt.ylim([1000,2000])
#plt.ylim([-2,2])
plt.grid(True)

ax4=plt.subplot(4,1,4,sharex=ax1)
#plt.plot(t_ard,np_forces[:,3],'m',linewidth=0.5)
plt.plot(t_ros,np_pwms[:,3],'k',linewidth=0.5)
plt.ylabel('F4(N)')
#plt.ylabel('Thrust_d')
plt.xlabel('time(s)')
plt.ylim([1000,2000])
#plt.ylim([0.7,1.2])
plt.grid(True)

plt.tight_layout()
plt.savefig('{0}-{1}.png'.format(filename,'force'),dpi=200)
plt.show()


bag.close()
