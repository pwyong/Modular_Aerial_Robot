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

for topic,msg,t in bag.read_messages():
    if topic=="/torque_d":
        np_force=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        np_force[0,0]=msg.x
        np_force[0,1]=msg.y
        np_force[0,2]=msg.z
        np_force[0,3]=msg.w
        np_force[0,4]=t.secs
        np_force[0,5]=t.nsecs
        if np_forces is None:
            np_forces=np_force
        else:
            np_forces=np.append(np_forces,np_force,axis=0)

start_sec=np_forces[0,4]
start_nsec=np_forces[0,5]
t=np.zeros(np_forces.shape[0],dtype='float32')
for i in range(np_forces.shape[0]):
    t[i]=(np_forces[i,4]-start_sec)+(np_forces[i,5]-start_nsec)/1000000000.0

ax1=plt.subplot(4,1,1)
plt.plot(t,np_forces[:,0],'r',linewidth=0.5)
plt.ylabel('tau_r_d')
plt.ylim([-2,2])
plt.xticks(visible=False)
plt.title("desired torque / Thrust")
plt.grid(True)

ax2=plt.subplot(4,1,2,sharex=ax1)
plt.plot(t,np_forces[:,1],'g',linewidth=0.5)
plt.ylabel('tau_p_d')
plt.ylim([-2,2])
plt.xticks(visible=False)
plt.grid(True)

ax3=plt.subplot(4,1,3,sharex=ax1)
plt.plot(t,np_forces[:,2],'b',linewidth=0.5)
plt.ylabel('tau_y_d')
plt.ylim([-2,2])
plt.grid(True)

ax4=plt.subplot(4,1,4,sharex=ax1)
plt.plot(t,np_forces[:,3],'k',linewidth=0.5)
plt.ylabel('Thrust_d')
plt.xlabel('time(s)')
plt.ylim([0,30])
plt.grid(True)

plt.tight_layout()
plt.savefig('{0}-{1}.png'.format(filename,'torque'),dpi=200)
plt.show()


bag.close()
