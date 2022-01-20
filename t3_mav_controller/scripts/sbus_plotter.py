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
np_sbus=None

for topic,msg,t in bag.read_messages():
    if topic=="/sbus":
        np_sbus_tmp=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        np_sbus_tmp[0,0]=msg.data[0]
        np_sbus_tmp[0,1]=msg.data[1]
        np_sbus_tmp[0,2]=msg.data[2]
        np_sbus_tmp[0,3]=msg.data[3]
        np_sbus_tmp[0,4]=msg.data[4]
        np_sbus_tmp[0,5]=msg.data[5]
        np_sbus_tmp[0,6]=msg.data[6]
        np_sbus_tmp[0,7]=msg.data[7]
        np_sbus_tmp[0,8]=t.secs
        np_sbus_tmp[0,9]=t.nsecs
        if np_sbus is None:
            np_sbus=np_sbus_tmp
        else:
            np_sbus=np.append(np_sbus,np_sbus_tmp,axis=0)

start_sec=np_sbus[0,8]
start_nsec=np_sbus[0,8]
t=np.zeros(np_sbus.shape[0],dtype='float32')
for i in range(np_sbus.shape[0]):
    t[i]=(np_sbus[i,8]-start_sec)+(np_sbus[i,9]-start_nsec)/1000000000.0

ax1=plt.subplot(4,2,1)
plt.plot(t,np_sbus[:,0],'r',linewidth=0.5)
plt.ylabel('CH1')
plt.ylim([0,2047])
plt.xticks(visible=False)
plt.title("sbus")
plt.grid(True)

ax2=plt.subplot(4,2,2)
plt.plot(t,np_sbus[:,1],'g',linewidth=0.5)
plt.ylabel('CH2')
plt.ylim([0,2047])
plt.xticks(visible=False)
plt.grid(True)

ax3=plt.subplot(4,2,3,sharex=ax1)
plt.plot(t,np_sbus[:,2],'b',linewidth=0.5)
plt.ylabel('CH3')
plt.ylim([0,2047])
plt.grid(True)

ax4=plt.subplot(4,2,4,sharex=ax2)
plt.plot(t,np_sbus[:,3],'c',linewidth=0.5)
plt.ylabel('CH4')
plt.ylim([0,2047])
plt.xticks(visible=False)
plt.grid(True)

ax5=plt.subplot(4,2,5,sharex=ax1)
plt.plot(t,np_sbus[:,4],'m',linewidth=0.5)
plt.ylabel('CH5')
plt.ylim([0,2047])
plt.grid(True)

ax6=plt.subplot(4,2,6,sharex=ax2)
plt.plot(t,np_sbus[:,5],'y',linewidth=0.5)
plt.ylabel('CH6')
plt.ylim([0,2047])
plt.xticks(visible=False)
plt.grid(True)

ax7=plt.subplot(4,2,7,sharex=ax1)
plt.plot(t,np_sbus[:,6],'k',linewidth=0.5)
plt.ylabel('CH7')
plt.xlabel('time(s)')
plt.ylim([0,2047])
plt.grid(True)

ax8=plt.subplot(4,2,8,sharex=ax2)
plt.plot(t,np_sbus[:,7],'r',linewidth=0.5)
plt.ylabel('CH8')
plt.xlabel('time(s)')
plt.ylim([0,2047])
plt.xticks(visible=False)
plt.grid(True)

plt.tight_layout()
plt.savefig('{0}-{1}.png'.format(filename,'sbus'),dpi=200)
plt.show()


bag.close()
