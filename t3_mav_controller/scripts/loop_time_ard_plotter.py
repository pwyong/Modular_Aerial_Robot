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
np_loop_times=None

for topic,msg,t in bag.read_messages():
    if topic=="/loop":
        np_loop_time=np.array([[0.0, 0.0, 0.0]])
        np_loop_time[0,0]=msg.data
        np_loop_time[0,1]=t.secs
        np_loop_time[0,2]=t.nsecs
        if np_loop_times is None:
            np_loop_times=np_loop_time
        else:
            np_loop_times=np.append(np_loop_times,np_loop_time,axis=0)

start_sec=np_loop_times[0,1]
start_nsec=np_loop_times[0,2]
t=np.zeros(np_loop_times.shape[0])
for i in range(np_loop_times.shape[0]):
    t[i]=(np_loop_times[i,1]-start_sec)+(np_loop_times[i,2]-start_nsec)/1000000000.0

ax1=plt.subplot(1,1,1)
plt.plot(t,np_loop_times[:,0],".",linewidth=0.5)
# plt.ylim([0,0.01])
plt.ylabel('loop_time(us)')
plt.xticks(visible=False)
plt.title("Arduino loop time")
plt.grid(True)

plt.tight_layout()
plt.savefig('{0}-{1}.png'.format(filename,'ard_loop'),dpi=200)
plt.show()


bag.close()
