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
np_pos=None
np_pos_d=None


for topic,msg,t in bag.read_messages():
    if topic=="/t265_pos":
        np_pos_tmp=np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        np_pos_tmp[0,0]=msg.x
        np_pos_tmp[0,1]=msg.y
        np_pos_tmp[0,2]=msg.z
        np_pos_tmp[0,3]=t.secs
        np_pos_tmp[0,4]=t.nsecs
        if np_pos is None:
            np_pos=np_pos_tmp
        else:
            np_pos=np.append(np_pos,np_pos_tmp,axis=0)
    elif topic=="/desired_altitude":
        np_pos_d_tmp=np.array([[0.0, 0.0, 0.0]])
        np_pos_d_tmp[0,0]=msg.data
        np_pos_d_tmp[0,1]=t.secs
        np_pos_d_tmp[0,2]=t.nsecs
        if np_pos_d is None:
            np_pos_d=np_pos_d_tmp
        else:
            np_pos_d=np.append(np_pos_d,np_pos_d_tmp,axis=0)
    


start_sec=np_pos[0,3]
start_nsec=np_pos[0,4]
t=np.zeros(np_pos.shape[0],dtype='float32')
for i in range(np_pos.shape[0]):
    t[i]=(np_pos[i,3]-start_sec)+(np_pos[i,4]-start_nsec)/1000000000.0


# fig = plt.figure()
# ax=fig.add_subplot(111, projection='3d')

# ax.plot(np_pos[:,0],np_pos[:,1],np_pos[:,2])
# ax.set_xlim(-5,5)
# ax.set_ylim(-5,5)
# ax.set_zlim(0,5)
# ax.set_xlabel("X(m)")
# ax.set_ylabel("Y(m)")
# ax.set_zlabel("Z(m)")

ax1=plt.subplot(1,1,1)
plt.plot(t,np_pos[:,2],'r',linewidth=0.5)
plt.plot(t,np_pos_d[np_pos_d.shape[0]-np_pos.shape[0]:,0],'k',linewidth=0.5)
plt.xlabel('time(s)')
plt.ylabel('z(m)')
# plt.xlim([t[0],t[np_pos.shape[0]-1]])
plt.ylim([-1,3])
plt.xticks(visible=False)
plt.title("Altitude")
plt.grid(True)


# plt.tight_layout()
plt.savefig('{0}-{1}.png'.format(filename,'position'),dpi=200)
plt.show()


bag.close()
