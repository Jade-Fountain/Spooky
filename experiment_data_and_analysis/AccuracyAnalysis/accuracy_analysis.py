# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import numpy as np
import matplotlib.pyplot as plt

# Analyse the accuracy of fused tracking vs individuals

def getDataFromFile(file,names,converters=[]):
    return np.array(genfromtxt(file,
                      delimiter=" ", 
                      comments="#"))


leap_log_l = np.genfromtxt("LeapPosition_hand_l.csv")
leap_log_r = np.genfromtxt("LeapPosition_hand_r.csv")

# optitrack_rot = np.array(
#                 [[1,0,0],
#                  [0,1,0],
#                  [0,0,1]])
optitrack_rot = np.array(
                [[0,1,0],
                 [-1,0,0],
                 [0,0,1]])
optitrack_log_r = np.genfromtxt("Optitrack_hand_r.csv")
opdata_r = np.transpose(np.dot(optitrack_rot,np.transpose(optitrack_log_r)))
optitrack_log_l = np.genfromtxt("Optitrack_hand_l.csv")
opdata_l = np.transpose(np.dot(optitrack_rot,np.transpose(optitrack_log_l)))

# x = forward
# y = left
# z = up

def scatterUE4Positions(ax,data, colormap="spring",m='x'):
    t = np.arange(len(data))
    ax.scatter(data[:,0],-data[:,1],data[:,2],c=t,cmap=colormap)




fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

ax.plot([0,10],[0,0],[0,0],c='r')
ax.plot([0,0],[0,10],[0,0],c='g')
ax.plot([0,0],[0,0],[0,10],c='b')
ax.plot([0],[0],[0],marker="o",c='k')


# ax.set_xlim3d(-100,100)
# ax.set_ylim3d(-100,100)
# ax.set_zlim3d(-100,100)
print("Leap shape = ",leap_log_l.shape,leap_log_r.shape)
scatterUE4Positions(ax,leap_log_l,colormap="Purples")
scatterUE4Positions(ax,leap_log_r,colormap="Oranges")

print("Opti shape = ",opdata_l.shape,opdata_r.shape)
scatterUE4Positions(ax,opdata_l,colormap="Blues")    
scatterUE4Positions(ax,opdata_r,colormap="Reds")

l = np.min([len(opdata_l),len(opdata_r),len(leap_log_r),len(leap_log_l)])
leap_error_l = np.linalg.norm(opdata_l[0:l]-leap_log_l[0:l],axis=1)
leap_error_r = np.linalg.norm(opdata_r[0:l]-leap_log_r[0:l],axis=1)

plt.figure()
plt.plot(leap_error_r)
plt.plot(leap_error_l)

plt.show()