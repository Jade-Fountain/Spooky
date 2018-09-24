# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


import numpy as np
import matplotlib.pyplot as plt

# Analyse the accuracy of fused tracking vs individuals

def getDataFromFile(file,names,converters=[]):
    return np.array(genfromtxt(file,
                      delimiter=" ", 
                      comments="#"))


leap_log_l = np.genfromtxt("LeapPositionLog_hand_l.csv")
leap_log_r = np.genfromtxt("LeapPositionLog_hand_r.csv")

# x = forward
# y = left
# z = up

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(leap_log_l[:,0],-leap_log_l[:,1],leap_log_l[:,2],c='r')
ax.scatter(leap_log_r[:,0],-leap_log_r[:,1],leap_log_r[:,2],c='b')
plt.show()