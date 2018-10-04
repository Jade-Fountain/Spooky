# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import numpy as np
import matplotlib.pyplot as plt

# Analyse the accuracy of fused tracking vs individuals
def colourMap(i):
    #colour brewer http://colorbrewer2.org/#type=qualitative&scheme=Set2&n=3
    return {
        0 : '#66c2a5',
        1 : '#8da0cb',
        2 : '#fc8d62'
    }[int(i)]

def getDataFromFile(file,names,converters=[]):
    return np.array(genfromtxt(file,
                      delimiter=" ", 
                      comments="#"))
# x = forward
# y = left
# z = up
def scatterUE4Positions(ax,data, colormap="spring",m='x',label=""):
    t = np.arange(len(data))
    ax.plot(data[:,0],-data[:,1],data[:,2],label=label)#,c=t,cmap=colormap)


def plotPairedErrors(title,lefthand,righthand,leftref,rightref):
    l = np.min([len(lefthand),len(righthand),len(leftref),len(rightref)])
    error_l = np.linalg.norm(leftref[0:l]-lefthand[0:l],axis=1)
    error_r = np.linalg.norm(rightref[0:l]-righthand[0:l],axis=1)

    plt.plot(error_r,label=title+" L")
    plt.plot(error_l,label=title+" R")
    plt.legend()

    plt.ylabel("Error (cm)")
    plt.xlabel("Frame")
    # plt.title(title)

def plotErrors(title,labels,data_streams,ref, first_frame=0):
    # print(data_streams.shape)
    print("Error analysis ("+title+")")
    i = 0
    for labl,data in zip(labels,data_streams):
        l = np.min([len(data),len(ref)])
        errors = np.linalg.norm(data[first_frame:l]-ref[first_frame:l],axis=1)
        plt.plot(errors,label=labl,c=colourMap(i))
        print("Mean error ("+labl+") = "+ str(np.mean(errors)));
        i=(i+1)%3
    plt.legend()

    plt.ylabel("Error (cm)")
    plt.xlabel("Frame")
    plt.title(title)

def positionalHeadRelativeErrorAnalysis(folder):
    leap_log_l = np.genfromtxt(folder+"/LeapPosition_hand_l.csv")
    leap_log_r = np.genfromtxt(folder+"/LeapPosition_hand_r.csv")


    PN_log_l = np.genfromtxt(folder+"/PN_LeftHand.csv")
    PN_log_r = np.genfromtxt(folder+"/PN_RightHand.csv")

    Fused_log_l = np.genfromtxt(folder+"/Fused_hand_l.csv")
    Fused_log_r = np.genfromtxt(folder+"/Fused_hand_r.csv")

    # optitrack_rot = np.array(
    #                 [[1,0,0],
    #                  [0,1,0],
    #                  [0,0,1]])
    optitrack_rot = np.array(
                    [[0,1,0],
                     [-1,0,0],
                     [0,0,1]])
    optitrack_log_r = np.genfromtxt(folder+"/Optitrack_hand_r.csv")
    opdata_r = np.transpose(np.dot(optitrack_rot,np.transpose(optitrack_log_r)))
    optitrack_log_l = np.genfromtxt(folder+"/Optitrack_hand_l.csv")
    opdata_l = np.transpose(np.dot(optitrack_rot,np.transpose(optitrack_log_l)))


    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.plot([0,10],[0,0],[0,0],c='r')
    ax.plot([0,0],[0,10],[0,0],c='g')
    ax.plot([0,0],[0,0],[0,10],c='b')
    ax.plot([0],[0],[0],marker="o",c='k')


    ax.set_xlim3d(-100,100)
    ax.set_ylim3d(-100,100)
    ax.set_zlim3d(-100,100)


    print("Leap shape = ",leap_log_l.shape,leap_log_r.shape)
    scatterUE4Positions(ax,leap_log_l,colormap="Purples",label="leap L")
    scatterUE4Positions(ax,leap_log_r,colormap="Oranges",label="leap R")

    print("PN shape = ",PN_log_l.shape,PN_log_r.shape)
    scatterUE4Positions(ax,PN_log_l,colormap="Purples",label="PN L")
    scatterUE4Positions(ax,PN_log_r,colormap="Oranges",label="PN R")


    print("Fused shape = ",Fused_log_l.shape,Fused_log_r.shape)
    scatterUE4Positions(ax,Fused_log_l,colormap="Purples",label="Fused L")
    scatterUE4Positions(ax,Fused_log_r,colormap="Oranges",label="Fused R")


    print("Opti shape = ",opdata_l.shape,opdata_r.shape)
    scatterUE4Positions(ax,opdata_l,colormap="Blues",label="Ref L")    
    scatterUE4Positions(ax,opdata_r,colormap="Reds",label="Ref R")

    plt.legend()

    plt.figure()
    plotErrors("Left Hand",["LP","PN","FT"],[leap_log_l,PN_log_l,Fused_log_l],opdata_l,first_frame=100)
    plt.figure()
    plotErrors("Right Hand",["LP","PN","FT"],[leap_log_r,PN_log_r,Fused_log_r],opdata_r,first_frame=100)


positionalHeadRelativeErrorAnalysis("test1")
positionalHeadRelativeErrorAnalysis("test2")
positionalHeadRelativeErrorAnalysis("balltest")
plt.show()