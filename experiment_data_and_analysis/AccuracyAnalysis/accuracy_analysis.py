# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

from matplotlib2tikz import save as tikz_save
import numpy as np
import matplotlib.pyplot as plt
import copy,os

# Analyse the accuracy of fused tracking vs individuals
def colourMap(i):
    #colour brewer http://colorbrewer2.org/#type=qualitative&scheme=Set2&n=3
    return {
        0 : '#66c2a5',
        1 : '#8da0cb',
        2 : '#fc8d62'
    }[int(i)]

CL = '#1f77b4'
CR = '#ff7f0e'

thesis_folder = "/Users/jake/MEGA/PhD/Documents/Thesis/chapters/user_study/figure/accuracy/"
table_folder = "/Users/jake/MEGA/PhD/Documents/Thesis/chapters/user_study/table/accuracy/"

def saveFigure(name,pgf=True):
    print("Saving ",name)
    if(pgf):
        tikz_save("figure/"+name+".tex")
        if(os.name=='posix'):
            tikz_save(thesis_folder+name+".tex")
    plt.savefig("figure/"+name+".pdf",bbox_inches='tight')
    if(os.name=='posix'):
        plt.savefig(thesis_folder+name+".pdf",bbox_inches='tight')

def getDataFromFile(file,names,converters=[]):
    return np.array(genfromtxt(file,
                      delimiter=" ", 
                      comments="#"))
# x = forward
# y = left
# z = up
def scatterUE4Positions(ax,data, colormap="spring",m='x',label=""):
    t = np.arange(len(data))
    ax.scatter(data[:,0],-data[:,1],data[:,2],label=label,c=t,marker=m,cmap=colormap)


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

def plotErrors(title,labels,data_streams,ref,sections=[[],[],[]]):
    # print(data_streams.shape)
    print("Error analysis ("+title+")")
    i = 0
    for labl,data,sec in zip(labels,data_streams,sections):
        l = np.min([len(data),len(ref)])
        errors = np.linalg.norm(data[0:l]-ref[0:l],axis=1)

        if(len(sec)==0):
            plt.plot(errors,label=labl,c=colourMap(i))
        else:
            frames = range(len(errors))
            for s in sec:
                plt.plot(frames[s[0]:min(s[1],l)],errors[s[0]:min(s[1],l)],label=labl,c=colourMap(i))

            
        print("Mean error ("+labl+") = "+ str(np.mean(errors)));
        i=(i+1)%3
    # plt.legend()

    plt.ylabel("Error (cm)")
    plt.xlabel("Frame")
    plt.title(title)

def plot2DTraces(title,left,right,refLeft=None,refRight=None, secL=None,secR=None):
    plt.figure()
    # plt.xlabel("$x$ (cm)")
    # plt.ylabel("$y$ (cm)")
    fig=plt.gca()
    fig.tick_params(length=0)
    fig.axes.get_xaxis().set_ticklabels([])
    fig.axes.get_yaxis().set_ticklabels([])
    fig.set_aspect('equal')
    plt.grid(True)

    # Head
    plt.plot([0,10],[0,0],c='r')
    plt.plot([0,0],[0,10],c='g')
    plt.plot([0],[0],marker="o",c='k')

    plt.xlim(-75,75)
    plt.ylim(-75,75)

    if(refLeft is not None and refRight is not None):
        plt.plot(refLeft[:,0],-refLeft[:,1],c='grey',alpha=0.2,label="GT Left")
        plt.plot(refRight[:,0],-refRight[:,1],c='grey',alpha=0.2,label="GT Right")

    if(secL is not None):
        for i1,i2 in secL:
            plt.plot(left[i1:i2,0],-left[i1:i2,1],c=CL,label=title+" Left")
    else:
        plt.plot(left[:,0],-left[:,1],c=CL,label=title+" Left")
        

    if(secR is not None):
        for i1,i2 in secR:
            plt.plot(right[i1:i2,0],-right[i1:i2,1],c=CR,label=title+" Right")
    else:
        plt.plot(right[:,0],-right[:,1],c=CR,label=title+" Right")
      

    # plt.legend()

def getValidSections(valid,data):
    validSections=[]
    lastv=False
    section=[0,0]
    for i in range(len(data)):
        v=valid(data[i])
       
        if(v and not lastv):
            section[0]=i
        if(not v and lastv):
            # Not inclusive range, and we dont want this one
            section[1]=i
            print("Adding section", section)
            validSections += [copy.deepcopy(section)]
        lastv=v
    if(lastv):
        # non inclusive range, but we do want the last one
        section[1]=len(data)
        validSections += [copy.deepcopy(section)]
    return validSections


def positionalHeadRelativeErrorAnalysis(folder,ghost_trace=False):

    r = [100,500]

    def leapPointValid(p):
        offpointR = [8.720749, 56.646088, -85.070961]
        offpointL = [8.720749, -56.646088, -85.070961]
        return np.linalg.norm(p-offpointR) > 1 and np.linalg.norm(p-offpointL) > 1

    leap_log_l = np.genfromtxt(folder+"/LeapPosition_hand_l.csv")[r[0]:r[1]]
    leap_log_r = np.genfromtxt(folder+"/LeapPosition_hand_r.csv")[r[0]:r[1]]
    leap_secL = getValidSections(leapPointValid,leap_log_l)
    leap_secR = getValidSections(leapPointValid,leap_log_r)

    # print("Leap left sections:",leap_secL)
    # print("Leap right sections:",leap_secR)

    PN_log_l = np.genfromtxt(folder+"/PN_LeftHand.csv")[r[0]:r[1]]
    PN_log_r = np.genfromtxt(folder+"/PN_RightHand.csv")[r[0]:r[1]]

    Fused_log_l = np.genfromtxt(folder+"/Fused_hand_l.csv")[r[0]:r[1]]
    Fused_log_r = np.genfromtxt(folder+"/Fused_hand_r.csv")[r[0]:r[1]]


    optitrack_rot = np.array(
                    [[0,1,0],
                     [-1,0,0],
                     [0,0,1]])
    optitrack_log_r = np.genfromtxt(folder+"/Optitrack_hand_r.csv")[r[0]:r[1]]
    opdata_r = np.transpose(np.dot(optitrack_rot,np.transpose(optitrack_log_r)))
    optitrack_log_l = np.genfromtxt(folder+"/Optitrack_hand_l.csv")[r[0]:r[1]]
    opdata_l = np.transpose(np.dot(optitrack_rot,np.transpose(optitrack_log_l)))

    # Plot range in frames
    

    # =======================
    # 3D
    # =======================
    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.set_xlabel("x")
    # ax.set_ylabel("y")
    # ax.set_zlabel("z")

    # ax.plot([0,10],[0,0],[0,0],c='r')
    # ax.plot([0,0],[0,10],[0,0],c='g')
    # ax.plot([0,0],[0,0],[0,10],c='b')
    # ax.plot([0],[0],[0],marker="o",c='k')


    # ax.set_xlim3d(-100,100)
    # ax.set_ylim3d(-100,100)
    # ax.set_zlim3d(-100,100)


    # print("Leap shape = ",leap_log_l.shape,leap_log_r.shape)
    # scatterUE4Positions(ax,leap_log_l,colormap="hsv",label="leap L",m="o")
    # scatterUE4Positions(ax,leap_log_r,colormap="hsv",label="leap R",m="o")

    # # print("PN shape = ",PN_log_l.shape,PN_log_r.shape)
    # # scatterUE4Positions(ax,PN_log_l,colormap="hsv",label="PN L",m="o")
    # # scatterUE4Positions(ax,PN_log_r,colormap="hsv",label="PN R",m="o")


    # # print("Fused shape = ",Fused_log_l.shape,Fused_log_r.shape)
    # # scatterUE4Positions(ax,Fused_log_l,colormap="Purples",label="Fused L")
    # # scatterUE4Positions(ax,Fused_log_r,colormap="Oranges",label="Fused R")


    # print("Opti shape = ",opdata_l.shape,opdata_r.shape)
    # # scatterUE4Positions(ax,opdata_l,colormap="hsv",label="Ref L")    
    # # scatterUE4Positions(ax,opdata_r,colormap="hsv",label="Ref R")
    # plt.legend()

    # =======================

    # =======================
    # 2D Traces
    # =======================
    if(ghost_trace):
        plot2DTraces("LP",leap_log_l,leap_log_r,opdata_l,opdata_r,secL=leap_secL,secR=leap_secR)
        saveFigure("LPWristTrace("+folder+")")
        # unfiltered data:
        # plot2DTraces("LP",leap_log_l,leap_log_r,opdata_l,opdata_r)

        plot2DTraces("PN",PN_log_l,PN_log_r,opdata_l,opdata_r)
        saveFigure("PNWristTrace("+folder+")")

        plot2DTraces("FT",Fused_log_l,Fused_log_r,opdata_l,opdata_r)
        saveFigure("FTWristTrace("+folder+")")
    else:
        plot2DTraces("LP",leap_log_l,leap_log_r,secL=leap_secL,secR=leap_secR)
        saveFigure("LPWristTrace("+folder+")")
        # unfiltered data:
        # plot2DTraces("LP",leap_log_l,leap_log_r,opdata_l,opdata_r)

        plot2DTraces("PN",PN_log_l,PN_log_r)
        saveFigure("PNWristTrace("+folder+")")

        plot2DTraces("FT",Fused_log_l,Fused_log_r)
        saveFigure("FTWristTrace("+folder+")")

    plot2DTraces("GT",opdata_l,opdata_r)
    saveFigure("GTWristTrace("+folder+")")

    # =======================


    # =======================
    # Errors
    # =======================

    # plt.figure()
    # plotErrors("Left Hand",["LP","PN","FT"],[leap_log_l,PN_log_l,Fused_log_l],opdata_l)
    plt.figure()
    plotErrors("Left Hand",["LP","PN","FT"],[leap_log_l,PN_log_l,Fused_log_l],opdata_l,sections=[leap_secL,[],[]])
    saveFigure("LeftHandErrors("+folder+")")

    # plt.figure()
    # plotErrors("Right Hand",["LP","PN","FT"],[leap_log_r,PN_log_r,Fused_log_r],opdata_r)
    plt.figure()
    plotErrors("Right Hand",["LP","PN","FT"],[leap_log_r,PN_log_r,Fused_log_r],opdata_r,sections=[leap_secR,[],[]])
    saveFigure("RightHandErrors("+folder+")")



positionalHeadRelativeErrorAnalysis("test1")
# positionalHeadRelativeErrorAnalysis("test2")
# positionalHeadRelativeErrorAnalysis("balltest",ghost_trace=True)
plt.show()