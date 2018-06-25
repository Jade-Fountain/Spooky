
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
import numpy as np
import math
import scipy.stats
from numpy import genfromtxt
import os
import csv
from matplotlib2tikz import save as tikz_save
from collections import Counter
from textblob import TextBlob
from wordcloud import WordCloud
import colorcet as cc
# from sets import Sets
#Regex
import re

plt.rc('font', family='serif')

button_task_file = "ButtonBoard.csv"
sorting_task_file = "SortingTask.csv"
throw_task_file = "ThrowingTask.csv"

thesis_folder = "/Users/jake/MEGA/PhD/Documents/Thesis/chapters/user_study/figure/"
table_folder = "/Users/jake/MEGA/PhD/Documents/Thesis/chapters/user_study/table/"

def shorten_float(x):
    try:
        result = round(x,3)
        return result
    except TypeError:
        return x

def list_transpose(l):
    return list(map(list, zip(*l)))

def saveFigure(name,pgf=True):
    print("Saving ",name)
    if(pgf):
        tikz_save("figure/"+name+".tex")
        if(os.name=='posix'):
            tikz_save(thesis_folder+name+".tex")
    plt.savefig("figure/"+name+".pdf")
    if(os.name=='posix'):
        plt.savefig(thesis_folder+name+".pdf")

def saveTable(name,data,header=[],delim=',',shorten_floats=False):
    if(len(header) > 0 and len(data[0]) != len(header)):
        raise ValueError("Header doesnt match data")
    print(data)
    fname = name
    file = open(fname,'w')
    writer = csv.writer(file,delimiter=delim,quoting=csv.QUOTE_NONE)
    writer.writerow(header)
    for row in data:
        try:
            if(shorten_floats):
                shortened = [shorten_float(x) for x in row]
                print(shortened)
                writer.writerow(shortened)
            else:
                writer.writerow(row.tolist())
        except AttributeError:
            #If already a list
            if(shorten_floats):
                writer.writerow([shorten_float(x) for x in row])
                print(['{0:.2f}'.format(x) for x in row])
            else:
                writer.writerow(row)

def installTexTable(name,data):
    filename_local = "data/"+name+".tex"
    saveTable(filename_local,data,delim="&",shorten_floats=True)
    # if(os.name=='posix'):
        # thesis_filename = table_folder+name+".tex"
        # saveTable(thesis_filename,data,delim="&",shorten_floats=True)

def boolFromString(s):
    if(s.lower() == "true" or s.lower() == b"true"):
        return 1
    elif(s.lower() == "false" or s.lower() == b"false"):
        return 0
    else:
        raise ValueError('Failed to read boolean value')

def techFromString(s):
    #LP = 0 = Leap Motion
    #PN = 1 = Perception Neuron
    #FT = 2 = Fused Tracking
    if(s.upper() == "LP" or s.upper() == b"LP"):
        return 0
    elif(s.upper() == "PN" or s.upper() == b"PN"):
        return 1
    elif(s.upper() == "FT" or s.upper() == b"FT"):
        return 2
    else:
        raise ValueError("String " + s + " doesnt correspond to a tracking technology")

def stringFromTechID(i):
    #LP = 0 = Leap Motion
    #PN = 1 = Perception Neuron
    #FT = 2 = Fused Tracking
    if(i==0):
        return "LP"
    elif(i==1):
        return "PN"
    elif(i==2):
        return "FT"
    else:
        raise ValueError("ID " + i + " doesnt correspond to a tracking technology")

def taskIDFromString(s):
    #LP = 0 = Leap Motion
    #PN = 1 = Perception Neuron
    #FT = 2 = Fused Tracking
    if(s == "keyboard"):
        return 0
    elif(s == "sorting"):
        return 1
    elif(s == "throwing"):
        return 2
    else:
        raise ValueError("String " + s + " doesnt correspond to a task")

def stringFromTaskID(i):
    #LP = 0 = Leap Motion
    #PN = 1 = Perception Neuron
    #FT = 2 = Fused Tracking
    if(i==0):
        return "keyboard"
    elif(i==1):
        return "sorting"
    elif(i==2):
        return "throwing"
    else:
        raise ValueError("ID " + i + " doesnt correspond to a tracking technology")

def colourFromString(s):
    if(s.lower() == "red" or s.lower() == b"red"):
        return 0
    elif(s.lower() == "green" or s.lower() == b"green"):
        return 1
    elif(s.lower() == "blue" or s.lower() == b"blue"):
        return 2
    else:
        raise ValueError("String " + s + " doesnt correspond to a colour")

def split(X, column):
    splitX = {X[0][column] : np.array([X[0]])}
    order_count = 1
    orders = {X[0][column] : order_count}
    for i in range(1,len(X)):
        val_i = X[i][column]
        if(not (val_i in splitX.keys())):
            splitX[val_i] = np.array([X[i]])
            order_count = order_count+1
            orders[val_i] = order_count
        else:
            splitX[val_i] = np.append(splitX[val_i], [X[i]])
    return splitX, orders

def colourMap(i):
    #colour brewer http://colorbrewer2.org/#type=qualitative&scheme=Set2&n=3
    return {
        0 : '#66c2a5',
        1 : '#8da0cb',
        2 : '#fc8d62'
    }[int(i)]

def markerMap(i):
    return {
        0 : 's',
        1 : 'o',
        2 : '*'
    }[int(i)]

def getDataFromFile(file,names,converters=[]):
    return np.array(genfromtxt(file,
                      delimiter=" ", 
                      comments="#", 
                      names=names,
                      converters=converters))

def getRawParticipantData(folder,task_file):
    if(task_file == button_task_file):
        return getDataFromFile(folder+"/"+button_task_file,
                      names=["Tech", "Time", "Correct","CorrectPosX", "CorrectPosY", "CorrectPosZ", "TouchPosX", "TouchPosY", "TouchPosZ", "ButtonSize", "ResponseTime"],
                      converters={"Tech":techFromString,"Correct": boolFromString})
    elif(task_file == sorting_task_file):
        return getDataFromFile(folder+"/"+sorting_task_file,
                      names=["Tech", "Time", "Correct", "Floor", "CubeNumber", "CubeColour", "ResponseTime", "NHands", "Grasps"],
                      converters={"Tech":techFromString, "Correct": boolFromString, "Floor": boolFromString, "CubeColour": colourFromString})
    elif(task_file == throw_task_file):
        return getDataFromFile(folder+"/"+throw_task_file,
                      names=["Tech", "Time", "Success", "HitPosX", "HitPosY", "HitPosZ", "ResponseTime", "GraspCount", "ThrowVelX", "ThrowVelY", "ThrowVelZ"],
                      converters={"Tech":techFromString, "Success": boolFromString})
    else:
        raise ValueError('task not found')

def sumStat(data):
    return np.median(data)


#Specific analysis functions for each task
def getParticipantDataButton(folder):
    data = getRawParticipantData(folder,button_task_file)
    
    splitData, splitOrders = split(data,0)
    # print("splitOrders", splitOrders)
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    mistakes = np.array([0.0,0.0,0.0])
    orders = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Correct'].sum()
        success = splitData[i]['Correct'] == 1
        responseTimes[int(i)] = sumStat(splitData[i]['ResponseTime'][success])

        deltaX = splitData[i]['CorrectPosX'] - splitData[i]["TouchPosX"]
        deltaY = splitData[i]['CorrectPosY'] - splitData[i]["TouchPosY"]
        deltaZ = splitData[i]['CorrectPosZ'] - splitData[i]["TouchPosZ"]
        errors = np.sqrt(deltaX**2+deltaY**2+deltaZ**2)
        mistakes[int(i)] = np.logical_not(success).sum()
        orders[int(i)] = splitOrders[i]
    return scores,mistakes,responseTimes,orders

def getParticipantDataSort(folder):
    data = getRawParticipantData(folder,sorting_task_file)
    splitData, splitOrders = split(data,0)
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    mistakes = np.array([0.0,0.0,0.0])
    orders = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Correct'].sum()
        success = splitData[i]['Floor'] == 0 #aka false
        responseTimes[int(i)] = sumStat(splitData[i]['ResponseTime'][success])
        mistakes[int(i)] = splitData[i]['Floor'].sum()
        orders[int(i)] = splitOrders[i]
    return scores,mistakes, responseTimes,orders

FIELD = {
    "TARGET" : {
        "CENTRE_R" : 5,
        "RED_R" : 28,
        "BLUE_R" : 80,
        "OUTER_R" : 130
    },
    "PLAYER":{
        "SIZE" : [30,30],
        "POS" : [-265,-15]
    },
    "BALL":{
        "RADIUS" : 5,
        "POS" : [-220,-50],
        "STAND" : {
            "SIZE" : [40,40],
            "POS" : [-240,-70]
        }
    },
    "VALID_RANGE" : [[-300,150],[-150,150]]
}

def getParticipantDataThrow(folder):
    data = getRawParticipantData(folder,throw_task_file)

    splitData, splitOrders = split(data,0)
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    drops = np.array([0.0,0.0,0.0])
    orders = np.array([0.0,0.0,0.0])
    error_distances = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Success'].sum()


        deltaX = splitData[i]['HitPosX']
        deltaY = splitData[i]['HitPosY']
        errors = np.sqrt(deltaX**2+deltaY**2) 

        xtest = np.logical_and(deltaX >= FIELD["VALID_RANGE"][0][0], deltaX <= FIELD["VALID_RANGE"][0][1])
        ytest = np.logical_and(deltaY >= FIELD["VALID_RANGE"][1][0], deltaY <= FIELD["VALID_RANGE"][1][1])
        test = np.logical_and(xtest, ytest)

        #Filter large errors (corresponding to ball disappearing)
        valid_errors = errors[test]
        success = errors < FIELD["TARGET"]["OUTER_R"]
        error_distances[int(i)] = sumStat(valid_errors)
        drops[int(i)] = np.logical_not(success).sum()
        #Only count response times which are successful
        orders[int(i)] = splitOrders[i]
        if(np.any(success)):
            responseTimes[int(i)] = sumStat(splitData[i]['ResponseTime'][success])
        else:
            responseTimes[int(i)] = sumStat(splitData[i]['ResponseTime'])
    return scores, drops, responseTimes,orders,error_distances


def drawThrowingBG(ax):
    ax.set_aspect(1)
    outer = patches.Circle([0,0], radius=FIELD["TARGET"]["OUTER_R"], color='w',linewidth=1,linestyle='solid',ec='w',fill=False)
    outmiddle = patches.Circle([0,0], radius=FIELD["TARGET"]["BLUE_R"], color='b',fill=False)
    middle = patches.Circle([0,0], radius=FIELD["TARGET"]["RED_R"], color='r',fill=False)
    inner = patches.Circle([0,0], radius=FIELD["TARGET"]["CENTRE_R"], color='w',fill=False)
    # player = patches.Rectangle(FIELD["PLAYER"]["POS"], width=FIELD["PLAYER"]["SIZE"][0], height=FIELD["PLAYER"]["SIZE"][1], color='g',fill=False)
    player_1 = mlines.Line2D([FIELD["PLAYER"]["POS"][0],FIELD["PLAYER"]["POS"][0]+FIELD["PLAYER"]["SIZE"][0]], [FIELD["PLAYER"]["POS"][1],FIELD["PLAYER"]["POS"][1]+FIELD["PLAYER"]["SIZE"][1]],color='green')
    player_2 = mlines.Line2D([FIELD["PLAYER"]["POS"][0],FIELD["PLAYER"]["POS"][0]+FIELD["PLAYER"]["SIZE"][0]], [FIELD["PLAYER"]["POS"][1]+FIELD["PLAYER"]["SIZE"][1],FIELD["PLAYER"]["POS"][1]],color='green')
    ax.add_line(player_1)
    ax.add_line(player_2)

    standsize = 40.0
    ball_stand = patches.Rectangle(FIELD["BALL"]["POS"]-np.array(FIELD["BALL"]["STAND"]["SIZE"])/2, width=FIELD["BALL"]["STAND"]["SIZE"][0], height=FIELD["BALL"]["STAND"]["SIZE"][1], color='w',linestyle='solid',ec='w',linewidth=1,fill=False)
    ball = patches.Circle(FIELD["BALL"]["POS"], radius=FIELD["BALL"]["RADIUS"], color='w',linestyle='solid',ec='w',linewidth=1)
    ax.add_patch(outer)
    ax.add_patch(outmiddle)
    ax.add_patch(middle)
    ax.add_patch(inner)
    # ax.add_patch(player)
    ax.add_patch(ball_stand)
    ax.add_patch(ball)

def drawThrowingBGSolid(ax):
    ax.set_aspect(1)
    outer = patches.Circle([0,0], radius=FIELD["TARGET"]["OUTER_R"], color='w',linewidth=1,linestyle='solid',ec='k')
    outmiddle = patches.Circle([0,0], radius=FIELD["TARGET"]["BLUE_R"], color='b')
    middle = patches.Circle([0,0], radius=FIELD["TARGET"]["RED_R"], color='r')
    inner = patches.Circle([0,0], radius=FIELD["TARGET"]["CENTRE_R"], color='k')
    # player = patches.Rectangle(FIELD["PLAYER"]["POS"], width=FIELD["PLAYER"]["SIZE"][0], height=FIELD["PLAYER"]["SIZE"][1], color='k')
    player_1 = mlines.Line2D([FIELD["PLAYER"]["POS"][0],FIELD["PLAYER"]["POS"][0]+FIELD["PLAYER"]["SIZE"][0]], [FIELD["PLAYER"]["POS"][1],FIELD["PLAYER"]["POS"][1]+FIELD["PLAYER"]["SIZE"][1]],color='green')
    player_2 = mlines.Line2D([FIELD["PLAYER"]["POS"][0],FIELD["PLAYER"]["POS"][0]+FIELD["PLAYER"]["SIZE"][0]], [FIELD["PLAYER"]["POS"][1]+FIELD["PLAYER"]["SIZE"][1],FIELD["PLAYER"]["POS"][1]],color='green')
    ax.add_line(player_1)
    ax.add_line(player_2)

    standsize = 40.0
    ball_stand = patches.Rectangle(FIELD["BALL"]["POS"]-np.array(FIELD["BALL"]["STAND"]["SIZE"])/2, width=FIELD["BALL"]["STAND"]["SIZE"][0], height=FIELD["BALL"]["STAND"]["SIZE"][1], color='w',linestyle='solid',ec='k',linewidth=1)
    ball = patches.Circle(FIELD["BALL"]["POS"], radius=FIELD["BALL"]["RADIUS"], color='w',linestyle='solid',ec='k',linewidth=1)
    ax.add_patch(outer)
    ax.add_patch(outmiddle)
    ax.add_patch(middle)
    ax.add_patch(inner)
    # ax.add_patch(player)
    ax.add_patch(ball_stand)
    ax.add_patch(ball)

def plotThrowingHeatmaps(folders,saveNames=[]):
    data = np.array([])
    for folder in folders:
        if len(data) == 0:
            data = getRawParticipantData(folder,throw_task_file)
        else:
            data = np.append(data,getRawParticipantData(folder,throw_task_file))

    splitData,splitOrders = split(data,0)

    plot_range = FIELD["VALID_RANGE"]
    legend_counts,deltaFilteredX,deltaFilteredY,heatmaps,xstddev,ystddev = [],[],[],[],[],[]
    for i in splitData.keys():
        deltaX = splitData[i]['HitPosX']
        deltaY = splitData[i]['HitPosY']

        xtest = np.logical_and(deltaX < 150, -deltaX < 300) 
        ytest = np.abs(deltaY) < 150 
        rtest = np.square(deltaY) + np.square(deltaX) < 130**2
        test = np.logical_and(xtest, ytest)
        deltaFilteredX += [deltaX[test]]
        deltaFilteredY += [-deltaY[test]]
        legend_counts += [len(deltaFilteredX[int(i)])]
        hm, xedges, yedges = np.histogram2d(deltaFilteredX[int(i)], deltaFilteredY[int(i)],range=plot_range, bins=20)
        heatmaps += [hm]

        xstddev += [np.std(deltaFilteredX[int(i)])]
        ystddev += [np.std(deltaFilteredY[int(i)])]

    titles = ['Leap Motion\n(Valid throws: '+ str(legend_counts[0]) + '/'+ str(len(splitData[0]['HitPosX'])) + ')',
              'Perception Neuron\n(Valid throws: '+ str(legend_counts[1]) + '/'+ str(len(splitData[1]['HitPosX'])) + ')',
              'Fused Tracking\n(Valid throws: '+ str(legend_counts[2])+  '/'+ str(len(splitData[2]['HitPosX'])) + ')']
    max_throw_density = np.max(heatmaps)
    
    fig, axes = plt.subplots(2,2,sharex=True, sharey=True)
    fig.set_figheight(4.5)
    fig.set_figwidth(5.7)
    cbar_ax = fig.add_axes([0.1, 0.05, 0.8, 0.03])
    cbar_ax.set_xlabel('Hit Count')
        
    #Test variances are different:
    result = scipy.stats.levene(deltaFilteredX[0],deltaFilteredX[1],deltaFilteredX[2],center='mean')
    print("Levenes test X: result Leap vs PN = ",scipy.stats.levene(deltaFilteredX[0],deltaFilteredX[1]))
    print("Levenes test X: result Leap vs FT = ",scipy.stats.levene(deltaFilteredX[0],deltaFilteredX[2]))
    print("Levenes test X: result FT vs PN= ",scipy.stats.levene(deltaFilteredX[1],deltaFilteredX[2]))
    print("stddev = ", xstddev)
    # exit()

    for i,ax in enumerate(axes.flat):
        # plt.plot(deltaFilteredX[int(i)],deltaFilteredY[int(i)],markerMap(i),c=colourMap(2),ms=10,markeredgewidth=1,markeredgecolor='black')
        
        if(i>0):
            i=i-1
            norm_heatmap = heatmaps[i]
            extent = [plot_range[0][0],plot_range[0][1],plot_range[1][0],plot_range[1][1]]
            #colorcet for perceptually stable colour mapping
            im = ax.imshow(norm_heatmap.T, extent=extent, origin='lower',interpolation='nearest',cmap=cc.m_fire)
            drawThrowingBG(ax)
            if(i==0):
                fig.colorbar(im, cax=cbar_ax,orientation='horizontal')
            im.set_clim(0,max_throw_density)
            # plt.plot(deltaFilteredX.mean(),deltaFilteredY.mean(),markerMap(i),c=colourMap(i),ms=20,markeredgewidth=1,markeredgecolor='black')
            ax.set_title(titles[i])
            ax.set_ylabel("Y (cm)")
            ax.axis('off')
            if(i==2):
                ax.set_xlabel("X (cm)")
        else:
            ax.set_title("Task Diagram")
            drawThrowingBGSolid(ax)
            # ax.ticks('off')
            ax.set_xticks([], [])
            ax.set_yticks([], [])

    saveFigure("ThrowingHeatmaps",pgf=False)
    print("Plotting projected heatmap X")
    #Plot x projected hist
    plotProjectedHeatmaps(splitData,heatmaps,plot_range,stddev=xstddev,axis=0)
    if(len(saveNames)>0):
        saveFigure("ThrowXPlot")

    print("Plotting projected heatmap Y")
    #Plot y projected hist
    plotProjectedHeatmaps(splitData,heatmaps,plot_range,stddev=ystddev,axis=1)
    if(len(saveNames)>0):
        saveFigure("ThrowYPlot")
    print("Projected Heatmapts plotted")
    
def plotProjectedHeatmaps(splitData,heatmaps,plot_range,stddev,axis):
    max_y = 0
    fig = plt.figure()
    legends = []
    names = ["Leap","Neuron","Fused"]
    for i in splitData.keys():
        
        norm_heatmap = heatmaps[int(i)]
        x_hist = np.sum(norm_heatmap.T,axis=axis)
        x_pos = np.linspace(plot_range[axis][0],plot_range[axis][1],len(x_hist))
        x_pos = x_pos + (x_pos[1]-x_pos[0])*0.5
        plt.plot(x_pos,x_hist,alpha=1,color=colourMap(i),label="{:s}".format(names[int(i)]))
        plt.fill_between(x_pos,0,x_hist,alpha=0.5,edgecolor='k',facecolor=colourMap(i))
        max_y = np.max(np.append(x_hist,[max_y]))
    
    max_y +=10


    red_patch = patches.Patch(color='red', label='The red data')
    print(legends)
    leg = plt.legend(names)
    for legobj in leg.legendHandles:
        legobj.set_linewidth(5)
    plt.xlim(plot_range[axis])
    plt.ylim(0,max_y)

    # plt.title("Throw Density vs. Distance")
    # True centre
    plt.plot([0,0],[0,max_y],'-k')
    # inner circle
    plt.plot([+FIELD["TARGET"]["CENTRE_R"],+FIELD["TARGET"]["CENTRE_R"]],[0,max_y],':k')
    plt.plot([-FIELD["TARGET"]["CENTRE_R"],-FIELD["TARGET"]["CENTRE_R"]],[0,max_y],':k')
    #Red circle
    plt.plot([FIELD["TARGET"]["RED_R"],FIELD["TARGET"]["RED_R"]],[0,max_y],':r')
    plt.plot([-FIELD["TARGET"]["RED_R"],-FIELD["TARGET"]["RED_R"]],[0,max_y],':r')    
    #Blue circle
    plt.plot([FIELD["TARGET"]["BLUE_R"],FIELD["TARGET"]["BLUE_R"]],[0,max_y],':b')
    plt.plot([-FIELD["TARGET"]["BLUE_R"],-FIELD["TARGET"]["BLUE_R"]],[0,max_y],':b')
    #Outer circle
    plt.plot([FIELD["TARGET"]["OUTER_R"],FIELD["TARGET"]["OUTER_R"]],[0,max_y],'--k')
    plt.plot([-FIELD["TARGET"]["OUTER_R"],-FIELD["TARGET"]["OUTER_R"]],[0,max_y],'--k')
    plt.ylabel("Throws")
    if(axis==0):
        plt.xlabel("X (cm)")
    elif(axis==1):
        plt.xlabel("Y (cm)")

def plotThrowingData(folders):
    data = np.array([])
    for folder in folders:
        if len(data) == 0:
            data = getRawParticipantData(folder,throw_task_file)
        else:
            data = np.append(data,getRawParticipantData(folder,throw_task_file))

    splitData,splitOrders = split(data,0)

    fig, ax = plt.subplots()
    drawThrowingBGSolid(ax)

    legend_counts = []
    for i in splitData.keys():
        deltaX = splitData[i]['HitPosX']
        deltaY = splitData[i]['HitPosY']

        xtest = np.abs(deltaX) < 300
        ytest = np.abs(deltaY) < 200 
        rtest = np.square(deltaY) + np.square(deltaX) < 130**2
        test = np.logical_and(xtest, ytest)
        deltaFilteredX = deltaX[test]
        deltaFilteredY = -deltaY[test]
        plt.plot(deltaFilteredX,deltaFilteredY,markerMap(i),c=colourMap(i),ms=10,markeredgewidth=1,markeredgecolor='black')
        # plt.plot(deltaFilteredX.mean(),deltaFilteredY.mean(),markerMap(i),c=colourMap(i),ms=20,markeredgewidth=1,markeredgecolor='black')
        legend_counts += [len(deltaFilteredX)]
    plt.legend(['Leap Motion (' + "{:3.1f}".format(100 * legend_counts[0]/float(len(splitData[0]['HitPosX']))) + '% valid / ' + str(len(splitData[0]['HitPosX'])) + ' throws)',
                'Perception Neuron (' + "{:3.1f}".format(100 * legend_counts[1]/float(len(splitData[1]['HitPosX']))) + '% valid / ' + str(len(splitData[1]['HitPosX'])) + ' throws)',
                'Fused Tracking (' + "{:3.1f}".format(100 * legend_counts[2]/float(len(splitData[2]['HitPosX']))) + '% valid / ' + str(len(splitData[2]['HitPosX'])) + ' throws)'])
    
def boxPlotColumns(data,data_subclasses=None,labels=None,scatter=False):
    plt.figure()
    bp = plt.boxplot(data,widths=0.9,zorder=1)# patch_artist=True)
    
    
    # Labels
    if(labels):
        x = range(1,len(labels)+1)
        plt.xticks(x,labels)
    
    #Colours
    # for i,patch in enumerate(bp['boxes']):
    #     # patch.set(facecolor=colors[i%2])
    #     patch.set(facecolor=colourMap(i%3))

    if(scatter):
        # Scatter points
        if(data_subclasses is None):        
            data_subclasses = np.zeros(data.shape)
        max_subclass = data_subclasses.max()
        min_subclass = data_subclasses.min()
        centre_subclass = (max_subclass - min_subclass)/ 0.5
        subclass_spacing = 1
        if(max_subclass!=min_subclass):
            subclass_spacing = 1/(max_subclass-min_subclass)
        subclass_width = 0.5
        for i in range(data.shape[0]):
            y = data[i,:]
            x = range(1,data.shape[1]+1) + ((data_subclasses[i,:] - min_subclass) * subclass_spacing - 0.5)*subclass_width  # np.random.normal(0, 0.0, size=len(y))
            plt.plot(x, y, 'ok', alpha=1)

    #X axis 
    x = [0,data.shape[1]+1]
    y = [0,0]
    plt.plot(x,y,'--k',linewidth=1,zorder=2)
    
def histogramPlotColumns(data,labels,title,bins = 5):
    #One plot for each column
    try:
        fig, axes = plt.subplots(data.shape[1], sharey=True,sharex=True)
    except(IndexError):
        fig, ax = plt.subplots()
        axes = [ax]
        data = np.transpose([data])
    if(title):
        plt.title(title)

    # Scatter points
    for i,ax in enumerate(axes):
        x = data[:,i]
        print(x)
        ax.hist(x, bins=10, color=colourMap(i))
        ax.set_ylabel(labels[i])

    for i,ax in enumerate(axes):
        x = data[:,i]
        y = ax.get_ylim()[1]
        ax2 = ax.twinx()
        ax2.boxplot(x,vert=False,widths=[0.75])
        ax2.tick_params(
            axis='y',          # changes apply to the y-axis
            which='both',      # both major and minor ticks are affected
            right=False,      # ticks along the bottom edge are off
            left=False,        # ticks along the top edge are off
            labelright=False,
            labelbottom=False) # labels along the bottom edge are off
        ax2.spines['right'].set_visible(False)
        # ax2.set_ylim(ax.get_ylim())
        ax2.plot([0,0],[0,y],'--k')

def wilcoxonAnalysisCols(data):
    results = [["Median","Min Sum Rank $T$", "Sig. $p$ (2-tailed)", "Effect $r$"]]
    for col_i in range(data.shape[1]):
        col = data[:,col_i]
        median = np.median(col)
        wilcoxon = scipy.stats.wilcoxon(col)
        z_score = scipy.stats.norm.ppf(wilcoxon.pvalue/2) 
        effect_r = z_score / np.sqrt(2*len(col)) #*2 because each participant was measured twice
        results += [[median,wilcoxon.statistic,wilcoxon.pvalue,effect_r]]
    # print("results size = ",len(results),"x",len(results[0]))
    results += [["\\\\\\hline","\\\\\\hline","\\\\\\hline","\\\\\\hline"]]
    return results



def getParticipantSummaryStats(participant):
    #Button
    b_scores, b_errors, b_rtimes, b_orders = getParticipantDataButton(participant)
    #Sort
    s_scores, s_errors, s_rtimes, s_orders = getParticipantDataSort(participant)
    #Throw
    t_scores, t_errors, t_rtimes, t_orders, d_error = getParticipantDataThrow(participant)

    # Order which fused was attempted (1,2,3)
    delta_orders = np.array([ b_orders[2]-b_orders[0],b_orders[2]-b_orders[1],
                                s_orders[2]-s_orders[0],s_orders[2]-s_orders[1],
                                t_orders[2]-t_orders[0],t_orders[2]-t_orders[1]])

    #SCORE IMPROVEMENTS
    improvements = np.array([b_scores[2]-b_scores[0],b_scores[2]-b_scores[1],
                               s_scores[2]-s_scores[0],s_scores[2]-s_scores[1],
                               t_scores[2]-t_scores[0],t_scores[2]-t_scores[1]]).astype(float)

    basescores = np.array([b_scores[0],b_scores[1],
                          s_scores[0],s_scores[1],
                          t_scores[0],t_scores[1]]).astype(float)
   
    # improvements = 100 * improvements / basescores
    
    #TIME IMPROVEMENTS
    time_improvements = np.array([b_rtimes[0]-b_rtimes[2],b_rtimes[1]-b_rtimes[2],
                                   s_rtimes[0]-s_rtimes[2],s_rtimes[1]-s_rtimes[2],
                                   t_rtimes[0]-t_rtimes[2],t_rtimes[1]-t_rtimes[2]])

    basetimes = np.array([b_rtimes[0],b_rtimes[1],
                          s_rtimes[0],s_rtimes[1],
                          t_rtimes[0],t_rtimes[1]]).astype(float)
    # time_improvements = 100 * time_improvements/basetimes
   
    #MISTAKE IMPROVEMENTS
    error_improvements = np.array([b_errors[0]-b_errors[2],b_errors[1]-b_errors[2],
                                    s_errors[0]-s_errors[2],s_errors[1]-s_errors[2],
                                    t_errors[0]-t_errors[2],t_errors[1]-t_errors[2]])
    baseerror = np.array([b_errors[0],b_errors[1],
                          s_errors[0],s_errors[1],
                          t_errors[0],t_errors[1]]).astype(float)
    # error_improvements = 100 * error_improvements / baseerror

    #Distance Error Improvements
    d_error_improvements = np.array([d_error[2]-d_error[0],d_error[2]-d_error[1]])

    base_d_error = d_error[0:1]

    # d_error_improvements = 100 * d_error_improvements / base_d_error

    scores = np.append(b_scores,[s_scores,t_scores])
    times = np.append(b_rtimes,[s_rtimes,t_rtimes])
    errors = np.append(b_errors,[s_errors,t_errors])
    orders = np.append(b_orders,[s_orders,t_orders])

    return scores, times, errors, orders, improvements, time_improvements, error_improvements, delta_orders, d_error, d_error_improvements

#Pvalue 
# Probability that we would see such large mean if population mean was zero
def getPValueNormGT0(data):
    sigma = data.std(axis=0)
    mean = data.mean(axis=0)
    pval = 1 - scipy.stats.norm.cdf(mean,scale=sigma/np.sqrt(data.shape[0]))
    return pval

#returns tech_order[i] = t, map from slot to technology used
perms = [[0,1,2],[2,0,1],[1,2,0],[1,0,2],[2,1,0],[0,2,1]]
def techOrder(pID,taskID):
    p = pID-1
    if(pID <= 13):
        #Every second participant gets the second half of permutations
        p_odd = p%2
    elif(pID <= 25):
        #Fill in missed perms
        #x12 participants
        p_odd = (p+1)%2
    elif(pID <= 28):
        # go back and fix up
        p_odd = p%2
    else:
        p_odd = (p/len(perms))%2
    #3 perms one for each tech, but order of which one goes to which task is selected by task_perm
    tech_perms = perms[p_odd*3:p_odd*3+3]
    #task perm permutes tech_perms
    task_perm = perms[p%len(perms)]
    #The index for the permutation for this task
    tech_index = task_perm[taskID]
    #Return the actual perm
    return tech_perms[tech_index]

    
def getNumber(c):
    if(c == ord('A')):
        return 0
    if(c == ord('B')):
        return 1
    if(c == ord('C')):
        return 2
    else:
        raise ValueError(c+" is not a valid preference!")



def inversePermutation(p):
    p_inv = np.zeros(len(p))
    for i in range(len(p)):
        # print(p[i]," -> ", i)
        p_inv[p[i]] = i
    return p_inv

def plotRowFrequencies(data):
    setOfTypes = set(data.flatten())
    n_types = len(setOfTypes)
    n_rows = data.shape[1]
    result = np.zeros(n_rows*n_types)
    listOfTypes = sorted(setOfTypes)
    for row in range(n_rows):
        counts_row = {e:0 for e in setOfTypes}
        for element in data[:,row]:
            counts_row[element] += 1
        result[row*n_types:(row+1)*n_types] = list(counts_row.values())
    
    plt.figure()
    width = 1
    indicies = np.array(range(len(result))) + np.array([a/n_types for a in range(len(result))])
    plt.bar(indicies,result,width)
    x = [x.mean() + width/2 for x in np.reshape(indicies.astype(float),[n_rows,n_types])]
    labels = ['Leap\nKeyboard', 'PN\nKeyboard','Fused\nKeyboard', 'Leap\nSorting', 'PN\nSorting','Fused\nSorting','Leap\nThrowing', 'PN\nThrowing','Fused\nThrowing']
    plt.xticks(x,labels)

def plotTestTechOrders():
    orders = []
    first_part = 5
    #any multiple of 4 balances
    n_part = 15
    participants = range(first_part,first_part+n_part)
    for pID in participants:
        participant_orders = []
        for taskID in range(3):
            #Task changes every three trials
            order = inversePermutation(techOrder(pID,taskID))
            participant_orders = np.append(participant_orders,order)
        orders += [participant_orders]
    print(orders)
    plotRowFrequencies(np.array(orders))
    plt.title("Predicted Order Frequencies "+str(len(participants)) + "\n" + str(participants))
# plotTestTechOrders()

def printTechOrders():
    for p in range(4,28):
        pID = p+1
        message = "P"+str(pID) + " "
        for taskID in range(3):
            message += stringFromTaskID(taskID) + " "
            #Task changes every three trials
            order = techOrder(pID,taskID)
            for i in order:
                message += stringFromTechID(i) + " "
        print(message)
# printTechOrders()

#Returns vector of preferences for 1st,2nd,3rd attempts
# prefs = A rank (0-2), B rank (0-2), C rank (0-2)
# so 0,2,1 => ACB
# or 1,2,0 => CAB
def parsePref(pref_string):
    rankings = [0,0,0]
    i = 0
    for c in pref_string.strip():
        n = getNumber(c)
        rankings[n] = i
        i+=1
    return rankings


def checkOrders(orders,pID):
        # print("orders ",orders)
    for taskID in range(3):
        # print(techOrder(pID,taskID) )
        #inverse because techOrder returns which tech given the task, attempt
        # the actual order returns which order a given task was performed
        predicted_order = inversePermutation(techOrder(pID,taskID))
        actual_order = orders[taskID*3:taskID*3+3] - 1
        if (predicted_order != actual_order).any():
            print("Order wrong!!")
            print("predicted_order = ", predicted_order        )
            print("actual_order = ", actual_order  )
            raise ValueError("Something has gone wrong with the orders!")      


def decodePreferences(participantIDs,preferences,task):
    # Preference counts for each tech
    taskID = taskIDFromString(task)
    # rankings[Tech][N] = number of times Tech was ranked Nth best
    #               Frequency
    #           1st,   2nd,   3rd
    # Tech1     0,     1,     1
    # Tech2     ....
    # Tech3
    rank_counts = np.zeros([3,3])
    for i in range(len(preferences)):
        pID = participantIDs[i]
        pref = preferences[i]
        rankings = parsePref(pref)
        t_order = techOrder(pID,taskID)
        for j in range(len(rankings)):
            rank = rankings[j]
            tech_id = t_order[j]
            rank_counts[tech_id][rank] += 1
    return rank_counts

def decodeComments(participantIDs,verbals,task):
     # Preference counts for each tech
    taskID = taskIDFromString(task)

    comments = [[],[],[]]
    for i in range(len(verbals)):
        pID = participantIDs[i]
        vs = verbals[i]
        t_order = techOrder(pID,taskID)
        for j in range(len(vs)):
            v = vs[j]
            tech_id = t_order[j]
            comments[tech_id] += [v]
    return comments

def decodeRanks(prefs,taskID,pID):
    # prefs = A rank (0-2), B rank (0-2), C rank (0-2)
    # t order = 
    t_order = techOrder(pID,taskID)
    
    best_attempt = prefs.index(0)
    second_best_attempt = prefs.index(1)
    third_best_attempt = prefs.index(2)

    first_choice_tech_id = t_order[best_attempt]
    second_choice_tech_id = t_order[second_best_attempt]
    third_choice_tech_id = t_order[third_best_attempt]

    result = [0,0,0]
    result[first_choice_tech_id] = 3
    result[second_choice_tech_id] = 2
    result[third_choice_tech_id] = 1
    return result

def saveOutPreferenceTable(filename,k_pref,s_pref,t_pref,sum=False):
    pref_table = []
    p_loc = {}
    for i in range(len(k_pref)):
        #Set participant number
        p = k_pref[i][0]
        pref_table += [[p]]
        #store participant location for later
        p_loc[p] = i

        #include keyboard prefs
        pref = parsePref(k_pref[i][1])
        pref_table[i] += decodeRanks(pref,taskID=taskIDFromString("keyboard"),pID=p)
    
    for i in range(len(s_pref)):
        #Set participant number
        p = s_pref[i][0]

        #include sorting prefs
        pref = parsePref(s_pref[i][1])
        if(sum):
            ranks = decodeRanks(pref,taskID=taskIDFromString("sorting"),pID=p)
            pref_table[p_loc[p]][0+1] += ranks[0]
            pref_table[p_loc[p]][1+1] += ranks[1]
            pref_table[p_loc[p]][2+1] += ranks[2]
        else:
            pref_table[p_loc[p]] += decodeRanks(pref,taskID=taskIDFromString("sorting"),pID=p)

    for i in range(len(t_pref)):
        #Set participant number
        p = t_pref[i][0]

        #include sorting prefs
        pref = parsePref(t_pref[i][1])
        if(sum):
            ranks = decodeRanks(pref,taskID=taskIDFromString("throwing"),pID=p)
            pref_table[p_loc[p]][0+1] += ranks[0]
            pref_table[p_loc[p]][1+1] += ranks[1]
            pref_table[p_loc[p]][2+1] += ranks[2]
        else:
            pref_table[p_loc[p]] += decodeRanks(pref,taskID=taskIDFromString("throwing"),pID=p)

    # print("pref_table",pref_table)
    if(sum):
        header = ["Participant ID","LP Ranks","PN Ranks","FT Ranks"]
    else:
        header = ["Participant ID","Keyboard LP Ranks","Keyboard PN Ranks","Keyboard FT Ranks",
                                   "Sorting LP Ranks","Sorting PN Ranks","Sorting FT Ranks",
                                   "Throwing LP Ranks","Throwing PN Ranks","Throwing FT Ranks"]
    saveTable(filename,pref_table,header)

def getResponseData(task):
    return genfromtxt("ParticipantResponses/"+task+"_responses.txt",
                      delimiter=",", 
                      comments="#", 
                      names=["Participant", "Quality", "Utility", "CommentsA", "CommentsB", "CommentsC", "GeneralComments"],
                      dtype=None)


def plotPreferenceAnalysis(GraphName,prefs):
    #Preferences totalled over all tasks
    prefsTotal = np.sum(prefs,axis=0)

    # print("prefs")
    # print(prefs)
    # print("prefsTotal",prefsTotal)
    width = 0.2
    tick_pos = width*1
    #-----------------------------
    # Number of times each system was ranked 1st, 2nd, and 3rd
    #-----------------------------
    # plt.figure()
    # ind = np.array(range(3))
    # plt.title(GraphName + " - Tech Ranking Frequencies")
    # for i in range(prefsTotal.shape[0]):
    #     plt.bar(ind+i*width,prefsTotal[i],width,color=colourMap(i))
    # x = np.array([0,1,2]) + tick_pos
    # labels = ['1st Count', '2nd Count', '3rd Count']
    # plt.xticks(x,labels)
    #-----------------------------

    #-----------------------------
    #THIS GRAPH IS ACE:
    # Plotting scores 
    #-----------------------------
    plt.figure()
    #points per ranking
    n_participants = np.sum(prefs[0][0])
    print("n_participants",n_participants)
    weight_vector = np.array([3,2,1]) / n_participants
    print("prefs",prefs)

    techIDs = np.array([0,1,2])
    techID_widths = width * np.array([0,1,2])
    plt.title(GraphName+" Preferences")
    plt.ylabel("Mean Preference Rank")

    total_tech_scores = 0
    for taskID in range(len(prefs)):
        #Vector of tech scores
        tech_scores = np.dot(prefs[taskID],weight_vector)
        total_tech_scores += tech_scores
        print(map(colourMap,techIDs))
        rects = plt.bar(techID_widths + taskID, tech_scores, width, color=list(map(colourMap,techIDs)))
    
    # plt.legend((rects[0], rects[1], rects[2]), ('LP', 'PN', 'FT'))
    x = np.array([0,1,2]) + tick_pos
    labels = ['Keyboard', 'Sorting', 'Throwing']
    plt.xticks(x,labels)
    plt.ylim([1,3])
    saveFigure(GraphName+"Responses")
    #-----------------------------
    #-----------------------------
    # Plotting sum scores 
    #-----------------------------
    plt.figure()
    plt.title(GraphName + " (All Tasks)")
    plt.bar(np.array([0,1,2]), total_tech_scores / 3, 1, color=list(map(colourMap,techIDs)))
    plt.ylim([1,3])
    print(GraphName + " total_tech_scores",total_tech_scores/3)
    
    x = np.array([0,1,2])
    labels = ['LP', 'PN', 'FT']
    plt.ylabel("Mean Preference Rank")
    plt.xticks(x,labels)
    #-----------------------------

    #TODO: I thought it might be a good idea to plot preferences against order of tech seen - is it worth the effort?

def plotPValues(improvements_p):
    plt.figure()
    horiz = np.arange(6)
    ticks = ("FT>LP","FT>PN","FT>LP","FT>PN","FT>LP","FT>PN")
    plt.bar(horiz, improvements_p)
    plt.plot([0,6],[0.05,0.05],"--r")
    plt.xticks(horiz+0.5, ticks)

def commentWordCloud(comments):

    sumComments = ["","",""]
    for i in range(len(comments)):
        for j in range(len(comments[i])):
            sumComments[i] += " " + str(comments[i][j])

    counters = []
    titles = ["Leap","Neuron","Fused"]
    for i,comm in enumerate(sumComments):
        comm = comm.replace("b\'", " ")
        comm = comm.replace('\"', " ")
        comm = comm.replace('\'', " ")
        comm = comm.replace('hand', " ")
        comm = comm.replace('Hand', " ")
        comm = comm.replace('right', " ")
        comm = comm.replace('left', " ")
        comm = comm.replace('finger', " ")
        comm = comm.replace('fingers', " ")
        counters += [Counter(comm.split())]
        plt.figure()
        plt.title(titles[i])
        word_cloud = WordCloud().generate(comm)
        plt.imshow(word_cloud)
        plt.axis("off")
    return counters

def combineComments(comments_by_task):
    combinedComments = [[],[],[]]
    for task,comments_by_tech in enumerate(comments_by_task):
        for tech,comment_list in enumerate(comments_by_tech):
            combinedComments[tech] += comment_list
    return combinedComments

def extractParticipantComments(text):      
  matches=re.findall(r'\"(.+?)\"',str(text))
  # matches is now ['String 1', 'String 2', 'String3']
  return ",".join(matches)

def anylize_sentiment(statement):
    '''
    Utility function to classify the polarity of a statement
    using textblob.
    '''
    analysis = TextBlob(str(statement))
    print(analysis.sentiment.subjectivity)
    if analysis.sentiment.polarity > 0:
        return 1
    elif analysis.sentiment.polarity == 0:
        return 0
    else:
        return -1

def getSumSentiment(comments_by_tech):
    #           neg,   neutral,   positive
    # Tech1     0,     1,          1
    # Tech2     ....
    # Tech3
    result = np.zeros([3,3])
    for i,comments in enumerate(comments_by_tech):
        for j,c in enumerate(comments_by_tech[i]):
            if(len(c) < 2):
                continue
            print("Comment:",c)
            sentiment = anylize_sentiment(c)
            print("Sentiment:",sentiment)
            index = 1 + sentiment
            result[i][index] += 1
    return result


keyboard_responses = getResponseData("keyboard")
sorting_responses = getResponseData("sorting")
throwing_responses = getResponseData("throwing")
# print(keyboard_responses)
# print(sorting_responses)
# print(throwing_responses)

# prefs[taskid][techid][rank] gives the number of times that task ranked that tech as rank rank
Qprefs = [decodePreferences(keyboard_responses["Participant"],keyboard_responses["Quality"],"keyboard") 
    , decodePreferences(sorting_responses["Participant"],sorting_responses["Quality"],"sorting")
    , decodePreferences(throwing_responses["Participant"],throwing_responses["Quality"],"throwing")]

Uprefs = [decodePreferences(keyboard_responses["Participant"],keyboard_responses["Utility"],"keyboard") 
    , decodePreferences(sorting_responses["Participant"],sorting_responses["Utility"],"sorting")
    , decodePreferences(throwing_responses["Participant"],throwing_responses["Utility"],"throwing")]

comments_by_task = [decodeComments(keyboard_responses["Participant"],keyboard_responses[["CommentsA","CommentsB","CommentsC"]],"keyboard") 
    , decodeComments(sorting_responses["Participant"],sorting_responses[["CommentsA","CommentsB","CommentsC"]],"sorting")
    , decodeComments(throwing_responses["Participant"],throwing_responses[["CommentsA","CommentsB","CommentsC"]],"throwing")]


comments_by_tech = combineComments(comments_by_task)

#Filter for just participant comments in quotes
participantUtterances = comments_by_tech
for i,utterances in enumerate(participantUtterances):
    print("Adding utterances for tech=",i, " length = ", len(utterances))
    for j,utt in enumerate(utterances):
        utt = extractParticipantComments(utt)
        print(utt)

# commentWordCloud(participantUtterances)
# plt.show()

sumSent = getSumSentiment(participantUtterances)
print(sumSent)

print(comments_by_tech)
# Wordcloud:
# word_counters = commentWordCloud(comments_by_task)
# print("word_counters",word_counters)



plotPreferenceAnalysis("Quality",Qprefs)
saveFigure("QualitySumResponses")
plotPreferenceAnalysis("Utility",Uprefs)
saveFigure("UtilitySumResponses")
# plt.show()

saveOutPreferenceTable("data/QualityPreferences.csv",keyboard_responses[["Participant","Quality"]],sorting_responses[["Participant","Quality"]],throwing_responses[["Participant","Quality"]])
saveOutPreferenceTable("data/QualityPreferencesSummed.csv",keyboard_responses[["Participant","Quality"]],sorting_responses[["Participant","Quality"]],throwing_responses[["Participant","Quality"]],sum=True)
saveOutPreferenceTable("data/UtilityPreferences.csv",keyboard_responses[["Participant","Utility"]],sorting_responses[["Participant","Utility"]],throwing_responses[["Participant","Utility"]])
saveOutPreferenceTable("data/UtilityPreferencesSummed.csv",keyboard_responses[["Participant","Utility"]],sorting_responses[["Participant","Utility"]],throwing_responses[["Participant","Utility"]],sum=True)


def performanceAnalysis():
    participants = [5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,21,22,23]
    # improvements, time_improvements, error_improvements = np.array([]),np.array([]),np.array([])
    # scores, times, errors = np.array([]),np.array([]),np.array([])
    parNames = []
    first = True
    dataTable = []
    for p in participants:
        participantName = "Participant"+str(p)
        parNames += [participantName]
        s,T,E,o,i,t,e,do,de,dei = getParticipantSummaryStats(participantName)
        # print("s,T,E,o,i,t,e,do,de,dei",s,T,E,o,i,t,e,do,de,dei)
        dataTable+=[np.concatenate([[participantName],s,T,E,o,i,t,e,do,de,dei])]
        checkOrders(o,p)
        if(first):
            improvements = [i]
            time_improvements = [t]
            error_improvements = [e]
            orders = [o]
            scores = [s]
            times = [T]
            errors = [E]
            deltaOrders = [do]
            distanceError = [de]
            distanceErrorImprovements = [dei]

            first = False
        else:
            improvements = np.append(improvements,[i],axis=0)
            time_improvements = np.append(time_improvements,[t],axis=0)
            error_improvements = np.append(error_improvements,[e],axis=0)
            scores = np.append(scores,[s],axis=0)
            times = np.append(times,[T],axis=0)
            errors = np.append(errors,[E],axis=0)
            orders = np.append(orders,[o],axis=0)
            deltaOrders = np.append(deltaOrders,[do],axis=0)
            distanceError = np.append(distanceError,[de],axis=0)
            distanceErrorImprovements = np.append(distanceErrorImprovements,[dei],axis=0)
    # print("dataTable",dataTable)
    table_header = ["#Participant","Score Leap Keyboard", "Score Neuron Keyboard","Score Fused Keyboard",
                                   "Score Leap Sorting", "Score Neuron Sorting","Score Fused Sorting",
                                   "Score Leap Throwing", "Score Neuron Throwing","Score Fused Throwing",
                                   
                                   "Time Leap Keyboard", "Time Neuron Keyboard","Time Fused Keyboard",
                                   "Time Leap Sorting", "Time Neuron Sorting","Time Fused Sorting",
                                   "Time Leap Throwing", "Time Neuron Throwing","Time Fused Throwing",
                                   
                                   "Mistake Leap Keyboard", "Mistake Neuron Keyboard","Mistake Fused Keyboard",
                                   "Mistake Leap Sorting", "Mistake Neuron Sorting","Mistake Fused Sorting",
                                   "Mistake Leap Throwing", "Mistake Neuron Throwing","Mistake Fused Throwing",

                                   "Order Leap Keyboard", "Order Neuron Keyboard","Order Fused Keyboard",
                                   "Order Leap Sorting", "Order Neuron Sorting","Order Fused Sorting",
                                   "Order Leap Throwing", "Order Neuron Throwing","Order Fused Throwing",

                                   "Score Improvement Leap Keyboard", "Score Improvement Neuron Keyboard",
                                   "Score Improvement Leap Sorting", "Score Improvement Neuron Sorting",
                                   "Score Improvement Leap Throwing", "Score Improvement Neuron Throwing",

                                   "Time Improvement Leap Keyboard", "Time Improvement Neuron Keyboard",
                                   "Time Improvement Leap Sorting", "Time Improvement Neuron Sorting",
                                   "Time Improvement Leap Throwing", "Time Improvement Neuron Throwing",

                                   "Mistake Improvement Leap Keyboard", "Mistake Improvement Neuron Keyboard",
                                   "Mistake Improvement Leap Sorting", "Mistake Improvement Neuron Sorting",
                                   "Mistake Improvement Leap Throwing", "Mistake Improvement Neuron Throwing",

                                   "Delta Orders Leap Keyboard", "Delta Orders Neuron Keyboard",
                                   "Delta Orders Leap Sorting", "Delta Orders Neuron Sorting",
                                   "Delta Orders Leap Throwing", "Delta Orders Neuron Throwing",

                                   "Throwing Error Leap", "Throwing Error Neuron", "Throwing Error Fused",
                                   "Throwing Error Improvement Leap", "Throwing Error Improvement Neuron"]

                                   
    saveTable("data/ObjectiveData.csv",np.array(dataTable),header=table_header)


    # print("distanceError = ", distanceError)
    # print("orders",orders)
    plotRowFrequencies(orders)
    plt.title("Actual Orders")
    saveFigure("Orders")

    #All Participants
    plotThrowingHeatmaps(parNames,["ThrowsLP","ThrowsPN","ThrowsFT"])
    plotThrowingData(parNames)
    plt.title("All Throws")    
    saveFigure("AllThrows")

    #This participant
    plotThrowingData(["Participant23"])
    plt.title("Participant 22 Throws")    
    # saveFigure("Participant22Throws")

    #===============
    # Wilcoxon analysis
    #===============
    wilcoxonScore = list_transpose(wilcoxonAnalysisCols(improvements))
    wilcoxonMistakes = list_transpose(wilcoxonAnalysisCols(-error_improvements))
    # wilcoxonHeader = ["Median","Min Sum Rank $T$", "Sig. $p$ (2-tailed)", "Effect $r$"]
    installTexTable("DeltaScoreWilcoxonResults",wilcoxonScore)
    installTexTable("DeltaMistakesWilcoxonResults",wilcoxonMistakes)

    #===============
    # Delta data
    #===============
    #Score Improvements
    labels = ["FT$-$LP","FT$-$PN"]
    boxPlotColumns(improvements[:,0:2],labels=labels)
    
    # histogramPlotColumns(np.sum(improvements[:,0:2],axis=1),labels=["Total Fused Improvement"])
    saveFigure("DeltaScoreKeyboard")
    boxPlotColumns(improvements[:,2:4],labels=labels)
    saveFigure("DeltaScoreSorting")
    boxPlotColumns(improvements[:,4:6],labels=labels)
    saveFigure("DeltaScoreThrowing")

    labels = ["FT$-$LP","FT$-$PN"]
    #Mistake change
    boxPlotColumns(-error_improvements[:,0:2],labels=labels)
    # boxPlotColumns(np.sum(error_improvements,axis=1),labels=["Total Fused Improvement"])
    saveFigure("DeltaErrorsKeyboard")
    boxPlotColumns(-error_improvements[:,2:4],labels=labels)
    saveFigure("DeltaErrorsSorting")
    boxPlotColumns(-error_improvements[:,4:6],labels=labels)
    saveFigure("DeltaErrorsThrowing")
    
    # boxPlotColumns(np.sum(error_improvements,axis=1),labels=["Total Fused Improvement"])
    # saveFigure("SuccessRateKeyboard")
    # boxPlotColumns(success_rate[:,2:4],labels=labels)
    # saveFigure("SuccessRateSorting")
    # boxPlotColumns(success_rate[:,4:6],labels=labels)
    # saveFigure("SuccessRateThrowing")

    #Old box plots:
    # plt.title("Change in Score (Fusion vs. X)")
    # boxPlotColumns(-time_improvements, deltaOrders)
    # plt.title("Change in Time (Fusion vs. X)")
    # saveFigure("ImprovementsTime")
    # boxPlotColumns(-error_improvements, deltaOrders)
    # plt.title("Change in Mistakes (Fusion vs. X)")    
    # saveFigure("ImprovementsError")
    
    #===============
    # Raw data
    #===============

    success_rate = scores / (scores+errors)
    # labels = ["$M_{FT}- M_{LP}$","$M_{FT}- M_{PN}$"]
    tech_labels=["LP","PN","FT"]
    #Success rate
    boxPlotColumns(success_rate,labels=[tech_labels,tech_labels,tech_labels])
    saveFigure("SuccessRates")

    # print(scores, orders)
    # boxPlotColumns(scores, orders)
    boxPlotColumns(scores[:,0:3],labels=tech_labels)
    plt.ylim([0,np.max(scores[:,0:3])+5])
    # fig = plt.gcf()
    # fig.set_size_inches(20,10)
    plt.ylabel("Score")
    plt.title('Keyboard')
    saveFigure("RawScoresKeyboard")
    boxPlotColumns(scores[:,3:6],labels=tech_labels)
    plt.ylim([0,np.max(scores[:,3:6])+5])
    # fig = plt.gcf()
    # fig.set_size_inches(20,10)
    # plt.ylabel("Score")
    plt.title('Sorting')
    saveFigure("RawScoresSorting")
    boxPlotColumns(scores[:,6:9],labels=tech_labels)
    plt.ylim([0,np.max(scores[:,6:9])+5])
    # fig = plt.gcf()
    # fig.set_size_inches(20,10)
    # plt.ylabel("Score")
    plt.title('Throwing')
    saveFigure("RawScoresThrowing")

    boxPlotColumns(errors, orders)
    fig = plt.gcf()
    fig.set_size_inches(20,10)
    plt.title("Raw Errors")
    saveFigure("RawErrors")
    
    boxPlotColumns(times, orders)
    fig = plt.gcf()
    fig.set_size_inches(20,10)
    plt.title("Raw Times")
    saveFigure("RawTimes")

    #Throwing task specifically:
    boxPlotColumns(distanceError, orders[:,6:9])
    plt.title("Mean Throwing Error Distance")
    saveFigure("MeanThrowingErrorDistance")
    
    boxPlotColumns(-distanceErrorImprovements, deltaOrders[:,4:5])
    plt.title("Change in Mean Throwing Error (Fusion vs. X)")
    saveFigure("ImprovementsMeanThrowingError")

    #test with repeated same measurements
    # improvements = np.repeat(improvements,5,axis=0)
    # time_improvements = np.repeat(time_improvements,5,axis=0)
    # error_improvements = np.repeat(error_improvements,5,axis=0)
    # # Offset to avoid zero stddev
    # improvements[4] = improvements[4]-1
    # time_improvements[4] = time_improvements[4]-0.1
    # error_improvements[4] = error_improvements[4]-0.1

    #Statistical tests
    print("improvements ")
    print(improvements )
    print("time_improvements ")
    print(time_improvements)
    print("error_improvements ")
    print(error_improvements)
    
    print("getPValueNormGT0(improvements) ")
    print(getPValueNormGT0(improvements) )
    improvements_p = getPValueNormGT0(improvements) #< 0.05
    plotPValues(improvements_p)
    saveFigure("PValuesImprovements")

    print("getPValueNormGT0(time_improvements) ")
    print(getPValueNormGT0(time_improvements) )
    time_improvements_p = getPValueNormGT0(time_improvements) #< 0.05
    plotPValues(time_improvements_p)
    saveFigure("PValuesTimes")

    print("getPValueNormGT0(error_improvements) ")
    print(getPValueNormGT0(error_improvements) )
    error_improvements_p = getPValueNormGT0(error_improvements) #< 0.05
    plotPValues(error_improvements_p)
    saveFigure("PValuesErrors")

performanceAnalysis()

def plotResponsesPieChart(vals):
    def make_autopct(values):
        def my_autopct(pct):
            total = sum(values)
            val = int(round(pct*total/100.0))
            return '{p:.0f}\%  ({v:d})'.format(p=pct,v=val)
        return my_autopct

    plt.figure()
    labels = None
    colors = 'xkcd:green', 'lightgreen', 'xkcd:gold', 'xkcd:coral'
    # explode = (0, 0.05, 0, 0)

    plt.pie(vals, labels=labels, colors = colors, explode=[0,0,0,0.05], autopct=make_autopct(vals))
    plt.axis("off")
#'Positive', 'No Mention', 'Mixed','Negative',
#Subtractions are from participant 20
leap_resp_count = [2,18-1,2,35-2]
neuron_resp_count = [5,23-2,6,23-1]
fused_resp_count = [5,28-2,4,20-1]

plotResponsesPieChart(leap_resp_count)
saveFigure("ResponsePieLeap")

plotResponsesPieChart(neuron_resp_count)
saveFigure("ResponsePieNeuron")

plotResponsesPieChart(fused_resp_count)
saveFigure("ResponsePieFused")

print("END OF CODE REACHED SUCCESSFULLY!")
# plt.show()