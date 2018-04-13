
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
import scipy.stats
from numpy import genfromtxt

button_task_file = "ButtonBoard.csv"
sorting_task_file = "SortingTask.csv"
throw_task_file = "ThrowingTask.csv"

def boolFromString(s):
    if(s.lower() == "true"):
        return 1
    elif(s.lower() == "false"):
        return 0
    else:
        raise ValueError('Failed to read boolean value')

def techFromString(s):
    #LP = 0 = Leap Motion
    #PN = 1 = Perception Neuron
    #FT = 2 = Fused Tracking
    if(s.upper() == "LP"):
        return 0
    elif(s.upper() == "PN"):
        return 1
    elif(s.upper() == "FT"):
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

def colourFromString(s):
    if(s.lower() == "red"):
        return 0
    elif(s.lower() == "green"):
        return 1
    elif(s.lower() == "blue"):
        return 2
    else:
        raise ValueError("String " + s + " doesnt correspond to a colour")

def split(X, column):
    splitX = {X[0][column] : np.array([X[0]])}
    for i in range(1,len(X)):
        val_i = X[i][column]
        if(not (val_i in splitX.keys())):
            splitX[val_i] = np.array([X[i]])
        else:
            splitX[val_i] = np.append(splitX[val_i], [X[i]])
    return splitX

def colourMap(i):
    return {
        0 : 'tab:green',
        1 : 'tab:blue',
        2 : 'tab:orange'
    }[int(i)]

def markerMap(i):
    return {
        0 : 's',
        1 : 'o',
        2 : '*'
    }[int(i)]

def getDataFromFile(file,names,converters):
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

#Specific analysis functions for each task
def getParticipantDataButton(folder):
    data = getRawParticipantData(folder,button_task_file)
    
    splitData = split(data,0)
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    mean_errors = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Correct'].sum()
        responseTimes[int(i)] = splitData[i]['ResponseTime'].mean()

        deltaX = splitData[i]['CorrectPosX'] - splitData[i]["TouchPosX"]
        deltaY = splitData[i]['CorrectPosY'] - splitData[i]["TouchPosY"]
        deltaZ = splitData[i]['CorrectPosZ'] - splitData[i]["TouchPosZ"]
        errors = np.sqrt(deltaX**2+deltaY**2+deltaZ**2)
        mean_errors[int(i)] = errors.mean()
    return scores,mean_errors,responseTimes

def getParticipantDataSort(folder):
    data = getRawParticipantData(folder,sorting_task_file)
    splitData = split(data,0)
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Correct'].sum()
        responseTimes[int(i)] = splitData[i]['ResponseTime'].mean()
    return scores, responseTimes


def getParticipantDataThrow(folder):
    data = getRawParticipantData(folder,throw_task_file)

    splitData = split(data,0)
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    mean_errors = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Success'].sum()
        responseTimes[int(i)] = splitData[i]['ResponseTime'].mean()

        deltaX = splitData[i]['HitPosX']
        deltaY = splitData[i]['HitPosY']
        errors = np.sqrt(deltaX**2+deltaY**2) 
        #Filter large errors (corresponding to ball disappearing)
        errors = (errors < 1000).astype(float) * errors
        mean_errors[int(i)] = errors.mean()
    return scores, mean_errors, responseTimes

def plotThrowingData(folder):
    data = getRawParticipantData(folder,throw_task_file)

    splitData = split(data,0)

    fig, ax = plt.subplots()

    ax.set_aspect(1)
    outer = patches.Circle([0,0], radius=130, color='w',linewidth=1,linestyle='solid',ec='k')
    outmiddle = patches.Circle([0,0], radius=80, color='b')
    middle = patches.Circle([0,0], radius=28, color='r')
    inner = patches.Circle([0,0], radius=5, color='k')
    ssize = 30.0
    player = patches.Rectangle([-250 - ssize/2,-ssize/2], width=ssize, height=ssize, color='k')
    standsize = 40.0
    ball_stand = patches.Rectangle([-220.0 - standsize/2.0,-50-standsize/2.0], width=standsize, height=standsize, color='w',linestyle='solid',ec='k',linewidth=1)
    ball = patches.Circle([-220,-50], radius=5, color='w',linestyle='solid',ec='k',linewidth=1)
    ax.add_patch(outer)
    ax.add_patch(outmiddle)
    ax.add_patch(middle)
    ax.add_patch(inner)
    ax.add_patch(player)
    ax.add_patch(ball_stand)
    ax.add_patch(ball)

    legend_counts = []
    for i in splitData.keys():
        deltaX = splitData[i]['HitPosX']
        deltaY = splitData[i]['HitPosY']

        xtest = np.abs(deltaX) < 200
        ytest = np.abs(deltaY) < 200 
        test = np.logical_and(xtest, ytest)
        deltaFilteredX = deltaX[test]
        deltaFilteredY = deltaY[test]
        plt.plot(deltaFilteredX,deltaFilteredY,markerMap(i),c=colourMap(i),ms=10,markeredgewidth=1)
        legend_counts += [str(len(deltaFilteredX))]
    plt.legend(['Leap Motion (' + legend_counts[0] + '/' + str(len(splitData[0]['HitPosX'])) + ' valid throws)',
                'Perception Neuron (' + legend_counts[1] + '/' + str(len(splitData[1]['HitPosX'])) + ' valid throws)',
                'Fused Tracking (' + legend_counts[2] + '/' + str(len(splitData[2]['HitPosX'])) + ' valid throws)'])
    

    plt.show()


def getParticipantSummaryStats(participant):
    #Button
    b_scores, b_errors, b_rtimes = getParticipantDataButton(participant)
    #Sort
    s_scores, s_rtimes = getParticipantDataSort(participant)
    #Throw
    t_scores, t_errors, t_rtimes = getParticipantDataThrow(participant)

    improvements = np.array([[b_scores[2]-b_scores[0],b_scores[2]-b_scores[1],
                               s_scores[2]-s_scores[0],s_scores[2]-s_scores[1],
                               t_scores[2]-t_scores[0],t_scores[2]-t_scores[1]]])

    time_improvements = np.array([[b_rtimes[0]-b_rtimes[2],b_rtimes[1]-b_rtimes[2],
                                   s_rtimes[0]-s_rtimes[2],s_rtimes[1]-s_rtimes[2],
                                   t_rtimes[0]-t_rtimes[2],t_rtimes[1]-t_rtimes[2]]])
    
    error_improvements = np.array([[b_errors[0]-b_errors[2],b_errors[1]-b_errors[2],
                                    t_errors[0]-t_errors[2],t_errors[1]-t_errors[2]]])

    return improvements, time_improvements, error_improvements

#Pvalue 
# Probability that we would see such large mean if population mean was zero
def getPValueNormGT0(data):
    sigma = data.std(axis=0)
    mean = data.mean(axis=0)
    pval = 1 - scipy.stats.norm.cdf(mean,scale=sigma/np.sqrt(data.shape[0]))    
    return pval

plotThrowingData("MattTest2")

participants = ["JakeTest_12_4_18","MattTest","MattTest2"]
improvements, time_improvements, error_improvements = np.array([]),np.array([]),np.array([])

first = True
for p in participants:
    i,t,e = getParticipantSummaryStats(p)
    if(first):
        improvements = i
        time_improvements = t
        error_improvements = e
        first = False
    else:
        improvements = np.append(improvements,i,axis=0)
        time_improvements = np.append(time_improvements,t,axis=0)
        error_improvements = np.append(error_improvements,e,axis=0)


#test with repeated same measurements
# improvements = np.repeat(improvements,5,axis=0)
# time_improvements = np.repeat(time_improvements,5,axis=0)
# error_improvements = np.repeat(error_improvements,5,axis=0)
# # Offset to avoid zero stddev
# improvements[4] = improvements[4]-1
# time_improvements[4] = time_improvements[4]-0.1
# error_improvements[4] = error_improvements[4]-0.1

print "improvements "
print improvements
print "time_improvements "
print time_improvements
print "error_improvements "
print error_improvements
print "getPValueNormGT0(improvements) "
print getPValueNormGT0(improvements) < 0.05
print "getPValueNormGT0(time_improvements) "
print getPValueNormGT0(time_improvements) < 0.05
print "getPValueNormGT0(error_improvements) "
print getPValueNormGT0(error_improvements) < 0.05