
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
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

def getDataFromFile(file,names,converters):
    return np.array(genfromtxt(       file,
                      delimiter=" ", 
                      comments="#", 
                      names=names,
                      converters=converters))

#Specific analysis functions for each task
def getParticipantDataButton(folder):
    data = getDataFromFile(folder+"/"+button_task_file,
                      names=["Tech", "Time", "Correct","CorrectPosX", "CorrectPosY", "CorrectPosZ", "TouchPosX", "TouchPosY", "TouchPosZ", "ButtonSize", "ResponseTime"],
                      converters={"Tech":techFromString,"Correct": boolFromString})
    
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
    data = getDataFromFile(folder+"/"+sorting_task_file,
                      names=["Tech", "Time", "Correct", "Floor", "CubeNumber", "CubeColour", "ResponseTime", "NHands", "Grasps"],
                      converters={"Tech":techFromString, "Correct": boolFromString, "Floor": boolFromString, "CubeColour": colourFromString})
    splitData = split(data,0)
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Correct'].sum()
        responseTimes[int(i)] = splitData[i]['ResponseTime'].mean()
    return scores, responseTimes

def getParticipantDataThrow(folder):
    data = getDataFromFile(folder+"/"+throw_task_file,
                      names=["Tech", "Time", "Success", "HitPosX", "HitPosY", "HitPosZ", "ResponseTime", "GraspCount", "ThrowVelX", "ThrowVelY", "ThrowVelZ"],
                      converters={"Tech":techFromString, "Success": boolFromString})

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

print getParticipantDataButton("JakeTest_5_4_18")
print getParticipantDataSort("JakeTest_5_4_18")
print getParticipantDataThrow("JakeTest_5_4_18")