
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
    return {
        0 : 'green',
        1 : 'blue',
        2 : 'orange'
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

#Specific analysis functions for each task
def getParticipantDataButton(folder):
    data = getRawParticipantData(folder,button_task_file)
    
    splitData, splitOrders = split(data,0)
    # print "data", data
    # print "splitOrders", splitOrders
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    mean_errors = np.array([0.0,0.0,0.0])
    orders = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Correct'].sum()
        success = splitData[i]['Correct'] == 1
        responseTimes[int(i)] = splitData[i]['ResponseTime'][success].mean()

        deltaX = splitData[i]['CorrectPosX'] - splitData[i]["TouchPosX"]
        deltaY = splitData[i]['CorrectPosY'] - splitData[i]["TouchPosY"]
        deltaZ = splitData[i]['CorrectPosZ'] - splitData[i]["TouchPosZ"]
        errors = np.sqrt(deltaX**2+deltaY**2+deltaZ**2)
        mean_errors[int(i)] = errors.mean()
        orders[int(i)] = splitOrders[i]
    return scores,mean_errors,responseTimes,orders

def getParticipantDataSort(folder):
    data = getRawParticipantData(folder,sorting_task_file)
    splitData, splitOrders = split(data,0)
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    mistakes = np.array([0.0,0.0,0.0])
    orders = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Correct'].sum()
        success = splitData[i]['Floor'] == 0
        responseTimes[int(i)] = splitData[i]['ResponseTime'][success].mean()
        mistakes[int(i)] = splitData[i]['Floor'].sum()
        orders[int(i)] = splitOrders[i]
    return scores,mistakes, responseTimes,orders


def getParticipantDataThrow(folder):
    data = getRawParticipantData(folder,throw_task_file)

    splitData, splitOrders = split(data,0)
    scores = np.array([0,0,0])
    responseTimes = np.array([0.0,0.0,0.0])
    mean_errors = np.array([0.0,0.0,0.0])
    orders = np.array([0.0,0.0,0.0])
    for i in splitData.keys():
        scores[int(i)] = splitData[i]['Success'].sum()


        deltaX = splitData[i]['HitPosX']
        deltaY = splitData[i]['HitPosY']
        errors = np.sqrt(deltaX**2+deltaY**2) 
        #Filter large errors (corresponding to ball disappearing)
        errors = (errors < 1000).astype(float) * errors
        mean_errors[int(i)] = errors.mean()
        #Only count response times which are successful
        success = errors < 130
        orders[int(i)] = splitOrders[i]
        responseTimes[int(i)] = splitData[i]['ResponseTime'][success].mean()
    return scores, mean_errors, responseTimes,orders

def plotThrowingData(folders):
    data = np.array([])
    for folder in folders:
        if len(data) == 0:
            data = getRawParticipantData(folder,throw_task_file)
        else:
            data = np.append(data,getRawParticipantData(folder,throw_task_file))

    splitData,splitOrders = split(data,0)

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
        rtest = np.square(deltaY) + np.square(deltaX) < 130**2
        test = np.logical_and(xtest, ytest)
        deltaFilteredX = deltaX[test]
        deltaFilteredY = -deltaY[test]
        plt.plot(deltaFilteredX,deltaFilteredY,markerMap(i),c=colourMap(i),ms=10,markeredgewidth=1,markeredgecolor='black')
        plt.plot(deltaFilteredX.mean(),deltaFilteredY.mean(),markerMap(i),c=colourMap(i),ms=20,markeredgewidth=1,markeredgecolor='black')
        legend_counts += [str(len(deltaFilteredX))]
    plt.legend(['Leap Motion (' + legend_counts[0] + '/' + str(len(splitData[0]['HitPosX'])) + ' valid throws)',
                'Perception Neuron (' + legend_counts[1] + '/' + str(len(splitData[1]['HitPosX'])) + ' valid throws)',
                'Fused Tracking (' + legend_counts[2] + '/' + str(len(splitData[2]['HitPosX'])) + ' valid throws)'])
    


def boxPlotColumns(data,data_subclasses=None):
    plt.figure()
    
    #X axis 
    x = [0,data.shape[1]]
    y = [0,0]
    plt.plot(x,y,'k',linewidth=1)

    bp = plt.boxplot(data)
    
    # Labels
    if(data.shape[1] == 6):
        x = [1,2,3,4,5,6]
        labels = ['Leap\nKeyboard', 'PN\nKeyboard', 'Leap\nSorting', 'PN\nSorting','Leap\nThrowing', 'PN\nThrowing']
        plt.xticks(x,labels)
    else:
        x = [1,2,3,4,5,6,7,8,9]
        labels = ['Leap\nKeyboard', 'PN\nKeyboard','Fused\nKeyboard', 'Leap\nSorting', 'PN\nSorting','Fused\nSorting','Leap\nThrowing', 'PN\nThrowing','Fused\nThrowing']
        plt.xticks(x,labels)

    #Colours
    colors = ["red","blue"]
    i = 0
    for patch in bp['boxes']:
        # patch.set(facecolor=colors[i%2])
        patch.set(color=colors[i%2])
        i+=1

    if(data_subclasses is None):        
        data_subclasses = np.zeros(data.shape)
    max_subclass = data_subclasses.max()
    min_subclass = data_subclasses.min()
    centre_subclass = (max_subclass - min_subclass)/ 0.5
    subclass_spacing = 1
    if(max_subclass!=min_subclass):
        subclass_spacing = 1/(max_subclass-min_subclass)
    subclass_width = 0.5
    # Scatter points
    for i in range(data.shape[0]):
        y = data[i,:]
        x = range(1,data.shape[1]+1) + ((data_subclasses[i,:] - min_subclass) * subclass_spacing - 0.5)*subclass_width  # np.random.normal(0, 0.0, size=len(y))
        plt.plot(x, y, 'ok', alpha=1)


def getParticipantSummaryStats(participant):
    #Button
    b_scores, b_errors, b_rtimes, b_orders = getParticipantDataButton(participant)
    #Sort
    s_scores, s_errors, s_rtimes, s_orders = getParticipantDataSort(participant)
    #Throw
    t_scores, t_errors, t_rtimes, t_orders = getParticipantDataThrow(participant)

    # Order which fused was attempted (1,2,3)
    delta_orders = np.array([[         b_orders[2]-b_orders[0],b_orders[2]-b_orders[1],
                                s_orders[2]-s_orders[0],s_orders[2]-s_orders[1],
                                t_orders[2]-t_orders[0],t_orders[2]-t_orders[1]]])

    improvements = np.array([[b_scores[2]-b_scores[0],b_scores[2]-b_scores[1],
                               s_scores[2]-s_scores[0],s_scores[2]-s_scores[1],
                               t_scores[2]-t_scores[0],t_scores[2]-t_scores[1]]])
    

    time_improvements = np.array([[b_rtimes[0]-b_rtimes[2],b_rtimes[1]-b_rtimes[2],
                                   s_rtimes[0]-s_rtimes[2],s_rtimes[1]-s_rtimes[2],
                                   t_rtimes[0]-t_rtimes[2],t_rtimes[1]-t_rtimes[2]]])
    
    error_improvements = np.array([[b_errors[0]-b_errors[2],b_errors[1]-b_errors[2],
                                    s_errors[0]-s_errors[2],s_errors[1]-s_errors[2],
                                    t_errors[0]-t_errors[2],t_errors[1]-t_errors[2]]])

    scores = np.array([np.append(b_scores,[s_scores,t_scores])])
    times = np.array([np.append(b_rtimes,[s_rtimes,t_rtimes])])
    errors = np.array([np.append(b_errors,[s_errors,t_errors])])
    orders = np.array([np.append(b_orders,[s_orders,t_orders])])

    return scores, times, errors,orders, improvements, time_improvements, error_improvements, delta_orders

#Pvalue 
# Probability that we would see such large mean if population mean was zero
def getPValueNormGT0(data):
    sigma = data.std(axis=0)
    mean = data.mean(axis=0)
    pval = 1 - scipy.stats.norm.cdf(mean,scale=sigma/np.sqrt(data.shape[0]))
    return pval

#TODO: fix this:
perms = [[0,1,2],[2,0,1],[1,2,0],[1,0,2],[2,1,0],[0,2,1]]
def techOrder(pID,taskID):
    p = pID-1
    #Every second participant gets the second half of permutations
    p_odd = p%2
    #3 perms one for each tech, but order of which one goes to which task is selected by task_perm
    tech_perms = perms[p_odd*3:p_odd*3+3]
    #task perm permutes tech_perms
    task_perm = perms[p%len(perms)]
    #The index for the permutation for this task
    tech_index = task_perm[taskID]
    #Return the actual perm
    return tech_perms[tech_index]

    
def getNumber(c):
    if(c == 'A'):
        return 0
    if(c == 'B'):
        return 1
    if(c == 'C'):
        return 2


def testTechOrders():
    for p in range(20):
        pID = p+1
        message = "P"+str(pID) + " "
        for taskID in range(3):
            message += stringFromTaskID(taskID) + " "
            #Task changes every three trials
            order = techOrder(pID,taskID)
            for i in order:
                message += stringFromTechID(i) + " "
        print message
# testTechOrders()

#Returns vector of preferences for 1st,2nd,3rd tasks
def parsePref(pref):
    rankings = [0,0,0]
    i = 0
    for c in pref.strip():
        n = getNumber(c)
        rankings[n] = i
        i+=1
    return rankings

def decodePreferences(participantIDs,preferences,task):
    # Preference counts for each tech
    taskID = taskIDFromString(task)

    # rankings 
    #           Tech1, Tech2, Tech3
    # 1st Count     0,     1,     1
    # 2nd Count
    # 3rd Count
    rank_counts = np.zeros([3,3])
    for i in range(len(preferences)):
        pID = participantIDs[i]
        pref = preferences[i]
        rankings = parsePref(pref)
        t_order = techOrder(pID,taskID)
        for j in range(len(rankings)):
            rank = rankings[j]
            rank_counts[t_order[j]][rank] += 1
    return rank_counts




def getResponseData(task):
    return genfromtxt("ParticipantResponses/"+task+"_responses.txt",
                      delimiter=",", 
                      comments="#", 
                      names=["Participant", "Quality", "Utility", "CommentsA", "CommentsB", "CommentsC", "GeneralComments"],
                      dtype=None)


keyboard_responses = getResponseData("keyboard")
sorting_responses = getResponseData("sorting")
throwing_responses = getResponseData("throwing")
print keyboard_responses
print sorting_responses
print throwing_responses
print (decodePreferences(keyboard_responses["Participant"],keyboard_responses["Quality"],"throwing") 
    + decodePreferences(sorting_responses["Participant"],sorting_responses["Quality"],"throwing")
    + decodePreferences(throwing_responses["Participant"],throwing_responses["Quality"],"throwing"))
prefsTotal = (decodePreferences(keyboard_responses["Participant"],keyboard_responses["Quality"],"throwing") 
    + decodePreferences(sorting_responses["Participant"],sorting_responses["Quality"],"throwing")
    + decodePreferences(throwing_responses["Participant"],throwing_responses["Quality"],"throwing"))

plt.figure()
ind = np.array(range(3))
width = 0.2
for i in range(prefsTotal.shape[0]):
    plt.bar(ind+i*width,prefsTotal[i],width,color=colourMap(i))
# plt.show()

def inversePermutation(p):
    p_inv = np.zeros(len(p))
    for i in range(len(p)):
        # print p[i]," -> ", i
        p_inv[p[i]] = i
    return p_inv

def checkOrders(orders,pID):
        # print "orders ",orders
    for taskID in range(3):
        # print techOrder(pID,taskID) 
        #inverse because techOrder returns which tech given the task, attempt
        # the actual order returns which order a given task was performed
        predicted_order = inversePermutation(techOrder(pID,taskID))
        actual_order = orders[taskID*3:taskID*3+3] - 1
        if (predicted_order != actual_order).any():
            print "Order wrong!!"
            print "predicted_order = ", predicted_order        
            print "actual_order = ", actual_order  
            raise ValueError("Something has gone wrong with the orders!")      


def performanceAnalysis():
    participants = [5,6,7,8,9,10,11,12,13]
    # improvements, time_improvements, error_improvements = np.array([]),np.array([]),np.array([])
    # scores, times, errors = np.array([]),np.array([]),np.array([])
    parNames = []
    first = True
    for p in participants:
        participantName = "Participant"+str(p)
        parNames += [participantName]
        s,T,E,o,i,t,e,do = getParticipantSummaryStats(participantName)
        checkOrders(o[0],p)
        if(first):
            improvements = i
            time_improvements = t
            error_improvements = e
            orders = o
            scores = s
            times = T
            errors = E
            deltaOrders = do
            first = False
        else:
            improvements = np.append(improvements,i,axis=0)
            time_improvements = np.append(time_improvements,t,axis=0)
            error_improvements = np.append(error_improvements,e,axis=0)
            scores = np.append(scores,s,axis=0)
            times = np.append(times,T,axis=0)
            errors = np.append(errors,E,axis=0)
            orders = np.append(orders,o,axis=0)
            deltaOrders = np.append(deltaOrders,do,axis=0)

    print "orders",orders
    plotThrowingData(parNames)
    plotThrowingData(["Participant13"])
    boxPlotColumns(improvements,deltaOrders)
    plt.title("Score Improvements")
    boxPlotColumns(time_improvements, deltaOrders)
    plt.title("Time Improvements")
    boxPlotColumns(error_improvements, deltaOrders)
    plt.title("Error Improvements")

    print scores, orders
    boxPlotColumns(scores, orders)
    plt.title("Raw Scores")
    boxPlotColumns(errors, orders)
    plt.title("Raw Errors")
    boxPlotColumns(times, orders)
    plt.title("Raw Times")

    plt.show()

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
performanceAnalysis()