# experiment_analysis.py

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
from numpy import genfromtxt
from covariance_plot import plot_point_cov

def plotData(folderName, N, point_color, ellipse_color):
	errors = np.zeros([N,2])
	for i in range(N):
		my_data = genfromtxt(folderName + '/' + str(i+1), delimiter=',')
		errors[i] = my_data.mean(axis=0)

	mean = errors.mean(axis=0)
	print errors
	print "Mean = ", errors.mean(axis=0)
	print "StdDev = ", errors.std(axis=0)
	print "Worst Individual= ", errors.max(axis=0)
	print "Best Individual= ", errors.min(axis=0)
	
	plot_point_cov(errors,color=ellipse_color,alpha=0.5)
	plt.plot(errors[:,0], errors[:,1], '.', color = point_color)
	plt.axis([0,30,0,8])
	plt.plot(mean[0],mean[1], 'rx',color = point_color)

def plotDataUnified(folderName, N, point_color, ellipse_color):
	errors = np.zeros([0,2])
	times = np.array([])
	for i in range(N):
		my_data = genfromtxt(folderName + '/' + str(i+1), delimiter=',')
		start = 0
		last_value = my_data[0,0]
		for j in range(my_data.shape[0]):
			change = np.linalg.norm(my_data[j,0] - last_value)
			if(change > 0.2 or j==my_data.shape[0]-1):
				errors = np.append(errors, [my_data[start:j,0:2].mean(axis=0)], axis=0)
				times = np.append(times, [my_data[start:j,2].min()],axis=0)
				start = j + 1
				last_value = my_data[j,0]

	mean = errors.mean(axis=0)
	print errors
	print "Mean Time = ", times.max(), times.mean(), times.min()
	print "Mean = ", errors.mean(axis=0)
	print "StdDev = ", errors.std(axis=0)
	print "Worst Individual= ", errors.max(axis=0)
	print "Best Individual= ", errors.min(axis=0)
	
	plot_point_cov(errors,color=ellipse_color,alpha=0.5)
	plt.plot(errors[:,0], errors[:,1], '.', color = point_color)
	
	plt.plot(mean[0],mean[1], 'x', color = point_color)


def plotDataSeparatedWithTime(folderName, N, point_color, ellipse_color):
	errors = np.zeros([0,2])
	times = np.array([])
	for i in range(N):
		my_data = genfromtxt(folderName + '/' + str(i+1), delimiter=',')
		start = 0
		last_value = my_data[0,0]
		for j in range(my_data.shape[0]):
			change = np.linalg.norm(my_data[j,0] - last_value)
			if(change > 0.2 or j==my_data.shape[0]-1):
				errors = np.append(errors, [my_data[start:j,1:3].mean(axis=0)], axis=0)
				times = np.append(times, [my_data[start:j,3].min()],axis=0)
				start = j + 1
				last_value = my_data[j,0]

	mean = errors.mean(axis=0)
	print errors
	print "Mean Time = ", times.max(), times.mean(), times.min()
	print "Mean = ", errors.mean(axis=0)
	print "StdDev = ", errors.std(axis=0)
	print "Worst Individual= ", errors.max(axis=0)
	print "Best Individual= ", errors.min(axis=0)
	
	plot_point_cov(errors,color=ellipse_color,alpha=0.5)
	plt.plot(errors[:,0], errors[:,1], '.', color = point_color)
	
	plt.plot(mean[0],mean[1], 'x', color = point_color)


# plotData('CustomMovementVive1',10, point_color = 'red', ellipse_color=(0.9,0.5,0.5))
# plotDataUnified('CustomMovementRift1',1, point_color = 'blue', ellipse_color=(0.5,0.5,0.9))
# plt.axis([0,40,0,10])
# plt.title('Custom Movement Rift vs. Vive')

# plt.figure()
# plotDataUnified('CustomMovementLongGripRift1',1, point_color = 'red', ellipse_color=(0.9,0.5,0.5))
# plotDataUnified('CustomMovementLongGripRiftLitMethod1',1, point_color = 'blue', ellipse_color=(0.5,0.5,0.9))
# plt.axis([0,40,0,10])
# plt.title('Long Grip Mine vs. Lit')


# plt.figure()
# plotDataUnified('CustomMovementRift1',1, point_color = 'red', ellipse_color=(0.9,0.5,0.5))
# plotDataUnified('CustomMovementRiftLitMethod1',1, point_color = 'blue', ellipse_color=(0.5,0.5,0.9))
# plt.axis([0,40,0,10])
# plt.title('Rift Mine vs. Lit')



#Problems: 
# -times are missing for Shooting
# - only right hand and head was used
plt.figure()
plotDataSeparatedWithTime('Experiment2/Rift/FreeMovement',N=1, point_color = 'red', ellipse_color=(0.9,0.5,0.5))
plotDataSeparatedWithTime('Experiment2/Rift/Sorting',1, point_color = 'green', ellipse_color=(0.5,0.9,0.5))
plotDataSeparatedWithTime('Experiment2/Rift/Shooting',1, point_color = 'blue', ellipse_color=(0.5,0.5,0.9))
plt.axis([0,50,0,10])
plt.title('Rift ')

plt.figure()
plotDataSeparatedWithTime('Experiment3/Vive/FreeMovement',N=1, point_color = 'red', ellipse_color=(0.9,0.5,0.5))
plotDataSeparatedWithTime('Experiment3/Vive/Sorting',1, point_color = 'green', ellipse_color=(0.5,0.9,0.5))
plotDataSeparatedWithTime('Experiment3/Vive/Shooting',1, point_color = 'blue', ellipse_color=(0.5,0.5,0.9))
plt.axis([0,50,0,10])
plt.title('Vive ')

plt.show()

# plotDataUnified('CustomMovementViveLitMethod1',1, point_spec = 'k.', ellipse_color='grey')
# plotDataUnified('SortingTaskVive1',1, point_spec = 'k.', ellipse_color='grey')

