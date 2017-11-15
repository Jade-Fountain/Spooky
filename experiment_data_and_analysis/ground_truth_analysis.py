import json
import numpy as np
import matplotlib.pyplot as plt

def identifyMarkers(p):
	dp = []
	for i in range(len(p)):
		dp = dp + [np.linalg.norm(p[(i+1) % len(p)] - p[i])]
	upper = 0
	upper_i = 0
	lower = 100000
	lower_i = 0
	for i in range(len(dp)):
		if(dp[i] < lower):
			lower = dp[i]
			lower_i = i
		if(dp[i] > upper):
			upper = dp[i]
			upper_i = i

	middle_i = 0
	for i in range(len(dp)):
		if(i != upper_i and i != lower_i):
			middle_i = i
	return [upper_i, middle_i, lower_i]


def getFusionKitData(file_name):
	file = open(file_name)

	json_string = file.read()
	file.close()

	json_data = json.loads(json_string)

	data = np.zeros([len(json_data), 9])
	timestamps = np.zeros(len(json_data))
	t_0 = json_data[0]["timestamp"]

	for i in range(len(json_data)):
		frame = json_data[i]
		if(len(frame["Markers"]) == 3):
			p = [np.zeros(3),np.zeros(3),np.zeros(3)]
			p[0] = np.array((frame["Markers"][0]["Position"]["X"], frame["Markers"][0]["Position"]["Y"], frame["Markers"][0]["Position"]["Z"]))
			p[1] = np.array((frame["Markers"][1]["Position"]["X"], frame["Markers"][1]["Position"]["Y"], frame["Markers"][1]["Position"]["Z"]))
			p[2] = np.array((frame["Markers"][2]["Position"]["X"], frame["Markers"][2]["Position"]["Y"], frame["Markers"][2]["Position"]["Z"]))

			#todo: add identification
			identity = identifyMarkers(p)

			for j in range(3):
				data[i,3*j:3*j+3] = p[identity[j]]
			timestamps[i] = (frame["timestamp"] - t_0)/ 1e7
		else:
			timestamps[i] = (frame["timestamp"] - t_0) / 1e7
			data[i, :] = float('nan') * np.zeros(9)
			# print "Bad Frame"
		# print i,np.isnan(data[i, 0]),frame["timestamp"], t_0, timestamps[i]

		# print i,data[i]
	return data, timestamps

def getOptitrackData(file_name):
	seconds_per_frame = 1 / 120.0
	optitrack_data_raw = np.genfromtxt(file_name, delimiter=',')
	optitrack_data = np.array([[0,0,0,0,0,0,0,0,0]])
	timestamps = np.zeros(optitrack_data_raw.shape[0])
	for j in range(len(optitrack_data_raw)):
		p = optitrack_data_raw[j]
		p_ = []
		for i in range(3):
			p_ += [p[3*i:3*i+3]]
		ids = identifyMarkers(p_)
		new_p = np.append(p_[ids[0]], [p_[ids[1]], p_[ids[2]]])
		optitrack_data = np.append(optitrack_data, [new_p], axis = 0)
		timestamps[j] = j * seconds_per_frame
		# print p, ids, new_p, p_[ids[0]], p_[ids[1]], p_[ids[2]]
	# print optitrack_data
	return optitrack_data[1:], timestamps

def getVelocity(x, t):
	v = np.zeros([x.shape[0],x.shape[1]/3])
	print "v.shape = ", v.shape
	for i in range(len(x)-1):
		for j in range(3):
			v[i, j] = np.linalg.norm(x[i+1][3*j:3*j+3] - x[i][3*j:3*j+3]) / (t[i+1] - t[i])
	return v

fkdata, fkt = getFusionKitData('calib_attempt1.fkmcp')
opdata, opt = getOptitrackData("optitrack_data.csv")

velfk = getVelocity(fkdata,fkt)
velop = getVelocity(opdata,opt)

print opdata
print opt
plt.plot(fkt,velfk[:,1])
plt.plot(opt,velop[:,1])
plt.show()




