/*  This file is part of Spooky, a sensor fusion plugin for VR in the Unreal Engine
    
   Copyright 2017 Jake Fountain

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include "Spooky.h"
#include "Core.h"
#include <chrono>

namespace spooky {
	Core::SpookyConfig Core::config;

	Core::Core(){
		calibrator.getNodeGlobalPose = std::bind(&Core::getNodeGlobalPose, this, std::placeholders::_1);
		calibrator.getNodeLocalPose = std::bind(&Core::getNodeLocalPose, this, std::placeholders::_1);
	}
	
	void Core::addFixedNode(const NodeDescriptor & node, const NodeDescriptor & parent,
							 const Transform3D& pose)
	{
		skeleton.addNode(node, parent);
		skeleton.setFixedNode(node, pose);
	}

	void Core::addBoneNode(const NodeDescriptor & node, const NodeDescriptor & parent,
							 const Transform3D& boneTransform,
							 const Eigen::VectorXf& constraint_centre, const Eigen::MatrixXf& constraint_variance,
							 const Eigen::MatrixXf& process_noise, const bool& modelVelocity)
	{
		skeleton.addNode(node, parent);
		skeleton.setBoneForNode(node, boneTransform, Node::State::Parameters(constraint_centre, constraint_variance), process_noise, modelVelocity);
	}

	void Core::addPoseNode(const NodeDescriptor & node, const NodeDescriptor & parent,
							 const Transform3D& poseInitial,
							 const Eigen::VectorXf& constraint_centre, const Eigen::MatrixXf& constraint_variance,
							 const Eigen::MatrixXf& process_noise, const bool& modelVelocity)
	{
		skeleton.addNode(node, parent);
		skeleton.setPoseNode(node, poseInitial, Node::State::Parameters(constraint_centre, constraint_variance), process_noise, modelVelocity);
	}

	void Core::addScalePoseNode(const NodeDescriptor & node, const NodeDescriptor & parent,
							 const Transform3D& poseInitial,
							 const Eigen::VectorXf& constraint_centre, const Eigen::MatrixXf& constraint_variance,
							 const Eigen::MatrixXf& process_noise, const bool& modelVelocity)
	{
		skeleton.addNode(node, parent);
		skeleton.setScalePoseNode(node, poseInitial, Node::State::Parameters(constraint_centre, constraint_variance),process_noise, modelVelocity);
	}


	void Core::setReferenceSystem(const SystemDescriptor & system) {
		skeleton.setReferenceSystem(system);
	}

	void Core::initSensor(const SystemDescriptor& system, const int& sensorID){
		//If we haven't seen this sensor already, initialise
		if (utility::safeAccess(sensors, system).count(sensorID) == 0) {
			sensors[system][sensorID] = std::make_unique<Sensor>();
			sensors[system][sensorID]->system = system;
			sensors[system][sensorID]->id = sensorID;
			//If we have system latencies for this system, use those and override with individual sensor latencies
			sensors[system][sensorID]->latency = sysLatencies.count(system) == 0 ? 0 : sysLatencies[system];
			sensors[system][sensorID]->rootNode = rootNodes.count(system) == 0 ? NodeDescriptor("") : rootNodes[system];

		}
	}

	void Core::setSensorLatency(const SystemDescriptor& system, const int& sensorID, const float& latency){
		initSensor(system,sensorID);
		utility::safeAccess(sensors, system)[sensorID]->latency = latency;
	}

	void Core::setSystemLatency(const SystemDescriptor& system, const float& latency){
		//Set existing sensors
		for(auto& sensor : utility::safeAccess(sensors,system)){
			sensor.second->latency = latency;
		}
		//Record for future sensors 
		sysLatencies[system] = latency;
	}
	
	void Core::setJointStiffness(const float& stiffness) {
		skeleton.setAllJointStiffness(stiffness);
	}
	
	void Core::setSystemRootNode(const SystemDescriptor& system, const NodeDescriptor& node){
		//Set existing sensors
		for(auto& sensor : utility::safeAccess(sensors,system)){
			sensor.second->rootNode = node;
		}
		//Record for future sensors 
		rootNodes[system] = node;
	}



	// =================
	//Saving and loading
	// =================
	void Core::setSaveDirectory(const std::string& dir) {
		saveManager.setWorkingDirectory(dir);
	}

	void Core::loadCalibration(const SystemDescriptor& s1, const SystemDescriptor& s2, bool tryReverse) {
		CalibrationResult cal(s1, s2);
		//TODO: if fail, try reverse order
		bool success = saveManager.load(&cal);
		if (success) {
			calibrator.setResults(cal);
			//SPOOKY_LOG("Loaded Calibration[" + s1.name + ", " + s2.name + "] SUCCESSFULLY");
		}
		else {
			SPOOKY_LOG("Loading Calibration[" + s1.name + ", " + s2.name + "] FAILED");
			if(tryReverse){
				loadCalibration(s2,s1,false);
			}
		}
	}
	
	void Core::saveCalibration(const SystemDescriptor& s1, const SystemDescriptor& s2) {
		bool success = saveManager.save(calibrator.getResultsFor(s1, s2));
		if (success) {
			//SPOOKY_LOG("Saved Calibration[" + s1.name + ", " + s2.name + "] SUCCESSFULLY");
		}
		else {
			SPOOKY_LOG("Saving Calibration[" + s1.name + ", " + s2.name + "] FAILED");
		}
	}


	// =================
	//END: Saving and loading
	// =================
	void Core::finaliseSetup()
	{
		skeleton.enumerateHeirarchy();
	}
	//Adds a new measurement to the system
	void Core::addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node) {
		m->getSensor()->addNode(node);
		skeleton.addGenericNode(node);
		measurement_buffer.push_back(m);
	}

	//Adds a new measurement to the system
	void Core::addMeasurement(const Measurement::Ptr& m, const std::vector<NodeDescriptor>& nodes) {
		//Add nodes which the measurement might correspond to - actually gets stored in the sensor pointer
		for(auto& n : nodes){
			m->getSensor()->addNode(n);
			skeleton.addGenericNode(n);
		}
		measurement_buffer.push_back(m);
	}

	//Computes data added since last fuse() call. Should be called repeatedly	
	void Core::fuse(const float& time) {
		utility::profiler.startTimer("CoreMainLoop");

		frame_count++;
		last_time = time;
		//TODO: add ifdefs for profiling

		//Get measurements offset by the largest latency so all measurements are valid
		utility::profiler.startTimer("Sync (Offset)");
		std::vector<Measurement::Ptr> sync_measurements = measurement_buffer.getOffsetSynchronizedMeasurements(time);
		utility::profiler.endTimer("Sync (Offset)");

		///////////////////////
		// FUSE
		///////////////////////
		utility::profiler.startTimer("Fuse");
		auto lastMeasurements = measurement_buffer.getLatestMeasurements();
		skeleton.addMeasurementGroup(lastMeasurements);

		//DEBUG FOR LATENCY TESTING:
		//skeleton.addMeasurementGroup(sync_measurements);
		skeleton.fuse(calibrator);
		utility::profiler.endTimer("Fuse");
		///////////////////////
		

		for(auto& m : sync_measurements){
			NodeDescriptor rootNode = m->getSensor()->getRootNode();
			if(rootNode.name.size() != 0){
				if(rootPoses.count(rootNode) == 0){
					rootPoses[rootNode] = DataBuffer<Transform3D>();
				}
				rootPoses[rootNode].time_width = std::fmax(m->getLatency(),rootPoses[rootNode].time_width);
				//Insert current pose of root node based on 
				rootPoses[rootNode].insert(skeleton.getNodeLastFusionTime(rootNode), getNodeGlobalPose(rootNode), time);
				//Search back in time for 
				bool valid = false;
				Transform3D p = rootPoses[rootNode].get(m->getTimestamp() - m->getLatency(), &valid);
				if(valid){
					m->setRootNodePose(p);
				}
			}
		}


		///////////////////////
		// CORRELATE
		///////////////////////
		utility::profiler.startTimer("Correlator");
		correlator.addMeasurementGroup(sync_measurements);
		correlator.identify();
		utility::profiler.endTimer("Correlator");
		///////////////////////
		

		///////////////////////
		// CALIBRATE
		///////////////////////
		utility::profiler.startTimer("Calibrator add");
		calibrator.addMeasurementGroup(sync_measurements);
		utility::profiler.endTimer("Calibrator add");

		utility::profiler.startTimer("Calibrate");
		calibrator.calibrate();
		utility::profiler.endTimer("Calibrate");
		///////////////////////

	
		utility::profiler.endTimer("CoreMainLoop");
	}

	CalibrationResult Core::getCalibrationResult(SystemDescriptor s1, SystemDescriptor s2) {
		return calibrator.getResultsFor(s1, s2);
	}

	Transform3D Core::getNodeGlobalPose(const NodeDescriptor& node){
		return skeleton.getNodeGlobalPose(node);
	}

	Transform3D Core::getNodeLocalPose(const NodeDescriptor & node)
	{
		return skeleton.getNodeLocalPose(node);
	}

	NodeDescriptor Core::getCorrelationResult(SystemDescriptor system, SensorID id) {
		if (sensors.count(system) > 0 &&
			sensors[system].count(id) > 0)
		{
			return sensors[system][id]->getNode();
		}
		else {
			return "UNKNOWN";
		}
	}

	//Called by owner of the Core object
	void Core::setMeasurementSensorInfo(Measurement::Ptr & m, const SystemDescriptor& system, const SensorID& id)
	{
		initSensor(system,id);
		//Set pointer in measurement
		m->setSensor(sensors[system][id]);
	}

	std::string Core::getCalibratorStateSummary() {
		return calibrator.getStateSummary();
	}

	std::string Core::getTimingSummary() {
		return utility::profiler.getReport() + "\n framerate = "  + std::to_string(frame_count / last_time);
	}

	ArticulatedModel& Core::getSkeleton() {
		return skeleton;
	}
}
