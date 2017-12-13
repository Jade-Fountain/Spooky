/*  This file is part of UnrealFusion, a sensor fusion plugin for VR in the Unreal Engine
    Copyright (C) 2017 Jake Fountain
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Spooky.h"
#include "Core.h"
#include <chrono>

namespace spooky {

	void Core::addBoneNode(const NodeDescriptor & node, const NodeDescriptor & parent, const Transform3D& boneTransform)
	{
		skeleton.addNode(node, parent);
		skeleton.setBoneForNode(node, boneTransform);
	}

	void Core::addPoseNode(const NodeDescriptor & node, const NodeDescriptor & parent, const Transform3D& poseInitial)
	{
		skeleton.addNode(node, parent);
		skeleton.setPoseNode(node, poseInitial);
	}

	void Core::addScalePoseNode(const NodeDescriptor & node, const NodeDescriptor & parent, const Transform3D& poseInitial, const Eigen::Vector3f& scaleInitial)
	{
		skeleton.addNode(node, parent);
		skeleton.setScalePoseNode(node, poseInitial, scaleInitial);
	}


	void Core::setReferenceSystem(const SystemDescriptor & system) {
		skeleton.setReferenceSystem(system);
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
		//TODO: add ifdefs for profiling
		//Add new data to calibration, with checking for usefulness
		utility::profiler.startTimer("Correlator");
		utility::profiler.startTimer("All");
		//SPOOKY_LOG("Fusing: " + std::to_string(measurement_buffer.size()) + "measurements");

		//Get measurements offset by the largest latency so all measurements are valid
		std::vector<Measurement::Ptr> sync_measurements = measurement_buffer.getOffsetSynchronizedMeasurements(time);
		correlator.addMeasurementGroup(sync_measurements);
		correlator.identify();
		utility::profiler.endTimer("Correlator");
		if(correlator.isStable() || true){
			utility::profiler.startTimer("Calibrator add");
			calibrator.addMeasurementGroup(sync_measurements);
			utility::profiler.endTimer("Calibrator add");
			utility::profiler.startTimer("Calibrate");
			calibrator.calibrate();
			utility::profiler.endTimer("Calibrate");
			if(calibrator.isStable() || true){
				utility::profiler.startTimer("Fuse");
				skeleton.addMeasurementGroup(measurement_buffer.getMeasurements(time));
				skeleton.fuse(calibrator);
				utility::profiler.endTimer("Fuse");
			}
		}	
		measurement_buffer.clearLast();
		//TODO: do this less often
		utility::profiler.endTimer("All");
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
	void Core::setMeasurementSensorInfo(Measurement::Ptr & m, SystemDescriptor system, SensorID id)
	{
		//If we haven't seen this sensor already, initialise
		if (utility::safeAccess(sensors, system).count(id) == 0) {
			utility::safeAccess(sensors, system)[id] = std::make_unique<Sensor>();
			utility::safeAccess(sensors, system)[id]->system = system;
			utility::safeAccess(sensors, system)[id]->id = id;
		}
		//Set pointer in measurement
		m->setSensor(sensors[system][id]);
	}

	std::string Core::getCalibratorStateSummary() {
		return calibrator.getStateSummary();
	}

	std::string Core::getTimingSummary() {
		return utility::profiler.getReport();
	}
}
