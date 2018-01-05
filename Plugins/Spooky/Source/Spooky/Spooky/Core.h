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
#pragma once

#include <string>
#include <map>
#include "Eigen/Core"
#include "FusionTypes.h"
#include "Calibration.h"
#include "Correlator.h"
#include "ArticulatedModel.h"
#include "Utilities/DataStructures.h"
#include "Utilities/TimeProfiling.h"
#include "SaveManager.h"
namespace spooky {


	//Centre of a fusion plant linking all skeleton elements
	class Core{
	private:
		//Object responsible for saving and loading calibrations
		SaveManager saveManager;

		//Measurement buffer for this frame
		MeasurementBuffer measurement_buffer;

		//Class responsible for distinguishing ambiguous sensors
		Correlator correlator;

		//Calibration data per system pair (A,B) = std::pair<SystemDescriptor,SystemDescriptor>
		//sensorTransforms[(A,B)]: A -> B
		Calibrator calibrator;

		//Fused data
		ArticulatedModel skeleton;

		//Sensor list
		std::map<SystemDescriptor, std::map<SensorID, Sensor::Ptr>> sensors;

		//Latency data per system
		std::map<SystemDescriptor, float> sysLatencies;
		
	public:

		struct SpookyConfig{
			//Units config
			struct {
				float input_m = 1;
				float output_m = 1;
			} units;
			//Correlator config
			Correlator::Config correlator;
			//Calibrator config
			Calibrator::Config calibrator;
		} config;



		////////////////////////////////////////////////////
		//					Initialisation
		////////////////////////////////////////////////////

		//Adds a node to the fusion graph model
		void addBoneNode(const NodeDescriptor& node, const NodeDescriptor& parent, const Transform3D& boneTransform);
		void addPoseNode(const NodeDescriptor& node, const NodeDescriptor& parent, const Transform3D& poseInitial);
		void addScalePoseNode(const NodeDescriptor& node, const NodeDescriptor& parent, const Transform3D& poseInitial, const Eigen::Vector3f& scaleInitial);
		
		//Sets the reference system for the fused skeleton
		//Joint values will be reported relative to this system
		void setReferenceSystem(const SystemDescriptor & system);

		//Sets saveManager's save directory where calibration results will be saved and loaded
		void setSaveDirectory(const std::string & dir);

		//Save and load calibration results
		void loadCalibration(const SystemDescriptor & s1, const SystemDescriptor & s2, bool tryReverse = true);
		void saveCalibration(const SystemDescriptor & s1, const SystemDescriptor & s2);

		//Computes necessary metadata after setup
		void finaliseSetup();
		
		//If we haven't seen this sensor, add it to list
		void initSensor(const SystemDescriptor& system, const int& sensorID);

		//Latency config
		void setSensorLatency(const SystemDescriptor& system, const int& sensorID, const float& latency);
		void setSystemLatency(const SystemDescriptor& system, const float& latency);

		////////////////////////////////////////////////////
		//					Input at runtime
		////////////////////////////////////////////////////
		
		//Adds a new measurement to the system (unambiguous)
		void addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node);
		
		//Adds a new measurement to the system (ambiguous or unambiguous, less efficient)
		void addMeasurement(const Measurement::Ptr & m, const std::vector<NodeDescriptor>& nodes);
		
		//Computes data added since last fuse() call. Should be called repeatedly	
		void fuse(const float& time);

		////////////////////////////////////////////////////
		//					Results
		////////////////////////////////////////////////////

		//Returns the global pose of node
		Transform3D getNodeGlobalPose(const NodeDescriptor& node);
		
		//Returns the local orientation of a node
		Transform3D getNodeLocalPose(const NodeDescriptor& node);

		//Returns mapping from s1 to s2
		CalibrationResult getCalibrationResult(SystemDescriptor s1, SystemDescriptor s2);

		//Returns most likely node location of the given sensor
		NodeDescriptor getCorrelationResult(SystemDescriptor system, SensorID id);

		//Called by owner of the Core object to set a measurement sensor pointer
		void setMeasurementSensorInfo(Measurement::Ptr& m, const SystemDescriptor& system, const SensorID& id);

		//Returns a string summarising the state of calibration in the system
		std::string getCalibratorStateSummary();
		//Returns info on compute time
		std::string getTimingSummary();

	};

}