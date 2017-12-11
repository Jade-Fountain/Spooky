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
		std::vector<Measurement::Ptr> measurement_buffer;

		//Class responsible for distinguishing ambiguous sensors
		Correlator correlator;

		//Calibration data per system pair (A,B) = std::pair<SystemDescriptor,SystemDescriptor>
		//sensorTransforms[(A,B)]: A -> B
		Calibrator calibrator;

		//Fused data
		ArticulatedModel skeleton;

		//Sensor list
		std::map<SystemDescriptor, std::map<SensorID, Sensor::Ptr>> sensors;
		
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

		class MesurementBuffer {
			int max_buffer_length = 1000;
			std::map<SystemDescriptor, std::vector<Measurement::Ptr>> measurements;
			void push_back(const Measurement::Ptr& m){
				//TODO: replace safeAccess with some kind of global check early on
				safeAccess(measurements, m->system).push_back(m);
				if(measurement[m->system].size() > max_buffer_length){
					measurement[m->system].erase(measurement[m->system].begin());
				}
			}

			const std::vector<Measurement::Ptr>& getNewSynchronizedMeasurements(){

			}
		};


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

		////////////////////////////////////////////////////
		//					Input at runtime
		////////////////////////////////////////////////////
		
		//Adds a new measurement to the system (unambiguous)
		void addMeasurement(const Measurement::Ptr& m, const NodeDescriptor& node);
		
		//Adds a new measurement to the system (ambiguous or unambiguous, less efficient)
		void addMeasurement(const Measurement::Ptr & m, const std::vector<NodeDescriptor>& nodes);
		
		//Computes data added since last fuse() call. Should be called repeatedly	
		void fuse();

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
		void setMeasurementSensorInfo(Measurement::Ptr& m, SystemDescriptor system, SensorID id);

		//Returns a string summarising the state of calibration in the system
		std::string getCalibratorStateSummary();
		//Returns info on compute time
		std::string getTimingSummary();

	};

}