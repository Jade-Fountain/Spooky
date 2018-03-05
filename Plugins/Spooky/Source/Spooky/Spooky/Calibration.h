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
#include <vector>
#include <map>
#include <set>
#include "Eigen/Core"
#include "FusionTypes.h"
#include "Utilities/DataStructures.h"
#include "Utilities/CalibrationUtilities.h"

namespace spooky {

	//Encapsulation for storing measurements	
	class CalibrationDataSet {
	public:

		//Encapsulation for accessing measurements corresponding to 
		class Stream {
		public:
			//Upper bound for number of samples. 
			//Ideally shouldnt reach this, but included as safety
			int max_samples = 1000;
			//Stores sensor samples per ID
			std::map<SensorID, utility::MultiUseStream<Measurement::Ptr,std::string>> sensors;
			
			//Adds a measurement to the stream
			void addMeasurement(const Measurement::Ptr& m);
			//Gets total sensor measurement count for this node and sensor
			int totalCount(const SystemDescriptor& system1, const SystemDescriptor& system2);
			//Counts measurements independent of their use
			int rawCount();
			//Add job to streams
			void addCalibrationJob(const SystemDescriptor& system1, const SystemDescriptor& system2);

		};


		//Stores the data for each System and each node which has sensors from that system
		//Picture it as a table of sensor streams with Systems naming the rows and Nodes naming the columns
		//Example:
		//			N1	N2	N3		...
		//		S1	s1	s2	s3,s4
		//		S2	r1	-	-
		//		S3	-	q1	-
		//
		std::map<SystemNodePair, Stream> systemNodeTable;
		//Row labels:
		std::set<SystemDescriptor> systems;
		//Column labels:
		std::set<NodeDescriptor> nodes;

		//Helper methods
		void addMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node);
		float compareMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node);
		std::string getStateSummary();
		


	};


	class Calibrator {
	private:
		//----------------
		//PRIVATE MEMBERS
		//----------------
		bool fault_detection_disabled = false;


		//Table for looking up data relevant to determining transforms
		CalibrationDataSet calibrationSet;

		//Data for resulting calibrations
		std::map<SystemPair, CalibrationResult> calibrationResults;
		
		//----------------
		//PRIVATE METHODS
		//----------------

		//Checks there is data corresponding to more than one system for a given node in a measurement group
		std::vector<Measurement::Ptr> filterLonelyData(const std::vector<Measurement::Ptr>& measurementQueue);

		//Returns true if sufficient movement has occurred to warrant recording of data
		std::vector<Measurement::Ptr> filterChanges(const std::vector<Measurement::Ptr>& measurements);

		//Calibrate two systems with respect to one another
		void calibrateSystems(SystemDescriptor system1, SystemDescriptor system2);

		//Gets the measurements relevant to calibration of system1 and system2
		//Returns an empty list if calibration not ready yet
		void getRelevantMeasurements(SystemDescriptor system1,
									SystemDescriptor system2, 
									std::vector<Measurement::Ptr>* measurements1, 
									std::vector<Measurement::Ptr>* measurements2, 
									int minCountPerNode,
									bool clearMeasurementsWhenDone = true);

		//Counts the number of measurements that will be returned by getRelevantMeasurements
		std::pair<int, int> Calibrator::countRelevantSynchronisedMeasurements(SystemDescriptor system1,
																				SystemDescriptor system2,
																				int minCountPerNode);		
		
		//Determines what sensors are available to perform calibrations
		void Calibrator::determineCalibrationsRequired(SystemDescriptor system1,
																	SystemDescriptor system2,
																	int minCountPerNode);

		//Calibrate two particular data streams
		CalibrationResult calibrateStreams(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult& calib);
		
		//----------------
		//CALIBRATION METHODS

		//Different types of calibration procedures:
		//see CalibrationProcedures.cpp for definitions
		//----------------

		//Update state of calibration, filtering over time
		CalibrationResult updateCalibration(const CalibrationResult& newCalibration,const CalibrationResult& currentCalibration) const;

		//Calibrate two correlated positional measurements
		CalibrationResult calPos(const std::vector<Measurement::Ptr>& measurements1, const std::vector<Measurement::Ptr>& measurements2, const CalibrationResult& calib) const;

		//Calibrate two rigidly linked 6DoF sensors
		CalibrationResult cal6DoF(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult & currentCalibration, const bool& includePosition) const;

		//Estimate latencies of multiple concatenated streams of measurements
		float estimateLatencies(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2);

		//Returns the estimated latency l between two streams: m2[t] <-> m1[t+l]
		//Aka, m2 lags begind by l
		float estimateLatency(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2);

	public:
		//Config		
		struct Config{
			//Difference threshold: store new measurement if difference to last measurement is larger than this
			float diff_threshold = 0.075f;
			//TODO: change diff threshold for different calibration stages
			//{
			//	{ CalibrationResult::State::UNCALIBRATED, 0.1 },
			//	{ CalibrationResult::State::REFINING, 0.5 },
			//	{ CalibrationResult::State::CALIBRATED, 0.5 }
			//};

			//Smallest allowed count for a single node before it can be included in calibration
			//Hard minimum = 4
			int min_count_per_node = 4; //>=4

			//Count Threshold: Calibrate when this many samples acquired
			std::map<CalibrationResult::State, int> count_threshold = 
			{	
				{CalibrationResult::State::UNCALIBRATED,100},
				{CalibrationResult::State::REFINING,100 },
				{CalibrationResult::State::CALIBRATED,100}
			};			

			//CalibrationProcedures parameters
			// CalibrationResult Calibrator::updateCalibration(const CalibrationResult& newCalibration, const CalibrationResult& currentCalibration) const{
			//TODO: make these some logical values
			//TODO: make this different for each system pair
			float initial_quality_threshold = 0.5;
			float quality_convergence_threshold = 0.01;
			float fault_hysteresis_rate = 0.25;
			float relevance_decay_rate = 0.1;
			float settle_threshold = 0.90;
			float fault_angle_threshold = M_PI * 5 / 180;
			float fault_distance_threshold = 0.1;
		} config;

		//Set config method
		void configure(const Config& cfg){config=cfg;}

		//Add data for later calibration
		void addMeasurement(const Measurement::Ptr& m);
		void addMeasurementGroup(const std::vector<Measurement::Ptr>& measurementQueue);

		//Calibrate - integrates data and calibrates if possible
		void calibrate();

		//Returns true if useable data is now available
		bool isStable();
		
		//Sets calibration result from external data
		void setResults(const CalibrationResult & r);

		//Searches for calibration results and returns them for use in fusion
		CalibrationResult getResultsFor(SystemDescriptor s1, SystemDescriptor s2) const;

		//Gets string summarising state of calibrator
		std::string getStateSummary();

	};

}
