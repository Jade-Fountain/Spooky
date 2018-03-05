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
#include "Calibration.h"
#include "Logging.h"
#include "Utilities/Conventions.h"
#include "Utilities/TimeProfiling.h"
#include <math.h>

namespace spooky {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									CalibrationDataSet
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	//-------------------------------------------------------------------------------------------------------
	//									Stream
	//-------------------------------------------------------------------------------------------------------

	void CalibrationDataSet::Stream::addMeasurement(const Measurement::Ptr& m) {
		utility::safeAccess(sensors, m->getSensorID()).push_back(m);
		while (sensors[m->getSensorID()].raw_size() > max_samples) {
			//Erase oldest data
			sensors[m->getSensorID()].eraseFront();
		}
	}


	int CalibrationDataSet::Stream::totalCount(const SystemDescriptor& system1, const SystemDescriptor& system2)
	{
		int result = 0;
		for (auto& sensor : sensors) {
			result += sensor.second.size(system1.name + system2.name);
		}
		return result;
	}


	int CalibrationDataSet::Stream::rawCount()
	{
		int result = 0;
		for (auto& sensor : sensors) {
			result += sensor.second.raw_size();
		}
		return result;
	}

	void CalibrationDataSet::Stream::addCalibrationJob(const SystemDescriptor& system1, const SystemDescriptor& system2){
		for(auto& sensor : sensors){
			sensor.second.addInitialCounter(system1.name + system2.name);
		}
	}


	//-------------------------------------------------------------------------------------------------------
	//									CalibrationDataSet Members
	//-------------------------------------------------------------------------------------------------------


	void CalibrationDataSet::addMeasurement(const Measurement::Ptr& m, const SystemDescriptor& system, const NodeDescriptor& node) {
		SystemNodePair sysNode = SystemNodePair(system, node);
		systemNodeTable[sysNode].addMeasurement(m);
		//Store info for later
		systems.insert(system);
		nodes.insert(node);
	}

	float CalibrationDataSet::compareMeasurement(const Measurement::Ptr & m, const SystemDescriptor & system, const NodeDescriptor & node)
	{
		SystemNodePair sysNode = SystemNodePair(system, node);
		//utility::MultiStream<Measurement::Ptr,std::string>&
		auto& stream = utility::safeAccess(systemNodeTable[sysNode].sensors, m->getSensorID());
		//If no previous recorded data, return max difference
		if (stream.raw_size() == 0) {
			return float(std::numeric_limits<float>::max());
		}
		//Otherwise compare the measurements
		return stream.back()->compare(m);
	}

	std::string CalibrationDataSet::getStateSummary() {
		std::stringstream ss;
		std::string spacer = "                     ";
		auto spaceString = [spacer](const std::string& s){
			std::string new_s = s + spacer.substr(0, std::max(int(spacer.size() - s.size()), 0)) + "|";
			return new_s;
		};
		//Initial newline and gap for columns
		ss << spacer << "|";
		//Column labels
		for (auto & sys : systems) {
			ss << spaceString(sys.name);
		}
		ss << std::endl;
		//Rows
		for (auto & node : nodes) {
			//Node name
			ss << spaceString(node.name);
			//For each col
			for (auto & sys : systems) {
				SystemNodePair sysNode(sys, node);
				//Create entry containing each sensor count
				std::stringstream entry;
				for (auto& sensor : systemNodeTable[sysNode].sensors) {
					entry << sensor.first << ":" << sensor.second.raw_size();
					for (auto& i : sensor.second.sizes()) {
						entry << "(" << i << ")";
					}
				}
				ss << spaceString(entry.str());
			}
			ss << std::endl;
		}
		ss << "Key: (x:n) = sensor x has n stored measurements" << std::endl;
		return ss.str();
	}



	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Calibrator:Private
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::vector<Measurement::Ptr>
	Calibrator::filterLonelyData(const std::vector<Measurement::Ptr>& measurementQueue) {
		//Init result
		std::vector<Measurement::Ptr> result;
		//Structure for counting systems per node
		utility::SafeMap<NodeDescriptor, std::set<SystemDescriptor>> systemsPerNode;
		//utility::profiler.startTimer("Calibration: Filter - Count");

		//Count
		for (auto& m : measurementQueue) {
			systemsPerNode[m->getNode()].insert(m->getSystem());
		}
		//utility::profiler.endTimer("Calibration: Filter - Count");
		//Push back relevant measurments
		//utility::profiler.startTimer("Calibration: Filter - Pushback");
		for (auto& m : measurementQueue) {
			if (systemsPerNode[m->getNode()].size() > 1) {
				result.push_back(m);
			}
		}
		//utility::profiler.endTimer("Calibration: Filter - Pushback");
		return result;
	}

	std::vector<Measurement::Ptr> Calibrator::filterChanges(const std::vector<Measurement::Ptr>& measurements) {
		//TODO: need to check that measurements aren't missing to:
		//	-Store sensors for each node. or sensors for each node. 

		std::vector<Measurement::Ptr> results;
		for (auto& mes : measurements) {
			NodeDescriptor node = mes->getNode();
			float diff = calibrationSet.compareMeasurement(mes, mes->getSystem(), node);

			//Result is true if all sensors on the given node exceed the threshold
			bool changed = (diff > config.diff_threshold);
			if (changed) {
				results.push_back(mes);
			}
		}

		//If any nodes move then change has occured
		return results;
	}

	void Calibrator::calibrateSystems(SystemDescriptor system1, SystemDescriptor system2)
	{
		//Create key for the pair of systems
		SystemPair sysPair(system1, system2);

		//Initialise vectors of measurements relevant to calibrating system1 and system2
		std::vector<Measurement::Ptr> measurements1;
		std::vector<Measurement::Ptr> measurements2;

		int thres = config.count_threshold[getResultsFor(system1, system2).state];

		std::pair<int,int> count = countRelevantSynchronisedMeasurements(system1, system2, config.min_count_per_node);

		//Calibrate
		if (count.first > thres && count.second > thres) {
			//TODO:latency estimation
			utility::profiler.startTimer("Calibrator Calibrate Systems " + system1.name + ", " + system2.name);
			CalibrationResult latestResult = getResultsFor(system1, system2);
			getRelevantMeasurements(system1, system2, &measurements1, &measurements2, config.min_count_per_node, true);
			calibrationResults[sysPair] = calibrateStreams(measurements1, measurements2, latestResult);
			utility::profiler.endTimer("Calibrator Calibrate Systems " + system1.name + ", " + system2.name);

			//Debug
			//std::stringstream ss;
			//ss << "Results for X: " << system1.name << " --> " << system2.name << "(Combined nodes)\n" << calibrationResults[sysPair].transform.matrix() << "\n";
			//SPOOKY_LOG(ss.str());
		}
	}

	void Calibrator::getRelevantMeasurements(
		SystemDescriptor system1, 
		SystemDescriptor system2,
		std::vector<Measurement::Ptr>* measurements1,
		std::vector<Measurement::Ptr>* measurements2,
		int minCountPerNode,
		bool clearMeasurementsWhenDone
	){
		//TODO:
		// - incorporate latency 

		//Loop through nodes and build up relevant measurements
		for (auto& node : calibrationSet.nodes) {

			//Keys for accessing data streams
			SystemNodePair sysNode1(system1, node);
			SystemNodePair sysNode2(system2, node);


			//If there is an entry for each system in the table, check if there is sufficient data for calibration
			if (calibrationSet.systemNodeTable.count(sysNode1) > 0 &&
				calibrationSet.systemNodeTable.count(sysNode2) > 0)
			{
				//Get maximum length of sensor stream
				int count1 = calibrationSet.systemNodeTable[sysNode1].totalCount(system1, system2);
				int count2 = calibrationSet.systemNodeTable[sysNode2].totalCount(system1, system2);

				//Streams of different length or not long enough- we cant use this data
				if (count1 < minCountPerNode || count2 < minCountPerNode) {
					continue; //cannot calibrate this pair of sensors yet
				}

				//Calibrate with complete bipartite graph of relationships
				for (auto& pair1 : calibrationSet.systemNodeTable[sysNode1].sensors) {
					SensorID id1 = pair1.first;
					//utility::MultiStream<Measurement::Ptr,std::string>&
					auto& stream_m1 = pair1.second;
					//Get measurements
					for (auto& pair2 : calibrationSet.systemNodeTable[sysNode2].sensors) {
						//utility::MultiStream<Measurement::Ptr,std::string>&
						auto& stream_m2 = pair2.second;

						//Synchronise the two streams
						std::vector<Measurement::Ptr> m1 = stream_m1.get(system1.name + system2.name);
						std::vector<Measurement::Ptr> m2 = stream_m2.get(system1.name + system2.name);

						bool streamsCompatible = m1.back()->calibrationCompatible(m2.back()->type);
						if(!streamsCompatible) continue;
						
						//TODO: retarget high noise measurements, not high latency - I no longer know what I meant by this
						if (m1.size() < m2.size()) {
							Measurement::synchronise(m2, m1);
						}
						else if(m1.size() > m2.size()){
							Measurement::synchronise(m1, m2);
						}
						
						if (m1.size() > minCountPerNode && m2.size() > minCountPerNode)
						{
							measurements1->insert(measurements1->end(), m1.begin(), m1.end());
							measurements2->insert(measurements2->end(), m2.begin(), m2.end());
						}

						//Clear the data used for calibration
						//Clear data even if it isnt used because it is not synchronised
						if (clearMeasurementsWhenDone) {
							stream_m1.clear(system1.name + system2.name);
							stream_m2.clear(system1.name + system2.name);
						}
					}
				}
			}
		}
	}


	std::pair<int, int> Calibrator::countRelevantSynchronisedMeasurements(
		SystemDescriptor system1,
		SystemDescriptor system2,
		int minCountPerNode
	) {
		//TODO:
		// - incorporate latency 
		std::vector<Measurement::Ptr> measurements1;
		std::vector<Measurement::Ptr> measurements2;

		getRelevantMeasurements(
			system1,
			system2,
			&measurements1,
			&measurements2,
			minCountPerNode,
			false
		);

		return std::make_pair<int,int>(measurements1.size(),measurements2.size());
	}

	void Calibrator::determineCalibrationsRequired(
		SystemDescriptor system1,
		SystemDescriptor system2,
		int minCountPerNode
	) {
		//Loop through nodes and build up relevant measurements
		for (auto& node : calibrationSet.nodes) {

			//Keys for accessing data streams
			SystemNodePair sysNode1(system1, node);
			SystemNodePair sysNode2(system2, node);


			//If there is an entry for each system in the table, check if there is sufficient data for calibration
			if (calibrationSet.systemNodeTable.count(sysNode1) > 0 &&
				calibrationSet.systemNodeTable.count(sysNode2) > 0)
			{
				//Get maximum length of sensor stream
				int count1 = calibrationSet.systemNodeTable[sysNode1].rawCount();
				int count2 = calibrationSet.systemNodeTable[sysNode2].rawCount();

				//Streams not long enough- we cant use this data
				if (count1 < minCountPerNode || count2 < minCountPerNode) {
					continue; //cannot calibrate this pair of sensors yet
				}

				//Add job for this system pair
				calibrationSet.systemNodeTable[sysNode1].addCalibrationJob(system1, system2);
				calibrationSet.systemNodeTable[sysNode2].addCalibrationJob(system1, system2);

			}
		}
	}

	CalibrationResult Calibrator::calibrateStreams(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult& calib)
	{
		//TODO: measurements are not sorted chronologically here
		Measurement::Type t1 = m1.front()->type;
		Measurement::Type t2 = m2.front()->type;
		//Bulk logic to route calibration procedures at runtime
		switch (t1) {
		case Measurement::Type::POSITION:
			if (t2 == Measurement::Type::POSITION || t2 == Measurement::Type::RIGID_BODY) {
				return calPos(m1, m2, calib);
			}
			break;
		case Measurement::Type::RIGID_BODY:
			if (t2 == Measurement::Type::RIGID_BODY) {
				//return calPos(m1, m2, calib);
				//TODO: implement this:
				return cal6DoF(m1, m2, calib, true);
			}
			else if (t2 == Measurement::Type::POSITION) {
				return calPos(m1, m2, calib);
			}
			else if (t2 == Measurement::Type::ROTATION) {
				return cal6DoF(m1, m2, calib, false);
			}
			break;
		case Measurement::Type::ROTATION:
			if (t2 == Measurement::Type::ROTATION || t2 == Measurement::Type::RIGID_BODY) {
				return cal6DoF(m1, m2, calib, false);
			} break;
		}
		SPOOKY_LOG("WARNING : no calibration model found for measurement types: " + std::to_string(t1) + " and " + std::to_string(t2));
		return CalibrationResult();
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Calibrator:Public
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Calibrator::addMeasurement(const Measurement::Ptr& m) {
		calibrationSet.addMeasurement(m, m->getSystem(), m->getNode());
	}

	void Calibrator::addMeasurementGroup(const std::vector<Measurement::Ptr>& measurementQueue) {
		
		//Decide if data is new and useful
		std::vector<Measurement::Ptr> measurements = filterChanges(measurementQueue);
		
		//Check there is data corresponding to more than one system for a given node, otherwise useless
		//TODO: optimise this filterLonelyData - currently takes way too long
		measurements = filterLonelyData(measurements);

		//Store the (refs to) the relevant measurements
		for (auto& m : measurements) {
			addMeasurement(m);
		}
	}

	void Calibrator::calibrate()
	{
		//For each unordered pairing of systems, check if there are common nodes
		for (std::set<SystemDescriptor>::iterator system1 = calibrationSet.systems.begin(); system1 != calibrationSet.systems.end(); system1++) {
			for (std::set<SystemDescriptor>::iterator system2 = std::next(system1); system2 != calibrationSet.systems.end(); system2++) {
				//Quickly check if we might be able to calibrate:
				determineCalibrationsRequired(*system1, *system2, config.min_count_per_node);
				//Calibrate if we can:
				calibrateSystems(*system1, *system2);
				
			}
		}
	}

	bool Calibrator::isStable()
	{
		for(auto& cal : calibrationResults){
			if (!(cal.second.state == CalibrationResult::State::CALIBRATED)) return false;
		}
		return true;
	}

	void  Calibrator::setResults(const CalibrationResult & r) {
		SystemPair forward = r.systems;
		SystemPair reverse(forward.second, forward.first);
		calibrationResults[forward] = r;
		calibrationResults[reverse] = r.inverse();
	}

	CalibrationResult Calibrator::getResultsFor(SystemDescriptor s1, SystemDescriptor s2) const
	{
		SystemPair forward(s1, s2);
		if (s1 == s2) {
			CalibrationResult cr = CalibrationResult();
			cr.systems = forward;
			return cr;
		}
		if (calibrationResults.count(forward) > 0) {
			return calibrationResults.at(forward);
		}
		SystemPair reverse(s2, s1);
		if(calibrationResults.count(reverse) > 0) {
			return calibrationResults.at(reverse).inverse();
		}
		CalibrationResult cr = CalibrationResult();
		cr.systems = forward;
		return cr;
	}

	std::string Calibrator::getStateSummary() {
		return calibrationSet.getStateSummary();
	}

}
