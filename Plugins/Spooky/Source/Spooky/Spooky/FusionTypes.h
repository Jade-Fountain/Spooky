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

#include <string>
#include <queue>
#include <set>
#include <memory>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Logging.h"
#include "Utilities/DataStructures.h"
#include "Utilities/CommonMath.h"
#pragma once


namespace spooky {

	//For convenience and abstraction, typedef some basic structs

	//Mapping between two affine spaces
	typedef Eigen::Transform<float, 3, Eigen::Affine> Transform3D;
	typedef Eigen::Transform<std::complex<double>, 3, Eigen::Affine> Transform3Dcd;
	
	//Define isometry for fast inverses when possible
	class Isometry3D : Transform3D {
		Isometry3D inverse(){
			this->Transform3D::inverse(Eigen::Isometry);
		}
	};

	static inline size_t hashTransform3D(const Transform3D& T) {
		size_t result = 0;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				//XOR
				result = result ^ (std::hash<float>{}(T.matrix()(i, j)) << 1);
			}
		}
		return result;
	}

	/** System descriptor - abstraction for a string type to be used as a map key - might be changed later
	*
	*/
	class SystemDescriptor {
	public:
		//Name of the system - used for checking system identity
		std::string name;

		//Overloaded operators check for valid system name and check equality
		bool SystemDescriptor::operator==(const SystemDescriptor &other) const {
			return name.size() > 0 && other.name.size() > 0 && name.compare(other.name) == 0;  // Compare the values, and return a bool result.
		}
		bool SystemDescriptor::operator!=(const SystemDescriptor &other) const {
			return !(*this == other);
		}
		//Comparators for maps
		bool operator<(const SystemDescriptor& rhs) const { return name < rhs.name; }
		bool operator>(const SystemDescriptor& rhs) const { return name > rhs.name; }
		//Constructor
		SystemDescriptor(std::string n = "") : name(n) {}
	};

	//Node descriptor uses the same class as systemdescriptor
	class NodeDescriptor : public SystemDescriptor {
	public:
		NodeDescriptor(std::string n = "") :  SystemDescriptor(n){}
	};

	static const NodeDescriptor SPOOKY_WORLD_ROOT_DESC = "SPOOKY_WORLD_ROOT_DESC";

	//Map key types
	typedef std::pair<SystemDescriptor, SystemDescriptor> SystemPair;
	typedef std::pair<SystemDescriptor, NodeDescriptor> SystemNodePair;
	
	//Sensor ID type
	typedef int SensorID;

	//Function to overload system pair map keying
	struct SystemPairCompare {
		bool operator() (const SystemPair& lhs, const SystemPair& rhs) {
			std::string lhs_combined = lhs.first.name + lhs.second.name;
			std::string rhs_combined = rhs.first.name + rhs.second.name;
			return lhs_combined < rhs_combined;
		}
	};


	//Results of a calibration are stored in this struct and returned
	class CalibrationResult {
	public:

		enum State {
			UNCALIBRATED = 0,
			REFINING = 1,
			CALIBRATED = 2
		};

		//systems = [domain,range]
		SystemPair systems;
		//Calibration state describes
		State state = UNCALIBRATED;
		//Maps systems.first to systems.second
		Transform3D transform;
		//Latency in ms of systems.second compared to systems.first
		// float latency = 0;
		//Time when calibration was carried out
		double timestamp = 0;

		//Error is the mean re-projection error
		float error = 0;
		//Quality is a qualitative measure in [0,1] of the estimated accuracy of the result
		float quality = 0;
		//Relevance - filtered error result from current calibration
		Transform3D relevance;
		//Weight counts the number of samples incorporated into this calibration result
		float weight = 0;

		//Constructors
		CalibrationResult(){
			reset();
		}

		CalibrationResult(const SystemDescriptor& s1, const SystemDescriptor& s2) {
			reset();
			systems = std::make_pair(s1, s2);
		}

		//Returns the inverse of the calibration result
		CalibrationResult inverse() const{
			CalibrationResult result = *this;
			result.transform = transform.inverse();
			return result;
		}

		//Checks if sensor is calibrated
		bool calibrated() const{
			return state != UNCALIBRATED;
		}

		//Checks if sensor is calibrated
		bool refining() const{
			return state != CALIBRATED;
		}


		//Combines two calibration results associatively
		void updateResult(const CalibrationResult& new_cal) {
			assert(systems == new_cal.systems);
			//Get mean transform
			std::vector<Transform3D> trans(2);
			std::vector<float> weights(2);
			trans.push_back(transform);
			weights.push_back(weight);
			trans.push_back(new_cal.transform);
			weights.push_back(new_cal.weight);
			transform = utility::getMeanTransform(trans, weights);

			//new timestamps
			//latency = new_cal.latency;
			timestamp = new_cal.timestamp;
			state = new_cal.state;
			relevance = new_cal.relevance;

			//Interpolate error and quality
			//TODO: make this correct error/quality amount
			float sum_weight = (weight + new_cal.weight);
			if (sum_weight == 0) return; //Return if both are zero weight (aka uncomputed) results
			error = (weight * error + new_cal.weight * new_cal.error) / sum_weight;
			quality = (weight * quality + new_cal.weight * new_cal.quality) / sum_weight;

			//Update weight to reflect new information integrated
			weight += new_cal.weight;
		}

		void reset() {
			error = 0;
			quality = 0;
			relevance = Transform3D::Identity();
			transform = Eigen::Matrix4f::Identity();
			state = State::UNCALIBRATED;
		}
	};

	/** Structs describing measurements
	*
	*/


	//Sensor describes all persistent parameters of a given sensor
	//WARNING TO DEVS: Measurement::Ptr cannot be stored in here or a memory leak will occur (shared ptr loop)
	class Sensor {
	public:
		//=================================================
		//Name of the sensor system from which the measurement came
		SystemDescriptor system;

		//Sensor number corresponding to measurement
		SensorID id = 0;

		//Possible nodes which this sensor is attached to
		std::set<NodeDescriptor> nodes;
		//Set of eliminated nodes
		std::set<NodeDescriptor> eliminatedNodes;
		//Stats for node scores:
		float meanScore = 1; // computed and set by correlator
		//=================================================
		//Latency of this sensor relative to the game engine (sec)
		float latency = 0;
		//=================================================
		//Typedef ptr to this class for neater code later
		typedef std::shared_ptr<Sensor> Ptr;

		//Root node from to which the sensor is attached
		//Defaults to the first node in the model
		NodeDescriptor rootNode = "";

		//Accessors:
		bool isResolved() { return utility::setDiff(nodes, eliminatedNodes).size() == 1; }
		bool isAmbiguous() { return nodes.size() != 1; }
		NodeDescriptor getRootNode(){return rootNode;}

		//Returns a valid node only when there is only one possibility
		NodeDescriptor getNode() {
			std::set<NodeDescriptor> nodesFinal = utility::setDiff(nodes, eliminatedNodes);
			if (nodesFinal.size() != 1) {
				//TODO: fix this log with a second getNodes method
				//SPOOKY_LOG(__FILE__ + __LINE__ + std::string(" : attempted to get node of ambiguous sensor"));
				return "__AMBIGUOUS__"; 
			}
			return *nodesFinal.begin();
		}

		//gets all possible nodes
		std::set<NodeDescriptor> getNodes() {
			return nodes;
		}

		std::set<NodeDescriptor> getRemainingNodes(){
			return utility::setDiff(nodes,eliminatedNodes);
		}
		
		//Adds a node as a possible sensor location
		void addNode(const NodeDescriptor& node) {
			nodes.insert(node);
		}

		bool nodeEliminated(const NodeDescriptor& node){
			return eliminatedNodes.count(node) > 0 || nodes.count(node) < 1;
		}

		void eliminateNode(const NodeDescriptor& node){
			eliminatedNodes.insert(node);
		}

		void resetNodesIfEmpty() {
			if (getRemainingNodes().size() == 0) {
				eliminatedNodes.clear();
			}
		}

		float getLatency() {
			return latency;
		}

		void setLatency(const float& l) {
			latency = l;
		}

	};

	//Class describing individual sensor reading taken at a particular time
	class Measurement {
	public:
		typedef std::shared_ptr<Measurement> Ptr;	

		enum Type {
			GENERIC = 0,
			POSITION = 1,
			ROTATION = 2,
			RIGID_BODY = 3,
			SCALE = 4
		};

		//=========================
		//			Members
		//=========================
	private:		
		//Measurement dimensions
		int size;
		
		//Value of measurement
		//Quaternions stored in (x,y,z,w) format
		Eigen::VectorXf data;

		//Uncertainty in T
		Eigen::MatrixXf uncertainty;

		//Sensor information
		Sensor::Ptr sensor;

		//Timestamp (sec; from device)
		//TODO: ensure double precision input
		double timestamp = -1;
	public:

		//Confidence in T in [0,1]
		float confidence = 0;

		//Type of measurement
		Type type;

		//--------------------
		//Postprocessing flags
		//--------------------
		//Is measurement in global or parent relative space
		bool globalSpace = true;

		//Should the skeletal constraints and prior be relaxed during multiple fusion steps?
		bool relaxConstraints = false;

		//Should the measurements 
		bool sensorDrifts = false;
		//--------------------

		//=========================
		//			Methods
		//=========================

		//Setup Methods
		bool check_consistent();

		bool setMetaData(float timestamp_sec, float confidence_);

		//=========================
		//Static factory methods:
		//=========================
		static Measurement::Ptr createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma);
		static Measurement::Ptr createQuaternionMeasurement(Eigen::Quaternionf quaternion, Eigen::Matrix<float,4,4> sigma);
		static Measurement::Ptr createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma);
		static Measurement::Ptr createPoseMeasurement(Eigen::Vector3f position, Eigen::Quaternionf quaternion, Eigen::Matrix<float,7,7> sigma);

		//=========================
		//Data Out Interface
		//=========================
		//Config:
		static constexpr float max_var = std::numeric_limits<float>::max();
		
		//Methods:
		Eigen::Vector3f getPosition();
		Eigen::Matrix3f getPositionVar();

		Eigen::Quaternionf getRotation();
		Eigen::Matrix4f getRotationVar();

		Eigen::Matrix<float,7,1>  getPosQuat();
		Eigen::Matrix<float,7,7> getPosQuatVar();

		Eigen::Vector3f getScale();
		Eigen::Matrix3f getScaleVar();

		Transform3D getTransform();
		Eigen::Matrix4f getTransformMatrix() { return getTransform().matrix(); }
		Eigen::Matrix4f getRotationTransformMatrix() { 
			auto M = getTransform().matrix(); 
			M.col(3).head(3) = Eigen::Vector3f(0,0,0);
			return M;
		}


		//=========================
		//Data helpers
		//=========================
		
		//Method for getting the distance between two measurements
		Eigen::VectorXf difference(const Measurement::Ptr & other);
		float compare(const Measurement::Ptr & other);


		//Transform this measurement to a new space
		Measurement transform(const Transform3D& T) const;

		//Synchronises the source stream with the target stream
		// It is assumed that the two  streams are chronologically sorted
		static void synchronise(std::vector<Measurement::Ptr>& source,
														 std::vector<Measurement::Ptr>& target);

		//Interpolates between two measurements of the same type
		static const float uncertainty_growth_max;
		static Eigen::VectorXf interpolateData(const Measurement::Ptr & x, const Measurement::Ptr & y, const float & t, const Measurement::Type & type);
		static Measurement::Ptr interpolate(const Measurement::Ptr& m0, const Measurement::Ptr& m1, float t);
		static Measurement::Ptr extrapolate(const Measurement::Ptr& m, float time_sec);
		
		//Calls set latency for each listed measurement's sensor
		static void setLatencies(std::vector<Measurement::Ptr>& m, float latency);
		
		//returns the vector corresponding to the transform T
		static Eigen::Matrix<float, 7, 1> getPosQuatFromTransform(const Transform3D& T);


		//Sort measurements based on the node of the measurements and return relevant data in one go
		template <class ReturnType, ReturnType(Measurement::*Getter)()>
		void static chunkMeasurements(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2,
			std::vector<std::vector<ReturnType>> * m1_out, std::vector<std::vector<ReturnType>> * m2_out)
		{
			std::map<NodeDescriptor, int> nodes;
			for (int i = 0; i < m1.size(); i++) {
				const auto& currentNode = m1[i]->getNode();
				assert(currentNode.name == m2[i]->getNode().name);

				//If new node, create another list for that node
				if (nodes.count(currentNode) == 0) {
					m1_out->push_back(std::vector<ReturnType>());
					m2_out->push_back(std::vector<ReturnType>());
					nodes[currentNode] = m1_out->size() - 1;
				}

				//Push back data to correct list corresponding to its node
				int index = nodes[currentNode];
				(*m1_out)[index].push_back((m1[i].get()->*Getter)());
				(*m2_out)[index].push_back((m2[i].get()->*Getter)());
			}
			//Return m1_out, m2_out
		}

		//----------------------
		//Accessors
		//----------------------
		NodeDescriptor getNode() {
			return sensor->getNode();
		}
		std::set<NodeDescriptor> getNodes() {
			return sensor->getNodes();
		}
		SystemDescriptor getSystem() {
			return sensor->system;
		}
		SensorID getSensorID() {
			return sensor->id;
		}

		void setSensor(Sensor::Ptr& sensor_) {
			sensor = sensor_;
		}
		Sensor::Ptr getSensor() {
			return sensor;
		}

		//Accessors
		const Eigen::MatrixXf& getUncertainty() const { return uncertainty; }
		const Eigen::VectorXf& getData() const { return data; }

		bool isAmbiguous() {
			return sensor->isAmbiguous();
		}
		bool isResolved() {
			return sensor->isResolved();
		}

		//Returns the global timestamp corresponding to this measurement, compensating for latency
		double getTimestamp() {
			return timestamp;
		}

		//Sets the local timestamp from a global timestamp
		void setTimestamp(double global_t) {
			timestamp = global_t;
		}

		void setLatency(const float& l) {
			sensor->setLatency(l);
		}
		
		//Returns the global global latency
		float getLatency() {
			return sensor->getLatency();
		}

		int getRequiredPDoF(){
			switch(type){
				case(Type::POSITION):
					return 3;
				case(Type::GENERIC):
					return data.size();
				case(Type::RIGID_BODY):
					return 3;
				case(Type::ROTATION):
					return 0;
				case(Type::SCALE):
					return 0;
			}
			return 0;
		}
		int getRequiredRDoF(){
			switch(type){
				case(Type::POSITION):
					return 0;
				case(Type::GENERIC):
					return data.size();
				case(Type::RIGID_BODY):
					return 3;
				case(Type::ROTATION):
					return 3;
				case(Type::SCALE):
					return 0;
			}
			return 0;
		}
		int getRequiredSDoF(){
			if(type == Type::SCALE)
			{
				return 3;
			} else {
  				return 0;
			}
		}
		bool calibrationCompatible(const Type& other_type) {
			switch (type) {
				case(Type::POSITION):
					return other_type == Type::POSITION || other_type == Type::RIGID_BODY;
				case(Type::GENERIC):
					return false;
				case(Type::RIGID_BODY):
					return other_type == Type::ROTATION || other_type == Type::POSITION;
				case(Type::ROTATION):
					return other_type == Type::ROTATION || other_type == Type::RIGID_BODY;
				case(Type::SCALE):
					return false;
			}
			return false;
		}
	};

	class MeasurementBuffer {
		//maximum storage for each system
		int max_buffer_length = 1000;
		
		// time in seconds before last measurement is ignored
		float expiry = 0.1; 
		

		struct MeasurementsWithCounter {
			Measurement::Ptr meas;
			int count = 0;
			MeasurementsWithCounter(){}
			MeasurementsWithCounter(Measurement::Ptr meas_) { meas = meas_; }
		};
		//Data : Sensor x Time --> Measurement
		// measurements[sensor][time] = meas
		std::map<Sensor::Ptr,std::map<double,MeasurementsWithCounter>> measurements;
		
		//Latencies
		float maximum_latency = 0;

	private:

		//Adds measurements for a given time and system to a list for later use
		//Interpolates between available timestamps
		void insertMeasurements(std::vector<Measurement::Ptr>& output, const Sensor::Ptr& sensor, const double& t) {
			//If we have no measurements for this system
			//TODO disable this check for private method
			//if (measurements.count(sensor) == 0) return;

			//Otherwise get the measurements
			//Timestamp-->vec<Measurements>
			const std::map<double, MeasurementsWithCounter>& m = measurements[sensor];

			//Binary search for closest values to the requested time t
			//WARNING: cpp lower_bound(t) gets "container whose key is not considered to go before t"
			//I would call that the upper bound if you ask me (ub)
			auto ub = m.lower_bound(t);
			//If we are looking before our records for this stream
			auto lb = std::prev(ub);
			
			if (lb == m.end() && ub == m.end()) {
				return;
			} else if(ub == m.end()){
				//Dont include old data
				if (std::abs(lb->first - t) > expiry) return;
				//If lb is the last measurement, return it
				output.push_back(lb->second.meas);
			}
			else if (lb == m.end()) {
				//Dont include data from too far in the future
				if (std::abs(ub->first - t) > expiry) return;
				//If ub is the next measurement, return it
				output.push_back(ub->second.meas);
			}
			else {
				//Interpolate otherwise
				//TODO: check lb and ub are close together?
				float alpha = (t - lb->first) / (ub->first - lb->first);
				assert(alpha >= 0 && alpha <= 1);
				output.push_back(Measurement::interpolate(lb->second.meas, ub->second.meas, alpha));
			}
		}

	public:
		//Add data
		void push_back(const Measurement::Ptr& m){
			//TODO: replace safeAccess with some kind of global check early on
			auto& sensor_measurements = utility::safeAccess(measurements, m->getSensor());
			sensor_measurements[m->getTimestamp()] = MeasurementsWithCounter(m);
			if(measurements[m->getSensor()].size() > max_buffer_length){
				//Erase old measurements
				measurements[m->getSensor()].erase(measurements[m->getSensor()].begin());
			}
			//Compute max latency
			maximum_latency = std::fmax(maximum_latency, m->getSensor()->latency);
		}

		//Gets measurements naively for all systems at time t
		std::vector<Measurement::Ptr> getLatestMeasurements(){
			std::vector<Measurement::Ptr> result;
			for(auto& m : measurements){
				const Sensor::Ptr& sensor = m.first;
				//If we have done a basic get operation on this measurement before, ignore
				if (measurements[sensor].rbegin()->second.count < 1) {
					//Get latest measurement for this sensor
					result.push_back(measurements[sensor].rbegin()->second.meas);
					measurements[sensor].rbegin()->second.count++;
				}
			}
			return result;
		}

		//Gets measurements for time t by seeking ahead in time for sensors with latency
		std::vector<Measurement::Ptr> getMeasurements(const double& t) {
			std::vector<Measurement::Ptr> result;
			for (auto& m : measurements) {
				const Sensor::Ptr& sensor = m.first;
				insertMeasurements(result, sensor, t);
			}
			return result;
		}
		//Gets measurements for time t by seeking ahead in time for sensors with latency
		std::vector<Measurement::Ptr> getSynchronizedMeasurements(const double& t){
			std::vector<Measurement::Ptr> result;
			for(auto& m : measurements){
				const Sensor::Ptr& sensor = m.first;
				insertMeasurements(result, sensor, t + sensor->latency);
			}
			return result;					
		}
		
		//Gets latest complete measurement set for time t.
		//This is needed because high latency sensors haven't reported data corresponding to 
		//the current measurements from low latency sensors yet.
		//This is used for calibration and correlation to make sure data corresponds correctly.
		//Fusion should just use the latest data available
		std::vector<Measurement::Ptr> getOffsetSynchronizedMeasurements(const double& t){
			return getSynchronizedMeasurements(t-maximum_latency);
		}
	};

	//Simpler buffer class for generic data objects
	template <class Data>
	class DataBuffer {
	private:
		//Timestamped data
		std::map<float,Data> buffer;
		float expiry = 0.1;

	public:
		//How far back in time to store data
		float time_width = 0;

		void insert(const float& timestamp, const Data& d, const float& time_now){
			if(time_now - timestamp <= time_width + expiry){
				buffer[timestamp] = d;				
			}
		}
		

		Data get(const float& t, bool* valid){
			//Binary search for closest values to the requested time t
			//WARNING: cpp lower_bound(t) gets "container whose key is not considered to go before t" aka lower bound of set bounded by t
			//I would call that the upper bound if you ask me (ub)
			auto ub = buffer.lower_bound(t);
			auto lb = std::prev(ub);
			*valid = true;

			if (lb == buffer.end() && ub == buffer.end()) {
				*valid = false;
				return Data();
			} else if(ub == buffer.end()){
				//Dont include old data
				if (std::abs(lb->first - t) > expiry) return;
				//If lb is the last measurement, return it
				return lb->second;
			}
			else if (lb == buffer.end()) {
				//Dont include data from too far in the future
				if (std::abs(ub->first - t) > expiry) return;
				//If ub is the next measurement, return it
				return ub->second;
			}
			else {
				//TODO: interpolate
				float alpha = (t - lb->first) / (ub->first - lb->first);
				if(alpha > 0.5){
					return ub->second;
				} else {
					return lb->second;
				}
			}
		}

		void clearOld(const float& time_now){
			auto iter = buffer.begin();
			while (iter != buffer.end()) {
				thisiter = iter;
				iter = std::next(iter);
				if (time_now - thisiter->first > time_width + expiry)
				{
					buffer.erase(thisiter);
				}
			}
		}

	};

}
