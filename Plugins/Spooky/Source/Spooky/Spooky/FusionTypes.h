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
	typedef SystemDescriptor NodeDescriptor;

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
		float latency = 0;
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
			latency = new_cal.latency;
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
		//Latency of this sensor relative to the main system (sec)
		float latency = 0;
		//=================================================
		//Typedef ptr to this class for neater code later
		typedef std::shared_ptr<Sensor> Ptr;

		//Accessors:
		bool isResolved() { return utility::setDiff(nodes, eliminatedNodes).size() == 1; }
		bool isAmbiguous() { return nodes.size() != 1; }

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

		//Is measurement in global or parent relative space
		bool globalSpace = true;

		//=========================
		//			Methods
		//=========================

		//Setup Methods
		bool check_consistent() {
			return (size == data.size() == uncertainty.rows() == uncertainty.cols());
		}

		bool setMetaData(float timestamp_sec, float confidence_){
			timestamp = timestamp_sec;
			confidence = confidence_;
			return check_consistent();
		}

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


		//=========================
		//Data helpers
		//=========================
		
		//Method for getting the distance between two measurements
		Eigen::VectorXf difference(const Measurement::Ptr & other);
		float compare(const Measurement::Ptr & other);


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
			SPOOKY_LOG("node 0 = " + m1.front()->getNode().name);
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
			return timestamp - sensor->getLatency();
		}

		//Sets the local timestamp from a global timestamp
		void setTimestamp(double global_t) {
			timestamp = global_t + sensor->getLatency();
		}

		void setLatency(const float& l) {
			sensor->setLatency(l);
		}
		
		//Returns the global global latency
		float getLatency() {
			return sensor->getLatency();
		}

	};


}
