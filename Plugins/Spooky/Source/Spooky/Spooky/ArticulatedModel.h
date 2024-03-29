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
#include "Eigen/Core"
#include "Articulation.h"
#include "FusionTypes.h"
#include "Calibration.h"
#include "Utilities/DataStructures.h"

namespace spooky {

	class Node : public std::enable_shared_from_this<Node>{
	public:
		//Constructor
		Node();
		//Define ptr type for neater code
		typedef std::shared_ptr<Node> Ptr;

		//////////////////////////////////////////////////////////////////
		//Internal Info
		//////////////////////////////////////////////////////////////////
		class State{
		public:
			class Parameters {
				//Vectors of articulation states stored in columns
				//		theta1	phi1 ...
				//		theta2	phi2
				//		theta3	phi3
				//		..
				//	e.g. quat: 	w
				//				x
				//				y
				//				z
				//	e.g. twists:(theta1	theta2 theta3)
				Eigen::VectorXf expectation_;
				//Covariance associated with vec(expectation_)
				Eigen::MatrixXf variance_;
				Eigen::MatrixXf information_;

			public:
				//Getters
				const Eigen::MatrixXf& variance() const{return variance_;} 
				const Eigen::MatrixXf& information() const{return information_;} 
				const Eigen::VectorXf& expectation() const { return expectation_; }

				//Expectation can be modified directly
				Eigen::VectorXf& expectation(){return expectation_;}
				
				//Setters for variance and information 
				void set_variance(const Eigen::MatrixXf& v){variance_ = v; information_ = v.inverse();}
				void set_information(const Eigen::MatrixXf& i){information_ = i; variance_ = i.inverse();}

				Parameters getSubstate(const int& position, const int& dim) const {
					Parameters substate(dim);
					substate.expectation_ = expectation_.block(position, 0, dim, 1);
					substate.variance_ = variance_.block(position, position, dim, dim);
					substate.information_ = information_.block(position, position, dim, dim);
					return substate;
				}

				void insertSubstate(const int& position, const Parameters& p) {
					int dim = p.expectation_.size();
					expectation_.block(position, 0, dim, 1) = p.expectation_;
					variance_.block(position, position, dim, dim) = p.variance_;
					information_.block(position, position, dim, dim) = p.information_;
				}

				//TODO: make this work!
				//void addProcessNoiseApprox(const Eigen::MatrixXf& P) {
				//	variance_ += P;
				//	Eigen::VectorXf v_powneg = Eigen::VectorXf(variance_.diagonal()).cwiseInverse();
				//	//Assumes P diagonal, and info diagonal mostly
				//	information_.diagonal() = v_powneg;
				//	
				//	//assuming (1+X)^-1 = sum(i=0,inf,X^i), if ||X||<1  "binomial series" 
				//	//then (X+P)^-1 ~= P^-1 - XP^-2
				//	//Eigen::VectorXf diagPinv = P.diagonal().cwiseInverse();
				//	//Eigen::VectorXf diagPinv2 = diagPinv.pow(2);
				//	//information_ = Eigen::MatrixXf(diagPinv.asDiagonal()) + information_ * diagPinv2.asDiagonal();
				//}

				Parameters(int dim) :
					expectation_(dim),
					variance_(dim, dim),
					information_(dim, dim)
				{
					expectation_.setZero();
					variance_.setIdentity();
					information_.setIdentity();
				}

				Parameters(const Eigen::VectorXf& x, const Eigen::MatrixXf& V) : 
					expectation_(x), 
					variance_(V), 
					information_(V.inverse()){}

				int dimension() const {
					return expectation_.size();
				}
			};
			bool modelVelocity = true;

			//Paramters for each articulation
			std::vector<Parameters> articulation;
			std::vector<Parameters> constraints;
			std::vector<Parameters> process_noise;

									   
			//Last update time
			float last_update_time = 0;

			//TODO: generalize concepts of confidence
			//Info for non-offset confidence
			float non_offset_confidence = 0;
			float non_offset_last_update_time = 0;

			//Lowest latency from latest set of measurements
			float smallest_latency = 0;
			//State is valid - false if state poorly initialised, etc...
			bool valid = true;
		};
	private:
		//Current state
		State local_state;
		//Constant internal structure of the node
		std::vector<Articulation> articulations;

		struct TimestampedData {
			Eigen::VectorXf data;
			float t = 0;
			TimestampedData() {}
			TimestampedData(Eigen::VectorXf d, float t_) {
				data = d;
				t = t_;
			}
		};
		std::map<Sensor::Ptr, TimestampedData> measurementBuffer;

	
	public:
		//////////////////////////////////////////////////////////////////
		//Extrinsic info
		//////////////////////////////////////////////////////////////////

		//Own name
		NodeDescriptor desc;

		//Parent of this node
		Ptr parent = NULL;
		NodeDescriptor parent_desc;

		//Pending measurements 
		//TODO: ensure ordered by timestamp
		std::vector<Measurement::Ptr> measurements;


	public:
		//Joint stiffness - 0 => no restriction; inf => next state will be the constraint centre
		float joint_stiffness = 1; //in [0,inf]

		//Returns the final pose of this node in global space based on pose of all parents
		Transform3D getFinalGlobalPose();
		//Returns final local transform relative to parent transform
		Transform3D getLocalPose() const;
		//Allows for speculative evaluation of pose based on expectation vector
		Transform3D getLocalPoseAt(const Eigen::VectorXf& theta) const;

		//Gets timestamp associated with latest data fusion, including accounting for latency
		float getNodeLastFusionTime();

		//Gets the confidence based only on measurments that don't accumulate offsets
		float getNonOffsetConfidence(const float& t);

		//TODO: reimplement:
		////Get the local pose deltaT seconds into the future based on velocity
		//Transform3D getLocalPosePredicted(const float& deltaT) const;
		////Get the local pose deltaT seconds into the future based on velocity, at speculative state theta
		//Transform3D getLocalPosePredictedAt(const Eigen::VectorXf& theta, const float& deltaT) const;
		////Gets the global pose deltaT seconds into the future
		//Transform3D getGlobalPosePredicted(const float& deltaT);


		//Returns variance associated with pose
		Eigen::Matrix<float,6,6> getLocalPoseVariance() const;

		//Get rotational and translational degrees of freedom
		int getPDoF(bool hasLeverChild) const;
		int getRDoF() const;
		int getSDoF() const;

		//Get the total number of variables describing this node
		int getDimension() const;

		//Split and merging of state parameters:
		//---------------------------------------
		//Set state parameters
		void setState(const State::Parameters& new_state, const float& t);

		State::Parameters getState() const;
		//Get combined articulation constraints
		State::Parameters getConstraints() const;
		//Get process noise for node
		State::Parameters getProcessNoise() const;
		//Get time since the data was updated
		//Note: returns undefined variance
		State::Parameters getTimeSinceUpdated() const;
		//Get the predicted state at timestamp based on internal models
		State::Parameters getPredictedState(const float& timestamp) const;
		//Get which entries in state are velocities
		//Note: returns undefined expectation
		State::Parameters getVelocityMatrix() const;

		//Generic grouped parameter methods
		static State::Parameters getChainParameters(std::function<Node::State::Parameters(Node&)> getParams, const std::vector<Node::Ptr>& node_chain);
		
		//Set and get an entire chain of nodes starting with this and recursing up parents
		static void setChainState(const std::vector<Node::Ptr>& node_chain, const State::Parameters & state, const float& t);
		static State::Parameters getChainState(const std::vector<Node::Ptr>& node_chain);
		static State::Parameters getChainConstraints(const std::vector<Node::Ptr>& node_chain);
		static State::Parameters getChainProcessNoise(const std::vector<Node::Ptr>& node_chain);
		static Eigen::VectorXf getChainTimeSinceUpdated(const std::vector<Node::Ptr>& node_chain);
		static Eigen::MatrixXf getChainVelocityMatrix(const std::vector<Node::Ptr>& node_chain);
		static State::Parameters getPredictionForArticulation(const Articulation& art, const State::Parameters& state, const float& t);

    	//Get prediction
    	static State::Parameters getChainPredictedState(const std::vector<Node::Ptr>& fusion_chain, const float& timestamp);
		//Get the jocobian of an entire pose chain mapping state |-> (w,p,s) axis-angle, position and scale, each with 3D
		static Eigen::MatrixXf getPoseChainJacobian(const std::vector<Node::Ptr>& fusion_chain,
													const bool& globalSpace,
													const Transform3D& globalToRootNode,
													const std::function<Eigen::VectorXf(const Transform3D&)>& transformRepresentation);
		
		//---------------------------------------

		//Sets the model for the articulations associated with this node
		void setModel(std::vector<Articulation> art, const bool& modelVelocity);

		//Set fusion parameters
		void setConstraintForArticulation(const int& i, const Node::State::Parameters& c);
		void setConstraints(const Node::State::Parameters& c);
		void setProcessNoiseForArticulation(const int& i, const Node::State::Parameters& p);
		void setProcessNoises(const Node::State::Parameters& p);

		//Local fusion of all buffered measurements
		void fuse(const Calibrator& calib, const SystemDescriptor& referenceSystem, const std::map<NodeDescriptor,Node::Ptr>& nodes);

		//Get required parents for fusing measurement m
		std::vector<Node::Ptr> getAllParents();
	    std::vector<Node::Ptr> getRequiredChain(const Node::Ptr& destNode, const Measurement::Ptr& m);

		//Fusion of particular mesurement types (defined in FusionProcedures.cpp)
		//Returns the updated node chain for future reference
		std::vector<Node::Ptr> fusePositionMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace, const Node::Ptr& rootNode);
		std::vector<Node::Ptr> fuseRotationMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace, const Node::Ptr& rootNode);
		std::vector<Node::Ptr> fuseDeltaRotationMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace, const Node::Ptr& rootNode);
		std::vector<Node::Ptr> fuseRigidMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace, const Node::Ptr& rootNode);
		std::vector<Node::Ptr> fuseScaleMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace, const Node::Ptr& rootNode);
		
		//Main generic EKF algorithm
		void computeEKFUpdate(
			const float& timestamp,
			const std::vector<Node::Ptr>& fusion_chain,
			const State::Parameters& measurement,
			const State::Parameters& constraints,
			const float& stiffness,
			const std::function<State::Parameters(const std::vector<Node::Ptr>&)> getPredictedState,
			const std::function<Eigen::VectorXf(const std::vector<Node::Ptr>&)> getMeasurement,
			const std::function<Eigen::MatrixXf(const std::vector<Node::Ptr>&)> getMeasurementJacobian,
			bool relaxConstraints,
			int max_iterations = 1
	    );
		
		//Basic math for performing EKF with prior, constraints and measurement
	    static Node::State::Parameters 
	    customEKFMeasurementUpdate( const State::Parameters& prior, const State::Parameters& constraints, 
	    					const float& stiffness, const State::Parameters& measurement,
                            const Eigen::MatrixXf& measurementJacobian, const Eigen::VectorXf& state_measurement);

    	static Node::State::Parameters IKUpdate(const State::Parameters& prior, const State::Parameters& measurement, 
    						const Eigen::MatrixXf& measurementJacobian, const Eigen::VectorXf& predictedMeasurement);
  		
  		//Basic math for performing EKF with prior and measurement only
	    static Node::State::Parameters 
	    EKFMeasurementUpdate( const State::Parameters& prior, const State::Parameters& measurement,
                            const Eigen::MatrixXf& measurementJacobian, const Eigen::VectorXf& state_measurement);
		
		//UKF - untested as of yet
		static Node::State::Parameters UKFMeasurementUpdate(
			const State::Parameters& prior,
			const State::Parameters& measurement,
			const std::function<Eigen::VectorXf(const Eigen::VectorXf&)> measurementFunction);
		

	private:
		Transform3D getGlobalPose();
	
		//Merges measurement m into state at slot i
		void insertMeasurement(const int& i, const Measurement::Ptr& m, const Transform3D& parent_pose, State* state);


		float initial_covariance = 3.14;

		//Cached transforms
		bool recacheRequired = true;
		//Optimise traversal and caching
		Transform3D cachedPose;
		size_t cachedPoseHash = 0;
		size_t lastParentHash = 0;
		Transform3D getCachedPose();
		size_t getCachedPoseHash();
		//Checks the chain of parents for changes to transform
		bool parentHashesChanged();
	};

	class ArticulatedModel{
	public:

		ArticulatedModel();
		/*//////////////////////////////////////////////////////////////////
		*				Public methods
		*//////////////////////////////////////////////////////////////////
			//Sets reference system
			void setReferenceSystem(const SystemDescriptor& s);
			//Adds node to the skeleton
			void addNode(const NodeDescriptor & node, const NodeDescriptor & parent);
			//Adds a generic node if necessary
			void addGenericNode(const NodeDescriptor & node);

			//Assigns parents and children of all nodes
			void enumerateHeirarchy();

			//Returns a list of pending measurements
			std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> getMeasurements();

			//Adds a measurement to be fused on next fusion call
			void addMeasurement(const Measurement::Ptr& m);

			//Add group of measurements
			void addMeasurementGroup(const std::vector<Measurement::Ptr>& m);

			//Compute best model for given data and prior
			void fuse(const Calibrator& calib);

			//Sets the structure parameters for the specified articulation as a bone according to the boneVec
			void setFixedNode(const NodeDescriptor & node, const Transform3D& boneTransform);
			void setBoneForNode(const NodeDescriptor & node, const Transform3D& boneTransform, const Node::State::Parameters& constraints, const Eigen::MatrixXf& process_noise, const bool& modelVelocity);
			void setPoseNode(const NodeDescriptor & node, const Transform3D& poseTransform, const Node::State::Parameters& constraints, const Eigen::MatrixXf& process_noise, const bool& modelVelocity);
			void setScalePoseNode(const NodeDescriptor & node, const Transform3D& poseTransform, const Node::State::Parameters& constraints, const Eigen::MatrixXf& process_noise, const bool& modelVelocity);
			
			//Stiffness settings
			void setJointStiffness(const NodeDescriptor & node, const float& stiffness);
			void setAllJointStiffness(const float& stiffness);

		////////////////////////////////////////////////////
		//					Results
		////////////////////////////////////////////////////
			//Returns node pose
			Transform3D getNodeGlobalPose(const NodeDescriptor& node);
			//Returns local orientation of node
			Transform3D getNodeLocalPose(const NodeDescriptor& node);
			
			//Gets the time last fusion occured
			float getNodeLastFusionTime(const NodeDescriptor& node);

			//Get confidence for this bone 
			float getNodeNonOffsetConfidence(const NodeDescriptor& node, const float& t);

		/*//////////////////////////////////////////////////////////////////
		*				Private Data
		*//////////////////////////////////////////////////////////////////
		private:
			//SkeletonData
			std::map<NodeDescriptor,Node::Ptr> nodes;

			//Reference coordinate system
			SystemDescriptor reference_system = "";
		
			//Clears measurements in graph
			void clearMeasurements();

	};
}
