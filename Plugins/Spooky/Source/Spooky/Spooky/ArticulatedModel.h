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
			public:
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
				Eigen::VectorXf expectation;
				//Covariance associated with vec(expectation)
				Eigen::MatrixXf variance;

				Parameters getSubstate(const int& position, const int& dim) const {
					Parameters substate(dim);
					substate.expectation = expectation.block(position, 0, dim, 1);
					substate.variance = variance.block(position, position, dim, dim);
					return substate;
				}

				void insertSubstate(const int& position, const Parameters& p) {
					int dim = p.expectation.size();
					expectation.block(position, 0, dim, 1) = p.expectation;
					variance.block(position, position, dim, dim) = p.variance;
				}

				Parameters(int dim) :
					expectation(dim),
					variance(dim, dim)
				{
					expectation.setZero();
					variance.setIdentity();
				}

				Parameters(const Eigen::VectorXf& x, const Eigen::MatrixXf& V) : expectation(x), variance(V){

				}
			};
			bool modelVelocity = true;

			//Paramters for each articulation
			std::vector<Parameters> articulation;
			std::vector<Parameters> constraints;
			std::vector<Parameters> process_noise;

									   
			//Last update time
			float last_update_time = 0;
			//State is valid - false if state poorly initialised, etc...
			bool valid = true;
		};
	private:
		//Current state
		State local_state;
		//Constant internal structure of the node
		std::vector<Articulation> articulations;

		//Homepose is the default node pose (when theta = 0)
		//If the articulations are not twists, then the home pose is the identity
		Transform3D homePose;

	
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
		Transform3D getLocalPose();
		//Allows for speculative evaluation of pose based on expectation vector
		Transform3D getLocalPoseAt(const Eigen::VectorXf& theta);
		//Returns variance associated with pose
		Eigen::Matrix<float,6,6> getLocalPoseVariance();

		//Get rotational and translational degrees of freedom
		int getPDoF(bool hasLeverChild);
		int getRDoF();
		int getSDoF();

		//Get the total number of variables describing this node
		int getDimension();

		//Split and merging of state parameters:
		//---------------------------------------
		//Set state parameters
		void setState(const State::Parameters& new_state, const float& t);

		State::Parameters getState();
		//Get combined articulation constraints
		State::Parameters getConstraints();
		//Get process noise for node
		State::Parameters getProcessNoise();
		//Get time since the data was updated
		//Note: returns undefined variance
		State::Parameters getTimeSinceUpdated();
		//Get which entries in state are velocities
		//Note: returns undefined expectation
		State::Parameters getVelocityMatrix();

		//Generic grouped parameter methods
		static State::Parameters getChainParameters(std::function<Node::State::Parameters(Node&)> getParams, const std::vector<Node::Ptr>& node_chain);
		
		//Set and get an entire chain of nodes starting with this and recursing up parents
		static void setChainState(const std::vector<Node::Ptr>& node_chain, const State::Parameters & state, const float& t);
		static State::Parameters getChainState(const std::vector<Node::Ptr>& node_chain);
		static State::Parameters getChainConstraints(const std::vector<Node::Ptr>& node_chain);
		static State::Parameters getChainProcessNoise(const std::vector<Node::Ptr>& node_chain);
		static Eigen::VectorXf getChainTimeSinceUpdated(const std::vector<Node::Ptr>& node_chain);
		static Eigen::MatrixXf getChainVelocityMatrix(const std::vector<Node::Ptr>& node_chain);

    	//Get prediction
    	static State::Parameters getChainPredictedState(const std::vector<Node::Ptr>& fusion_chain, const float& timestamp);
		//Get the jocobian of an entire pose chain mapping state |-> (w,p,s) axis-angle, position and scale, each with 3D
		static Eigen::Matrix<float, 9, Eigen::Dynamic> getPoseChainJacobian(const std::vector<Node::Ptr>& fusion_chain, const bool& globalSpace, const Transform3D& globalToRootNode);
		
		//---------------------------------------

		//Updates the state of this node (e.g. angle, quaternion, etc.)
		void updateState(const State& new_state, const float& timestamp, const float& latency);
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
		void fusePositionMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace, const Node::Ptr& rootNode);
		void fuseRotationMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace, const Node::Ptr& rootNode);
		void fuseRigidMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace, const Node::Ptr& rootNode);
		void fuseScaleMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace, const Node::Ptr& rootNode);
		
		//Main generic EKF algorithm
	    void Node::computeEKFUpdate(
			const float& timestamp,
			const std::vector<Node::Ptr>& fusion_chain,
			const State::Parameters& measurement, 
			const State::Parameters& constraints, 
			const std::function<State::Parameters(const std::vector<Node::Ptr>&)> getPredictedState,
			const std::function<Eigen::VectorXf(const std::vector<Node::Ptr>&)> getMeasurement,
			const std::function<Eigen::MatrixXf(const std::vector<Node::Ptr>&)> getMeasurementJacobian
	    );
	       
		//Basic math for performing EKF with prior, constraints and measurement
	    Node::State::Parameters 
	    customEKFMeasurementUpdate( const State::Parameters& prior, const State::Parameters& constraints, const State::Parameters& measurement,
                            const Eigen::MatrixXf& measurementJacobian, const Eigen::VectorXf& state_measurement);
  		//Basic math for performing EKF with prior, constraints and measurement
	    Node::State::Parameters 
	    EKFMeasurementUpdate( const State::Parameters& prior, const State::Parameters& measurement,
                            const Eigen::MatrixXf& measurementJacobian, const Eigen::VectorXf& state_measurement);
  

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
			void setBoneForNode(const NodeDescriptor & node, const Transform3D& boneTransform, const Node::State::Parameters& constraints, const float& process_noise, const bool& modelVelocity);
			void setPoseNode(const NodeDescriptor & node, const Transform3D& poseTransform, const Node::State::Parameters& constraints, const float& process_noise, const bool& modelVelocity);
			void setScalePoseNode(const NodeDescriptor & node, const Transform3D& poseTransform, const Eigen::Vector3f& scaleInitial, const Node::State::Parameters& constraints, const float& process_noise, const bool& modelVelocity);
			
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
