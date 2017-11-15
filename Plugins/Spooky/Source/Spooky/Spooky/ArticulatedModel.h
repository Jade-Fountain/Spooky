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

	class Node{
	public:
		//Constructor
		Node();
		//Define ptr type for neater code
		typedef std::shared_ptr<Node> Ptr;

		//////////////////////////////////////////////////////////////////
		//Internal Info
		//////////////////////////////////////////////////////////////////
		struct State{
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
			Eigen::MatrixXf expectation;
			//Covariance associated with vec(expectation)
			Eigen::MatrixXf variance;
			//Last update time
			float last_update_time = 0;
		};
	
		//Current state
		State local_state;
		//Constant internal structure of the node
		std::vector<Articulation> articulations;

		//Homepose is the default node pose (when theta = 0)
		//If the articulations are not twists, then the home pose is the identity
		Transform3D homePose;

	
		//////////////////////////////////////////////////////////////////
		//External info
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
		//Returns the final pose of this node in global space based on pose of all parents
		Transform3D getFinalGlobalPose();
		//Returns final local transform relative to parent transform
		Transform3D getLocalPose();

		//Updates the state of this node (e.g. angle, quaternion, etc.)
		void updateState(const State& new_state, const float& timestamp, const float& latency);
		//Sets the model for the articulations associated with this node
		void setModel(std::vector<Articulation> art);
		//Local fusion of buffered measurements
		void fuse(const Calibrator& calib, const SystemDescriptor& referenceSystem);
	
	private:
		Transform3D getGlobalPose();

		float initial_covariance = 3.14;

		//Cached transforms
		bool rechacheRequired = true;
		//TODO: optimise traversal and caching
		Transform3D cachedPose;
		Transform3D lastParentCache;
		Transform3D getCachedPose();
	};

	class ArticulatedModel{
		/*//////////////////////////////////////////////////////////////////
		*				Public methods
		*//////////////////////////////////////////////////////////////////
		public:
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
			void setBoneForNode(const NodeDescriptor & node, const Transform3D& boneTransform);
			void setPoseNode(const NodeDescriptor & node, const Transform3D& poseTransform);
			void setScalePoseNode(const NodeDescriptor & node, const Transform3D& poseTransform, const Eigen::Vector3f& scaleInitial);


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
