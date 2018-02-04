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
#include "ArticulatedModel.h"
#include "Utilities/Conventions.h"
#include <functional>

namespace spooky {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									Node
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	//-------------------------------------------------------------------------------------------------------
	//									Public
	//-------------------------------------------------------------------------------------------------------
	Node::Node() {
		homePose = Transform3D::Identity();
		cachedPose = Transform3D::Identity();
	}

	Transform3D Node::getFinalGlobalPose(){
		return getGlobalPose() * homePose;
	}

	Transform3D Node::getLocalPose(){
		Transform3D pose = Transform3D::Identity();
		for (int i = 0; i < articulations.size(); i++) {
			pose = pose * articulations[i].getTransform(local_state.articulation[i].expectation);
		}	
		return pose;
	}

	Transform3D Node::getLocalPoseAt(const Eigen::VectorXf & theta)
	{
		Transform3D pose = Transform3D::Identity();
		assert(theta.size() == getDimension());
		int block_start = 0;
		for (int i = 0; i < articulations.size(); i++) {
			int block_size = local_state.articulation[i].size();
			pose = pose * articulations[i].getTransform<float>(theta.block(block_start,0,block_size,1));
			block_start += block_size;
		}
		return pose;
	}
	
	Eigen::Matrix<float,6,6> Node::getLocalPoseVariance(){
		Eigen::Matrix<float,6,6> var = Eigen::Matrix<float,6,6>::Zero();
		for (int i = 0; i < articulations.size(); i++) {
			//Assume decoupling of variances between articulations - only true for linear cases (position only)
			//TODO: implement this method in articulation
			assert(false);
			//var += articulations[i].getPoseVariance(local_state.articulation[i].expectation,local_state.articulation[i].variance);
		}	
		return var;
	}

	
	int Node::getPDoF(bool hasLeverChild){
		int pdof = 0;
		for (int i = 0; i < articulations.size(); i++) {
			pdof += articulations[i].getPDoF(hasLeverChild);
		}
		return pdof;
	}
	
	int Node::getRDoF(){
		int rdof = 0;
		for (int i = 0; i < articulations.size(); i++) {
			rdof += articulations[i].getRDoF();
		}
		return rdof;
	}

	int Node::getDimension() {
		int dim = 0;
		for (int i = 0; i < articulations.size(); i++) {
			dim += local_state.articulation[i].size();
		}
		return dim;
	}


	//Set state parameters
	void Node::setState(const State::Parameters& new_state){
		int pos = 0;
		for(int i = 0; i < articulations.size(); i++){
			int dim = local_state.articulation[i].size();
			local_state.articulation[i] = new_state.getSubstate(pos,dim);
			pos += dim;
		}
		//Indicate that this node needs its local pose to be recomputed
		recacheRequired = true;
		//TODO:Add timestamp?
		//local_state.last_update_time = t;
	}

	Node::State::Parameters Node::getState(){
		//Construct new parameters set for combined articulations
		State::Parameters p(getDimension());
		int pos = 0;
		for(int i = 0; i < articulations.size(); i++){
			int dim = local_state.articulation[i].size();
			p.insertSubstate(pos,local_state.articulation[i]);
			pos += dim;
		}
		return p;
	}

	Node::State::Parameters Node::getChainState(const int& chain_length) {
		//Precompute State size
		int inputDimension = 0;
		//TODO: dont use raw pointer
		Node* node = this;
		for (int i = 0; i < chain_length; i++) {
			inputDimension += node->getDimension();
			if (node->parent == NULL) break;
			node = node->parent.get();
		}
		//Reset for actual calculation
		node = this;
		State::Parameters result(inputDimension);
		int position = 0;
		for (int i = 0; i < chain_length; i++) {
			int dim = node->getDimension();
			result.insertSubstate(position,node->getState());
			position += dim;
			if (node->parent == NULL) break;
			node = node->parent.get();
		}
		return result;
	}

	void Node::setChainState(const int& chain_length, const State::Parameters& state_params) {
		//Precompute Jacobian size
		//TODO: dont use raw pointer
		Node* node = this;
		int last_block_end = 0;
		for (int i = 0; i < chain_length; i++) {
			int dim = node->getDimension();
			node->setState(state_params.getSubstate(last_block_end,dim));
			last_block_end += dim;
			if (node->parent == NULL) break;
			node = node->parent.get();
		}
	}



	void Node::updateState(const State& new_state, const float& timestamp, const float& latency) {
		recacheRequired = true;
		//TODO: add latency prediction
		local_state = new_state;
		local_state.last_update_time = timestamp;
	}

	void Node::setModel(std::vector<Articulation> art){
		articulations = art;
		//Clear old model
		local_state.articulation.clear();
		int max_n_rows = 1;
		for(int i = 0; i < articulations.size(); i++){	
			auto init = Articulation::getInitialState(articulations[i].getType());
			local_state.articulation.push_back(Node::State::Parameters(init.size()));
			local_state.articulation[i].expectation = init;
			//TODO: generate covariance initial per aticulation
			local_state.articulation[i].variance = initial_covariance * Eigen::MatrixXf::Identity(local_state.articulation[i].size(),
																							      local_state.articulation[i].size());
		}
	}

	void Node::fuse(const Calibrator& calib, const SystemDescriptor& referenceSystem){
		
		//If this node has a parent, recursively fuse until we know its transform
		if (parent != NULL) {
			parent->fuse(calib, referenceSystem);
		}

		for(auto& m : measurements){
			//Throwout bad measurements
			//TODO: use confidence better
			if (m->confidence < 0.75) {
				continue;
			}

			//Get mapping to correct reference frame
			//TODO: Optimise this access somehow?
			Transform3D toFusionSpace = Transform3D::Identity();
			CalibrationResult calibResult = calib.getResultsFor(m->getSystem(),referenceSystem);
			if (calibResult.calibrated()) {
				toFusionSpace = calibResult.transform;
			}

			switch(m->type){
				case(Measurement::Type::POSITION):
					fusePositionMeasurement(m,toFusionSpace);
					break;
				case(Measurement::Type::ROTATION):
					fuseRotationMeasurement(m,toFusionSpace);
					break;
				case(Measurement::Type::RIGID_BODY):
					fuseRigidMeasurement(m,toFusionSpace);
					break;
				case(Measurement::Type::SCALE):
					fuseScaleMeasurement(m,toFusionSpace);
					break;

			}
			//Transform3D error = utilites::getError(m,parent_pose * getLocalPose());

			//Node::State new_state = local_state;
			//new_state.valid = false;
			//for(int i = 0; i < articulations.size(); i++){
			//	//Iteratively enters data into new_state
			//	insertMeasurement(i,m,parent_pose,&new_state);
			//	//If we can use the data, update the local state
			//}
			//if(new_state.valid) updateState(new_state, m->getTimestamp(), m->getLatency());
		}
		//Dont use data twice
		measurements.clear();
	}

	void Node::insertMeasurement(const int& i, const Measurement::Ptr& m, const Transform3D& parent_pose, State* state){
		Articulation::Type articulationType = articulations[i].getType();
		//If measurement is rotation
		if(m->type == Measurement::Type::ROTATION ||
		(articulationType == Articulation::Type::BONE && m->type == Measurement::Type::RIGID_BODY) )
		{
			//Simple update based on parent pose
			state->articulation[i].expectation = Eigen::Quaternionf(parent_pose.rotation().inverse() * m->getRotation()).coeffs();
			state->articulation[i].expectation.normalize();
			state->articulation[i].variance = m->getRotationVar();
			state->valid = true;
		} 
		//If measurement also contains position
		else if (m->type == Measurement::Type::RIGID_BODY && articulationType == Articulation::Type::POSE)
		{
			//Simple update based on parent pose
			Eigen::Quaternionf qLocal= Eigen::Quaternionf(parent_pose.rotation().inverse() * m->getRotation());
			Eigen::Vector3f posLocal = parent_pose.inverse() * m->getPosition();
			state->articulation[i].expectation = Eigen::Matrix<float, 7, 1>::Zero();
			state->articulation[i].expectation << posLocal, qLocal.coeffs();
			state->articulation[i].variance = m->getPosQuatVar();
			state->valid = true;
		}
		//If scaling
		else if (m->type == Measurement::Type::SCALE && articulationType == Articulation::Type::SCALE) {
			//Scales are ALWAYS LOCAL
			state->articulation[i].expectation = m->getScale();
			state->articulation[i].variance = m->getScaleVar();
			state->valid = true;
		}
	}

	//-------------------------------------------------------------------------------------------------------
	//									Private
	//-------------------------------------------------------------------------------------------------------

	Transform3D Node::getGlobalPose() {
		//If this is a root node
		if (parent == NULL) {
			cachedPose = recacheRequired ? getLocalPose() : cachedPose;
			cachedPoseHash = hashTransform3D(cachedPose);
			recacheRequired = false;
			return cachedPose;
		}
		//If we have a parent node
		bool parentChanged =  parent->getCachedPoseHash() != lastParentHash;
		//Check if cached
		//If we need to recache, concatenate articulations
		if(recacheRequired || parentChanged){
			//If root node, return identity
			Transform3D parent_pose = parent->getGlobalPose();
			//Save cache
			cachedPose = parent_pose * getLocalPose();
			cachedPoseHash = hashTransform3D(cachedPose);
			recacheRequired = false;
			//The latest transform of our parent
			lastParentHash =  parent->getCachedPoseHash();
		}
		return cachedPose;
	}

	Transform3D Node::getCachedPose() {
		return cachedPose;
	}

	size_t Node::getCachedPoseHash() {
		return cachedPoseHash;
	}



	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									ArticulatedModel
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	//-------------------------------------------------------------------------------------------------------
	//									Public
	//-------------------------------------------------------------------------------------------------------

	void ArticulatedModel::setReferenceSystem(const SystemDescriptor& s) {
		reference_system = s;
	}

	void ArticulatedModel::addNode(const NodeDescriptor & node, const NodeDescriptor & parent) {
		//This line initialises the node entry if not already initialised
		utility::safeAccess(nodes, node)->desc = node;
		nodes[node]->parent_desc = parent;
	}
	
	void ArticulatedModel::enumerateHeirarchy(){
		for(auto& node : nodes){
			NodeDescriptor parent = node.second->parent_desc;
			if(nodes.count(parent) != 0 && parent != node.second->desc){
				node.second->parent = nodes[parent];
			}
		}
	}


	std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> ArticulatedModel::getMeasurements() {
		std::vector<std::pair<Measurement::Ptr, NodeDescriptor>> measurements;
		for (auto& node : nodes) {
			for (auto& measurement : node.second->measurements) {
				measurements.push_back(std::make_pair(measurement,node.first));
			}
		}
		return measurements;
	}

	void ArticulatedModel::addMeasurement(const Measurement::Ptr& m) {
		if(m->isResolved()){
			nodes[m->getNode()]->measurements.push_back(m);
		}
	}

	void ArticulatedModel::addMeasurementGroup(const std::vector<Measurement::Ptr>& measurements) {
		for(auto& m : measurements){
			addMeasurement(m);
		}
	}

	void ArticulatedModel::fuse(const Calibrator& calib) {
		for(auto& node : nodes){
			node.second->fuse(calib, reference_system);
		}
		clearMeasurements();
	}

	void ArticulatedModel::setFixedNode(const NodeDescriptor& node, const Transform3D& pose) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createFixed(pose));
		nodes[node]->setModel(art);
		nodes[node]->local_state.articulation[0].expectation = Eigen::VectorXf(0);
	}


	void ArticulatedModel::setBoneForNode(const NodeDescriptor& node, const Transform3D& boneTransform) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createBone(boneTransform.translation()));
		nodes[node]->setModel(art);
		Eigen::AngleAxisf aa(boneTransform.rotation());
		nodes[node]->local_state.articulation[0].expectation = aa.angle() * aa.axis();
	}


	void ArticulatedModel::setPoseNode(const NodeDescriptor& node, const Transform3D& poseTransform) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createPose());
		nodes[node]->setModel(art);
		nodes[node]->local_state.articulation[0].expectation = Measurement::getPosQuatFromTransform(poseTransform);
	}
	
	void ArticulatedModel::setScalePoseNode(const NodeDescriptor & node, const Transform3D& poseTransform, const Eigen::Vector3f& scaleInitial) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createPose());
		//Scale in local space x'=T*S*x
		art.push_back(Articulation::createScale());
		nodes[node]->setModel(art);
		nodes[node]->local_state.articulation[0].expectation = Measurement::getPosQuatFromTransform(poseTransform);
		nodes[node]->local_state.articulation[1].expectation = scaleInitial;
	}


	void ArticulatedModel::addGenericNode(const NodeDescriptor & node) {
		if (nodes.count(node) == 0) {
			addNode(node, NodeDescriptor("root"));
			std::vector<Articulation> art;
			art.push_back(Articulation::createPose());
			nodes[node]->setModel(art);
			nodes[node]->local_state.articulation[0].expectation = Measurement::getPosQuatFromTransform(Transform3D::Identity());
		}
	}


	Transform3D ArticulatedModel::getNodeGlobalPose(const NodeDescriptor& node){
		if(nodes.count(node) == 0){
			return Transform3D::Identity();
		} else {
			return nodes[node]->getFinalGlobalPose();
		}
	}

	Transform3D ArticulatedModel::getNodeLocalPose(const NodeDescriptor& node){
		if(nodes.count(node) == 0){
			return Transform3D::Identity();
		} else {
			return nodes[node]->getLocalPose();
		}
	}
	//-------------------------------------------------------------------------------------------------------
	//									Private
	//-------------------------------------------------------------------------------------------------------

	void  ArticulatedModel::clearMeasurements() {
		for (auto& node : nodes) {
			node.second->measurements.clear();
		}
	}




}

