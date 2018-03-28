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
		cachedPose = Transform3D::Identity();
		parent_desc = NodeDescriptor("");
	}

	Transform3D Node::getFinalGlobalPose(){
		return getGlobalPose();
	}

	Transform3D Node::getLocalPose() const{
		Transform3D pose = Transform3D::Identity();
		for (int i = 0; i < articulations.size(); i++) {
			pose = pose * articulations[i].getTransform(local_state.articulation[i].expectation);
		}	
		return pose;
	}

	Transform3D Node::getLocalPoseAt(const Eigen::VectorXf & theta) const
	{
		Transform3D pose = Transform3D::Identity();
		assert(theta.size() == getDimension());
		int block_start = 0;
		for (int i = 0; i < articulations.size(); i++) {
			int block_size = local_state.articulation[i].expectation.size();
			pose = pose * articulations[i].getTransform<float>(theta.block(block_start,0,block_size,1));
			block_start += block_size;
		}
		return pose;
	}

	float Node::getNodeLastFusionTime(){
		return local_state.last_update_time - local_state.smallest_latency;
	}

	//TODO:reimplement
	//
	//Transform3D Node::getLocalPosePredicted(const float& deltaT){
	//	return getLocalPosePredictedAt(getState().expectation,deltaT);
	//}

	//Transform3D Node::getLocalPosePredictedAt(const Eigen::VectorXf& theta, const float& deltaT){
	//	Transform3D pose = Transform3D::Identity();
	//	int dim = getDimension();
	//	getPredictedState(deltaT)
	//	return getLocalPoseAt();
	//}
	//
	//Transform3D Node::getGlobalPosePredicted(const float& deltaT) {
	//if (parent == NULL) {
	//return getLocalPosePredicted(deltaT);
	//}
	//return parent->getGlobalPosePredicted(deltaT) * getLocalPosePredicted(deltaT);
	//}
	Eigen::Matrix<float,6,6> Node::getLocalPoseVariance() const {
		Eigen::Matrix<float,6,6> var = Eigen::Matrix<float,6,6>::Zero();
		for (int i = 0; i < articulations.size(); i++) {
			//Assume decoupling of variances between articulations - only true for linear cases (position only)
			//TODO: implement this method in articulation
			assert(false);
			//var += articulations[i].getPoseVariance(local_state.articulation[i].expectation,local_state.articulation[i].variance);
		}	
		return var;
	}

	
	int Node::getPDoF(bool hasLeverChild) const {
		int pdof = 0;
		for (int i = 0; i < articulations.size(); i++)  {
			pdof += articulations[i].getPDoF(hasLeverChild);
		}
		return pdof;
	}
	
	int Node::getRDoF() const {
		int rdof = 0;
		for (int i = 0; i < articulations.size(); i++) {
			rdof += articulations[i].getRDoF();
		}
		return rdof;
	}

	int Node::getSDoF() const {
		int sdof = 0;
		for (int i = 0; i < articulations.size(); i++) {
			sdof += articulations[i].getSDoF();
		}
		return sdof;
	}

	int Node::getDimension()  const {
		int dim = 0;
		for (int i = 0; i < articulations.size(); i++) {
			dim += local_state.articulation[i].expectation.size();
		}
		return dim;
	}


	//Set state parameters
	void Node::setState(const State::Parameters& new_state, const float& t){
		int pos = 0;
		for(int i = 0; i < articulations.size(); i++){
			int dim = local_state.articulation[i].expectation.size();
			State::Parameters state = new_state.getSubstate(pos, dim);
			//TODO: make this less gross - call articulation.constrain(...);
			if (articulations[i].getType() == Articulation::Type::BONE || 
				articulations[i].getType() == Articulation::Type::POSE) {
				//If there is a rotation component, restrict state to less than pi magnitude
				state.expectation.head(3) = utility::twistClosestRepresentation(state.expectation.head(3), Eigen::Vector3f::Zero());
			}
			local_state.articulation[i] = state;
			pos += dim;
		}
		//Indicate that this node needs its local pose to be recomputed
		recacheRequired = true;
		local_state.last_update_time = t;
	}

	Node::State::Parameters Node::getState() const {
		//Construct new parameters set for combined articulations
		State::Parameters p(getDimension());
		int pos = 0;
		for(int i = 0; i < articulations.size(); i++){
			int dim = local_state.articulation[i].expectation.size();
			p.insertSubstate(pos,local_state.articulation[i]);
			pos += dim;
		}
		return p;
	}

	Node::State::Parameters Node::getConstraints() const {
		//Construct new parameters set for combined articulations
		State::Parameters p(getDimension());
		int pos = 0;
		for(int i = 0; i < articulations.size(); i++){
			int dim = local_state.constraints[i].expectation.size();
			p.insertSubstate(pos,local_state.constraints[i]);
			pos += dim;
		}
		return p;
	}

	Node::State::Parameters Node::getProcessNoise() const {
		//Construct new parameters set for combined articulations
		State::Parameters p(getDimension());
		int pos = 0;
		for(int i = 0; i < articulations.size(); i++){
			int dim = local_state.process_noise[i].expectation.size();
			p.insertSubstate(pos,local_state.process_noise[i]);
			pos += dim;
		}
		return p;
	}

	Node::State::Parameters Node::getTimeSinceUpdated() const {
		//Construct new parameters set for combined articulations
		State::Parameters p(getDimension());
		int pos = 0;
		for (int i = 0; i < articulations.size(); i++) {
			int dim = local_state.articulation[i].expectation.size();
			p.expectation.block(pos, 0, dim, 1) = Eigen::VectorXf::Ones(dim) * local_state.last_update_time;
			pos += dim;
		}
		return p;
	}
	

	Node::State::Parameters Node::getPredictedState(const float& timestamp) const {
		State::Parameters pstate(getDimension());
		int pos = 0;
		for (int i = 0; i < articulations.size(); i++) {
			int dim = local_state.articulation[i].expectation.size();
			float t = timestamp - local_state.last_update_time;
			pstate.insertSubstate(pos, getPredictionForArticulation(articulations[i],local_state.articulation[i],t));
			pos += dim;
		}
		return pstate;
	}

	Node::State::Parameters Node::getVelocityMatrix() const {
		//Construct new parameters set for combined articulations
		State::Parameters p(getDimension());
		//Zero
		p.variance = Eigen::MatrixXf::Zero(getDimension(),getDimension());
		//if velocity not modelled, return zeros
		if(!local_state.modelVelocity) return p;
		//Otherwise, return ones for second half of each vector
		int pos = 0;
		for (int i = 0; i < articulations.size(); i++) {
			int dim = local_state.articulation[i].expectation.size();
			int dim_2 = dim / 2;
			//Top right corner of each articulation is identity:
			//[ 0, I
			//  0, 0]
			p.variance.block(0,pos+dim_2, dim_2, dim_2) = Eigen::MatrixXf::Identity(dim_2,dim_2);
			pos += dim;
		}
		return p;
	}


	Node::State::Parameters Node::getChainParameters(std::function<Node::State::Parameters (Node&)> getParams, const std::vector<Node::Ptr>& node_chain) {
		//Precompute State size
		int inputDimension = 0;
		for (auto& node : node_chain) {
			inputDimension += node->getDimension();
		}
		//Reset for actual calculation
		State::Parameters result(inputDimension);
		int position = 0;
		for (auto& node : node_chain) {
			int dim = node->getDimension();
			result.insertSubstate(position,getParams(*node));
			position += dim;
		}
		return result;
	}

	Node::State::Parameters Node::getChainState(const std::vector<Node::Ptr>& node_chain) {		
		return getChainParameters(
			std::function<Node::State::Parameters(Node&)>(&Node::getState)
			, node_chain);
	}

	Node::State::Parameters Node::getChainConstraints(const std::vector<Node::Ptr>& node_chain) {		
		return getChainParameters(
			std::function<Node::State::Parameters(Node&)>(&Node::getConstraints)
			, node_chain);
	}

	Node::State::Parameters Node::getChainProcessNoise(const std::vector<Node::Ptr>& node_chain) {		
		return getChainParameters(
			std::function<Node::State::Parameters(Node&)>(&Node::getProcessNoise)
			, node_chain);
	}

	Eigen::VectorXf Node::getChainTimeSinceUpdated(const std::vector<Node::Ptr>& node_chain) {
		return getChainParameters(
			std::function<Node::State::Parameters(Node&)>(&Node::getTimeSinceUpdated)
			, node_chain).expectation;
	}

	Eigen::MatrixXf Node::getChainVelocityMatrix(const std::vector<Node::Ptr>& node_chain) {
		return getChainParameters(
			std::function<Node::State::Parameters(Node&)>(&Node::getVelocityMatrix)
			, node_chain).variance;
	}

	Node::State::Parameters Node::getPredictionForArticulation(const Articulation& art, const State::Parameters& state, const float& t) {
		State::Parameters new_state(state.expectation.size());
		auto updateFunc = [&t,&art](const Eigen::VectorXf& x) {
			return art.getPredictedExpectation(x, t);
		};
		new_state.expectation = updateFunc(state.expectation);
		Eigen::MatrixXf Ju = utility::numericalVectorDerivative<float>(updateFunc, state.expectation);
		new_state.variance = Ju * state.variance * Ju.transpose();
		return new_state;
	}

	void Node::setChainState(const std::vector<Node::Ptr>& node_chain, const State::Parameters& state_params, const float& t) {
		int last_block_end = 0;
		for (auto& node : node_chain) {
			int dim = node->getDimension();
			node->setState(state_params.getSubstate(last_block_end,dim),t);
			last_block_end += dim;
		}
	}

	void Node::setModel(std::vector<Articulation> art, const bool& modelVelocity){
		articulations = art;
		local_state.modelVelocity = modelVelocity;
		//Clear old model
		local_state.articulation.clear();
		int max_n_rows = 1;
		for(int i = 0; i < articulations.size(); i++){	
			auto init = Articulation::getInitialState(articulations[i].getType());
			//State is twice as big if velocity included
			local_state.articulation.push_back(Node::State::Parameters(init.size() * (modelVelocity ? 2 : 1)));
			local_state.constraints.push_back(Node::State::Parameters(init.size() * (modelVelocity ? 2 : 1)));
			local_state.process_noise.push_back(Node::State::Parameters(init.size() * (modelVelocity ? 2 : 1)));
			//Init model values for state
			local_state.articulation[i].expectation.head(init.size()) = init;
			//Init zeros for velocity
			if(modelVelocity){
				local_state.articulation[i].expectation.tail(init.size()) = Eigen::VectorXf::Zero(init.size());
			}
			//TODO: generate covariance initial per aticulation
			local_state.articulation[i].variance = initial_covariance * Eigen::MatrixXf::Identity(local_state.articulation[i].expectation.size(),
																							      local_state.articulation[i].expectation.size());
		}
	}

	void Node::setConstraintForArticulation(const int& i, const Node::State::Parameters& c){
		local_state.constraints[i] = c;
	}

	void Node::setConstraints(const Node::State::Parameters& c){
		int pos = 0;
		for(int i = 0; i < articulations.size(); i++){
			int dim = local_state.articulation[i].expectation.size();
			local_state.constraints[i] = c.getSubstate(pos,dim);
			pos += dim;
		}
	}

	void Node::setProcessNoiseForArticulation(const int& i, const Node::State::Parameters& p){
		local_state.process_noise[i] = p;
	}

	void Node::setProcessNoises(const Node::State::Parameters& p){
		int pos = 0;
		for(int i = 0; i < articulations.size(); i++){
			int dim = local_state.articulation[i].expectation.size();
			local_state.process_noise[i] = p.getSubstate(pos,dim);
			pos += dim;
		}
	}

	void Node::fuse(const Calibrator& calib, const SystemDescriptor& referenceSystem, const std::map<NodeDescriptor,Node::Ptr>& nodes){
		
		//If this node has a parent, recursively fuse until we know its transform
		//TODO: cache when we are all out of measurements
		if (parent != NULL) {
			parent->fuse(calib, referenceSystem, nodes);
		}
		float smallest_latency = 100000;
		for(auto& m : measurements){
			//Throwout bad measurements
			//TODO: use confidence better
			if (m->confidence < 0.25) {
				continue;
			}

			//Get mapping to correct reference frame
			//TODO: Optimise this access somehow?
			Transform3D toFusionSpace = Transform3D::Identity();
			CalibrationResult calibResult = calib.getResultsFor(m->getSystem(),referenceSystem);
			if (calibResult.calibrated()) {
				toFusionSpace = calibResult.transform;
			}
			
			NodeDescriptor rootName = m->getSensor()->getRootNode();
			//If no root node then go to static root
			Node::Ptr rootNode = nodes.count(rootName) > 0 ? nodes.at(rootName) : nodes.at(SPOOKY_WORLD_ROOT_DESC);
			//TODO: Make sure the root node is fused and avoid loops
			//rootNode->fuse(calib,referenceSystem,nodes);

			switch (m->type) {
				case(Measurement::Type::POSITION):
					fusePositionMeasurement(m, toFusionSpace, rootNode);
					break;
				case(Measurement::Type::ROTATION):
					if (m->sensorDrifts && measurementBuffer.count(m->getSensor()) > 0) {
						fuseDeltaRotationMeasurement(m, toFusionSpace, rootNode);
					}
					else {
						fuseRotationMeasurement(m, toFusionSpace, rootNode);
					}
					break;
				case(Measurement::Type::RIGID_BODY):
					fuseRigidMeasurement(m,toFusionSpace,rootNode);
					break;
				case(Measurement::Type::SCALE):
					fuseScaleMeasurement(m,toFusionSpace,rootNode);
					break;
			}
			smallest_latency = std::fmin(smallest_latency,m->getLatency());
		}
		local_state.smallest_latency = smallest_latency;
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
		bool parentChanged = parentHashesChanged();
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

	bool Node::parentHashesChanged() {
		if (parent == NULL) return false;
		return parent->getCachedPoseHash() != lastParentHash || parent->parentHashesChanged();
	}


	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									ArticulatedModel
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	//-------------------------------------------------------------------------------------------------------
	//									Public
	//-------------------------------------------------------------------------------------------------------
	ArticulatedModel::ArticulatedModel(){
		nodes[SPOOKY_WORLD_ROOT_DESC] = std::make_unique<Node>();
		nodes[SPOOKY_WORLD_ROOT_DESC]->desc = SPOOKY_WORLD_ROOT_DESC;
	}
	void ArticulatedModel::setReferenceSystem(const SystemDescriptor& s) {
		reference_system = s;
	}

	void ArticulatedModel::addNode(const NodeDescriptor & node, const NodeDescriptor & parent) {
		//This line initialises the node entry if not already initialised
		if (node == SPOOKY_WORLD_ROOT_DESC) {
			//TODO throw exceptions
			SPOOKY_LOG("******************************\n\n\n\n\n\n\n\n\n\n FATAL ERROR ArticulatedModel::addNode() - line " + __LINE__ + std::string(" " + SPOOKY_WORLD_ROOT_DESC.name + " is a protected node name!!!!\n\n\n\n\n\n\n\n\n\n******************************"));
			throw "******************************\n\n\n\n\n\n\n\n\n\n FATAL ERROR ArticulatedModel::addNode() - line " + __LINE__ + std::string(" " + SPOOKY_WORLD_ROOT_DESC.name + " is a protected node name!!!!\n\n\n\n\n\n\n\n\n\n******************************");
		}
		utility::safeAccess(nodes, node)->desc = node;
		if (parent.name.size() == 0){
			nodes[node]->parent_desc = SPOOKY_WORLD_ROOT_DESC;
		}
		else {
			nodes[node]->parent_desc = parent;
		}
	}
	
	void ArticulatedModel::enumerateHeirarchy(){
		for (auto& node : nodes) {
			NodeDescriptor parent = node.second->parent_desc;
			
			if(nodes.count(parent) != 0 && parent != node.second->desc){
				node.second->parent = nodes[parent];
			}
		}
		std::set<NodeDescriptor> roots;
		for (auto & node : nodes) {
			roots.insert(node.second->getAllParents().back()->desc);
		}
		if (roots.size() > 1) {
			//TODO throw exceptions
			SPOOKY_LOG("******************************\n\n\n\n\n\n\n\n\n\n ArticulatedModel::enumerateHeirarchy() - line " + __LINE__ + std::string(" multiple root nodes found. This is not currently supported!!!!\n\n\n\n\n\n\n\n\n\n******************************"));
			throw ("******************************\n\n\n\n\n\n\n\n\n\n ArticulatedModel::enumerateHeirarchy() - line " + __LINE__ + std::string(" multiple root nodes found. This is not currently supported!!!!\n\n\n\n\n\n\n\n\n\n******************************"));
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
			node.second->fuse(calib, reference_system, nodes);
		}
		clearMeasurements();
	}

	void ArticulatedModel::setFixedNode(const NodeDescriptor& node, const Transform3D& pose) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createFixed(pose));
		nodes[node]->setModel(art, false);
		nodes[node]->setState(Node::State::Parameters(0),0);
	}


	void ArticulatedModel::setBoneForNode(const NodeDescriptor& node, const Transform3D& boneTransform,
										  const Node::State::Parameters& constraints, const Eigen::MatrixXf& process_noise, const bool& modelVelocity) {
		//Create articulation
		std::vector<Articulation> art;
		art.push_back(Articulation::createBone(boneTransform.translation()));

		//Set initial state
		Eigen::Vector3f scale;
		Eigen::Matrix3f R = boneTransform.matrix().topLeftCorner(3, 3);
		Eigen::Vector3f startState = utility::toAxisAngle(R,&scale);
		
		Eigen::Vector3f defaultScale;
		defaultScale.setOnes();
		if(!scale.isApprox(defaultScale,0.001)){
			SPOOKY_LOG("WARNING ArticulatedModel::setBoneForNode - scale of bone " + node.name + " is non unit, adding fixed scale transform");
			Transform3D scaleT = Transform3D::Identity();
			scaleT.matrix().diagonal().head(3) = scale;
			art.push_back(Articulation::createFixed(scaleT));
		}


		nodes[node]->setModel(art,modelVelocity);
		Node::State::Parameters initial = nodes[node]->getState();
		initial.expectation.head(3) = startState;
		nodes[node]->setState(initial,0);
		
		if(constraints.expectation.size() != nodes[node]->getDimension()){
			SPOOKY_LOG("ERROR - constraint sizes do not match node dimension in setBoneForNode for node " + node.name + ".  Dimension is " + std::to_string(constraints.expectation.size()) + " but should be " + std::to_string(nodes[node]->getDimension()) + (modelVelocity ? " (modelling velocity)" : " (NOT modelling velocity)"));
			//TODO:
			//SPOOKY_CLEAN_EXIT();
		}

		if(process_noise.cols() != nodes[node]->getDimension()){
			SPOOKY_LOG("ERROR - process noise dimension does not match node dimension in setBoneForNode for node " + node.name + 
				".  PNDimension is " + std::to_string(process_noise.cols()) + "x" +std::to_string(process_noise.cols()) + 
				" but should be " + std::to_string(nodes[node]->getDimension()) + (modelVelocity ? " (modelling velocity)" : " (NOT modelling velocity)"));
		}


		auto transformFunc = [&boneTransform](const Eigen::Vector3f& w) {
			Eigen::Matrix3f Rc = utility::rodriguezFormula<float>(w);
			return utility::toAxisAngle(Eigen::Matrix3f(boneTransform.rotation() * Rc));
		};
		//Constraint is relative to bone default space
		Node::State::Parameters constraintsDefaultSpace = constraints;
		constraintsDefaultSpace.expectation.head(3) = transformFunc(constraints.expectation.head(3));
		Eigen::Matrix3f J = utility::numericalVectorDerivative<float>(transformFunc, constraints.expectation.head(3));
		constraintsDefaultSpace.variance.topLeftCorner(3, 3) = J * constraints.variance.topLeftCorner(3, 3) * J.transpose();
		if (modelVelocity) {
			constraintsDefaultSpace.expectation.tail(3) = J * constraintsDefaultSpace.expectation.tail(3);
			constraintsDefaultSpace.variance.bottomRightCorner(3, 3) = J * constraints.variance.bottomRightCorner(3, 3) * J.transpose();
		}
		nodes[node]->setConstraints(constraintsDefaultSpace);
		Node::State::Parameters PN(nodes[node]->getDimension());
		PN.variance = process_noise;
		nodes[node]->setProcessNoises(PN);
	}


	void ArticulatedModel::setPoseNode(const NodeDescriptor& node, const Transform3D& poseTransform,
									   const Node::State::Parameters& constraints, const Eigen::MatrixXf& process_noise, const bool& modelVelocity) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createPose());

		//Set initial state
		Eigen::Vector3f scale;
		Eigen::VectorXf startState = utility::toAxisAnglePos(poseTransform, &scale);

		Eigen::Vector3f defaultScale;
		defaultScale.setOnes();
		if(!scale.isApprox(defaultScale, 0.001)){
			SPOOKY_LOG("WARNING ArticulatedModel::setPoseNode - scale of bone " + node.name + " is non unit, adding fixed scale transform");
			Transform3D scaleT = Transform3D::Identity();
			scaleT.matrix().diagonal().head(3) = scale;
			art.push_back(Articulation::createFixed(scaleT));
		}

		nodes[node]->setModel(art,modelVelocity);
		Node::State::Parameters initial = nodes[node]->getState();
		initial.expectation.head(6) = startState;
		nodes[node]->setState(initial,0);

		if(constraints.expectation.size() != nodes[node]->getDimension()){
			SPOOKY_LOG("ERROR - constraint sizes do not match node dimension in setPoseNode for node " + node.name + ".  Dimension is " + std::to_string(constraints.expectation.size()) + " but should be " + std::to_string(nodes[node]->getDimension()) + (modelVelocity ? " (modelling velocity)" : " (NOT modelling velocity)"));
			//TODO:
			//SPOOKY_CLEAN_EXIT();
		}
		if(process_noise.cols() != nodes[node]->getDimension()){
			SPOOKY_LOG("ERROR - process noise dimension does not match node dimension in setPoseNode for node " + node.name + 
				".  PNDimension is " + std::to_string(process_noise.cols()) + "x" +std::to_string(process_noise.cols()) + 
				" but should be " + std::to_string(nodes[node]->getDimension()) + (modelVelocity ? " (modelling velocity)" : " (NOT modelling velocity)"));
		}

		nodes[node]->setConstraints(constraints);
		Node::State::Parameters PN(nodes[node]->getDimension());
		PN.variance = process_noise;
		nodes[node]->setProcessNoises(PN);
	}
	
	void ArticulatedModel::setScalePoseNode(const NodeDescriptor & node, const Transform3D& poseTransform,
											const Node::State::Parameters& constraints, const Eigen::MatrixXf& process_noise, const bool& modelVelocity) {
		std::vector<Articulation> art;
		art.push_back(Articulation::createPose());
		//Scale in local space x'=T*S*x
		art.push_back(Articulation::createScale());
		nodes[node]->setModel(art, modelVelocity);

		//Set initial state
		Node::State::Parameters initial = nodes[node]->getState();
		Eigen::VectorXf aaps = utility::toAxisAnglePosScale(poseTransform);
		initial.expectation.head(6) = aaps.head(6);
		if (modelVelocity) {
			initial.expectation.segment<3>(12) = aaps.tail(3);
		}
		else {
			initial.expectation.segment<3>(6) = aaps.tail(3);
		}
		nodes[node]->setState(initial, 0);

		if(constraints.expectation.size() != nodes[node]->getDimension()){
			SPOOKY_LOG("ERROR - constraint sizes do not match node dimension in setScalePoseNode for node " + node.name + ".  Dimension is " + std::to_string(constraints.expectation.size()) + " but should be " + std::to_string(nodes[node]->getDimension()) + (modelVelocity ? " (modelling velocity)" : " (NOT modelling velocity)"));
			//TODO:
			//SPOOKY_CLEAN_EXIT();
		}

		if(process_noise.cols() != nodes[node]->getDimension()){
			SPOOKY_LOG("ERROR - process noise dimension does not match node dimension in setScalePoseNode for node " + node.name + 
				".  PNDimension is " + std::to_string(process_noise.cols()) + "x" +std::to_string(process_noise.cols()) + 
				" but should be " + std::to_string(nodes[node]->getDimension()) + (modelVelocity ? " (modelling velocity)" : " (NOT modelling velocity)"));
		}

		nodes[node]->setConstraints(constraints);
		Node::State::Parameters PN(nodes[node]->getDimension());
		PN.variance = process_noise;
		nodes[node]->setProcessNoises(PN);
	}

	void ArticulatedModel::setJointStiffness(const NodeDescriptor & node, const float& stiffness) {
		utility::safeAccess(nodes,node)->joint_stiffness = stiffness;
	}

	void ArticulatedModel::setAllJointStiffness(const float& stiffness) {
		for (auto& node : nodes) {
			node.second->joint_stiffness = stiffness;
		}
	}

	//TODO:remove generic nodes
	void ArticulatedModel::addGenericNode(const NodeDescriptor & node) {
		if (nodes.count(node) == 0) {
			addNode(node, NodeDescriptor("root"));
			std::vector<Articulation> art;
			art.push_back(Articulation::createPose());
			nodes[node]->setModel(art, false);
			//nodes[node]->local_state.articulation[0].expectation = Measurement::getPosQuatFromTransform(Transform3D::Identity());
		}
	}


	Transform3D ArticulatedModel::getNodeGlobalPose(const NodeDescriptor& node){
		if(nodes.count(node) == 0){
			SPOOKY_LOG("WARNING - Transform3D ArticulatedModel::getNodeGlobalPose(" + node.name + ") - node does not exist");
			return Transform3D::Identity();
		} else {
			return nodes[node]->getFinalGlobalPose();
		}
	}

	Transform3D ArticulatedModel::getNodeLocalPose(const NodeDescriptor& node){
		if(nodes.count(node) == 0){
			SPOOKY_LOG("WARNING - Transform3D ArticulatedModel::getNodeLocalPose(" + node.name + ") - node does not exist");
			return Transform3D::Identity();
		} else {
			return nodes[node]->getLocalPose();
		}
	}

	float ArticulatedModel::getNodeLastFusionTime(const NodeDescriptor& node){
		if(nodes.count(node) == 0){
			SPOOKY_LOG("WARNING - Transform3D ArticulatedModel::getNodeLocalPose(" + node.name + ") - node does not exist");
			return 0;
		} else {
			return nodes[node]->getNodeLastFusionTime();
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

