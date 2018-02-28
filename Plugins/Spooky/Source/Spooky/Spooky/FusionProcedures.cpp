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
#include "Utilities/CommonMath.h"

namespace spooky{

    //TODO: try this?
    // Node::getRequiredParentsCts(const Measurement::Ptr& m, const Transform3D& toFusionSpace){
    //         Eigen::Vector<float,6> error = toFusionSpace * m->getPosRot() - getGlobalPose().toPosRot();
            
    //         //Iterate through parents until enough flex is found
    //         std::vector<Node::Ptr> parents;
    //         parents.push_back(std::make_shared<Node>(this));
    //         Eigen::Matrix<float,6> varTotal = Eigen::Vector3f::Zero();
    //         Transform3D pose = Transform3D::Identity();
    //         float flex = 0;
    //         const float flex_threshold = 0.9;
    //         while(true){
    //             //Assume decoupling between nodes
    //             pose = parents.back()->getLocalPose() * pose;
    //             varTotal = parents.back()->getLocalPoseVariance(pose,varTotal);
    //             flex = std::exp(-error.transpose()*varTotal*error)
    //             if(flex<flex_threshold){
    //                 //We dont need anymore nodes to fuse data
    //                 break;
    //             } else {
    //                 //We need more nodes
    //                 parents.push_back(parents.back()->parent);
    //             }
    //         }
    //         return parents;
    // }

    //Following method is static for shared ptr reasons
    std::vector<Node::Ptr> Node::getAllParents(){
        std::vector<Node::Ptr> result;
        Node::Ptr n = shared_from_this();
        while(true){
            result.push_back(n);
            if(n->parent == NULL){
                break;
            } else {
                n = n->parent;
            }
        }
        return result;
    }

    std::vector<Node::Ptr> Node::getRequiredChain(const Node::Ptr& destNode, const Measurement::Ptr& m){
        //TODO: support ignoring irrelevant nodes!!!
		Node::Ptr srcNode = shared_from_this();

        //If not in global space, we only receive info about this node
        if(!m->globalSpace){
            std::vector<Node::Ptr> single_node;
            single_node.push_back(srcNode);
            return single_node;
        }
        
        //Get all nodes from the sensor root to the global root node
        std::vector<Node::Ptr> destParents = destNode->getAllParents();
        std::vector<Node::Ptr> srcParents = srcNode->getAllParents();

        int max_search_distance = std::min(destParents.size(),srcParents.size());
        int diverge_point = 0;
        for(int i = 1; i <= max_search_distance; i++){
            if(destParents[destParents.size()-i] != srcParents[srcParents.size()-i]){
                //Tree has diverged at last node
                diverge_point = i-1;
				break;
            }
        }
        //Combine chains to make a chain from src to dest including both
        std::vector<Node::Ptr> node_chain;
        node_chain.insert(node_chain.end(),srcParents.begin(),srcParents.end()-diverge_point);
		//only include destination chain if diverge occurs after matrix start
		if (diverge_point > 0) {
			node_chain.insert(node_chain.end(), destParents.rbegin() + diverge_point - 1, destParents.rend());
		}


        //Iterate through parents until enough flex is found
        //Positional degrees of freedom
        int p_dof = 0;
        //rotational degrees of freedom
        int r_dof = 0;
        //Scale degrees of freedom
        int s_dof = 0;
        //Required dof:
        int p_dof_req = m->getRequiredPDoF();
        int s_dof_req = m->getRequiredSDoF();
        int r_dof_req = m->getRequiredRDoF();

        bool hasLeverChild = false;
		//Iterate down node chain and up dest chain until enough DOF found
        std::vector<Node::Ptr> result;
        for(auto& node : node_chain){
            result.push_back(node);
            //Assume decoupling between nodes
            p_dof += node->getPDoF(hasLeverChild);
            r_dof += node->getRDoF();
            s_dof += node->getSDoF();

            if(!hasLeverChild){
                //If there is much of a translation then it creates a lever which can cause position change
                hasLeverChild = node->getLocalPose().translation().norm() > 0.01;
            }
            
            if(p_dof>=p_dof_req && r_dof >= r_dof_req && s_dof >= s_dof_req){
                //We dont need anymore nodes to fuse data
                break;
            }
        }
        return result;
    }

    void Node::fusePositionMeasurement(const Measurement::Ptr& m_local, const Transform3D& toFusionSpace, const Node::Ptr& rootNode){

        //------------------------------------------------------------------
        //Transform measurement to fusion space
        //------------------------------------------------------------------
        //TODO: optimise: dont transform when possible
        Measurement::Ptr m = std::make_unique<Measurement>(m_local->transform(toFusionSpace));
        //------------------------------------------------------------------

        //------------------------------------------------------------------
        //Get fusion chain
        //------------------------------------------------------------------
        std::vector<Node::Ptr> fusion_chain = getRequiredChain(rootNode,m);
        //------------------------------------------------------------------
       
        //------------------------------------------------------------------
        //Models for Prediction, Measurement and Measurement Jacobian:
        //------------------------------------------------------------------
        auto getPredState = [&m](const std::vector<Node::Ptr>& fusion_chain){
            return getChainPredictedState(fusion_chain, m->getTimestamp());
        };

		std::function<Eigen::VectorXf(const Transform3D&)> transformRepresentation = [](const Transform3D& T) {
			return utility::toAxisAnglePosScale(T);
		};

        auto getMeasJac = [&rootNode, &m, &transformRepresentation](const std::vector<Node::Ptr>& fusion_chain) {
            //JACOBIAN:state -> measurement
            //Get Jacobian for the chain, mapping state to (w,v) global pose
            Eigen::Matrix<float, 9, Eigen::Dynamic> poseJac = getPoseChainJacobian(fusion_chain, m->globalSpace, rootNode->getGlobalPose().inverse(), transformRepresentation);
            Eigen::Matrix<float, 3, Eigen::Dynamic> measurementJacobian = poseJac.block(3,0,3, poseJac.cols());
            return measurementJacobian;
        };

        auto getMeas = [&rootNode, &m](const std::vector<Node::Ptr>& fusion_chain){
            
            Transform3D globalToRootNode = rootNode->getGlobalPose().inverse();
           
            Eigen::VectorXf pstate;
            if (m->globalSpace) {
                pstate = (globalToRootNode * fusion_chain[0]->getGlobalPose()).translation();
            }
            else {
                pstate = fusion_chain[0]->getLocalPose().translation();
            }
            return pstate;
        };
        //------------------------------------------------------------------

        //------------------------------------------------------------------
        //Transform measurement to appropriate coordinates
        //------------------------------------------------------------------
        //THIS MEASUREMENT
        //Measurement information matrix
        State::Parameters measurement(3);
        measurement.expectation = m->getPosition();
        measurement.variance = m->getPositionVar();
        //------------------------------------------------------------------
        
        //------------------------------------------------------------------
        //JOINT CONSTRAINTS
        //------------------------------------------------------------------
        //Read quadratic constraints from joints in chain
        State::Parameters constraints = getChainConstraints(fusion_chain);
        //------------------------------------------------------------------


        computeEKFUpdate(m->getTimestamp(),fusion_chain, measurement, constraints, getPredState, getMeas, getMeasJac);
    }

    void Node::fuseRotationMeasurement(const Measurement::Ptr& m_local, const Transform3D& toFusionSpace, const Node::Ptr& rootNode){

        //Transform measurement to fusion space
        //TODO: optimise: dont transform when possible
        Measurement::Ptr m = std::make_unique<Measurement>(m_local->transform(toFusionSpace));

        //Fuse by modifying some parents if necessary
        std::vector<Node::Ptr> fusion_chain = getRequiredChain(rootNode,m);

        //------------------------------------------------------------------
        //Models for Prediction, Measurement and Measurement Jacobian:
        //------------------------------------------------------------------
        auto getPredState = [&m](const std::vector<Node::Ptr>& fusion_chain){
            return getChainPredictedState(fusion_chain, m->getTimestamp());
        };

		std::function<Eigen::VectorXf(const Transform3D&)> transformRepresentation = [](const Transform3D& T) {
			return Eigen::Quaternionf(T.rotation()).coeffs();
		};

        auto getMeasJac = [&rootNode, &m, &transformRepresentation](const std::vector<Node::Ptr>& fusion_chain) {
            //JACOBIAN:state -> measurement
            //Get Jacobian for the chain, mapping state to (w,v) global pose
            Eigen::Matrix<float, 10, Eigen::Dynamic> poseJac = getPoseChainJacobian(fusion_chain, m->globalSpace, rootNode->getGlobalPose().inverse(), transformRepresentation);
            Eigen::Matrix<float, 4, Eigen::Dynamic> measurementJacobian = poseJac.block(0, 0, 4, poseJac.cols());
            return measurementJacobian;
        };

        auto getMeas = [&rootNode, &m, &transformRepresentation](const std::vector<Node::Ptr>& fusion_chain){

            Transform3D globalToRootNode = rootNode->getGlobalPose().inverse();
            
            Eigen::VectorXf wpstate;
            if (m->globalSpace) {
                wpstate = transformRepresentation(globalToRootNode * fusion_chain[0]->getGlobalPose()).head(3);
            }
            else {
                wpstate = transformRepresentation(fusion_chain[0]->getLocalPose()).head(3);
            }
            return wpstate;

        };

        //JOINT CONSTRAINTS
        //Read quadratic constraints from joints in chain
        State::Parameters constraints = getChainConstraints(fusion_chain);

		Eigen::Vector4f qstate = getMeas(fusion_chain);
		Eigen::Vector4f qmeas = m->getRotation().coeffs();
		
		State::Parameters measurement(4);
		measurement.expectation = utility::quatClosestRepresentation(qmeas,qstate);
		measurement.variance = m->getRotationVar();
		
		
		std::stringstream ss;
		ss << "quat closest rep:" << std::endl;
		ss << "qstate" << qstate.transpose() << std::endl;
		ss << "qmeas" << qmeas.transpose() << std::endl;
		ss << "result " << measurement.expectation.transpose() << std::endl;
		SPOOKY_LOG(ss.str());
                
        //Perform computation
        computeEKFUpdate(m->getTimestamp(), fusion_chain, measurement, constraints, getPredState, getMeas, getMeasJac);
    }

    void Node::fuseRigidMeasurement(const Measurement::Ptr& m_local, const Transform3D& toFusionSpace, const Node::Ptr& rootNode){
        //------------------------------------------------------------------
        //Transform measurement to fusion space
        //------------------------------------------------------------------
        //TODO: optimise: dont transform when possible
        Measurement::Ptr m = std::make_unique<Measurement>(m_local->transform(toFusionSpace));
        //------------------------------------------------------------------

        //------------------------------------------------------------------
        //Get fusion chain
        //------------------------------------------------------------------
        std::vector<Node::Ptr> fusion_chain = getRequiredChain(rootNode,m);
        //------------------------------------------------------------------
       
        //------------------------------------------------------------------
        //Models for Prediction, Measurement and Measurement Jacobian:
        //------------------------------------------------------------------
        auto getPredState = [&m](const std::vector<Node::Ptr>& fusion_chain){
            return getChainPredictedState(fusion_chain, m->getTimestamp());
		};

		std::function<Eigen::VectorXf(const Transform3D&)> transformRepresentation = [](const Transform3D& T) {
			return utility::toAxisAnglePosScale(T);
		};

		auto getMeasJac = [&rootNode, &m, &transformRepresentation](const std::vector<Node::Ptr>& fusion_chain) {
			//JACOBIAN:state -> measurement
			//Get Jacobian for the chain, mapping state to (w,v) global pose
			Eigen::Matrix<float, 9, Eigen::Dynamic> poseJac = getPoseChainJacobian(fusion_chain, m->globalSpace, rootNode->getGlobalPose().inverse(), transformRepresentation);
			Eigen::Matrix<float, 6, Eigen::Dynamic> measurementJacobian = poseJac.block(0, 0, 6, poseJac.cols());
			return measurementJacobian;
		};

        auto getMeas = [&rootNode, &m](const std::vector<Node::Ptr>& fusion_chain){
            Eigen::VectorXf wpstate;
            Transform3D globalToRootNode = rootNode->getGlobalPose().inverse();
            if (m->globalSpace) {
                wpstate = utility::toAxisAnglePos(globalToRootNode * fusion_chain[0]->getGlobalPose());
            }
            else {
                wpstate = utility::toAxisAnglePos(fusion_chain[0]->getLocalPose());
            }
            return wpstate;
		};
        //------------------------------------------------------------------

        //------------------------------------------------------------------
        //Transform measurement to appropriate coordinates
        //------------------------------------------------------------------
        Eigen::VectorXf wpstate = getMeas(fusion_chain);
        //THIS MEASUREMENT
        //WARNING: Quat to be consistent: x,y,z,w is how eigen stores it internally, but its consrtuctor uses Quat(w,x,y,z)    
        //Compute relation between measurement quaternion and twist representation w
        Eigen::Matrix<float, 3, 4> quatToAxisJacobian = utility::getQuatToAxisJacobian(m->getRotation());
        Eigen::MatrixXf sigmaW = quatToAxisJacobian * m->getRotationVar() * quatToAxisJacobian.transpose();
        //Measurement information matrix
        State::Parameters measurement(6);
        Eigen::Matrix<float, 6, 1> wpm = utility::toAxisAnglePos(m->getTransform());
        measurement.expectation.head(3) = utility::twistClosestRepresentation(wpm.head(3),wpstate.head(3));
        measurement.expectation.tail(3) = wpm.tail(3);
        measurement.variance.topLeftCorner(3, 3) = sigmaW;
        measurement.variance.bottomRightCorner(3, 3) = m->getPositionVar();
        //------------------------------------------------------------------
        
        //------------------------------------------------------------------
        //JOINT CONSTRAINTS
        //------------------------------------------------------------------
        //Read quadratic constraints from joints in chain
        State::Parameters constraints = getChainConstraints(fusion_chain);
        //------------------------------------------------------------------


        computeEKFUpdate(m->getTimestamp(),fusion_chain, measurement, constraints, getPredState, getMeas, getMeasJac);
        //DEBUG
  //       std::stringstream ss;
		// ss << std::endl << "wpstate = " << wpstate.transpose() << std::endl;
  //       ss << "measurement = " << measurement.expectation.transpose() << std::endl;
  //       SPOOKY_LOG(ss.str());

    }

    void Node::fuseScaleMeasurement(const Measurement::Ptr& m_local, const Transform3D& toFusionSpace, const Node::Ptr& rootNode){


        //Transform measurement to fusion space
        //TODO: optimise: dont transform when possible
        Measurement::Ptr m = std::make_unique<Measurement>(m_local->transform(toFusionSpace));

        //Fuse by modifying some parents if necessary
        std::vector<Node::Ptr> fusion_chain = getRequiredChain(rootNode,m);

        //------------------------------------------------------------------
        //Models for Prediction, Measurement and Measurement Jacobian:
        //------------------------------------------------------------------
        auto getPredState = [&m](const std::vector<Node::Ptr>& fusion_chain){
            return getChainPredictedState(fusion_chain, m->getTimestamp());
        };

		std::function<Eigen::VectorXf(const Transform3D&)> transformRepresentation = [](const Transform3D& T) {
			return utility::toAxisAnglePosScale(T);
		};

        auto getMeasJac = [&rootNode, &m, &transformRepresentation](const std::vector<Node::Ptr>& fusion_chain) {
            //JACOBIAN:state -> measurement
            //Get Jacobian for the chain, mapping state to (w,v) global pose
            Eigen::MatrixXf poseJac = getPoseChainJacobian(fusion_chain, m->globalSpace, rootNode->getGlobalPose().inverse(), transformRepresentation);
            Eigen::Matrix<float, 3, Eigen::Dynamic> measurementJacobian = poseJac.block(6, 0, 3, poseJac.cols());
            return measurementJacobian;
        };

        auto getMeas = [&rootNode, &m](const std::vector<Node::Ptr>& fusion_chain){

            Eigen::VectorXf pstate;
            Transform3D globalToRootNode = rootNode->getGlobalPose().inverse();
            if (m->globalSpace) {
                //TODO: change to return class so I can use .scale(), .pos(), etc?
                pstate = utility::toAxisAnglePosScale(globalToRootNode * fusion_chain[0]->getGlobalPose()).tail(3);
            }
            else {
                pstate = utility::toAxisAnglePosScale(fusion_chain[0]->getLocalPose()).tail(3);
            }
            return pstate;
        };
        //JOINT CONSTRAINTS
        //Read quadratic constraints from joints in chain
        State::Parameters constraints = getChainConstraints(fusion_chain);

        //THIS MEASUREMENT
        //Measurement information matrix
        State::Parameters measurement(3);
        measurement.expectation = m->getScale();
        measurement.variance = m->getScaleVar();
        
        //Perform computation
        computeEKFUpdate(m->getTimestamp(), fusion_chain, measurement, constraints, getPredState, getMeas, getMeasJac);

    }

	void Node::computeEKFUpdate(
	   const float& timestamp,
       const std::vector<Node::Ptr>& fusion_chain,
       const State::Parameters& measurement, 
       const State::Parameters& constraints, 
       const std::function<State::Parameters(const std::vector<Node::Ptr>&)> getPredictedState,
       const std::function<Eigen::VectorXf(const std::vector<Node::Ptr>&)> getMeasurement,
       const std::function<Eigen::MatrixXf(const std::vector<Node::Ptr>&)> getMeasurementJacobian
    ){
		std::stringstream ss;
		ss << "new Frame" << std::endl;
		//Iterative update
        State::Parameters chainState = getPredictedState(fusion_chain);
		State::Parameters originalChainState = chainState;
		State::Parameters lastChainState = chainState;
		State::Parameters newChainState(chainState.expectation.size());
		
		//Get the predicted measurment
		Eigen::VectorXf predictedMeasurement = getMeasurement(fusion_chain);
		//JACOBIAN:state -> measurement
		//Get Jacobian for the chain, mapping state to (w,v) global pose
		Eigen::MatrixXf measurementJacobian = getMeasurementJacobian(fusion_chain);
		float last_error = 9999999999999999;
		int iterations = 0;

		for (int i = 0; i < 5; i++) {
			iterations++;
            //TODO optimise ekf by using information matrices and inverting covariance per node
            newChainState = customEKFMeasurementUpdate(chainState, constraints, measurement, measurementJacobian, predictedMeasurement);
            //newChainState = EKFMeasurementUpdate(chainState, measurement, measurementJacobian, wpstate);

            chainState.expectation = newChainState.expectation;
            //Move to next approximation, but keep old variances
            setChainState(fusion_chain, chainState, timestamp);
			
			//Prepare for next update:
			predictedMeasurement = getMeasurement(fusion_chain);
			measurementJacobian = getMeasurementJacobian(fusion_chain);
			//Error = 0 when we are at the most likely state
			//Eigen::VectorXf error_vec =
			//			measurementJacobian.transpose() * measurement.variance.inverse() * (predictedMeasurement - measurement.expectation)
			//			+ originalChainState.variance.inverse() * (chainState.expectation - originalChainState.expectation)
			//			+ joint_stiffness * constraints.variance.inverse() * (chainState.expectation - constraints.expectation);
			//float error = error_vec.norm();

			
			//Measurement error:
			float m_error = ((predictedMeasurement - measurement.expectation).transpose() * measurement.variance.inverse() * (predictedMeasurement - measurement.expectation))(0, 0);
			//Prior error
			float p_error = ((chainState.expectation - originalChainState.expectation).transpose() * originalChainState.variance.inverse() * (chainState.expectation - originalChainState.expectation))(0, 0);
			//Constraint Error
			float c_error = (joint_stiffness *(chainState.expectation - constraints.expectation).transpose() * constraints.variance.inverse() * (chainState.expectation - constraints.expectation))(0, 0);
			//Energy:
			float error = m_error + p_error + c_error;

			//ss << "error_vec = " << error_vec.transpose() << std::endl;
			//ss << "error = " << error << std::endl;
			//ss << "m_error = " << m_error << std::endl;
			//ss << "p_error = " << p_error << std::endl;
			//ss << "c_error = " << c_error << std::endl;
			//Keep searching until error increases 
			if (error > last_error) {
				//But we want the last estimate:
				newChainState = lastChainState;
				break;
			}
			//or is very low
			if(error < 1e-6) {
				//We want the current state (aka newChainState)
				break;
			}
			//Update Last variables
			last_error = error;
			lastChainState = newChainState;
        }
        setChainState(fusion_chain, newChainState, timestamp);
		ss << "bone = " << fusion_chain[0]->desc.name << std::endl;
		ss << "newChainState = " << newChainState.expectation.transpose() << std::endl;
		ss << "iterations = " << iterations << std::endl;
		SPOOKY_LOG(ss.str());
	};

    Node::State::Parameters Node::customEKFMeasurementUpdate( const State::Parameters& prior, const State::Parameters& constraints, const State::Parameters& measurement,
                                                        const Eigen::MatrixXf& measurementJacobian, const Eigen::VectorXf& state_measurement)
    {
		//TODO optimise custom ekf by using information matrices and inverting covariance per node
		//TODO: change to proper KF algorithm
        assert(prior.size() == measurementJacobian.cols() && measurement.size() == measurementJacobian.rows() == state_measurement.size() == measurement.size());
        //Prior information matrix
        Eigen::MatrixXf sigmaP_info = prior.variance.inverse();
        //Constraint information matrix
        Eigen::MatrixXf sigmaC_info = constraints.variance.inverse();

        //New state initialisation:
        State::Parameters posterior(prior.expectation.size());

        //New variance (extended kalman filter measurement update)
        Eigen::MatrixXf sigmaM_info = measurement.variance.inverse();
        posterior.variance = (measurementJacobian.transpose() * sigmaM_info * measurementJacobian + sigmaP_info + joint_stiffness * sigmaC_info).inverse();
        
        //Expectation update components
        Eigen::VectorXf mVector = measurement.expectation - state_measurement + measurementJacobian * prior.expectation;
        Eigen::VectorXf measurementUpdate = measurementJacobian.transpose() * sigmaM_info * mVector;
        Eigen::VectorXf priorUpdate = sigmaP_info * prior.expectation;
        Eigen::VectorXf constraintUpdate = sigmaC_info * constraints.expectation;

        //New state
        posterior.expectation = posterior.variance * (priorUpdate + joint_stiffness * constraintUpdate + measurementUpdate);

        return posterior;
    };

	Node::State::Parameters Node::EKFMeasurementUpdate(const State::Parameters& prior, const State::Parameters& measurement,
		const Eigen::MatrixXf& measurementJacobian, const Eigen::VectorXf& state_measurement)
	{
		//TODO: change to proper KF algorithm
		assert(prior.size() == measurementJacobian.cols() && measurement.size() == measurementJacobian.rows() == state_measurement.size() == measurement.size());

		//Variables as named pg 59 Thrun, Probabalistic Robotics
		const auto& H = measurementJacobian;
		const auto& SigmaBar = prior.variance;
		const auto& Q = measurement.variance;
		const int n = prior.expectation.size();
		State::Parameters posterior(n);

		Eigen::MatrixXf K = SigmaBar * H.transpose() * (H * SigmaBar * H.transpose() + Q).inverse();
		//New state
		posterior.variance = (Eigen::MatrixXf::Identity(n,n) - K * H) * SigmaBar;
        Eigen::VectorXf error = (measurement.expectation - state_measurement);
        Eigen::VectorXf delta = K * error;
		posterior.expectation = prior.expectation + K * (measurement.expectation - state_measurement);
		return posterior;
	};


	//TODO: always update, even when no measurements recieved
    Node::State::Parameters Node::getChainPredictedState(const std::vector<Node::Ptr>& fusion_chain, const float& timestamp){
        //CURRENT STATE
        State::Parameters chainState = getChainState(fusion_chain);
        //Process noise: max of ten seconds variance added
		Eigen::MatrixXf time_matrix = Eigen::MatrixXf::Zero(chainState.expectation.size(), chainState.expectation.size());
		time_matrix.diagonal() = (timestamp * Eigen::VectorXf::Ones(chainState.expectation.size()) - getChainTimeSinceUpdated(fusion_chain));
        Eigen::MatrixXf process_noise = getChainProcessNoise(fusion_chain).variance * time_matrix;
        //Add process noise
		//Use arrays for componentwise multiplication
		int dim = chainState.expectation.size();
        Eigen::MatrixXf velocityMatrix = time_matrix * getChainVelocityMatrix(fusion_chain);
		Eigen::MatrixXf updateMatrix = velocityMatrix + Eigen::MatrixXf::Identity(dim, dim);
        
        //DEBUG
  //      std::stringstream ss;
  //      ss << "chainState.expectation before = " << std::endl << chainState.expectation.transpose() << std::endl;
  //      //TODO: support velocity
  //      ss << "time_matrix  = " << std::endl << time_matrix << std::endl;
  //      ss << "velmatrix  = " << std::endl << velocityMatrix << std::endl;
  //      ss << "updateMatrix  = " << std::endl << updateMatrix << std::endl;
		//SPOOKY_LOG(ss.str());
        //DEBUG END

		chainState.variance = updateMatrix * chainState.variance * updateMatrix.transpose() + process_noise;
		chainState.expectation = updateMatrix * chainState.expectation;
        return chainState;
    }

	Eigen::MatrixXf Node::getPoseChainJacobian(const std::vector<Node::Ptr>& fusion_chain, 
																	   const bool& globalSpace, 
																	   const Transform3D& globalToRootNode,
																	   const std::function<Eigen::VectorXf(const Transform3D&)>& transformRepresentation) {
		//Precompute Jacobian size
		int inputDimension = 0;
		for (auto& node : fusion_chain) {
			inputDimension += node->getDimension();
		}
        //Reset for actual calculation
		float h = 0.0001;

		//3DOF for each rot, pos, scale
		int m_dim = transformRepresentation(Transform3D::Identity()).size();
		Eigen::MatrixXf J(m_dim, inputDimension);
		Transform3D childPoses = Transform3D::Identity();

		int block = 0;
		for (auto& node : fusion_chain) {
			Transform3D parentPoses = (node->parent != NULL && globalSpace) ? globalToRootNode * node->parent->getGlobalPose() : Transform3D::Identity();
		
			//Lambda to be differentiated
			auto mapToGlobalPose = [&childPoses, &parentPoses, &node, &transformRepresentation](const Eigen::VectorXf& theta) {
				//return utility::toAxisAnglePosScale(parentPoses * node->getLocalPoseAt(theta) * childPoses);
				return transformRepresentation(parentPoses * node->getLocalPoseAt(theta) * childPoses);
			};

			//Loop through all dof of this node and get the jacobian (w,p) entries for each dof
			int dof = node->getDimension();

			//Watch out for block assignments - they cause horrible hard to trace memory errors if not sized properly
			J.block(0, block, m_dim, dof) = utility::numericalVectorDerivative<float>(mapToGlobalPose, node->getState().expectation, h);

			block += dof;

			//Update child poses for next node
			childPoses = node->getLocalPose() * childPoses;
		}
		return J;
	}

}