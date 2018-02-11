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

    int Node::getRequiredChainLength(const Measurement::Ptr& m){
        //Iterate through parents until enough flex is found
        int nodecount = 1;
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

        const float flex_threshold = 0.9;
        bool hasLeverChild = false;
		//TODO: do a better way?
		Node* node = this;
        while(true){
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
            } else if(node->parent == NULL){
                //We are out of nodes! the data cant be fused properly
                //TODO: handle -1 case
                //return -1;
                return nodecount;
            } else {
                //We need more nodes
				node = node->parent.get();
				nodecount++;
            }
        }
        return nodecount;
    }

    void Node::fusePositionMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

        Eigen::VectorXf pstate;
        int fusion_chain = 1;

        if (m->globalSpace) {
            pstate = getGlobalPose().translation();
            //Fuse by modifying some parents if necessary
			fusion_chain = 2;// getRequiredChainLength(m);
        }
        else {
            pstate = getLocalPose().translation();
        }

        //CURRENT STATE
        State::Parameters chainState = getChainState(fusion_chain);
        //Process noise: max of ten seconds variance added
        Eigen::MatrixXf process_noise = getChainProcessNoise(fusion_chain).variance * std::fmin(10,m->getTimestamp() - local_state.last_update_time);
        chainState.variance += process_noise;
                    
        //JOINT CONSTRAINTS
        //Read quadratic constraints from joints in chain
        State::Parameters constraints = getChainConstraints(fusion_chain);

        //THIS MEASUREMENT
        //Measurement information matrix
        State::Parameters measurement(3);
        measurement.expectation = m->getPosition();
        measurement.variance = m->getPositionVar();
        
        //JACOBIAN:state -> measurement
        //Get Jacobian for the chain, mapping state to (w,v) global pose
        //TODO:
        // Eigen::Matrix<float, 3, Eigen::Dynamic> measurementJacobian = getPositionChainJacobian(fusion_chain,m->globalSpace);
        Eigen::Matrix<float, 9, Eigen::Dynamic> poseJac = getPoseChainJacobian(fusion_chain,m->globalSpace);
        Eigen::Matrix<float, 3, Eigen::Dynamic> measurementJacobian = poseJac.block(3,0,3, poseJac.cols());

        // State::Parameters newChainState = customEKFMeasurementUpdate(chainState, constraints, measurement, measurementJacobian, pstate);
        //State::Parameters newChainState = EKFMeasurementUpdate(chainState, measurement, measurementJacobian, pstate);

		State::Parameters newChainState = chainState;
		float alpha = 1;
		//newChainState.expectation += - alpha * measurementJacobian.transpose() * (pstate - measurement.expectation);
		newChainState.expectation[4] += 0.01;

        std::stringstream ss;
		ss << std::endl << "pstate = " << std::endl << pstate.transpose() << std::endl;
		ss << std::endl << "measurement.expectation = " << std::endl << measurement.expectation.transpose() << std::endl;
        // ss << std::endl << "constraints = " << std::endl << constraints.variance << std::endl;
        // //ss << std::endl << "sigmaM_info = " << std::endl << sigmaM_info << std::endl;
        // //ss << std::endl << "sigmaP_info = " << std::endl << sigmaP_info << std::endl;
        // //ss << std::endl << "sigmaC_info * joint_stiffness = " << std::endl << sigmaC_info * joint_stiffness << std::endl;
        ss << std::endl << "chainState = " << std::endl << chainState.expectation.transpose() << std::endl;
        // ss << std::endl << "measurement.expectation = " << std::endl << measurement.expectation.transpose() << std::endl;
        // //ss << std::endl << "mVector = " << std::endl << mVector.transpose() << std::endl;
        ss << std::endl << "measurementJacobian.transpose() = " << std::endl << measurementJacobian.transpose() << std::endl;
		ss << std::endl << " (pstate - measurement.expectation) = " << std::endl << (pstate - measurement.expectation).transpose() << std::endl;
		ss << std::endl << " update = " << std::endl << (-alpha * measurementJacobian.transpose() * (pstate - measurement.expectation)).transpose() << std::endl;
        // //ss << std::endl << "measurementUpdate = " << std::endl << measurementUpdate.transpose() << std::endl;
        // //ss << std::endl << "priorUpdate = " << std::endl << priorUpdate.transpose() << std::endl;
        // //ss << std::endl << "constraintUpdate = " << std::endl << constraintUpdate.transpose() << std::endl;
        ss << std::endl << "new state = " << newChainState.expectation.transpose() << std::endl;
        // ss << std::endl << "new cov diag = " << std::endl << newChainState.variance.diagonal().transpose() << std::endl;
        SPOOKY_LOG(ss.str());
        setChainState(fusion_chain, newChainState);
        //TODO: do this per node!
        local_state.last_update_time = m->getTimestamp();
    }

    void Node::fuseRotationMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

        //Calculate error
        Eigen::VectorXf wpstate;
        int fusion_chain = 1;

        if (m->globalSpace) {
            wpstate = utility::toAxisAnglePos(getGlobalPose()).head(3);
            //Fuse by modifying some parents if necessary
            fusion_chain = getRequiredChainLength(m);
        }
        else {
            wpstate = utility::toAxisAnglePos(getLocalPose()).head(3);
        }

        //CURRENT STATE
        State::Parameters chainState = getChainState(fusion_chain);
        //Process noise: max of ten seconds variance added
        Eigen::MatrixXf process_noise = getChainProcessNoise(fusion_chain).variance * std::fmin(10,m->getTimestamp() - local_state.last_update_time);
        chainState.variance += process_noise;
                    
        //JOINT CONSTRAINTS
        //Read quadratic constraints from joints in chain
        State::Parameters constraints = getChainConstraints(fusion_chain);

        //Compute relation between measurement quaternion and twist representation w
        Eigen::Matrix<float, 3, 4> quatToAxisJacobian = utility::getQuatToAxisJacobian(m->getRotation());

        //THIS MEASUREMENT
        //WARNING: Quat to be consistent: x,y,z,w is how eigen stores it internally, but its consrtuctor uses Quat(w,x,y,z)
        //Information matrices (inverse Covariance)
        Eigen::MatrixXf sigmaW = quatToAxisJacobian * m->getRotationVar() * quatToAxisJacobian.transpose();
        //Measurement information matrix
        Eigen::Matrix<float, 6, 1> wpm = utility::toAxisAnglePos(m->getTransform());
        State::Parameters measurement(3);
        measurement.expectation = utility::twistClosestRepresentation(wpm.head(3),wpstate.head(3));
        measurement.variance = sigmaW;
        
        //JACOBIAN:state -> measurement
        //Get Jacobian for the chain, mapping state to (w,v) global pose
        Eigen::Matrix<float, 9, Eigen::Dynamic> poseJac = getPoseChainJacobian(fusion_chain,m->globalSpace);
        Eigen::Matrix<float, 3, Eigen::Dynamic> measurementJacobian = poseJac.block(0,0,3, poseJac.cols());

        //TODO optimise ekf by using information matrices and inverting covariance per node
        // State::Parameters newChainState = customEKFMeasurementUpdate(chainState, constraints, measurement, measurementJacobian, wpstate);
        State::Parameters newChainState = EKFMeasurementUpdate(chainState, measurement, measurementJacobian, wpstate);

        setChainState(fusion_chain, newChainState);
        //TODO: do this per node!
        local_state.last_update_time = m->getTimestamp();
    }

    void Node::fuseRigidMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){
		//Calculate error
        Eigen::VectorXf wpstate;
		int fusion_chain = 1;

		if (m->globalSpace) {
            wpstate = utility::toAxisAnglePos(getGlobalPose());
			//Fuse by modifying some parents if necessary
			fusion_chain = getRequiredChainLength(m);
        }
        else {
            wpstate = utility::toAxisAnglePos(getLocalPose());
        }

        //CURRENT STATE
		State::Parameters chainState = getChainState(fusion_chain);
        //Process noise: max of ten seconds variance added
		Eigen::MatrixXf process_noise = getChainProcessNoise(fusion_chain).variance * std::fmin(10,m->getTimestamp() - local_state.last_update_time);
		chainState.variance += process_noise;
					
        //JOINT CONSTRAINTS
		//Read quadratic constraints from joints in chain
		State::Parameters constraints = getChainConstraints(fusion_chain);

        //Compute relation between measurement quaternion and twist representation w
		Eigen::Matrix<float, 3, 4> quatToAxisJacobian = utility::getQuatToAxisJacobian(m->getRotation());

        //THIS MEASUREMENT
		//WARNING: Quat to be consistent: x,y,z,w is how eigen stores it internally, but its consrtuctor uses Quat(w,x,y,z)
        //Information matrices (inverse Covariance)
		Eigen::MatrixXf sigmaW = quatToAxisJacobian * m->getRotationVar() * quatToAxisJacobian.transpose();
        //Measurement information matrix
        State::Parameters measurement(6);
		Eigen::Matrix<float, 6, 1> wpm = utility::toAxisAnglePos(m->getTransform());
        measurement.expectation.head(3) = utility::twistClosestRepresentation(wpm.head(3),wpstate.head(3));
		measurement.expectation.tail(3) = wpm.tail(3);
		measurement.variance.topLeftCorner(3, 3) = sigmaW;
		measurement.variance.bottomRightCorner(3, 3) = m->getPositionVar();
        
        //JACOBIAN:state -> measurement
        //Get Jacobian for the chain, mapping state to (w,v) global pose
        Eigen::Matrix<float, 9, Eigen::Dynamic> poseJac = getPoseChainJacobian(fusion_chain,m->globalSpace);
        Eigen::Matrix<float, 6, Eigen::Dynamic> measurementJacobian = poseJac.block(0,0,6, poseJac.cols());

        //TODO optimise ekf by using information matrices and inverting covariance per node
        // State::Parameters newChainState = customEKFMeasurementUpdate(chainState, constraints, measurement, measurementJacobian, wpstate);
        State::Parameters newChainState = EKFMeasurementUpdate(chainState, measurement, measurementJacobian, wpstate);

  //      std::stringstream ss;
  //      ss << std::endl << "process_noise = " << std::endl << process_noise << std::endl;
  //      //ss << std::endl << "sigmaW_info = " << std::endl << sigmaW_info << std::endl;
  //      //ss << std::endl << "sigmaM_info = " << std::endl << sigmaM_info << std::endl;
  //      //ss << std::endl << "sigmaP_info = " << std::endl << sigmaP_info << std::endl;
  //      //ss << std::endl << "sigmaC_info * joint_stiffness = " << std::endl << sigmaC_info * joint_stiffness << std::endl;
  //      ss << std::endl << "wpstate = " << std::endl << wpstate.transpose() << std::endl;
  //      ss << std::endl << "wpm = " << std::endl << wpm.transpose() << std::endl;
  //      //ss << std::endl << "mVector = " << std::endl << mVector.transpose() << std::endl;
  //      ss << std::endl << "measurementJacobian = " << std::endl << measurementJacobian << std::endl;
  //      //ss << std::endl << "measurementJacobian.transpose() * sigmaM_info * measurementJacobian = " << std::endl << measurementJacobian.transpose() * sigmaM_info * measurementJacobian << std::endl;
  //      //ss << std::endl << "measurementUpdate = " << std::endl << measurementUpdate.transpose() << std::endl;
  //      //ss << std::endl << "priorUpdate = " << std::endl << priorUpdate.transpose() << std::endl;
  //      //ss << std::endl << "constraintUpdate = " << std::endl << constraintUpdate.transpose() << std::endl;
		//ss << std::endl << "new state = " << newChainState.expectation.transpose() << std::endl;
  //      //ss << std::endl << "new cov diag = " << std::endl << newChainState.variance.diagonal().transpose() << std::endl;
  //      SPOOKY_LOG(ss.str());
		setChainState(fusion_chain, newChainState);
        //TODO: do this per node!
		local_state.last_update_time = m->getTimestamp();

    }

    void Node::fuseScaleMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

        Eigen::VectorXf pstate;
        int fusion_chain = 1;

        if (m->globalSpace) {
			//TODO: change to return class so I can use .scale(), .pos(), etc?
            pstate = utility::toAxisAnglePosScale(getGlobalPose()).tail(3);
            //Fuse by modifying some parents if necessary
            fusion_chain = getRequiredChainLength(m);
        }
        else {
            pstate = utility::toAxisAnglePosScale(getLocalPose()).tail(3);
        }

        //CURRENT STATE
        State::Parameters chainState = getChainState(fusion_chain);
        //Process noise: max of ten seconds variance added
        Eigen::MatrixXf process_noise = getChainProcessNoise(fusion_chain).variance * std::fmin(10,m->getTimestamp() - local_state.last_update_time);
        chainState.variance += process_noise;
                    
        //JOINT CONSTRAINTS
        //Read quadratic constraints from joints in chain
        State::Parameters constraints = getChainConstraints(fusion_chain);

        //THIS MEASUREMENT
        //Measurement information matrix
        State::Parameters measurement(3);
        measurement.expectation = m->getScale();
        measurement.variance = m->getScaleVar();
        
        //JACOBIAN:state -> measurement
        //Get Jacobian for the chain, mapping state to (w,v) global pose
        //TODO:
        // Eigen::Matrix<float, 3, Eigen::Dynamic> measurementJacobian = getPositionChainJacobian(fusion_chain,m->globalSpace);
        Eigen::Matrix<float, 9, Eigen::Dynamic> poseJac = getPoseChainJacobian(fusion_chain,m->globalSpace);
        Eigen::Matrix<float, 3, Eigen::Dynamic> measurementJacobian = poseJac.block(6,0,3,poseJac.cols());

        //TODO optimise ekf by using information matrices and inverting covariance per node
        // State::Parameters newChainState = customEKFMeasurementUpdate(chainState, constraints, measurement, measurementJacobian, pstate);
        State::Parameters newChainState = EKFMeasurementUpdate(chainState, measurement, measurementJacobian, pstate);

        setChainState(fusion_chain, newChainState);
        //TODO: do this per node!
        local_state.last_update_time = m->getTimestamp();
    }

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
        State::Parameters posterior(prior.size());

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
        //DEBUG
            // std::stringstream ss;
            // Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
            // ss << std::endl << "mVector = " << std::endl << mVector.transpose().format(fmt) << std::endl;
            // ss << std::endl << "measurementUpdate = " << std::endl << measurementUpdate.transpose().format(fmt) << std::endl;
            // ss << std::endl << "sigmaM_info = " << std::endl << sigmaM_info.format(fmt) << std::endl;
            // ss << std::endl << "measurement.expecation. = " << std::endl << measurement.expectation.transpose().format(fmt) << std::endl;
            // ss << std::endl << "measurementJacobian = " << std::endl << measurementJacobian.format(fmt) << std::endl;
            // ss << std::endl << "new state = " << posterior.expectation.transpose().format(fmt) << std::endl;
            // SPOOKY_LOG(ss.str());
        //DEBUG END
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
		const int n = prior.size();
		State::Parameters posterior(n);

		Eigen::MatrixXf K = SigmaBar * H.transpose() * (H * SigmaBar * H.transpose() + Q).inverse();
		//New state
		posterior.variance = (Eigen::MatrixXf::Identity(n,n) - K * H) * SigmaBar;
        Eigen::VectorXf error = (measurement.expectation - state_measurement);
        Eigen::VectorXf delta = K * error;
		posterior.expectation = prior.expectation + K * (measurement.expectation - state_measurement);

		//DEBUG
		// std::stringstream ss;
		// Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
  //       ss << std::endl << "K = " << std::endl << K.format(fmt) << std::endl;
  //       ss << std::endl << "H = " << std::endl << H.format(fmt) << std::endl;
  //       Eigen::Vector3f x = H.col(3);
  //       Eigen::Vector3f y = H.col(4);
  //       Eigen::Vector3f z = H.col(5);
  //       Eigen::MatrixXf xyzcross(3,3);
		// xyzcross.col(0) = x.cross(y);
		// xyzcross.col(1) = y.cross(z);
		// xyzcross.col(2) = z.cross(x);
  //       Eigen::Vector3f norms(x.norm(),y.norm(),z.norm());
  //       ss << std::endl << "xyzcross = " << std::endl << xyzcross.format(fmt) << std::endl;
  //       ss << std::endl << "norms.transpose() = " << std::endl << norms.transpose().format(fmt) << std::endl;
		// ss << std::endl << "error = " << std::endl << error.transpose().format(fmt)  << std::endl;
  //       ss << std::endl << "delta = " << std::endl << delta.transpose().format(fmt)<< std::endl;
  //       ss << std::endl << "new state = " << posterior.expectation.transpose().format(fmt) << std::endl;
		// SPOOKY_LOG(ss.str());
		//DEBUG END
		return posterior;
	};



	Eigen::Matrix<float, 9, Eigen::Dynamic> Node::getPoseChainJacobian(const int& chain_length, const bool& globalSpace) {
		//Precompute Jacobian size
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
		float h = 0.0001;
		Transform3D childPoses = Transform3D::Identity();
		Transform3D parentPoses = (node->parent != NULL && globalSpace) ? parent->getGlobalPose() : Transform3D::Identity();
		//3D for each rot, pos, scale
        Eigen::Matrix<float, 9, Eigen::Dynamic > J(9, inputDimension);
		
		//Lambda to be differentiated
		auto mapToGlobalPose = [&childPoses, &parentPoses, &node](const Eigen::VectorXf& theta) {
			return utility::toAxisAnglePosScale(parentPoses * node->getLocalPoseAt(theta) * childPoses);
		};

		int block = 0;
		for (int i = 0; i < chain_length; i++) {
			//Loop through all dof of this node and get the jacobian (w,p) entries for each dof
			int dof = node->getDimension();
			//TODO: fix lambda function differentiation
			//Watch out for block assignments - they cause horrible hard to trace memory errors if not sized properly
			//J.block(0, block, 9, dof) = utility::numericalVectorDerivative<float>(mapToGlobalPose, node->getState().expectation, h);

			Eigen::VectorXf f_x = utility::toAxisAnglePosScale(parentPoses * node->getLocalPose() * childPoses);
			//Why is x infinite size?
			Eigen::VectorXf x = node->getState().expectation;

			Eigen::MatrixXf df_dx = Eigen::MatrixXf::Zero(x.size(), f_x.size());

			for (int j = 0; j < dof; j++) {
				Eigen::VectorXf xplush = x;
				xplush(j) += h;
				Eigen::VectorXf f2 = utility::toAxisAnglePosScale(parentPoses * node->getLocalPoseAt(xplush) * childPoses);
				Eigen::VectorXf delf_delx = (f2 - f_x) / h;
				df_dx.col(j) = delf_delx;
			}

			J.block(0, block, 9, dof) = df_dx;

			block += dof;

			//Move to next parent
			if (node->parent == NULL) break;
			childPoses = node->getLocalPose() * childPoses;
			node = node->parent.get();
			parentPoses = globalSpace ? parent->getGlobalPose() : Transform3D::Identity();
		}
		return J;
	}

}