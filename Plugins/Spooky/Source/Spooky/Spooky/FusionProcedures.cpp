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
        //Required dof:
        int p_dof_req = m->getRequiredPDoF();
        int r_dof_req = m->getRequiredRDoF();

        const float flex_threshold = 0.9;
        bool hasLeverChild = false;
		//TODO: do a better way?
		Node* node = this;
        while(true){
            //Assume decoupling between nodes
            p_dof += node->getPDoF(hasLeverChild);
            r_dof += node->getRDoF();

            if(!hasLeverChild){
                //If there is much of a translation then it creates a lever which can cause position change
                hasLeverChild = node->getLocalPose().translation().norm() > 0.01;
            }
            
            if(p_dof>=p_dof_req && r_dof>=r_dof_req || node->parent == NULL){
                //We dont need anymore nodes to fuse data, or we are out of nodes
                break;
            } else {
                //We need more nodes
				node = node->parent.get();
				nodecount++;
            }
        }
        return nodecount;
    }

    void Node::fusePositionMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

    }

    void Node::fuseRotationMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

    }

    void Node::fuseRigidMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){
		//Calculate error
		if (m->globalSpace) {
            Eigen::VectorXf wpstate = utility::toAxisAnglePos(getGlobalPose());
			//Fuse by modifying some parents if necessary
			int fusion_chain = 4;// getRequiredChainLength(m);

			State::Parameters chainState = getChainState(fusion_chain);
            //Process noise: max of ten seconds variance added
			Eigen::MatrixXf process_noise = getChainProcessNoise(fusion_chain).variance * std::fmin(10,m->getTimestamp() - local_state.last_update_time);
			chainState.variance += process_noise;
						
			//Read quadratic constraints from joints in chain
			State::Parameters constraints = getChainConstraints(fusion_chain);

            //Compute relation between measurement quaternion and twist representation w
			Eigen::Matrix<float, 3, 4> quatToAxisJacobian = utility::getQuatToAxisJacobian(m->getRotation());

			//WARNING: Fix quat to be consistent: x,y,z,w is how eigen stores it internally, but its consrtuctor uses Quat(w,x,y,z)
            //Information matrices (inverse Covariance)
			Eigen::MatrixXf sigmaW = quatToAxisJacobian * m->getRotationVar() * quatToAxisJacobian.transpose();
			
            //DEPENDS ON MEASUREMENT TYPE:
            //--------------------------------
            //Measurement information matrix
            State::Parameters measurement(6);
			//Block diagonal inverse is inverse of blocks
            measurement.expectation = utility::toAxisAnglePos(m->getTransform());
			measurement.variance.topLeftCorner(3, 3) = sigmaW;
			measurement.variance.bottomRightCorner(3, 3) = m->getPositionVar();
            //--------------------------------
            //Get Jacobian for the chain, mapping state to (w,v) global pose
            Eigen::Matrix<float, 6, Eigen::Dynamic> measurementJacobian = getPoseChainJacobian(fusion_chain);

            auto EKFMeasurementUpdate = [this]( const State::Parameters& prior, const State::Parameters& constraints, const State::Parameters& measurement,
                                                const Eigen::MatrixXf& measurementJacobian, const Eigen::VectorXf& state_measurement)
            {
                assert(prior.size() == measurementJacobian.cols() && sigmaM_info.rows() == sigmaM_info.cols() == measurementJacobian.rows() == state_measurement.size() == measurement.size());
                //Prior information matrix
                Eigen::MatrixXf sigmaP_info = prior.variance.inverse();
                //Constraint information matrix
                Eigen::MatrixXf sigmaC_info = constraints.variance.inverse();

                //New state initialisation:
                State::Parameters posterior(prior.size());

                //New variance (extended kalman filter measurement update)
                Eigen::MatrixXf sigmaM_info = measurement.variance.inverse();
                posterior.variance = (measurementJacobian.transpose() * sigmaM_info * measurementJacobian + sigmaP_info + joint_stiffness * sigmaC_info).inverse();
                
                //
                Eigen::VectorXf mVector = measurement.expectation - state_measurement + measurementJacobian * prior.expectation;

                Eigen::VectorXf measurementUpdate = measurementJacobian.transpose() * sigmaM_info * mVector;
                Eigen::VectorXf priorUpdate = sigmaP_info * prior.expectation;
                Eigen::VectorXf constraintUpdate = sigmaC_info * constraints.expectation;

                //New state
                posterior.expectation = posterior.variance * (priorUpdate + joint_stiffness * constraintUpdate + measurementUpdate);
                return posterior;
            };

            State::Parameters newChainState = EKFMeasurementUpdate(chainState, constraints, measurement, measurementJacobian, wpstate);

            std::stringstream ss;
            ss << std::endl << "process_noise = " << std::endl << process_noise << std::endl;
            //ss << std::endl << "sigmaW_info = " << std::endl << sigmaW_info << std::endl;
            //ss << std::endl << "sigmaM_info = " << std::endl << sigmaM_info << std::endl;
            //ss << std::endl << "sigmaP_info = " << std::endl << sigmaP_info << std::endl;
            //ss << std::endl << "sigmaC_info * joint_stiffness = " << std::endl << sigmaC_info * joint_stiffness << std::endl;
            //ss << std::endl << "wpstate = " << std::endl << wpstate.transpose() << std::endl;
            //ss << std::endl << "wpm = " << std::endl << wpm.transpose() << std::endl;
            //ss << std::endl << "mVector = " << std::endl << mVector.transpose() << std::endl;
            //ss << std::endl << "measurementJacobian = " << std::endl << measurementJacobian << std::endl;
            //ss << std::endl << "measurementJacobian.transpose() * sigmaM_info * measurementJacobian = " << std::endl << measurementJacobian.transpose() * sigmaM_info * measurementJacobian << std::endl;
            //ss << std::endl << "measurementUpdate = " << std::endl << measurementUpdate.transpose() << std::endl;
            //ss << std::endl << "priorUpdate = " << std::endl << priorUpdate.transpose() << std::endl;
            //ss << std::endl << "constraintUpdate = " << std::endl << constraintUpdate.transpose() << std::endl;
			ss << std::endl << "new state = "  << std::endl << newChainState.expectation.transpose() << std::endl;
            ss << std::endl << "new cov diag = " << std::endl << newChainState.variance.diagonal().transpose() << std::endl;
            SPOOKY_LOG(ss.str());
			setChainState(fusion_chain, newChainState);
			local_state.last_update_time = m->getTimestamp();
		}
		else {
			//TODO: Fuse locally with reg kalman filter
			//insertMeasurement(m);
		}
		//TODO: fuse chain
    }

    void Node::fuseScaleMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

    }


	Eigen::Matrix<float, 6, Eigen::Dynamic> Node::getPoseChainJacobian(const int& chain_length) {
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
		float h = 1e-20;
		Transform3D childPoses = Transform3D::Identity();
		Transform3D parentPoses = (node->parent != NULL) ? parent->getGlobalPose() : Transform3D::Identity();
		Eigen::Matrix<float, 6, Eigen::Dynamic > J(6, inputDimension);
		
		//Lambda to be differentiated
		auto mapToGlobalPose = [&childPoses, &parentPoses, &node](const Eigen::VectorXf& theta) {
			return utility::toAxisAnglePos(parentPoses * node->getLocalPoseAt(theta) * childPoses);
		};

		int block = 0;
		for (int i = 0; i < chain_length; i++) {
			//Loop through all dof of this node and get the jacobian (w,p) entries for each dof
			int dof = node->getDimension();

			//Watch out for block assignments - they cause horrible hard to trace memory errors if not sized properly
			J.block(0,block, 6, dof) = utility::numericalVectorDerivative<float>(mapToGlobalPose, node->getState().expectation, 0.01);
			block += dof;

			//Move to next parent
			if (node->parent == NULL) break;
			childPoses = node->getLocalPose() * childPoses;
			node = node->parent.get();
			parentPoses = parent->getGlobalPose();
		}
		return J;
	}

}