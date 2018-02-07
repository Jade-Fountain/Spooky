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
#include "Articulation.h"
#include "Utilities/CommonMath.h"

namespace spooky{
    
    Articulation::Articulation(){
    	fixedMatrix = Transform3D::Identity();
    	w = Eigen::Vector3f::Zero();
    	v = Eigen::Vector3f::Zero();
    }

    Articulation Articulation::createFromTransform(const Transform3D& T, const Type& type){
        Articulation result;
		result.type = type;

		switch (type) {
			case(FIXED):
			{
				result.fixedMatrix = T;
				break;
			}			
			case(AXIAL):
			{
				result.v = T.matrix().col(3).head(3);
				result.w = Sophus::SO3f::log(Sophus::SO3f(T.matrix().topLeftCorner(3, 3)));
				result.w.normalize();
				break;
			}
			case(TWIST):
			{
				Sophus::SE3f zeta(T.matrix());
				Sophus::Vector6f vec = zeta.log();
				result.v = vec.head(3);
				result.w = vec.tail(3);
				result.w.normalize();
				break;
			}
			case(BONE):
			{
				//Bones have a fixed displacement
				result.v = T.matrix().col(3).head(3);
				result.w = Eigen::Vector3f::Zero();
				break;
			}			
			case(POSE):
			{
				//Poses have no internal structure
				result.v = Eigen::Vector3f::Zero();
				result.w = Eigen::Vector3f::Zero();
				break;
			}
			case(SCALE):
			{
				//Scales have no internal structure, they scale in their parent reference frame
				result.v = Eigen::Vector3f::Zero();
				result.w = Eigen::Vector3f::Zero();
				break;
			}
		}
        return result;
    }

	Articulation Articulation::createFixed(const Transform3D& T){
		Articulation result;
		result.type = FIXED;
		result.fixedMatrix = T;
		return result;
	}

	Articulation Articulation::createBone(const Eigen::Vector3f& vec){
		Articulation result;
		result.type = BONE;
		result.v = vec;
		return result;
	}

	Articulation Articulation::createTwist(const Eigen::Vector3f& axis, const Eigen::Vector3f& position) {
		Articulation result;
		result.type = TWIST;
		result.w = axis;
		result.v = -axis.cross(position);
		return result;
	}

	Articulation Articulation::createCartesian(const Eigen::Vector3f& axis, const Eigen::Vector3f& position) {
		Articulation result;
		result.type = AXIAL;
		result.w = axis;
		result.v = position;
		return result;
	}	
	
	Articulation Articulation::createPose() {
		Articulation result;
		result.type = POSE;
		result.w = Eigen::Vector3f::Identity();
		result.v = Eigen::Vector3f::Identity();
		return result;
	}

	Articulation Articulation::createScale() {
		Articulation result;
		result.type = SCALE;
		//Scales have no internal structure, they scale in their parent reference frame
		result.v = Eigen::Vector3f::Zero();
		result.w = Eigen::Vector3f::Zero();
		return result;
	}

	int Articulation::getPDoF(bool hasLeverChild){
		  switch (type) {
            case(FIXED):
            	return 0;
            case(AXIAL):
            	return hasLeverChild ? 1 : 0;
            case(TWIST):
				//Twist rotates and translates simultaneously
            	return 1;
            case(BONE):
            //Roll doesnt help with position
            	return hasLeverChild ? 2 : 0;
			case(POSE):
			//Roll doesnt help with position
            	return hasLeverChild ? 5 : 2;
			case(SCALE):
				return hasLeverChild ? 3 : 0;
		}
		return 0;
	}
	int Articulation::getRDoF(){
		if(type == SCALE || type == FIXED){
			return 0;
		}
		else if (type == AXIAL || type == TWIST)
		{
			return 1;
		}
		else
		{
			return 3;
		}
	}
	int Articulation::getSDoF(){
		if(type == SCALE){
			return 3;
		} else {
			return 0;
		}
	}

	//Eigen::Matrix<float, 6, 6> Articulation::getPoseVariance(const Eigen::VectorXf& expectation, const Eigen::MatrixXf& variance) {
	//	Eigen::Matrix<float, 6, 6> result = Eigen::Matrix<float, 6, 6>::Identity();
	//	Eigen::Matrix<float, 6, Eigen::Dynamic> jacobian = getPoseJacobian(expectation);
	//	return jacobian * variance * jacobian.transpose();


	//}

	//Eigen::Matrix<float, 6, Eigen::Dynamic> Articulation::getPoseJacobian(const Eigen::VectorXf& expectation, const Eigen::MatrixXf& variance) {
	//	switch (type) {
	//		case(AXIAL):
	//		{
	//			//Single angle per articulation

	//			break;
	//		}
	//		case(TWIST):
	//		{
	//			//Single angle per articulation
	//			return Eigen::VectorXf::Zero(1);
	//			break;
	//		}
	//		case(BONE):
	//		{
	//			//quaternion representation
	//			return Eigen::Vector3f(0, 0, 0);
	//			break;
	//		}
	//		case(POSE):
	//		{
	//			//pos_quat representation
	//			Eigen::VectorXf vec = Eigen::Matrix<float, 6, 1>::Zero();
	//			vec << 0, 0, 0, 0, 0, 0;
	//			return vec;
	//			break;
	//		}
	//		case(SCALE):
	//		{
	//			return Eigen::Vector3f(1, 1, 1);
	//			break;
	//		}
	//		}
	//		return Eigen::VectorXf::Zero(1);
	//	}
	//}


    Eigen::VectorXf Articulation::getInitialState(const Articulation::Type& type){
        switch (type) {
        	case(FIXED):
            {
				//Zero articulations
				return Eigen::VectorXf(0);
            }
            case(AXIAL):
            {
				//Single angle per articulation
				return Eigen::VectorXf::Zero(1);
            }
            case(TWIST):
            {
				//Single angle per articulation
                return Eigen::VectorXf::Zero(1);
            }
            case(BONE):
            {
                //quaternion representation
                return Eigen::Vector3f(0,0,0);
            }
			case(POSE):
			{
				//pos_quat representation
				Eigen::VectorXf vec = Eigen::Matrix<float,6,1>::Zero();
				vec << 0, 0, 0, 0, 0, 0;
				return vec;
			}
			case(SCALE):
			{
				return Eigen::Vector3f(1,1,1);
			}
        }
		return Eigen::VectorXf::Zero(1);
    }
}
