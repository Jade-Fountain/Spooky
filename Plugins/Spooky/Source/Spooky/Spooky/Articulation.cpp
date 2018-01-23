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
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

namespace spooky{
    
    Articulation::Articulation(){

    }


    Transform3D Articulation::getTransform(Eigen::VectorXf theta){

		//TODO: make these cases into methods
		Transform3D T = Transform3D::Identity();
		Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
		Sophus::Vector6f vec;
		Eigen::Matrix3f W = Eigen::Matrix3f::Identity();

		switch(type){
		case(CARTESIAN):
            {
    			R = Sophus::SO3f::exp(theta(0) * w).matrix(); // = e^(theta * ^w)
    			T.translate(v);
    			T.rotate(R);
    			break;
            }
    		case(TWIST):
            {
    			vec.block<3, 1>(0, 0) = v;
    			vec.block<3, 1>(3, 0) = w;
    			T.matrix() = Sophus::SE3f::exp(theta(0) * vec).matrix();
    			break;
            }
    		case(BONE):
            {
                //Theta is an axis-angle
				Eigen::Vector3f rw = theta;
    			T.translate(v);
    			T.rotate(Sophus::SO3f::exp(rw).matrix());
    			break;
            }
			case(POSE):
			{
				//Theta is an axis-angle
				Eigen::Vector3f rw = theta.tail(3);
				Eigen::Vector3f pos = theta.head(3);
				T.translate(pos);
				T.rotate(Sophus::SO3f::exp(rw).matrix());
				break;
			}
			case(SCALE):
			{
				//Theta is a scale vector in x,y and z
				T.scale(Eigen::Vector3f(theta.head(3)));
				break;
			}
        }
		return T;
    }
    
    Articulation Articulation::createFromTransform(const Transform3D& T, const Type& type){
        Articulation result;
		result.type = type;

		switch (type) {
			case(CARTESIAN):
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
		result.type = CARTESIAN;
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

    Eigen::VectorXf Articulation::getInitialState(const Articulation::Type& type){
        switch (type) {
            case(CARTESIAN):
            {
				//Single angle per articulation
				return Eigen::VectorXf::Zero(1);
                break;
            }
            case(TWIST):
            {
				//Single angle per articulation
                return Eigen::VectorXf::Zero(1);
                break;
            }
            case(BONE):
            {
                //quaternion representation
                return Eigen::Vector3f(0,0,0);
                break;
            }
			case(POSE):
			{
				//pos_quat representation
				Eigen::VectorXf vec = Eigen::Matrix<float,6,1>::Zero();
				vec << 0, 0, 0, 0, 0, 0;
				return vec;
				break;
			}
			case(SCALE):
			{
				return Eigen::Vector3f(1,1,1);
				break;
			}
        }
		return Eigen::VectorXf::Zero(1);
    }

}
