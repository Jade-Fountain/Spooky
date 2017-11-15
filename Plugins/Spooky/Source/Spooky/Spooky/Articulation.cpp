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
    			w.normalize();
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
                //Theta is a quaternion
    			Eigen::Quaternionf q = Eigen::Quaternionf(Eigen::Vector4f(theta));
    			T.translate(v);
    			T.rotate(q);
    			break;
            }
			case(POSE):
			{
				//Theta is a quaternion
				Eigen::Quaternionf q = Eigen::Quaternionf(Eigen::Vector4f(theta.tail(4)));
				Eigen::Vector3f pos = theta.head(3);
				T.translate(pos);
				T.rotate(q);
				break;
			}
			case(SCALE):
			{
				//Theta is a scale vector
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
				result.w.normalized();
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
				//Scales have no internal structure
				result.v = Eigen::Vector3f::Zero();
				result.w = Eigen::Vector3f::Zero();
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
		result.w = Eigen::Vector3f::Identity();
		result.v = Eigen::Vector3f::Identity();
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
                return Eigen::Vector4f(0,0,0,1);
                break;
            }
			case(POSE):
			{
				//pos_quat representation
				Eigen::VectorXf vec = Eigen::Matrix<float,7,1>::Zero();
				vec << 0, 0, 0, 0, 0, 0, 1;
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
