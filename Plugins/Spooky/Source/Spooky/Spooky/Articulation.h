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
#pragma once
#include "FusionTypes.h"
#include "Eigen/Core"

namespace spooky{

	class Articulation{
	public:
		//Type of articulation
		enum Type {
			FIXED = 0,
			AXIAL = 1,
			TWIST = 2,
			BONE = 3,
			POSE = 4,
			SCALE = 5
		};
	private:

		Type type;

		//Axis or axis + angle
		Eigen::Vector3f w;
		//Vector: twist offest or displacement 
		Eigen::Vector3f v;
		//The constant matrix for fixed articulation
		Transform3D fixedMatrix;

	public:
		//Default constructor
		Articulation();

		//Get the transform associated with this articulation
		template <typename Scalar>
		Eigen::Transform<Scalar, 3, Eigen::Affine> Articulation::getTransform(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& theta) const{
			//Theta can include velocity in the second half, so get transform references only from head
			//TODO: make these cases into methods
			Eigen::Transform<Scalar, 3, Eigen::Affine> T = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();
			Eigen::Matrix<Scalar, 3, 3> R = Eigen::Matrix<Scalar, 3, 3>::Identity();
			Sophus::Matrix<Scalar, 6, 1> vec;
			Eigen::Matrix<Scalar, 3, 3> W = Eigen::Matrix<Scalar, 3, 3>::Identity();

			switch (type) {
			case(FIXED):
			{
				T = fixedMatrix.cast<Scalar>();
				break;
			}			
			case(AXIAL):
			{
				R = Sophus::SO3<Scalar>::exp(theta(0) * w.cast<Scalar>()).matrix(); // = e^(theta * ^w)
				T.translate(v.cast<Scalar>());
				T.rotate(R);
				break;
			}
			case(TWIST):
			{
				vec.block<3, 1>(0, 0) = v.cast<Scalar>();
				vec.block<3, 1>(3, 0) = w.cast<Scalar>();
				T.matrix() = Sophus::SE3<Scalar>::exp(theta(0) * vec).matrix();
				break;
			}
			case(BONE):
			{
				//Theta is an axis-angle
				Eigen::Matrix<Scalar, 3, 1> rw = theta.head(3);
				T.translate(v.cast<Scalar>());
				T.matrix().topLeftCorner(3, 3) = utility::rodriguezFormula(rw);
				break;
			}
			case(POSE):
			{
				//Theta is an axis-angle
				Eigen::Matrix<Scalar, 3, 1> rw = theta.head(3);
				Eigen::Matrix<Scalar, 3, 1> pos = theta.segment<3>(3);
				T.translate(pos);
				T.rotate(Sophus::SO3<Scalar>::exp(rw).matrix());
				break;
			}
			case(SCALE):
			{
				//Theta is a scale vector in x,y and z
				T.scale(Eigen::Matrix<Scalar, 3, 1>(theta.head(3)));
				break;
			}
			}
			return T;
		}

		//Constructor functions:
		static Articulation createFromTransform(const Transform3D& T, const Type& type);
		static Articulation createFixed(const Transform3D & vec);
		static Articulation createBone(const Eigen::Vector3f & vec);
		static Articulation createTwist(const Eigen::Vector3f & axis, const Eigen::Vector3f & position);
		static Articulation createCartesian(const Eigen::Vector3f & axis, const Eigen::Vector3f & position);
		static Articulation createPose();
		static Articulation createScale();

		//Get rotational and translational degrees of freedom
		int getPDoF(bool hasLeverChild) const;
		int getRDoF() const;
		int getSDoF() const;

		//Get pose variance
		//Eigen::Matrix<float, 6, 6> getPoseVariance(const Eigen::VectorXf& expectation, const Eigen::MatrixXf& variance);

		//TODO: Get the pose jacobian
		//Eigen::Matrix<float, 6, Eigen::Dynamic> getPoseJacobian(const Eigen::VectorXf& expectation, const Eigen::MatrixXf& variance);

		//Returns the initial state vector to operate this articulation
		static Eigen::VectorXf getInitialState(const Articulation::Type & type);

		//Get predicted state
		Eigen::VectorXf getPredictedExpectation(const Eigen::VectorXf& state, const float& t) const;

		//Accessors
		const Type& getType() const{
			return type;
		}
	
	};

}