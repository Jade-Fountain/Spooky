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
			CARTESIAN = 0,
			TWIST = 1,
			BONE = 2,
			POSE = 3,
			SCALE = 4
		};
	private:

		Type type;

		//Axis or axis + angle
		Eigen::Vector3f w;
		//Vector: twist offest or displacement 
		Eigen::Vector3f v;


	public:
		//Default constructor
		Articulation();

		//Get the transform associated with this articulation
		Transform3D getTransform(const Eigen::VectorXf& theta);

		//Constructor functions:
		static Articulation createFromTransform(const Transform3D& T, const Type& type);
		static Articulation createBone(const Eigen::Vector3f & vec);
		static Articulation createTwist(const Eigen::Vector3f & axis, const Eigen::Vector3f & position);
		static Articulation createCartesian(const Eigen::Vector3f & axis, const Eigen::Vector3f & position);
		static Articulation createPose();
		static Articulation createScale();

		//Get rotational and translational degrees of freedom
		int getPDoF();
		int getRDoF();

		//Returns the initial state vector to operate this articulation
		static Eigen::VectorXf getInitialState(const Articulation::Type & type);

		//Accessors
		const Type& getType() const{
			return type;
		}
	
	};

}