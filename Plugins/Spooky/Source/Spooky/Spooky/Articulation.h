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
		Transform3D getTransform(Eigen::VectorXf theta);

		//Constructor functions:
		static Articulation createFromTransform(const Transform3D& T, const Type& type);
		static Articulation createBone(const Eigen::Vector3f & vec);
		static Articulation createTwist(const Eigen::Vector3f & axis, const Eigen::Vector3f & position);
		static Articulation createCartesian(const Eigen::Vector3f & axis, const Eigen::Vector3f & position);
		static Articulation createPose();
		static Articulation createScale();

		//Returns the initial state vector to operate this articulation
		static Eigen::VectorXf getInitialState(const Articulation::Type & type);

		//Accessors
		const Type& getType() const{
			return type;
		}
	
	};

}