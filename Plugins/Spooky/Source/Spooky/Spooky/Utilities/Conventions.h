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
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace spooky {
	namespace utility {
		//Defines transforms between data of different types
		namespace convention{
			//TODO: move to measurements?
			static Eigen::Matrix4f unserialiseTo4x4f(Eigen::VectorXf data) {
				Eigen::Translation3f v(data.block<3, 1>(0, 0));
				Eigen::Quaternionf q(Eigen::Vector4f(data.bottomRows(4)));
				//assert(q.coeffs.isApprox(data.bottomRows()));
				Eigen::Transform<float, 3, Eigen::Affine> T(v);
				T.rotate(q);
				return T.matrix();
			}

			static Eigen::Vector4f quatToVec(Eigen::Quaternionf q) {
				return Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());
			}

		}
	}
}