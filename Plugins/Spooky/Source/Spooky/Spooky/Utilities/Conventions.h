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