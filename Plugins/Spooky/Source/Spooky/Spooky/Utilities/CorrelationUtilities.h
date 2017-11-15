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
#include<iostream>
#include<string>
#include<Eigen/Core>
#include<Eigen/SVD>
#include<Eigen/Geometry>
#include "CommonMath.h"
#include "Logging.h"

#pragma once
namespace spooky{
	namespace utility{
		namespace correlation {
			float error_scale = 0.1;

			//For correlating data with position only
			namespace Position {

				static inline float correlationQuality(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB
				) {
					//Compute velocity diffs
					float total_error = 0;
					for(int i = 1; i < samplesA.size() - 1; i++){
						float velA = (samplesA[i]-samplesA[i-1]).norm();
						float velB = (samplesB[i]-samplesB[i-1]).norm();
						total_error += std::fabs(velA - velB);
					}
					return qualityFromError(total_error / samplesA.size(), error_scale);
				}
				
				static inline float correlationQualityWeighted(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const std::vector<Eigen::Matrix3f>& invVariances
				) {
					//Compute velocity diffs
					float total_error = 0;
					for(int i = 1; i < samplesA.size() - 1; i++){
						float velA = (invVariances[i] * (samplesA[i]-samplesA[i-1])).norm();
						float velB = (invVariances[i] * (samplesB[i]-samplesB[i-1])).norm();
						total_error += std::fabs(velA - velB);
					}
					return qualityFromError(total_error / samplesA.size(), error_scale);
				}
				
				
			}

			//For correlating rotation only
			namespace Rotation {

			}

			//For correlating position and rotation data simultaneously
			namespace Transform {

			}
			
		}
	}
}
