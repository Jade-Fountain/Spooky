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
