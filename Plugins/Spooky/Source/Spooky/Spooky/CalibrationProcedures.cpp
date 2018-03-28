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
#include "Calibration.h"
#include "FusionTypes.h"
#include "Utilities/CalibrationUtilities.h"
#include "Logging.h"
#include "Utilities/Conventions.h"

namespace spooky {

	CalibrationResult Calibrator::updateCalibration(const CalibrationResult& newCalibration, const CalibrationResult& currentCalibration) const{

		//Compute transform, error, quality, relevance and weight
		CalibrationResult result = newCalibration;
		switch (currentCalibration.state) {
			case (CalibrationResult::State::UNCALIBRATED):
			{
				SPOOKY_LOG("CalibrationResult::State::UNCALIBRATED");
				//DEBUG:: Straight to calibrated
				if(result.quality > config.initial_quality_threshold){
					result.state = CalibrationResult::State::REFINING;
					SPOOKY_LOG("Calibration passed required threshold: " + std::to_string(config.initial_quality_threshold) + ", quality = " + std::to_string(result.quality) + " result.weight = " + std::to_string(result.weight));
				}
				else {
					result.reset();
					SPOOKY_LOG("Calibration DID NOT PASS required threshold: " + std::to_string(config.initial_quality_threshold) + ", quality = " + std::to_string(result.quality) + " result.weight = " + std::to_string(result.weight));
				}	
				break;
			}
			case (CalibrationResult::State::REFINING):
			{
				SPOOKY_LOG("CalibrationResult::State::REFINING");
				//Combine calibrations
				result.updateResult(currentCalibration);
				//refinement calibration
				float new_quality = result.quality;
				if (new_quality - currentCalibration.quality > config.quality_convergence_threshold) {
					//If large gains are being made keep going
					SPOOKY_LOG("REFINING: new_quality = " + std::to_string(new_quality) + ", quality = " + std::to_string(currentCalibration.quality) + " result.weight = " + std::to_string(result.weight));
					result.state = CalibrationResult::State::REFINING;
				}
				else if (new_quality > config.settle_threshold) {
					//If change is a small improvement, we are done
					SPOOKY_LOG("REFINEMENT FINISHED!!! new_quality = " + std::to_string(new_quality) + ", quality = " + std::to_string(currentCalibration.quality) + " result.weight = " + std::to_string(result.weight));
					result.state = CalibrationResult::State::CALIBRATED;
				}
				else {
					//If we cant improve, then we probably started in a bad state, so start again
					SPOOKY_LOG("Starting over: new_quality = " + std::to_string(new_quality) + ", quality = " + std::to_string(currentCalibration.quality) + " result.weight = " + std::to_string(result.weight));
					result.state = CalibrationResult::State::UNCALIBRATED;
				}
				break;
			}
			case (CalibrationResult::State::CALIBRATED):
			{
				SPOOKY_LOG("CalibrationResult::State::CALIBRATED");
				if (fault_detection_disabled) {
					SPOOKY_LOG("WARNING: FAULT DETECTION IS DISABLED");
					return currentCalibration;
				}
				//TODO: distinguish noise vs. actual movement
				//TODO: implement that fault decay detection thing
				//TODO: fix fault detection for new model
				//Track new transform and see how much it moves compared to accepted result
				Transform3D transform_error = result.transform.inverse() * currentCalibration.transform;
				Transform3D decayed_relevance = utility::slerpTransform3D(currentCalibration.relevance, Transform3D::Identity(), config.relevance_decay_rate);
				Transform3D filtered_relevance =  utility::slerpTransform3D(decayed_relevance, transform_error, config.fault_hysteresis_rate);
				auto relevance_norm = utility::transformNorm(filtered_relevance);

				//Debug
				std::stringstream relss;
				relss << filtered_relevance.matrix();
				SPOOKY_LOG("Filtered relevance = ");
				SPOOKY_LOG(relss.str());
				SPOOKY_LOG(" Already calibrated - watching for faults (" + currentCalibration.systems.first.name + ", " + currentCalibration.systems.second.name + ") - filtered relevance error = "
					+ std::to_string(relevance_norm.angle) + ", " + std::to_string(relevance_norm.distance) +
					" vs. threshold = " + std::to_string(config.fault_angle_threshold) + ", " + std::to_string(config.fault_distance_threshold) + 
					" result.weight = " + std::to_string(result.weight));

				//Regardless, dont change calibration
				result = currentCalibration;
				result.relevance = filtered_relevance;
				//Relevance represents the latest error from original transform, filtered with exponential filter
				//If our quality varies from the expected too much, we need to recalibrate
				if (relevance_norm.angle > config.fault_angle_threshold ||
					relevance_norm.distance > config.fault_distance_threshold) {
					//Always start again if faulted
					//if (result.quality < config.initial_quality_threshold) {
						//Start again
						SPOOKY_LOG("Starting over: result.quality = " + std::to_string(result.quality) + ", old quality = " + std::to_string(currentCalibration.quality) + " result.weight = " + std::to_string(result.weight));
						result.reset();
						result.state = CalibrationResult::State::UNCALIBRATED;
					//} else {
					//	//Try to improve the quality
					//	//TODO: fix refinement
					//	SPOOKY_LOG("Returning to refinement: result.quality = " + std::to_string(result.quality) + ", old quality = " + std::to_string(currentCalibration.quality) + " result.weight = " + std::to_string(result.weight));
					//	result.state = CalibrationResult::State::REFINING;
					//}
				}
				else {
					SPOOKY_LOG("No fault detected");
					result.state = CalibrationResult::State::CALIBRATED;
				}
				break;
			}
		}
		return result;
	}

	//TODO: refactor this mess
	CalibrationResult Calibrator::calPos(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult& currentCalibration)	const
	{
		float qualityScaleFactor = 0.05;


		//Data for each stream
		std::vector<Eigen::Vector3f> pos1(m1.size());
		std::vector<Eigen::Vector3f> pos2(m2.size());
		std::vector<Eigen::Matrix3f> inverse_variances(m1.size());
		std::stringstream ss;
		ss << "DATA[" << m1.front()->getSensor()->system.name << ", " << m2.front()->getSensor()->system.name << "]" << std::endl;
		SPOOKY_LOG(ss.str());
		for (int i = 0; i < m1.size(); i++) {
			
			pos1[i] = m1[i]->getPosition();
			pos2[i] = m2[i]->getPosition();
			ss << pos1[i].transpose() << " " << pos2[i].transpose() << std::endl;
			//TODO: Not strictly correct
			inverse_variances[i] = (m1[i]->getPositionVar() + m2[i]->getPositionVar()).inverse();
		}


		//Initialise current results
		CalibrationResult result;
		result.systems = SystemPair(m1.front()->getSystem(), m2.front()->getSystem());
		//result.latency = currentCalibration.latency;
		//TODO: get current time since start, perhaps with use of a Platform.h?
		result.timestamp = m1.back()->getTimestamp();
		
		//--------------------------------------
		//Compute current results
		//--------------------------------------
		//Clear previous
		result.transform.setIdentity();

		//Compute point cloud results
		//result.transform = utility::calibration::Position::calibrateWeightedIdenticalPair(pos1, pos2, inverse_variances, &result.error);
		result.transform = utility::calibration::Position::calibrateIdenticalPairTransform_Arun(pos1, pos2, &result.error);
		result.quality = utility::qualityFromError(result.error, qualityScaleFactor);
		result.relevance = Transform3D::Identity();
		result.weight = m1.size();

		//------------------------------------------------------------------
		//TODO:clean this up:
		//TODO: check if error is high enough and correct for rigid link
		//TODO: This doesnt even help Arun is too good already
		
		//Build chunked lists for later:
		//Groups the measurement data by node
		// std::vector<std::vector<Eigen::Vector3f>> chunked_pos1;
		// std::vector<std::vector<Eigen::Vector3f>> chunked_pos2;
		// Measurement::chunkMeasurements<Eigen::Vector3f, &Measurement::getPosition>(m1,m2,&chunked_pos1,&chunked_pos2);

		//Refine with rigid link model
		//for (int i = 0; i < 1; i++) {
		//	//TODO:clean up
		//	std::vector<Transform3D> transforms;
		//	std::vector<float> weights;

		//	for (int j = 0; j < chunked_pos1.size(); j++) {
		//		weights.push_back(100000);
		//		transforms.push_back(utility::calibration::Position::refineIdenticalPairPosition(chunked_pos1[j], chunked_pos2[j], result.transform, &weights.back()));
		//		weights.back() = utility::qualityFromError(weights.back(), qualityScaleFactor);
		//	}

		//	result.transform = utility::getMeanTransform(transforms, weights);


		//	transforms.clear();
		//	weights.clear();
		//	for (int j = 0; j < chunked_pos1.size(); j++) {
		//		weights.push_back(100000);
		//		transforms.push_back(utility::calibration::Position::refineIdenticalPairRotation(chunked_pos1[j], chunked_pos2[j], result.transform, &weights.back()));
		//		weights.back() = utility::qualityFromError(weights.back(), qualityScaleFactor);
		//	}
		//	result.transform = utility::getMeanTransform(transforms, weights);

		//------------------------------------------------------------------

		SPOOKY_LOG("Performed calibration on new data. error: " + std::to_string(result.error) + ", quality = " + std::to_string(result.quality) + " result.weight = " + std::to_string(result.weight));

		//--------------------------------------
		//Decide what to do with results
		//--------------------------------------

		ss << "Result: transform[" << result.systems.first.name << "->" << result.systems.second.name << "] = " << std::endl << result.transform.matrix() << std::endl;
		//SPOOKY_LOG(ss.str() + " result.weight = " + std::to_string(result.weight));
		SPOOKY_LOG("|||||");
		SPOOKY_LOG("|||||");
		return updateCalibration(result, currentCalibration);
	}

	CalibrationResult Calibrator::cal6DoF(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2, const CalibrationResult& currentCalibration, const bool& includePosition)const
	{
		float qualityScaleFactor = 1;

		//Debug
		std::stringstream ss;
		ss  << "cal6Dof" << (includePosition ? "(Pos+Rot)":"(Just Rotation)") << "[" << m1.front()->getSensor()->system.name << ", " << m2.front()->getSensor()->system.name 
			<< "], samples =[" << m1.size() << ", " << m2.size() << "]" << std::endl;

		//Chunk measurements based on node
		std::vector<std::vector<Eigen::Matrix4f>> pos1;
		std::vector<std::vector<Eigen::Matrix4f>> pos2;

		if (includePosition) {
			Measurement::chunkMeasurements<Eigen::Matrix4f, &Measurement::getTransformMatrix>(m1, m2, &pos1, &pos2);
		}
		else {
			Measurement::chunkMeasurements<Eigen::Matrix4f, &Measurement::getRotationTransformMatrix>(m1, m2, &pos1, &pos2);
		}

		//Initialise new result metadata
		CalibrationResult result;
		result.systems = SystemPair(m1.front()->getSystem(), m2.front()->getSystem());
		
		//Compute transforms and errors
		std::vector<float> weights;
		std::vector<Transform3D> transformsX;
		std::vector<Transform3D> transformsY;
		for (int i = 0; i < pos1.size(); i++) {
			float error = 100;
			// pos1[i][k] * X = Y * pos2[i][k] 
			//Y:System2->System1
			auto group_result = utility::calibration::Transform::twoSystems_Kronecker_Shah2013(pos1[i], pos2[i], &error, includePosition);
			transformsX.push_back(group_result.first);
			transformsY.push_back(group_result.second);
			weights.push_back(utility::qualityFromError(error, qualityScaleFactor));
		}
		//Compute mean transforms over each group
		Transform3D transformY = utility::getMeanTransform(transformsY, weights);
		result.transform = transformY.inverse(); //Y':System1->System2

		//Compute error
		result.error = 0;
		for (int i = 0; i < pos1.size(); i++) {
			result.error += utility::calibration::Transform::getTwoSystemsError(transformsX[i], transformY, pos1[i], pos2[i]);
		}
		result.error = result.error / pos1.size();

		//TODO: compute quality and weight
		result.quality = utility::qualityFromError(result.error, 1);
		result.state = CalibrationResult::State::CALIBRATED;
		result.weight = m1.size();
		
		ss << "Result: transformY = " << std::endl << transformY.matrix() << std::endl;
		ss << "Result: transform[" << result.systems.first.name << "->" << result.systems.second.name << "] = " << std::endl << result.transform.matrix() << std::endl;
		ss << "Result: error[" << result.systems.first.name << "->" << result.systems.second.name << "] = " << std::endl << result.error << std::endl;
		ss << "Result: quality[" << result.systems.first.name << "->" << result.systems.second.name << "] = " << std::endl << result.quality << std::endl;

		CalibrationResult combinedResult = updateCalibration(result, currentCalibration);

		ss << "CombinedResult: transform[" << combinedResult.systems.first.name << "->" << combinedResult.systems.second.name << "] = " << std::endl << combinedResult.transform.matrix() << std::endl;
		ss << "CombinedResult: error[" << combinedResult.systems.first.name << "->" << combinedResult.systems.second.name << "] = " << std::endl << combinedResult.error << std::endl;
		ss << "CombinedResult: quality[" << combinedResult.systems.first.name << "->" << combinedResult.systems.second.name << "] = " << std::endl << combinedResult.quality << std::endl;
		SPOOKY_LOG(ss.str());
		return combinedResult;
	}

	float Calibrator::estimateLatencies(const std::vector<Measurement::Ptr>& meas1, const std::vector<Measurement::Ptr>& meas2) {
		assert(false);//This method is no good. Needs to be updated
		float last_timestamp = meas1.front()->getTimestamp();
		int i = 0;
		int first = i;
		float result_sum = 0;
		int count_streams = 0;
		while (i < meas1.size()) {
			//Should be synchronised at this point
			if(i != first && meas1[i]->getTimestamp() < last_timestamp) {
				std::vector<Measurement::Ptr> m1(meas1.begin() + first, meas1.begin() + i);//Excludes last
				std::vector<Measurement::Ptr> m2(meas2.begin() + first, meas2.begin() + i);
				result_sum += estimateLatency(m1, m2);
				first = i;
				count_streams++;
			}
			last_timestamp = meas1[i]->getTimestamp();
			i++;
		}
		return result_sum / count_streams;

	}

	float Calibrator::estimateLatency(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2) {
		assert(false);//This method is no good. Needs to be updated
		Eigen::VectorXf data1(m1.size());
		Eigen::VectorXf times1(m1.size());
		for (int i = 0; i < m1.size() - 1; i++) {
			//Velocity data
			data1[i] = m1[i]->compare(m1[0]);
			//data1[i] = m1[i+1]->compare(m1[i]) / (m1[i + 1]->getTimestamp() - m1[i]->getTimestamp());
			//timestamps
			times1[i] = m1[i]->getTimestamp();
		}

		Eigen::VectorXf data2(m2.size());
		Eigen::VectorXf times2(m2.size());
		for (int i = 0; i < m2.size() - 1; i++) {
			//Velocity data
			data2[i] = m2[i]->compare(m2[0]);
			//data2[i] = m2[i+1]->compare(m2[i]) / (m2[i + 1]->getTimestamp() - m2[i]->getTimestamp());
			//timestamps
			times2[i] = m2[i]->getTimestamp();
		}
		//return latency of m2 relative to m1
		return utility::calibration::Time::estimateLatency(data1, times1, data2, times2);

	}

}