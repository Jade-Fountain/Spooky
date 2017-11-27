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
#include "FusionTypes.h"
#include "Utilities/Conventions.h"

//#define NDEBUG
#include <cassert>

namespace spooky {


	//Define config constants
	const float Measurement::uncertainty_growth_max = 0.01f; //Fractional growth per second

	//=========================
	//Static factory methods:
	//=========================
	Measurement::Ptr Measurement::createCartesianMeasurement(Eigen::Vector3f position, Eigen::Matrix<float,3,3> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = Type::POSITION;
		meas->data = position;
		meas->uncertainty = sigma;
		meas->size = position.rows();
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createQuaternionMeasurement(Eigen::Quaternionf quaternion, Eigen::Matrix<float,4,4> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = Type::ROTATION;
		meas->data = quaternion.coeffs();
		meas->uncertainty = sigma;
		meas->size = 4;
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createScaleMeasurement(Eigen::Vector3f scale, Eigen::Matrix<float,3,3> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = Type::SCALE;
		meas->data = scale;
		meas->uncertainty = sigma;
		meas->size = scale.rows();
		return std::move(meas);
	}
	Measurement::Ptr Measurement::createPoseMeasurement(Eigen::Vector3f position, Eigen::Quaternionf quaternion, Eigen::Matrix<float,7,7> sigma) {
		Measurement::Ptr meas = std::make_shared<Measurement>();
		meas->type = Type::RIGID_BODY;
		meas->data = Eigen::Matrix<float, 7, 1>();
		//Eigen coeffs are stored (x,y,z,w)
		//However, Eigen::Quaternionf(w,x,y,z) constructor doesnt obey this. Never use this constructor and should be fine
		meas->data << position, quaternion.coeffs();
		meas->uncertainty = sigma;
		meas->size = 7;
		return std::move(meas);
	}

	//=========================
	//Data Out Interface
	//=========================

	Eigen::Vector3f Measurement::getPosition(){
		if(type == Type::POSITION || type == Type::RIGID_BODY){
			return data.head(3);
		} else {
			return Eigen::Vector3f::Zero();
		}
	}

	Eigen::Matrix3f Measurement::getPositionVar(){
		if(type == Type::POSITION || type == Type::RIGID_BODY){
			return uncertainty.topLeftCorner(3,3);
		} else {
			return max_var * Eigen::Matrix3f::Identity();
		}
	}

	Eigen::Quaternionf Measurement::getRotation(){
		if(type == Type::ROTATION || type == Type::RIGID_BODY){
			return Eigen::Quaternionf(Eigen::Vector4f(data.tail(4)));
		} else {
			return Eigen::Quaternionf::Identity();
		}
	}

	Eigen::Matrix4f Measurement::getRotationVar(){
		if(type == Type::ROTATION || type == Type::RIGID_BODY){
			return uncertainty.bottomRightCorner(4,4);
		} else {
			return max_var * Eigen::Matrix4f::Identity();
		}
	}

	Eigen::Matrix<float,7,1>  Measurement::getPosQuat(){
		auto pos = getPosition();
		auto quat = getRotation();
		Eigen::Matrix<float,7,1> result;
		result << pos, quat.coeffs();
		return result;
	}

	Eigen::Matrix<float,7,7> Measurement::getPosQuatVar(){
		if(type == RIGID_BODY){
			return uncertainty;
		} else {
			Eigen::Matrix3f pVar = getPositionVar();
			Eigen::Matrix4f qVar = getRotationVar();
			Eigen::Matrix<float,7,7> result = Eigen::Matrix<float,7,7>::Identity();
			result.topLeftCorner(3,3) = pVar; 
			result.bottomRightCorner(4,4) = qVar;
			return result; 
		}
	}

	Eigen::Vector3f Measurement::getScale () {
		if (type == Type::SCALE) {
			return data.head(3);
		}
		else {
			return Eigen::Vector3f::Ones();
		}
	}

	Eigen::Matrix3f Measurement::getScaleVar() {
		if (type == Type::SCALE) {
			return uncertainty.topLeftCorner(3, 3);
		}
		else {
			return max_var * Eigen::Matrix3f::Identity();
		}
	}

	Transform3D Measurement::getTransform(){
		Transform3D T = Transform3D::Identity();
		bool rigid = type == Type::RIGID_BODY;
		bool pos = type == Type::POSITION || rigid;
		bool rot = type == Type::ROTATION || rigid;
		if(pos){
			T.translate(Eigen::Vector3f(data.head(3)));
		}
		if(rot){
			//Quat defined by tail 4 always
			Eigen::Quaternionf q(Eigen::Vector4f(data.tail(4)));
			T.rotate(q);
		}
		return T;
	}
	

	Eigen::VectorXf Measurement::difference(const Measurement::Ptr& other) {
		if (type != other->type) {
			throw std::runtime_error(__FILE__ + __LINE__ + std::string(" : Cannot compare two measurements of differing type"));
		}
		//TODO: deal with noisy measurements, esp rotation
		if (type == Type::POSITION || type == Type::RIGID_BODY) {
			return getPosition() - other->getPosition();
		}
		else if (type == Type::ROTATION) {
			return Eigen::Matrix<float, 1,1>(Eigen::AngleAxisf(other->getRotation().inverse() * getRotation()).angle());
		}
		else {
			return data - other->data;
		}
	}

	float Measurement::compare(const Measurement::Ptr& other) {
		return difference(other).norm();
	}
	//TODO: refactor using custom struct with two measurement streams
	void Measurement::synchronise(
		std::vector<Measurement::Ptr>& source, 
		std::vector<Measurement::Ptr>& target
		//std::vector<Measurement::Ptr>& target_out
	){
		//std::stringstream ss;
		//ss << "Source = " << source.front()->getSensor()->system.name << " count = " << source.size() << std::endl;
		//ss << "Target = " << target.front()->getSensor()->system.name << " count = " << target.size() << std::endl;

		std::vector<Measurement::Ptr> result;
		std::vector<Measurement::Ptr> target_out;

		std::vector<Measurement::Ptr>::const_iterator source_it = source.begin();
		std::vector<Measurement::Ptr>::const_iterator target_it = target.begin();
		
		if ((*source_it)->getTimestamp() == (*target_it)->getTimestamp()) {
			result.push_back(*source_it);
			target_out.push_back(*target_it);
		}

		while(target_it != target.end()){
			//Iterate to target after current source
			while(
				target_it != target.end() && 
				(*target_it)->getTimestamp() <= (*source_it)->getTimestamp()
			){
				target_it++;
			}
			
			//If we ran out of target measurements
			if(target_it == target.end()) break;

			//Increase source iterator until the next measurement is after or equal to the new target
			while(
				std::next(source_it) != source.end() && 
				(*std::next(source_it))->getTimestamp() < (*target_it)->getTimestamp()
			){
				source_it++;
			}

			//If there are no more source measurements
			if(std::next(source_it) == source.end()) break;
			
			//Interpolate between nearest measurements
			std::vector<Measurement::Ptr>::const_iterator lower_source_it = source_it;
			std::vector<Measurement::Ptr>::const_iterator upper_source_it = std::next(source_it);

			//Avoid interpolating if possible
			if (std::fabs((*lower_source_it)->getTimestamp() - (*target_it)->getTimestamp()) < 1e-6) {
				result.push_back(*lower_source_it);
				target_out.push_back(*target_it);
			} else {
				//Interpolate to synchronise
				float t0 = (*lower_source_it)->getTimestamp();
				float t1 = (*upper_source_it)->getTimestamp();
				float t = ((*target_it)->getTimestamp() - t0) / (t1 - t0);
				target_out.push_back(*target_it);

				result.push_back(Measurement::interpolate(*lower_source_it, *upper_source_it, t));

				//ss << "----------------------------------------" << std::endl;
				//ss << "resampling source point between " << (*lower_source_it)->getTimestamp() 
				//	<<" and " << (*upper_source_it)->getTimestamp() << 
				//	" at " << (*target_it)->getTimestamp() << std::endl;
				//ss  << "values: lower = " << (*lower_source_it)->getData().transpose() << std::endl
				//	<< " interpolated = " << result.back()->getData().transpose() << std::endl
				//	<< " upper =        " << (*upper_source_it)->getData().transpose() << std::endl;
				//ss << "----------------------------------------" << std::endl;
			}

			//Place source_it after/equal to current target_it
			source_it++;
		}
		//ss << "Source = " << source.front()->getSensor()->system.name << ", " << source.front()->getSensor()->getNode().name << " final count = " << result.size() << std::endl;
		//ss << "Target = " << target.front()->getSensor()->system.name << ", " << target.front()->getSensor()->getNode().name << " final count = " << target_out.size() << std::endl;
		//SPOOKY_LOG(ss.str());

		source = result;
		target = target_out;
	}

	Eigen::VectorXf Measurement::interpolateData(const Measurement::Ptr& x, const Measurement::Ptr& y, const float& t, const Measurement::Type& type) {
		Eigen::VectorXf result = x->getData() * (1-t) + y->getData() * t;
		if (type == Measurement::Type::ROTATION || type == Measurement::Type::RIGID_BODY) {
			Eigen::Quaternionf q1 = x->getRotation();
			Eigen::Quaternionf q2 = y->getRotation();
			Eigen::Quaternionf q = q1.slerp(t, q2);
			if (type == Measurement::Type::ROTATION) {
				return q.coeffs();
			}
			result.tail(4) = q.coeffs();
		}
		return result;
	}

	Measurement::Ptr Measurement::interpolate(const Measurement::Ptr& m0, const Measurement::Ptr& m1, float t){
		assert(m0->getSensor() == m1->getSensor());
		Measurement::Ptr result = std::make_shared<Measurement>(*m0);
		result->data = Measurement::interpolateData(m0,m1,t,m0->type);
		result->setTimestamp(m0->getTimestamp() * (1-t) + m1->getTimestamp() * t);

		float uncertainty_growth = 4 * t * (1-t) * uncertainty_growth_max * (m0->getTimestamp() - m1->getTimestamp()) / 2;
		result->uncertainty = (m0->uncertainty * (1-t) + m1->uncertainty * t) * (1 + uncertainty_growth);
		result->confidence = m0->confidence * (1-t) + m1->confidence * t;
		return result;
	}

	Measurement::Ptr Measurement::extrapolate(const Measurement::Ptr& m, float time_sec){
		Measurement::Ptr result = std::make_shared<Measurement>(*m);
		//Grow uncertainty linearly based on the time elapsed
		float uncertainty_growth = time_sec * uncertainty_growth_max;
		result->uncertainty = (m->uncertainty) * (1 + uncertainty_growth);
		//Otherwise guess same data, etc.
		return result;
	}

	void Measurement::setLatencies(std::vector<Measurement::Ptr>& m, float latency)
	{
		for (auto& meas : m) {
			meas->setLatency(latency);
		}
	}

	Eigen::Matrix<float, 7, 1> Measurement::getPosQuatFromTransform(const Transform3D& T) {
		Eigen::Matrix<float, 7, 1> result;
		result << T.translation(), Eigen::Quaternionf(T.rotation()).coeffs();
		return result;
	}


}


