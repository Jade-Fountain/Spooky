
#include "Spooky.h"
#include "Correlator.h"
#include "Utilities/CorrelationUtilities.h"

namespace spooky {

	float Correlator::getCorrelationScore(const std::vector<Measurement::Ptr>& measurements1, const std::vector<Measurement::Ptr>& measurements2)
	{
		//Synchronise 
		std::vector<Measurement::Ptr> m1 = measurements1;
		//Sync measurements2 to times of measurements1
		std::vector<Measurement::Ptr> m2 = measurements1;
		Measurement::synchronise(m2, m1);

		Measurement::Type t1 = m1.front()->type;
		Measurement::Type t2 = m2.front()->type;
		//Bulk logic to route calibration procedures at runtime
		switch (t1) {
		case Measurement::Type::POSITION:
			if (t2 == Measurement::Type::POSITION) {
				return correlatePos(m1,m2);
			}
			break;
		case Measurement::Type::RIGID_BODY:
			if (t2 == Measurement::Type::RIGID_BODY) {
				return 0;
			}
		}
		SPOOKY_LOG("WARNING : no correlationm model found for measurement types: " + std::to_string(t1) + " and " + std::to_string(t2));
		return 0;
	}

	float Correlator::getCorrelationScore(const std::vector<Measurement::Ptr>& ambiguousStream, const Data::Streams& hypothesisStreams)
	{
		float totalScore = 0;
		for(auto& stream : hypothesisStreams.sensors){
			Sensor::Ptr sensor = stream.first;
			std::vector<Measurement::Ptr> measurements = stream.second;
			totalScore += getCorrelationScore(ambiguousStream, measurements);
		}
		//Average score
		return totalScore / hypothesisStreams.sensors.size();
	}

	float Correlator::correlatePos(const std::vector<Measurement::Ptr>& m1, const std::vector<Measurement::Ptr>& m2){
		std::vector<Eigen::Vector3f> pos1(m1.size());
		std::vector<Eigen::Vector3f> pos2(m2.size());
		std::vector<Eigen::Matrix3f> inverse_variances(m1.size());
		for (int i = 0; i < m1.size(); i++) {
			pos1[i] = m1[i]->getData();
			pos2[i] = m2[i]->getData();
			//TODO: Not strictly correct
			inverse_variances[i] = (m1[i]->getUncertainty() + m2[i]->getUncertainty()).inverse();
		}
		
		return  utility::correlation::Position::correlationQualityWeighted(pos1,pos2,inverse_variances);
	}


}
