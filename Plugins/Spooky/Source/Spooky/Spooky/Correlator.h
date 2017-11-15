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

#include <string>
#include <map>
#include "Eigen/Core"
#include "FusionTypes.h"
#include "Utilities/DataStructures.h"
namespace spooky {


	//Centre of a fusion plant
	class Correlator {
	private:
		class Data {
		public:
			struct Streams{
				std::map<Sensor::Ptr, std::vector<Measurement::Ptr>> sensors;
			};
			
			//---------------------------------------------------------------------------------
			//MEMBERS
			//---------------------------------------------------------------------------------

			//Contains measurements for the ambiguous sensors
			Streams ambiguous_measurements;

			//Maps each node to set of unambiguous measurements from attached sensors
			//i.e. unamb measurements sorted by node, handles multiple sensors per node
			std::map<NodeDescriptor, Streams> unambiguous_measurements;

			//Nodes for which data is needed
			std::set<NodeDescriptor> relevant_nodes;

			//---------------------------------------------------------------------------------
			//HELPER FUNCTIONS
			//---------------------------------------------------------------------------------

			//Adds ambiguous measurement to data list
			void addAmbiguous(const Measurement::Ptr& m);
			//Adds unambiguous measurement to data list
			void addUnambiguous(const Measurement::Ptr& m);
			//Check if measurements from sensor s are useful for resolving ambiguities
			bool unambiguousMeasurementNeeded(const Sensor::Ptr& s);
			//Compare new measurement to last measurement of that sensor to see if useful
			float compareMeasurement(const Measurement::Ptr& m);
			//Returns true if no data recorded for the specified sensor
			bool unseen(const Sensor::Ptr& sensor);
			//Returns the unambiguous streams relevant to the specified node
			const Streams& getUnambiguousStreams(const NodeDescriptor& node);
			//Returns number of ambiguous measurements recorded for sensor
			int ambiguousCount(const Sensor::Ptr& sensor);
			//Clear ambiguous data associated with sensor s
			void clear(const Sensor::Ptr& s);
			//Cleanup - removes irrelevant and used unambiguous data
			void cleanUp();

			//Returns true if all sensors in the ambiguous streams are unambiguous
			bool isCorrelated();
			
		};
		//---------------------------------------------------------------------------------
		//MEMBERS
		//---------------------------------------------------------------------------------

		//Data structure for organising and filtering measurements
		Data data;


	public:
		//---------------------------------------------------------------------------------
		//FUNCTION INTERFACE
		//---------------------------------------------------------------------------------
		//Config
		struct Config{
			int ambiguous_threshold = 10;
			float elimination_threshold = 1;
			float diff_threshold = 0.1;
		} config;

		//Set config method
		void configure(const Config& cfg){config=cfg;}
		
		//Add data for later calibration
		void addMeasurement(const Measurement::Ptr& m);
		void addMeasurementGroup(const std::vector<Measurement::Ptr>& measurementQueue);

		//Performes identification procedure if possible
		void identify();

		//Returns true if useable data is now available
		bool isStable();

	private:
		//---------------------------------------------------------------------------------
		//INTERNALS
		//---------------------------------------------------------------------------------

		//Returns true if enough data has been collected for identification process for sensor
		bool dataSufficient(const Sensor::Ptr& sensor);

		//Removes measurements which do not give info about the current sensors of interest
		std::vector<Measurement::Ptr> filterLonelyData(const std::vector<Measurement::Ptr>& measurementQueue);

		bool checkChanges(const std::vector<Measurement::Ptr>& measurements);

		//Adds measurment for a sensor which is attached to an unknown node
		void addAmbiguousMeasurement(const Measurement::Ptr& m);

		//Adds measurement for a sensor attached to a known node
		void addUnambiguousMeasurementIfNeeded(const Measurement::Ptr& m);


		//---------------------------------------------------------------------------------
		//CORRELATION PROCEDURES (defined in CorrelationProcedures.cpp)
		//---------------------------------------------------------------------------------

		//Compares two streams of data to check if they are correlated
		float getCorrelationScore(const std::vector<Measurement::Ptr>& s1, const std::vector<Measurement::Ptr>& s2);
		//Returns correlation score between one stream and a collection of streams known to be correlated
		float getCorrelationScore(const std::vector<Measurement::Ptr>& ambiguousStream, const Data::Streams& hypothesisStreams);

		//Correlation of two position measurement streams
		float correlatePos(const std::vector<Measurement::Ptr>& s1, const std::vector<Measurement::Ptr>& s2);

	};

}

