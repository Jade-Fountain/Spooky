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
#include <fstream>

namespace spooky {
	
	/*	Savemanager is responsible for loading and saving data from/to files. In particular, calibration results
	*/
	class SaveManager {
		//Member variables
		std::string workingDirectory;
		std::string filename_ender = ".spky";

		////////////////////////////////////////////
		//Methods for templated functions
		////////////////////////////////////////////

		//Internal methods for calibration result
		std::string getPath(const CalibrationResult & r) const;
		CalibrationResult loadFromStream(std::ifstream& s) const;
		std::string toString(const CalibrationResult & r) const;

		//TODO: internal methods for Measurement
		// std::string getPath(const Measurement & r) const;
		// Measurement loadFromStream() const;
		// std::string toString(const Measurement & r) const;

	public:
		//Configuration
		bool setWorkingDirectory(const std::string& dir);

		//Loading objects
		template <class T>
		bool SaveManager::load(T * current)
		{
			//Get save location
			std::string pathname = getPath(*current);
			SPOOKY_LOG("Loading " +  pathname + "...");
			//Open file
			std::ifstream input;
			input.open(pathname, std::ios::in);
			//If success do load calibration
			bool success = input.is_open();
			if(success){
				T loadedT = loadFromStream(input);
				*current = loadedT;
				input.close();
			}
			return success;
		}

		//Saving objects
		template <class T>
		bool SaveManager::save(const T & result)
		{
			//Get save location
			std::string pathname = getPath(result);

			//Open file
			std::fstream output;
			output.open(pathname, std::ios::out);
			//If success do load calibration
			bool success = output.is_open();
			if(success){
				output << toString(result);
				output.close();
			}
			return success;
		}
	};

}
