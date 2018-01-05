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
#include "SaveManager.h"
#include "Eigen/Core"


namespace spooky {

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									SaveManager:Private
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::string SaveManager::getPath(const CalibrationResult & r) const {
		return workingDirectory + "/" + r.systems.first.name + "_" + r.systems.second.name + filename_ender;;
	}

	CalibrationResult SaveManager::loadFromStream(std::ifstream& s) const{
		CalibrationResult r;
		std::string dummy;

		//System names
		s >> dummy; s >> r.systems.first.name;	SPOOKY_LOG(dummy + " " + r.systems.first.name);
		s >> dummy; s >> r.systems.second.name;	SPOOKY_LOG(dummy + " " + r.systems.second.name);

		//State
		int state;
		s >> dummy; s >> state;	
		r.state = CalibrationResult::State(state);
		SPOOKY_LOG(dummy + " " + std::to_string(int(r.state)));

		//Transform
		s >> dummy;
		Eigen::RowVectorXf v(16);			
		for(int i = 0; i < 16; i++){
			s >> v[i];
		}
		Eigen::Matrix4f M(v.data());
		r.transform = Transform3D(M);
		std::stringstream ss;
		ss << r.transform.matrix();
		SPOOKY_LOG(dummy + " " + ss.str());

		//Other
//		s >> dummy; s >> r.latency;		SPOOKY_LOG(dummy + " " + std::to_string(r.latency));
		s >> dummy; s >> r.timestamp;	SPOOKY_LOG(dummy + " " + std::to_string(r.timestamp));
		s >> dummy; s >> r.error;		SPOOKY_LOG(dummy + " " + std::to_string(r.error));
		s >> dummy; s >> r.quality;		SPOOKY_LOG(dummy + " " + std::to_string(r.quality));

		return r;
	}

	std::string SaveManager::toString(const CalibrationResult & r) const{
		std::stringstream ss;
		ss << "domain: " << r.systems.first.name << std::endl;
		ss << "range: " << r.systems.second.name << std::endl;
		ss << "state: " << int(r.state) << std::endl;
		Eigen::Matrix4f M = r.transform.matrix();
		Eigen::Map<Eigen::RowVectorXf> v(M.data(), 16);
		ss << "transform: " << v << std::endl;
//		ss << "latency: " << r.latency << std::endl;
		ss << "timestamp: " << r.timestamp << std::endl;
		ss << "error: " << r.error << std::endl;
		ss << "quality: " << r.quality << std::endl;
		return ss.str();
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//									SaveManager:Public
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	bool SaveManager::setWorkingDirectory(const std::string & dir)
	{
		workingDirectory = dir;
		std::fstream testFile;
		std::string testDir = dir + "/test.txt";
		testFile.open(testDir, std::ios::out);
		bool success = testFile.is_open();
		if (!success) {
			SPOOKY_LOG("WARNING : Directory " + dir + "/test.txt could not be opened. Make sure the directory exists!");
		}
		testFile.close();
		return success;
	}

}
