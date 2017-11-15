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
#include <Eigen/Core>
#include <string>
#include <strstream>
#include <map>
#include <chrono>

#include "DataStructures.h"



namespace spooky {
	namespace utility {

		using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
		using Clock = std::chrono::steady_clock;

		//Class to aid in measuring time performance of the code
		class Profiler {

		private:
			struct TimeData {
				TimePoint start;
				TimePoint end;
				const static int max_count = 100;
				Eigen::Matrix<float, max_count, 1> durations;
				int timer_count = 0;
				int ringbuffer_index = 0;

				void computeDuration(){
					durations(ringbuffer_index) = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count() / 1000000.0;
					timer_count++;
					ringbuffer_index = timer_count % max_count;
				}

				float getMeanDuration(){
					return durations.head(std::min(timer_count,max_count)).mean();
				}
				
				float getMaxDuration() {
					return durations.head(std::min(timer_count, max_count)).maxCoeff();
				}

				float getMinDuration() {
					return durations.head(std::min(timer_count, max_count)).minCoeff();
				}
			};

			std::map<std::string, TimeData> timers;
			
		public:
			void startTimer(std::string s) {
				safeAccess(timers,s).start = Clock::now();
			}
			void endTimer(std::string s){
				timers[s].end = Clock::now();
				timers[s].computeDuration();
			}
			std::string getReport(){
				std::stringstream report;
				report << "Timing report: " << std::endl;
				for(auto& timer : timers){
					report << "Timer[" << timer.first << "] duration (ms) = ( " << timer.second.getMaxDuration() << ", "
																		   << timer.second.getMeanDuration() << ", "
																		   << timer.second.getMinDuration() << " )" << std::endl;
				}
				return report.str();
			}
		};

		//Profiling
		static Profiler profiler;
	}
}