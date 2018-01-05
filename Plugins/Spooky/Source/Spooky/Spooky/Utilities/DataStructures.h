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

#include <vector>
#include <memory>
#include <map>
#include <set>
#include <algorithm>
#include <iterator>


namespace spooky {
	namespace utility {
		template <class X, class Y>
		class SafeMap {
		private:
			std::map<X, Y> data;
		public:
			Y& operator[] (const X& x) {
				//TODO: make more efficient!
				if (data.count(x) == 0) data[x] = Y();
				return data[x];
			}
		};

		template<class X, class Y>
		Y& safeAccess(std::map<X, Y>& m, const X& x, const Y& initialVal = Y()) {
			if (m.count(x) == 0) m[x] = initialVal;
			return m[x];
		}

		template<class X, class Y>
		std::shared_ptr<Y>& safeAccess(std::map<X, std::shared_ptr<Y>>& m, const X& x) {
			if (m.count(x) == 0) m[x] = std::make_shared<Y>();
			return m[x];
		}

		template<class X>
		std::set<X> setDiff(const std::set<X>& A, const std::set<X>& B){
		    std::set<X> diff;
		 
		    std::set_difference(A.begin(), A.end(), B.begin(), B.end(), 
		                        std::inserter(diff, diff.begin()));

		    return diff;
		}


		//A vector that takes multiple clears
		template <class T,class Counter>
		class MultiUseStream {
		public:
			typedef std::set<Counter> CounterSet;

			//Number of uses for each piece of data
			CounterSet initial;
			//Data
			std::vector<T> data;
			//Individual uses for each piece of data
			std::vector<CounterSet> counters;
			
			void push_back(const T& x){
				data.push_back(x);
				counters.push_back(initial);
			}

			//On nth clear, the vector will actually be emptied
			void addInitialCounter(const Counter& n){
				initial.insert(n);
			}

			T& operator[] (const size_t& i) {
				return data[i];
			}

			//Clears when 0 clears remaining (or less)
			void clear(const Counter& counter) {
				
				std::vector<T>::iterator data_it = data.begin();
				std::vector<CounterSet>::iterator count_it = counters.begin();

				while(data_it != data.end()) {
					count_it->erase(counter);
				    if(count_it->empty()) {
				    	//If cleared enough times, erase the data
				        data_it = data.erase(data_it);
				        count_it = counters.erase(count_it);
				    }
				    else {
				    	//Otherwise, move to next element
				    	++data_it;
				    	++count_it;
				    }
				}
			}

			int raw_size() {
				return data.size();
			}

			int size(const Counter& c){
				int result = 0;
				std::vector<T>::iterator data_it = data.begin();
				std::vector<CounterSet>::iterator count_it = counters.begin();

				while (data_it != data.end()) {
					
					//If we havent cleared this counter yet, add it to the results
					if (count_it->count(c) > 0) {
						result++;
					}
					// move to next element
					++data_it;
					++count_it;
					
				}
				return result;
			}

			std::vector<int> sizes() {
				std::vector<int> result;
				//TODO: optimise
				for (const Counter& c : initial) {
					result.push_back(size(c));
				}
				return result;
			}

			std::vector<std::string> debug_sizes_str() {
				std::vector<std::string> result;
				//TODO: optimise and make work for not strings
				for (const Counter& c : initial) {
					result.push_back(c + ":" + std::to_string(size(c)));
				}
				return result;
			}

			std::vector<T> get(const Counter& c) {
				//Avoid repeated reallocations
				std::vector<T> result(data.size());
				int result_size = 0;
				for (int i = 0; i < data.size(); i++) {
					if (counters[i].count(c) > 0) {
						result[result_size++] = data[i];
					}
				}
				result.resize(result_size);
				return result;
			}


			void eraseFront()
			{
				data.erase(data.begin());
				counters.erase(counters.begin());
			}
			T& back(){
				return data.back();
			}
			
		};

	}
}
