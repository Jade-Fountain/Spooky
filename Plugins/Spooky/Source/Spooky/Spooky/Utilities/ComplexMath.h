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
#include <complex>

//Complex-complex comparisons
template <typename s>
inline bool operator< (const std::complex<s>& lhs, const std::complex<s>& rhs) { return std::abs(lhs) < std::abs(rhs); }

template <typename s>
inline bool operator> (const std::complex<s>& lhs, const std::complex<s>& rhs) { return rhs < lhs; }

template <typename s>
inline bool operator<= (const std::complex<s>& lhs, const std::complex<s>& rhs) { return std::abs(lhs) <= std::abs(rhs); }

template <typename s>
inline bool operator>= (const std::complex<s>& lhs, const std::complex<s>& rhs) { return rhs <= lhs; }

//Scalar-complex comparisons
template <typename s>
inline bool operator< (const s& lhs, const std::complex<s>& rhs) { return lhs < rhs.real(); }

template <typename s>
inline bool operator< (const std::complex<s>& lhs, const s& rhs) { return  lhs.real() < rhs; }

template <typename s>
inline bool operator> (const std::complex<s>& lhs, const s& rhs) { return  rhs < lhs; }

template <typename s>
inline bool operator> (const s& lhs, const std::complex<s>& rhs) { return  rhs < lhs; }

//Scalar-complex comparisons
template <typename s>
inline bool operator<= (const s& lhs, const std::complex<s>& rhs) { return lhs <= rhs.real(); }

template <typename s>
inline bool operator<= (const std::complex<s>& lhs, const s& rhs) { return  lhs.real() <= rhs; }

template <typename s>
inline bool operator>= (const std::complex<s>& lhs, const s& rhs) { return  rhs <= lhs; }

template <typename s>
inline bool operator>= (const s& lhs, const std::complex<s>& rhs) { return  rhs <= lhs; }