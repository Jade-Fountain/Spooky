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

#include "Spooky.h"
#include <EngineGlobals.h>
#include <Runtime/Engine/Classes/Engine/Engine.h>
#include <string>

//Unreal Engine specific log method. Would need replacement for use in other systems
inline void SPOOKY_LOG(std::string s){
	FString str(s.c_str());
	UE_LOG(LogTemp, Warning, TEXT("SPOOKY LOG : %s"),*str);
}

inline void SPOOKY_FLOG(FString s) {
	UE_LOG(LogTemp, Warning, TEXT("SPOOKY LOG : %s"), *s);
}

inline void SPOOKY_SCREEN_MESSAGE(std::string s) {
	FString str(s.c_str());
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("SPOOKY LOG : %s"), *str));
}