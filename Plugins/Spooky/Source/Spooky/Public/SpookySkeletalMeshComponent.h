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
#include <../Classes/Components/SkeletalMeshComponent.h>
#include <vector>
#include "Spooky/FusionTypes.h"
#include "SpookySkeletalMeshComponent.generated.h"

UENUM()
enum ESpookyMeasurementType {
	GENERIC = 0,
	POSITION = 1,
	ROTATION = 2,
	RIGID_BODY = 3,
	SCALE = 4
};

USTRUCT()
struct FSpookySkeletonBoneInfo{
	GENERATED_USTRUCT_BODY()
	FString name;
	FVector pos_variance;
	FVector4 quat_variance;
	FVector scale_variance;
	int bone_index;
	float confidence;
	double timestamp_sec;
	ESpookyMeasurementType measurementType;
};

UCLASS(ClassGroup=(Rendering, Spooky), hidecategories=Object, config=Engine, editinlinenew, meta=(BlueprintSpawnableComponent))
class USpookySkeletalMeshComponent :
	public USkeletalMeshComponent
{
	GENERATED_UCLASS_BODY()

private: 
	//TODO: read from config file?
	std::vector<FSpookySkeletonBoneInfo> activeBones;

public:

};

