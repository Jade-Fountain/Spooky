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


UENUM(BlueprintType)
enum class ESpookyReturnStatus : uint8
{
	Success,
	Failure
};

USTRUCT()
struct FSpookySkeletonBoneInfo{
	GENERATED_USTRUCT_BODY()

	//Name of the bone in the skeleton heirarchy
	FString name;
	//Variances
	struct {
		//Variance = sigma = diagonal of pos def variance matrix
		//TODO: support full matrix variance - currently not supported because unreal matrix types are limited
		struct {
			FVector sigma;
			bool isGlobal = false;
		} position;

		struct {
			FVector4 sigma;
			bool isGlobal = false;
		} quaternion;

		struct {
			FVector sigma;
			bool isGlobal = false;
		} scale;

	} variance;

	//Index of bone in skeleton->BoneSpaceTransforms[i]
	int bone_index;
	//A weighting of how reliable the measurement is
	float confidence;
	//When the measurement was recorded, relative to start of program
	double timestamp_sec;
	//The type of measurement provided by this bone.
	//Effectively masks out bone information which is known from being fused during runtime
	ESpookyMeasurementType measurementType;
};

UCLASS(ClassGroup=(Rendering, Spooky), hidecategories=Object, Config=Spooky, editinlinenew, meta=(BlueprintSpawnableComponent))
class USpookySkeletalMeshComponent :
	public USkeletalMeshComponent
{
	GENERATED_UCLASS_BODY()

private: 
	//List of bones which carry measurements of  with associated meta info
	std::map<std::string,FSpookySkeletonBoneInfo> activeBones;

	std::unique_ptr<FSpookySkeletonBoneInfo> defaultBoneInfo;
public:

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void SetDefaultBoneInfo(const FSpookySkeletonBoneInfo& info);

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void AddActiveBones(const TArray<FString>& bones, ESpookyReturnStatus& branch);

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void SetBoneInfo(const FSpookySkeletonBoneInfo& info);

	//TODO: read from config file?
	// UFUNCTION(BlueprintCallable, Category = "Spooky")
	// void LoadBoneConfig(const FString& filename);

	
};

