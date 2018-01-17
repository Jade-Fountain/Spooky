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


UENUM(BlueprintType)
enum class ESpookyMeasurementType : uint8 {
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

USTRUCT(BlueprintType)
struct FSpookySkeletonBoneInfo{
	GENERATED_BODY()

	//--------------------------------
	//		FIXED PARAMETERS
	//--------------------------------
	//Name of the system which the sensor belongs to
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FString system_name;

	//Name of the bone in the skeleton heirarchy
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FName name;

	//Index of bone in skeleton->BoneSpaceTransforms[i]
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	int bone_index;

	//The type of measurement provided by this bone.
	//Effectively masks out bone information which is known from being fused during runtime
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	ESpookyMeasurementType measurementType;

	//--------------------------------
	//		UPDATABLE PARAMETERS
	//--------------------------------
	//(these parameters will change frequently, every frame even)

	//Variances
	//.........................
	//Variance = sigma = diagonal of pos def variance matrix
	//TODO: support full matrix variance - currently not supported because unreal matrix types are limited
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector position_var;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	bool position_var_global = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector4 quaternion_var;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	bool quaternion_var_global = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector scale_var;	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	bool scale_var_global = false;

	//Confidence
	//.........................
	//A weighting of how reliable the measurement is
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	float confidence;

	//Timestamp
	//.........................
	//When the measurement was recorded, relative to start of program
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	float timestamp_sec; 	

};

UCLASS(ClassGroup=(Rendering, Spooky), hidecategories=Object, Config=Spooky, editinlinenew, meta=(BlueprintSpawnableComponent))
class USpookySkeletalMeshComponent :
	public USkeletalMeshComponent
{
	GENERATED_UCLASS_BODY()

private: 
	//List of bones which carry measurements of  with associated meta info
	std::map<FName,FSpookySkeletonBoneInfo> activeBones;

	std::unique_ptr<FSpookySkeletonBoneInfo> defaultBoneInfo;

public:


	//--------------------------------
	//		INITIALISATION
	//--------------------------------
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void SetDefaultBoneInfo(const FSpookySkeletonBoneInfo& info);

	UFUNCTION(BlueprintCallable, Category = "Spooky", Meta = (ExpandEnumAsExecs = "branch"))
	void AddActiveBones(const TArray<FName>& bones, ESpookyReturnStatus& branch);

	UFUNCTION(BlueprintCallable, Category = "Spooky", Meta = (ExpandEnumAsExecs = "branch"))
	void SetBoneInfo(const FSpookySkeletonBoneInfo& info, ESpookyReturnStatus& branch);


	//--------------------------------
	//		INPUT
	//--------------------------------

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void UpdateTimestamp(const FName& bone,const float& t_sec);

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void UpdateAllTimestamps(const float& t_sec);

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void UpdateConfidence(const FName& bone,const float& confidence);


	//--------------------------------
	//		OUTPUT
	//--------------------------------
	
	bool isBoneActive(const FName& name) {
		return activeBones.count(name) > 0;
	}

	const FSpookySkeletonBoneInfo& getSpookyBoneInfo(const FName& name) {
		return activeBones[name];
	}
};

