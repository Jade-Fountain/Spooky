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
enum class ESpookyFusionType : uint8 {
	FIXED = 0,
	BONE = 1,
	POSE = 2,
	SCALE_POSE = 3
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
	//Name of the bone in the skeleton heirarchy
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FName name;

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
	
	//Whether to use a local or global route to fusion
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	bool useGlobalData; 	
	float timestamp_sec;
};

//Fusion parameters describe how the skeleton behaves when targeted by spooky
USTRUCT(BlueprintType)
struct FSpookyBoneFusionParameters{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	ESpookyFusionType type;

	//Quadratic constraints Energy = centre^T * var^-1 * centre
	//Position constraints
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector position_centre;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector position_var;

	//Rotation constraints
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector4 quaternion_centre;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector4 quaternion_var;

	//Scale constraints
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector scale_centre;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector scale_var;

	//Process noises:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector position_process_noise;
	FVector vel_position_process_noise;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector4 quaternion_process_noise;
	FVector4 vel_quaternion_process_noise;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky")
	FVector scale_process_noise;
	FVector vel_scale_process_noise;

};

UCLASS(ClassGroup=(Rendering, Spooky), hidecategories=Object, Config=Spooky, editinlinenew, meta=(BlueprintSpawnableComponent))
class USpookySkeletalMeshComponent :
	public USkeletalMeshComponent
{
	GENERATED_UCLASS_BODY()

private: 

	//List of bones which carry measurements of  with associated meta info
	std::map<FName, FSpookySkeletonBoneInfo> activeBones;
	//List of target nodes for each active bone
	std::map<FName, spooky::NodeDescriptor> targetNodes;
	std::map<FName, FRotator> retargetRotators;

	//List of bones which can be modified by measurements when this skeleton is in output mode
	std::map<FName,FSpookyBoneFusionParameters> fusionBones;

	//Default data for a new bones
	std::unique_ptr<FSpookySkeletonBoneInfo> defaultBoneInfo;
	std::unique_ptr<FSpookyBoneFusionParameters> defaultBoneFusionParams;

	bool setup = false;

public:

	//Name of the system which the sensor belongs to
	FString system_name;
	//Name of the root node that the sensor system rests on
	FString root_node;
	//The transform of the sensor system relative to the root node
	FTransform root_node_offset;
	
	//--------------------------------
	//		INITIALISATION
	//--------------------------------

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void SetSystemInfo(const FString& systemName, const FString& rootNode, const FTransform& rootNodeOffset);

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void SetDefaultBoneInfo(const FSpookySkeletonBoneInfo& info);

	UFUNCTION(BlueprintCallable, Category = "Spooky", Meta = (ExpandEnumAsExecs = "branch"))
	void AddActiveBones(const TArray<FName>& bones, const TArray<FName>& boneTargetNodes, const TArray<FRotator>& boneRetargetRotators,ESpookyReturnStatus& branch);

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
	
	ESpookyFusionType GetBoneFusionType(const FName& bone);
	Eigen::VectorXf GetConstraintCentre(const FName& bone);
	Eigen::MatrixXf GetConstraintVariance(const FName& bone);
	Eigen::MatrixXf GetProcessNoise(const FName& bone);

	bool isBoneActive(const FName& name) {
		return activeBones.count(name) > 0;
	}

	const FSpookySkeletonBoneInfo& getSpookyBoneInfo(const FName& name) {
		return activeBones[name];
	}

	//Retrieve the spooky skeleton node being targeted by the given bone
	//Defaults to bone.string if not specified otherwise
	spooky::NodeDescriptor getTargetNode(const FName& bone);
	FRotator getRetargetRotator(const FName& bone);

};

