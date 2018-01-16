/*  This file is part of SpookyUnreal, a sensor fusion plugin for VR in the Unreal Engine
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

#include "Spooky.h"

#include "Components/ActorComponent.h"
#include "Components/PoseableMeshComponent.h"
#include "BoneContainer.h"

#include "Spooky/Core.h"
#include "Spooky/FusionTypes.h"
#include "SpookySkeletalMeshComponent.h"

#include <iostream>
#include <vector>
#include <string>

//Must be last include
#include "SpookyFusionPlant.generated.h"

//Unreal engine specific struct containing the results of a calibration between two 3D sensor systems
USTRUCT()
struct FCalibrationResult {
	GENERATED_BODY()
		
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky") FTransform transform;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky") bool calibrated = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky") bool refining = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky") float quality = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky") FString system1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spooky") FString system2;
};

//Unreal engine interface layer linking to generic code from the fusion module
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class USpookyFusionPlant : public UActorComponent
{
	GENERATED_BODY()

	//FusionspookyCore
	spooky::Core spookyCore;

	//Input Skeletons
	std::vector<USpookySkeletalMeshComponent*> skeletal_spirits;

public:	

	// Sets default values for this component's properties
	USpookyFusionPlant();

	// Called when the game starts
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction ) override;

//===========================
//Setup and initialisation
//===========================

	//Add complete skeleton to list of fusion objects
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void Configure(float input_units_m = 1, float output_units_m = 1);

	//Add complete skeleton to list of fusion objects
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void AddSkeleton(USpookySkeletalMeshComponent* spooky_skeletal_mesh);

	//Set the output target which will have the complete fused skeleton pose applied
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void AddOutputTarget(USkeletalMeshComponent* skeletal_mesh);
	
	//Perform some setup postprocessing
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void FinaliseSetup();

	//Set the reference frame for the skeleton
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void SetReferenceFrame(FString system_name);

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void SetSensorLatency(FString system_name, int sensorID, float latency);

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void SetSystemLatency(FString system_name, float latency);
//TODO: Contruction of sensor nodes

	////Add a new sensor node model
	//UFUNCTION(BlueprintCallable, Category = "Spooky")
	//void AddSensorNode(FString nodeName, FTransform initialState, FTransform initialCovariance);

	////Add a new sensor node model
	//UFUNCTION(BlueprintCallable, Category = "Spooky")
	//void SetHomeCoordinateSpace(FString systemName);

//===========================
//Update functions
//===========================
	//Add vec3 measurement
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void AddPositionMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, bool globalSpace = true, float confidence = 1);
	
	//Add rotation quaternion method
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void AddRotationMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FRotator measurement, FVector4 covariance, bool globalSpace = true, float confidence = 1);

	//Add transform measurement
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void AddPoseMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FTransform measurement, FVector position_var, FVector4 quaternion_var, bool globalSpace = true, float confidence = 1);
	
	//Add scale measurement in local space
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void AddScaleMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, float confidence = 1);

	//Adds measurements for whole skeleton
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void addSkeletonMeasurement(int skel_index);

	//Align, calibrate and fuse all added data
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void Fuse(float current_time);

	//Gets animation details for driving the skeleton
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	FTransform getBoneTransform(const FString& name);

//===========================
//Data retrieval functions
//===========================
	//Gets the calibration result mapping T:s1->s2
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	FCalibrationResult getCalibrationResult(FString s1, FString s2);
	
	//Gets the name of the node which the specified sensor is attached to
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	FString getCorrelationResult(FString s1, int sensorID);

	//Gets the result of fusion for a node
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	FTransform getNodeGlobalPose(FString node);

//===========================
//Data saving/loading functions
//===========================

	//Sets save/load location	
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void setSaveDirectory(FString dir);

	//Saves the calibration result mapping T:s1->s2
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void saveCalibrationResult(FString s1, FString s2);
	
	//Loads the calibration result mapping T:s1->s2
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	void loadCalibrationResult(FString s1, FString s2);
	
//===========================
//Utility
//===========================

	//Compute axis angle representation (x,y,z,alpha)
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	FVector4 getRotatorAxisAngle(FRotator R);

	UFUNCTION(BlueprintCallable, Category = "Spooky")
	FString getDefaultUserSavePath();

	//Method to copy data from one poseable mesh to another
	void CopyPose(UPoseableMeshComponent* target, const UPoseableMeshComponent* input);

	//Methods for creating measurements which can then be sent to the fusion spookyCore
	spooky::Measurement::Ptr CreatePositionMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector position, FVector uncertainty, float confidence = 1);
	spooky::Measurement::Ptr CreateRotationMeasurement(FString system_name, int sensorID, float timestamp_sec, FQuat rotation, FVector4 uncertainty, float confidence = 1);
	spooky::Measurement::Ptr CreateScaleMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector scale, FVector uncertainty, float confidence = 1);
	spooky::Measurement::Ptr CreatePoseMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector v, FQuat q, Eigen::Matrix<float,7,1> uncertainty, float confidence = 1);
	spooky::Measurement::Ptr CreatePoseMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector v, FQuat q, FVector position_var, FVector4 quaternion_var, float confidence);

	
	//Sets data common to all types of measurements
	void SetCommonMeasurementData(spooky::Measurement::Ptr& m, FString system_name, int sensorID, float timestamp_sec, float confidence);

	//Convert names to nodeDescriptors
	std::vector<spooky::NodeDescriptor> convertToNodeDescriptors(const TArray<FString>& names);

	//Convert Transform3D to FMatrix
	FMatrix convert(const spooky::Transform3D& T);
	spooky::Transform3D convert(const FMatrix& T);

//===========================
//DEBUG
//===========================

	//For testing blueprints: TODO delete
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	FString GetCalibrationStateSummary();

	//For testing blueprints: TODO delete
	UFUNCTION(BlueprintCallable, Category = "Spooky")
	FString GetCalibrationTimingSummary();

};
