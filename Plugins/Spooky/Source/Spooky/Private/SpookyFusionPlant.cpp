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

#include "Spooky.h"
#include "SpookyFusionPlant.h"
#include "Spooky/Utilities/TimeProfiling.h"
#include <iostream>


using spooky::Measurement;
//===========================
//Setup and initialisation
//===========================


// Sets default values for this component's properties
USpookyFusionPlant::USpookyFusionPlant()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	// ...
}


// Called when the game starts
void USpookyFusionPlant::BeginPlay()
{
	Super::BeginPlay();
	// ...
	
}


// Called every frame
void USpookyFusionPlant::TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction )
{
	Super::TickComponent( DeltaTime, TickType, ThisTickFunction );
}


UFUNCTION(BlueprintCallable, Category = "Spooky") void USpookyFusionPlant::Configure(float input_units_m, float output_units_m)
{
	plant.config.units.input_m = input_units_m;
	plant.config.units.output_m = output_units_m;
	/*
	plant.config.correlator.ambiguous_threshold = correlator_ambiguous_threshold;
	plant.config.correlator.elimination_threshold = correlator_elimination_threshold;
	plant.config.correlator.diff_threshold = correlator_diff_threshold;

	plant.config.calibrator.diff_threshold = calibration_diff_threshold;
	plant.config.calibrator.min_count_per_node = calibration_min_count_per_node;
	plant.config.calibrator.count_threshold = 
		{	
			{spooky::CalibrationResult::State::UNCALIBRATED,100},
			{spooky::CalibrationResult::State::REFINING,100 },
			{spooky::CalibrationResult::State::CALIBRATED,100}
		};
	plant.config.calibrator.initial_quality_threshold = calibration_initial_quality_threshold;
	plant.config.calibrator.quality_convergence_threshold = calibration_quality_convergence_threshold;
	plant.config.calibrator.fault_hysteresis_rate = calibration_fault_hysteresis_rate;
	plant.config.calibrator.relevance_decay_rate = calibration_relevance_decay_rate;
	plant.config.calibrator.settle_threshold = calibration_settle_threshold;
	plant.config.calibrator.fault_angle_threshold = calibration_fault_angle_threshold;
	plant.config.calibrator.fault_distance_threshold = calibration_fault_distance_threshold;*/
}

UFUNCTION(BlueprintCallable, Category = "Spooky") void USpookyFusionPlant::AddSkeleton(USkeletalMeshComponent* skeletal_mesh, FVector position_var, FVector4 quaternion_var)
{
	//TODO: enable complete skeleton fusion
	//Add skeleton reference
	skeletons.push_back(skeletal_mesh);

	//Store uncertainties for later
	Eigen::Vector3f vv(&position_var[0]);
	Eigen::Vector4f vq(&quaternion_var[0]);
	Eigen::Matrix<float, 7, 1> uncertainty;
	uncertainty << vv, vq;
	skeletonCovariances.push_back(uncertainty);
	return;
}


UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddOutputTarget(USkeletalMeshComponent * skeletal_mesh)
{
	skeletal_mesh = skeletal_mesh;
	TArray<FMeshBoneInfo> boneInfo = skeletal_mesh->SkeletalMesh->RefSkeleton.GetRefBoneInfo();
	for (int i = 0; i < boneInfo.Num(); i++) {
		FMeshBoneInfo& bone = boneInfo[i];
		//TODO: make more efficient
		FTransform b = FTransform(skeletal_mesh->SkeletalMesh->GetRefPoseMatrix(i));
		//Scale to spooky units
		b.SetTranslation(b.GetTranslation() * plant.config.units.input_m);
		spooky::Transform3D bonePoseLocal = convert(b.ToMatrixNoScale());
		//Set parent
		spooky::NodeDescriptor parent_desc = (bone.ParentIndex >= 0) ?
			spooky::NodeDescriptor(TCHAR_TO_UTF8(*(boneInfo[bone.ParentIndex].Name.GetPlainNameString()))) :
			spooky::NodeDescriptor();
		//Set bone name		
		spooky::NodeDescriptor bone_desc = spooky::NodeDescriptor(TCHAR_TO_UTF8(*(bone.Name.GetPlainNameString())));
		//TODO: find better way to do this check for pose nodes
		if (bone.Name.GetPlainNameString() == "pelvis") {
			//The pelvis has 6DoF pose and 3DoF scale
			plant.addScalePoseNode(bone_desc, parent_desc, bonePoseLocal, Eigen::Vector3f::Ones());
		}
		else {
			plant.addBoneNode(bone_desc, parent_desc, bonePoseLocal);
		}
		SPOOKY_LOG("Adding Bone: " + bone_desc.name + ", parent = " + parent_desc.name);
	}
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::FinaliseSetup() {
	plant.finaliseSetup();
}

//Set the reference frame for the skeleton
UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::SetReferenceFrame(FString system_name) {
	plant.setReferenceSystem(spooky::SystemDescriptor(TCHAR_TO_UTF8(*system_name)));
}

//===========================
//Update functions
//===========================


UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddPositionMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, bool globalSpace, float confidence)
{
	Measurement::Ptr m = CreatePositionMeasurement(systemName, sensorID, timestamp_sec, measurement, covariance, confidence);
	m->globalSpace = globalSpace;
	plant.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddRotationMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FRotator measurement, FVector4 covariance, bool globalSpace, float confidence)
{
	Measurement::Ptr m = CreateRotationMeasurement(systemName,sensorID,timestamp_sec, measurement.Quaternion(),covariance,confidence);
	m->globalSpace = globalSpace;
	plant.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddPoseMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FTransform measurement, FVector position_var, FVector4 quaternion_var, bool globalSpace, float confidence)
{
	Eigen::Vector3f vv(&position_var[0]);
	Eigen::Vector4f vq(&quaternion_var[0]);
	Eigen::Matrix<float, 7, 1> uncertainty;
	uncertainty << vv, vq;
	Measurement::Ptr m = CreatePoseMeasurement(systemName, sensorID, timestamp_sec, measurement.GetTranslation(), measurement.GetRotation(), uncertainty, confidence);
	m->globalSpace = globalSpace;
	plant.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddScaleMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, float confidence)
{
	Measurement::Ptr m = CreateScaleMeasurement(systemName, sensorID, timestamp_sec, measurement, covariance, confidence);
	//Scales always local to the node
	m->globalSpace = false;
	plant.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::addSkeletonMeasurement(int skel_index) {
	//For each bone
	auto& skeleton = skeletons[skel_index];
	TArray<FMeshBoneInfo> boneInfo = skeleton->SkeletalMesh->RefSkeleton.GetRefBoneInfo();
	for (int i = 0; i < boneInfo.Num(); i++) {
		FMeshBoneInfo& bone = boneInfo[i];
		spooky::NodeDescriptor bone_name = spooky::NodeDescriptor(TCHAR_TO_UTF8(*(bone.Name.GetPlainNameString())));
		FTransform measurement = skeleton->BoneSpaceTransforms[i];
		//TODO: support confidences
		//TODO: doesnt seem like the best way to do this!
		//TODO: support skeleton group measurement input properly: need skeleton->getUncertianty(i), get confidence, time stamp, etc
		float timestamp_sec = 0;// skeleton->getLatestMeasurementTime();
		Measurement::Ptr m = CreatePoseMeasurement(skeleton->GetName(), i, timestamp_sec, measurement.GetTranslation(), measurement.GetRotation(), skeletonCovariances[skel_index], 1);
		m->globalSpace = false;
		plant.addMeasurement(m, bone_name);
	}
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::Fuse(float current_time)
{
	spooky::utility::profiler.startTimer("AAA FUSION TIME");
	for (int i = 0; i < skeletons.size(); i++) {
		addSkeletonMeasurement(i);
	}
	plant.fuse(current_time);
	spooky::utility::profiler.endTimer("AAA FUSION TIME");
	//SPOOKY_LOG(spooky::utility::profiler.getReport());
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
FTransform USpookyFusionPlant::getBoneTransform(const FString& name) {
	spooky::NodeDescriptor bone_name = spooky::NodeDescriptor(TCHAR_TO_UTF8(*(name)));
	spooky::Transform3D T = plant.getNodeLocalPose(bone_name);
	FTransform result(convert(T));
	result.SetTranslation(result.GetTranslation() / plant.config.units.output_m);
	return result;
}


//===========================
//Data retrieval functions
//===========================

FCalibrationResult USpookyFusionPlant::getCalibrationResult(FString s1, FString s2)
{
	spooky::CalibrationResult T = plant.getCalibrationResult(spooky::SystemDescriptor(TCHAR_TO_UTF8(*s1)),spooky::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
	Eigen::Quaternionf q(T.transform.matrix().block<3,3>(0,0));
	Eigen::Vector3f v(T.transform.matrix().block<3, 1>(0, 3) / plant.config.units.output_m);
	FQuat fq(q.x(), q.y(), q.z(), q.w());
	
	FCalibrationResult result;
	result.transform.SetRotation(fq);
	result.transform.SetTranslation(FVector(v[0], v[1], v[2]));
	result.calibrated = T.calibrated();
	result.refining = T.refining();
	result.quality = T.quality;
	result.system1 = FString(T.systems.first.name.c_str());
	result.system2 = FString(T.systems.second.name.c_str());
	return result;
}

FString USpookyFusionPlant::getCorrelationResult(FString s1, int sensorID)
{
	return plant.getCorrelationResult(spooky::SystemDescriptor(TCHAR_TO_UTF8(*s1)),sensorID).name.c_str();
}

FTransform USpookyFusionPlant::getNodeGlobalPose(FString node)
{
	spooky::Transform3D result = plant.getNodeGlobalPose(spooky::NodeDescriptor(TCHAR_TO_UTF8(*node)));
	FMatrix unrealMatrix = convert(result);
	unrealMatrix.ScaleTranslation(FVector(1,1,1) * 1 / plant.config.units.output_m);
	//UE_LOG(LogTemp, Warning, TEXT("getNodePose : %s"), *(unrealMatrix.ToString()));
	return FTransform(unrealMatrix);
}
//===========================
//Data saving/loading functions
//===========================

//Sets save/load location	
UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::setSaveDirectory(FString dir) {
	plant.setSaveDirectory(TCHAR_TO_UTF8(*dir));
}

//Saves the calibration result mapping T:s1->s2
UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::saveCalibrationResult(FString s1, FString s2){
	plant.saveCalibration(spooky::SystemDescriptor(TCHAR_TO_UTF8(*s1)),spooky::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
}

//Loads the calibration result mapping T:s1->s2
UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::loadCalibrationResult(FString s1, FString s2){
	plant.loadCalibration(spooky::SystemDescriptor(TCHAR_TO_UTF8(*s1)), spooky::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
}

//===========================
//Utility
//===========================

//Compute axis angle representation (x,y,z,alpha)
UFUNCTION(BlueprintCallable, Category = "Spooky")
FVector4 USpookyFusionPlant::getRotatorAxisAngle(FRotator R) {
	float angle;
	FVector axis;
	R.Quaternion().ToAxisAndAngle(axis,angle);
	return FVector4(axis[0], axis[1], axis[2], angle * 180 / M_PI);
}

//TODO: optimise with const ref
Measurement::Ptr USpookyFusionPlant::CreatePositionMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector position, FVector uncertainty, float confidence)
{
	//Create basic measurement
	Eigen::Vector3f meas(position[0],position[1],position[2]);
	meas = meas * plant.config.units.input_m;

	Eigen::Matrix<float, 3, 3> un = Eigen::Matrix<float,3,3>::Identity();
	un.diagonal() = Eigen::Vector3f(uncertainty[0], uncertainty[1], uncertainty[2]);
	Measurement::Ptr result = Measurement::createCartesianMeasurement(meas, un);
	
	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);

	return std::move(result);
}

Measurement::Ptr USpookyFusionPlant::CreateRotationMeasurement(FString system_name, int sensorID, float timestamp_sec, FQuat rotation, FVector4 uncertainty, float confidence)
{
	//Create basic measurement
	//BEWARE: dumb format mismatch:
	Eigen::Quaternionf meas(rotation.W, rotation.X, rotation.Y, rotation.Z);
	Eigen::Matrix<float, 4, 4> un = Eigen::Matrix<float, 4, 4>::Identity();
	un.diagonal() = Eigen::Vector4f(&uncertainty[0]);
	Measurement::Ptr result = Measurement::createQuaternionMeasurement(meas, un);

	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);
	
	return std::move(result);
}

Measurement::Ptr USpookyFusionPlant::CreateScaleMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector scale, FVector uncertainty, float confidence)
{
	//Create basic measurement
	Eigen::Vector3f meas(&scale[0]);
	Eigen::Matrix<float, 3, 3> un = Eigen::Matrix<float, 3, 3>::Identity();
	un.diagonal() = Eigen::Vector3f(&uncertainty[0]);
	Measurement::Ptr result = Measurement::createScaleMeasurement(meas, un);

	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);
	
	return std::move(result);
}

Measurement::Ptr USpookyFusionPlant::CreatePoseMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector v, FQuat q, Eigen::Matrix<float, 7, 1> uncertainty, float confidence)
{
	//Convert transform to state vector (v,q)
	Eigen::Vector3f ev(&v[0]);
	ev = ev * plant.config.units.input_m;
	//BEWARE: dumb format mismatch:
	Eigen::Quaternionf eq(q.W,q.X,q.Y,q.Z);
	//Create basic measurement
	Eigen::Matrix<float, 7, 7> un = Eigen::Matrix<float, 7, 7>::Identity();
	un.diagonal() = uncertainty;
	Measurement::Ptr result = Measurement::createPoseMeasurement(ev, eq, un);
	
	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);

	return std::move(result);
}

void USpookyFusionPlant::SetCommonMeasurementData(Measurement::Ptr& m, FString system_name, int sensorID, float timestamp_sec, float confidence){
	//Add metadata
	plant.setMeasurementSensorInfo(m, spooky::SystemDescriptor(TCHAR_TO_UTF8(*system_name)), spooky::SensorID(sensorID));
	bool measurementConsistent = m->setMetaData(timestamp_sec, confidence);
	if (!measurementConsistent) {
		std::cout << "WARNING - Measurement not created correctly - " << __FILE__ << " : " << __LINE__ << std::endl;
	}
}

std::vector<spooky::NodeDescriptor> USpookyFusionPlant::convertToNodeDescriptors(const TArray<FString>& names){
	std::vector<spooky::NodeDescriptor> result;
	for(auto& name : names){
		result.push_back(spooky::NodeDescriptor(TCHAR_TO_UTF8(*name)));
	}
	return result;
}

FMatrix USpookyFusionPlant::convert(const spooky::Transform3D& T) {
	FMatrix unrealMatrix;
	memcpy(&(unrealMatrix.M[0][0]), T.data(), sizeof(float) * 16);
	return unrealMatrix;
}

spooky::Transform3D USpookyFusionPlant::convert(const FMatrix& T) {
	spooky::Transform3D matrix;
	memcpy(matrix.data(), &(T.M[0][0]), sizeof(float) * 16);
	return matrix;
}

//===========================
//DEBUG
//===========================

//For testing blueprints: TODO delete
UFUNCTION(BlueprintCallable, Category = "Spooky")
FString USpookyFusionPlant::GetCalibrationStateSummary() {
	std::string s = plant.getCalibratorStateSummary();
	return s.c_str();
}
//For testing blueprints: TODO delete
UFUNCTION(BlueprintCallable, Category = "Spooky")
FString USpookyFusionPlant::GetCalibrationTimingSummary() {
	std::string s = plant.getTimingSummary();
	return s.c_str();
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
FString USpookyFusionPlant::getDefaultUserSavePath() {
	return FPaths::GameSavedDir();
}
