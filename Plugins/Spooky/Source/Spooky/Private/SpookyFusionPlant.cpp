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
#include "Spooky/FusionTypes.h"
#include "SpookyFusionPlant.h"
#include "Spooky/Utilities/TimeProfiling.h"
#include "Spooky/Utilities/DataStructures.h"
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
	spookyCore.config.units.input_m = input_units_m;
	spookyCore.config.units.output_m = output_units_m;
	/*
	spookyCore.config.correlator.ambiguous_threshold = correlator_ambiguous_threshold;
	spookyCore.config.correlator.elimination_threshold = correlator_elimination_threshold;
	spookyCore.config.correlator.diff_threshold = correlator_diff_threshold;

	spookyCore.config.calibrator.diff_threshold = calibration_diff_threshold;
	spookyCore.config.calibrator.min_count_per_node = calibration_min_count_per_node;
	spookyCore.config.calibrator.count_threshold = 
		{	
			{spooky::CalibrationResult::State::UNCALIBRATED,100},
			{spooky::CalibrationResult::State::REFINING,100 },
			{spooky::CalibrationResult::State::CALIBRATED,100}
		};
	spookyCore.config.calibrator.initial_quality_threshold = calibration_initial_quality_threshold;
	spookyCore.config.calibrator.quality_convergence_threshold = calibration_quality_convergence_threshold;
	spookyCore.config.calibrator.fault_hysteresis_rate = calibration_fault_hysteresis_rate;
	spookyCore.config.calibrator.relevance_decay_rate = calibration_relevance_decay_rate;
	spookyCore.config.calibrator.settle_threshold = calibration_settle_threshold;
	spookyCore.config.calibrator.fault_angle_threshold = calibration_fault_angle_threshold;
	spookyCore.config.calibrator.fault_distance_threshold = calibration_fault_distance_threshold;*/
}

UFUNCTION(BlueprintCallable, Category = "Spooky") void USpookyFusionPlant::SetJointStiffness(float stiffness) {
	spookyCore.setJointStiffness(stiffness);
}

UFUNCTION(BlueprintCallable, Category = "Spooky") void USpookyFusionPlant::AddSkeleton(USpookySkeletalMeshComponent* spooky_skeletal_mesh)
{
	//Add skeleton reference
	skeletal_spirits.push_back(spooky_skeletal_mesh);
	//Set the root node of the system associated with the skeleton
	SetSystemRootNode(spooky_skeletal_mesh->system_name, spooky_skeletal_mesh->root_node, spooky_skeletal_mesh->root_node_offset);
	return;
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::SetSystemRootNode(FString system, FString rootNode, const FTransform& rootNodeOffset) {
	spooky::SystemDescriptor sys(TCHAR_TO_UTF8(*system));
	spooky::NodeDescriptor root(TCHAR_TO_UTF8(*rootNode));
	//Add node representing offset from the root node
	//The node has the same as the system with the suffix "_root"
	if (root.name != "" && root != spooky::SPOOKY_WORLD_ROOT_DESC) {
		spookyCore.addFixedNode(spooky::NodeDescriptor(sys.name + "_root"), root, convert(rootNodeOffset.ToMatrixWithScale()));
		//Attach to sensor model
		spookyCore.setSystemRootNode(sys, spooky::NodeDescriptor(sys.name + "_root"));
	} 
}


UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddOutputTarget(USpookySkeletalMeshComponent * skeletal_mesh)
{
	TArray<FMeshBoneInfo> boneInfo = skeletal_mesh->SkeletalMesh->RefSkeleton.GetRefBoneInfo();
	for (int i = 0; i < boneInfo.Num(); i++) {
		FMeshBoneInfo& bone = boneInfo[i];
		//TODO: make more efficient
		FTransform b = FTransform(skeletal_mesh->SkeletalMesh->GetRefPoseMatrix(i));
		//Scale to spooky units
		spooky::Transform3D bonePoseLocal = convert(b.ToMatrixWithScale());
		//Set parent
		spooky::NodeDescriptor parent_desc = (bone.ParentIndex >= 0) ?
			spooky::NodeDescriptor(TCHAR_TO_UTF8(*(boneInfo[bone.ParentIndex].Name.GetPlainNameString()))) :
			spooky::NodeDescriptor();
		//Set bone name		
		spooky::NodeDescriptor bone_desc = spooky::NodeDescriptor(TCHAR_TO_UTF8(*(bone.Name.GetPlainNameString())));


		ESpookyFusionType fusionType = skeletal_mesh->GetBoneInputFusionType(bone.Name);
		switch(fusionType){
			case(ESpookyFusionType::FIXED):
			{
				spookyCore.addFixedNode(bone_desc, parent_desc, bonePoseLocal);
			}break;
			case(ESpookyFusionType::BONE):
			{
				spookyCore.addBoneNode(bone_desc, parent_desc, bonePoseLocal,
						skeletal_mesh->GetInputConstraintCentre(bone.Name), 
						skeletal_mesh->GetInputConstraintVariance(bone.Name), 
						skeletal_mesh->GetInputProcessNoise(bone.Name),
						skeletal_mesh->DoesBoneInputModelVelocity(bone.Name));
			}break;
			case(ESpookyFusionType::POSE):
			{
				spookyCore.addPoseNode(bone_desc, parent_desc, bonePoseLocal,
						skeletal_mesh->GetInputConstraintCentre(bone.Name), 
						skeletal_mesh->GetInputConstraintVariance(bone.Name), 
						skeletal_mesh->GetInputProcessNoise(bone.Name),
						skeletal_mesh->DoesBoneInputModelVelocity(bone.Name));
			}break;
			case(ESpookyFusionType::SCALE_POSE):
			{
				spookyCore.addScalePoseNode(bone_desc, parent_desc, bonePoseLocal,
						skeletal_mesh->GetInputConstraintCentre(bone.Name), 
						skeletal_mesh->GetInputConstraintVariance(bone.Name), 
						skeletal_mesh->GetInputProcessNoise(bone.Name),
						skeletal_mesh->DoesBoneInputModelVelocity(bone.Name));
			}break;
		}


		SPOOKY_LOG("Adding Bone: " + bone_desc.name + ", parent = " + parent_desc.name);
	}
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::FinaliseSetup() {
	spookyCore.finaliseSetup();
}

//Set the reference frame for the skeleton
UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::SetReferenceFrame(FString system_name) {
	spookyCore.setReferenceSystem(spooky::SystemDescriptor(TCHAR_TO_UTF8(*system_name)));
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::SetSensorLatency(FString system_name, int sensorID, float latency) {
	spookyCore.setSensorLatency(spooky::SystemDescriptor(TCHAR_TO_UTF8(*system_name)), sensorID, latency);
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::SetSystemLatency(FString system_name, float latency){
	spookyCore.setSystemLatency(spooky::SystemDescriptor(TCHAR_TO_UTF8(*system_name)), latency);
}

//===========================
//Update functions
//===========================


UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddPositionMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, FSpookyMeasurementFlags flags, float confidence)
{
	Measurement::Ptr m = CreatePositionMeasurement(systemName, sensorID, timestamp_sec, measurement, covariance, confidence);
	setFlags(m,flags);
	spookyCore.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddRotationMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FRotator measurement, FVector4 covariance, FSpookyMeasurementFlags flags, float confidence)
{
	Measurement::Ptr m = CreateRotationMeasurement(systemName,sensorID,timestamp_sec, measurement.Quaternion(),covariance,confidence);
	setFlags(m,flags);	
	spookyCore.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddPoseMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FTransform measurement, FVector position_var, FVector4 quaternion_var, FSpookyMeasurementFlags flags, float confidence)
{
	Measurement::Ptr m = CreatePoseMeasurement(systemName, sensorID, timestamp_sec, measurement.GetTranslation(), measurement.GetRotation(), position_var, quaternion_var, confidence);
	setFlags(m,flags);	
	spookyCore.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::AddScaleMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, FSpookyMeasurementFlags flags, float confidence)
{
	Measurement::Ptr m = CreateScaleMeasurement(systemName, sensorID, timestamp_sec, measurement, covariance, confidence);
	setFlags(m,flags);	
	//Scales always local to the node
	m->globalSpace = false;
	spookyCore.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}


UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::addSkeletonMeasurement(int skel_index) {
	//For each bone
	USpookySkeletalMeshComponent* skeleton = skeletal_spirits[skel_index];
	TArray<FMeshBoneInfo> boneInfo = skeleton->SkeletalMesh->RefSkeleton.GetRefBoneInfo();
	TArray<FTransform> componentSpaceTransforms = skeleton->GetComponentSpaceTransforms();
	for (int i = 0; i < boneInfo.Num(); i++) {
		//Bone info
		FMeshBoneInfo& bone = boneInfo[i];
		spooky::NodeDescriptor bone_name = spooky::NodeDescriptor(TCHAR_TO_UTF8(*(bone.Name.GetPlainNameString())));
		//If this bone is active then make new measurement
		//TODO: search other way around
		if (skeleton->isBoneOutputActive(bone.Name)) {
			spooky::NodeDescriptor targetNode = skeleton->getOutputTargetNode(bone.Name);
			FTransform T = skeleton->BoneSpaceTransforms[i];
			const FSpookySkeletonBoneOutputParams& spookyBoneInfo = skeleton->getSpookyBoneOutputParams(bone.Name);

			//Create measurement
			Measurement::Ptr m;
			if(spookyBoneInfo.flags.globalSpace){
				T = componentSpaceTransforms[i];
			}
			//Retarget to new skeleton
			FRotator retargetRotationOffset = skeleton->getOutputRetargetRotator(bone.Name);
			T.SetRotation(T.GetRotation() * retargetRotationOffset.Quaternion());
			if (spookyBoneInfo.flags.filterUnchanged) {
				if(spooky::utility::safeAccess(lastHash,bone_name) == hashFTransform(skeleton->BoneSpaceTransforms[i])){
					continue;
				} else {
					lastHash[bone_name] = hashFTransform(skeleton->BoneSpaceTransforms[i]);
				}
			}
			switch (spookyBoneInfo.measurementType) {
				//TODO: local vs global variance?
				case(ESpookyMeasurementType::GENERIC):
					//TODO: support generic measurement
					m = CreatePositionMeasurement(skeleton->system_name, i, spookyBoneInfo.timestamp_sec, T.GetTranslation(), spookyBoneInfo.position_var, spookyBoneInfo.confidence);
					break;
				case(ESpookyMeasurementType::POSITION):
					m = CreatePositionMeasurement(skeleton->system_name, i, spookyBoneInfo.timestamp_sec,T.GetTranslation(), spookyBoneInfo.position_var, spookyBoneInfo.confidence);
					break;
				case(ESpookyMeasurementType::ROTATION):
					m = CreateRotationMeasurement(skeleton->system_name, i, spookyBoneInfo.timestamp_sec, T.GetRotation(), spookyBoneInfo.quaternion_var, spookyBoneInfo.confidence);
					break;
				case(ESpookyMeasurementType::RIGID_BODY):
					m = CreatePoseMeasurement(skeleton->system_name, i, spookyBoneInfo.timestamp_sec, T.GetTranslation(), T.GetRotation(), spookyBoneInfo.position_var, spookyBoneInfo.quaternion_var, spookyBoneInfo.confidence);
					break;
				case(ESpookyMeasurementType::SCALE):
					m = CreateScaleMeasurement(skeleton->system_name, i, spookyBoneInfo.timestamp_sec, T.GetScale3D(),  spookyBoneInfo.scale_var, spookyBoneInfo.confidence);
					break;
			}
			setFlags(m, spookyBoneInfo.flags);
			spookyCore.addMeasurement(m, targetNode);
		}
	}
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::Fuse(float current_time)
{
	spooky::utility::profiler.startTimer("AAA FUSION TIME");
	for (int i = 0; i < skeletal_spirits.size(); i++) {
		addSkeletonMeasurement(i);
	}
	spookyCore.fuse(current_time);
	spooky::utility::profiler.endTimer("AAA FUSION TIME");
	//SPOOKY_LOG(spooky::utility::profiler.getReport());
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
FTransform USpookyFusionPlant::getBoneTransform(const FString& name) {
	spooky::NodeDescriptor bone_name = spooky::NodeDescriptor(TCHAR_TO_UTF8(*(name)));
	spooky::Transform3D T = spookyCore.getNodeLocalPose(bone_name);
	FTransform result(convert(T));
	return result;
}


//===========================
//Data retrieval functions
//===========================

FCalibrationResult USpookyFusionPlant::getCalibrationResult(FString s1, FString s2)
{
	spooky::CalibrationResult T = spookyCore.getCalibrationResult(spooky::SystemDescriptor(TCHAR_TO_UTF8(*s1)),spooky::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
	Eigen::Quaternionf q(T.transform.matrix().block<3,3>(0,0));
	Eigen::Vector3f v(T.transform.matrix().block<3, 1>(0, 3) / spookyCore.config.units.output_m);
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
	return spookyCore.getCorrelationResult(spooky::SystemDescriptor(TCHAR_TO_UTF8(*s1)),sensorID).name.c_str();
}

FTransform USpookyFusionPlant::getNodeGlobalPose(FString node)
{
	spooky::Transform3D result = spookyCore.getNodeGlobalPose(spooky::NodeDescriptor(TCHAR_TO_UTF8(*node)));
	//UE_LOG(LogTemp, Warning, TEXT("getNodePose : %s"), *(unrealMatrix.ToString()));
	return convert(result);
}
//===========================
//Data saving/loading functions
//===========================

//Sets save/load location	
UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::setSaveDirectory(FString dir) {
	spookyCore.setSaveDirectory(TCHAR_TO_UTF8(*dir));
}

//Saves the calibration result mapping T:s1->s2
UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::saveCalibrationResult(FString s1, FString s2){
	spookyCore.saveCalibration(spooky::SystemDescriptor(TCHAR_TO_UTF8(*s1)),spooky::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
}

//Loads the calibration result mapping T:s1->s2
UFUNCTION(BlueprintCallable, Category = "Spooky")
void USpookyFusionPlant::loadCalibrationResult(FString s1, FString s2){
	spookyCore.loadCalibration(spooky::SystemDescriptor(TCHAR_TO_UTF8(*s1)), spooky::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
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
	meas = meas * spookyCore.config.units.input_m;

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
	ev = ev * spookyCore.config.units.input_m;
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

Measurement::Ptr USpookyFusionPlant::CreatePoseMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector v, FQuat q, FVector position_var, FVector4 quaternion_var, float confidence)
{
	Eigen::Vector3f vv(&position_var[0]);
	Eigen::Vector4f vq(&quaternion_var[0]);
	Eigen::Matrix<float, 7, 1> uncertainty;
	uncertainty << vv, vq;
	return CreatePoseMeasurement(system_name, sensorID, timestamp_sec, v, q, uncertainty, confidence);
}

void USpookyFusionPlant::SetCommonMeasurementData(Measurement::Ptr& m, FString system_name, int sensorID, float timestamp_sec, float confidence){
	//Add metadata
	spookyCore.setMeasurementSensorInfo(m, spooky::SystemDescriptor(TCHAR_TO_UTF8(*system_name)), spooky::SensorID(sensorID));
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

FTransform USpookyFusionPlant::convert(const spooky::Transform3D& T) {
	FMatrix unrealMatrix;
	memcpy(&(unrealMatrix.M[0][0]), T.data(), sizeof(float) * 16);
	FTransform m(unrealMatrix);
	m.SetTranslation(m.GetTranslation() / spookyCore.config.units.output_m);
	return m;
}

spooky::Transform3D USpookyFusionPlant::convert(const FMatrix& T) {
	spooky::Transform3D matrix;
	memcpy(matrix.data(), &(T.M[0][0]), sizeof(float) * 16);
	matrix.translation() *= spookyCore.config.units.input_m;
	return matrix;
}
	
size_t USpookyFusionPlant::hashFTransform(const FTransform& T){
	size_t result = 0;
	FMatrix Tmat = T.ToMatrixWithScale();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			//XOR
			result = result ^ (std::hash<float>{}(Tmat.GetColumn(j)[i]) << 1);
		}
	}
	return result;
}

void USpookyFusionPlant::setFlags(spooky::Measurement::Ptr m, const FSpookyMeasurementFlags& flags){
	m->globalSpace = flags.globalSpace;
	m->relaxConstraints = flags.relaxConstraints;
	m->sensorDrifts = flags.sensorDrifts;
}
//===========================
//DEBUG
//===========================

//For testing blueprints: TODO delete
UFUNCTION(BlueprintCallable, Category = "Spooky")
FString USpookyFusionPlant::GetCalibrationStateSummary() {
	std::string s = spookyCore.getCalibratorStateSummary();
	return s.c_str();
}
//For testing blueprints: TODO delete
UFUNCTION(BlueprintCallable, Category = "Spooky")
FString USpookyFusionPlant::GetCalibrationTimingSummary() {
	std::string s = spookyCore.getTimingSummary();
	return s.c_str();
}

UFUNCTION(BlueprintCallable, Category = "Spooky")
FString USpookyFusionPlant::getDefaultUserSavePath() {
	return FPaths::GameSavedDir();
}
