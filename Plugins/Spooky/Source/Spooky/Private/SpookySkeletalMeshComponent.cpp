
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
#include "Logging.h"
#include "SpookySkeletalMeshComponent.h"
#include "SpookyFusionPlant.h"
#include "Classes/Kismet/KismetMathLibrary.h"

USpookySkeletalMeshComponent::USpookySkeletalMeshComponent(class FObjectInitializer const &)
{
}


void USpookySkeletalMeshComponent::SetSystemInfo(const FString& systemName, const FString& rootNode, const FTransform& rootNodeOffset){
	system_name = systemName;
	root_node = rootNode;
	root_node_offset = rootNodeOffset;
	setup = true;
}

void USpookySkeletalMeshComponent::SetDefaultBoneOutputParams(const FSpookySkeletonBoneOutputParams& info){
	defaultBoneOutputParams = std::make_unique<FSpookySkeletonBoneOutputParams>(info);
}

void USpookySkeletalMeshComponent::SetDefaultBoneInputParams(const FSpookyBoneInputParams& params){
	defaultBoneInputParams = std::make_unique<FSpookyBoneInputParams>(params);
}


void USpookySkeletalMeshComponent::AddOutputBones(const TArray<FName>& bones, const TArray<FName>& boneTargetNodes, const TArray<FRotator>& boneRetargetRotators, ESpookyReturnStatus& branch){
	if(!defaultBoneOutputParams){
		branch = ESpookyReturnStatus::Failure;
		SPOOKY_LOG("ERROR: NO DEFAULT BONE INFO SET SO CANNOT ADD ACTIVE BONES");
		return;	
	}
	std::vector<FName> missingBones;
	for(int i = 0; i < bones.Num(); i++){
		bool thisBoneExists = this->SkeletalMesh->RefSkeleton.FindBoneIndex(bones[i]) != INDEX_NONE;
		if (thisBoneExists) {
			outputBones[bones[i]] = *defaultBoneOutputParams;
			outputBones[bones[i]].id = i;
			if (i < boneTargetNodes.Num() && boneTargetNodes[i].Compare("") != 0) {
				targetNodes[bones[i]] = spooky::NodeDescriptor(TCHAR_TO_UTF8(*(boneTargetNodes[i].ToString())));
			}
			if (i < boneRetargetRotators.Num()) {
				retargetRotators[bones[i]] = boneRetargetRotators[i];
			}
			if (outputBones[bones[i]].flags.accumulateOffsets) {
				outputOffsets[bones[i]] = spooky::Transform3D::Identity();
			}
		}
		else {
			missingBones.push_back(bones[i]);
		}
	}
	if (missingBones.size() > 0) {
		branch = ESpookyReturnStatus::Failure;
		SPOOKY_LOG("ERROR: THE FOLLOWING REQUESTED ACTIVE BONES DO NOT EXIST: ");
		for (auto& b : missingBones) {
			SPOOKY_LOG(TCHAR_TO_UTF8(*(b.ToString())));
		}
	}
	else {
		branch = ESpookyReturnStatus::Success;
	}
}

void USpookySkeletalMeshComponent::AddInputBones(const TArray<FName>& bones, ESpookyReturnStatus& branch){
	if(!defaultBoneInputParams){
		branch = ESpookyReturnStatus::Failure;
		SPOOKY_LOG("ERROR: NO DEFAULT BONE INFO SET SO CANNOT ADD ACTIVE BONES");
		return;	
	}
	std::vector<FName> missingBones;
	for(int i = 0; i < bones.Num(); i++){
		bool thisBoneExists = this->SkeletalMesh->RefSkeleton.FindBoneIndex(bones[i]) != INDEX_NONE;
		if (thisBoneExists) {
			inputBones[bones[i]] = *defaultBoneInputParams;
		}
		else {
			missingBones.push_back(bones[i]);
		}
	}
	if (missingBones.size() > 0) {
		branch = ESpookyReturnStatus::Failure;
		SPOOKY_LOG("ERROR: THE FOLLOWING REQUESTED ACTIVE BONES DO NOT EXIST: ");
		for (auto& b : missingBones) {
			SPOOKY_LOG(TCHAR_TO_UTF8(*(b.ToString())));
		}
	}
	else {
		branch = ESpookyReturnStatus::Success;
	}
}


void USpookySkeletalMeshComponent::SetBoneOutputParams(const FSpookySkeletonBoneOutputParams& info, ESpookyReturnStatus& branch){
	if(outputBones.count(info.name) == 0){
		branch = ESpookyReturnStatus::Failure;
	} else {
		branch = ESpookyReturnStatus::Success;
		auto info_ = info;
		info_.id = outputBones[info.name].id;
		outputBones[info.name] = info_;
		if (outputBones[info.name].flags.accumulateOffsets) {
			outputOffsets[info.name] = spooky::Transform3D::Identity();
		}
	}
}

void USpookySkeletalMeshComponent::SetBoneInputParams(const FSpookyBoneInputParams& info, ESpookyReturnStatus& branch){
	if(inputBones.count(info.name) == 0){
		branch = ESpookyReturnStatus::Failure;
	} else {
		branch = ESpookyReturnStatus::Success;
		inputBones[info.name] = info;
	}
}



void USpookySkeletalMeshComponent::UpdateTimestamp(const FName& bone,const float& t_sec){
	if(!setup) throw "SpookySkeletalMeshComponent - not set up!!!!!!!!!!!";
	outputBones[bone].timestamp_sec = t_sec;
}


void USpookySkeletalMeshComponent::UpdateAllTimestamps(const float& t_sec){
	if(!setup) throw "SpookySkeletalMeshComponent - not set up!!!!!!!!!!!";
	for(auto& bone : outputBones){
		bone.second.timestamp_sec = t_sec;
	}
}


void USpookySkeletalMeshComponent::UpdateOutputConfidence(const FName& bone,const float& confidence){
	if(!setup) throw "SpookySkeletalMeshComponent - not set up!!!!!!!!!!!";
	outputBones[bone].confidence = confidence;
}
void USpookySkeletalMeshComponent::SetAllFlags(const FSpookyMeasurementFlags& flags) {
	if (!setup) throw "SpookySkeletalMeshComponent - not set up!!!!!!!!!!!";
	for (auto& bone : outputBones) {
		bone.second.flags = flags;
	}
}

void USpookySkeletalMeshComponent::AccumulateOffsets(spooky::ArticulatedModel& skeleton, const float& t) {
	TArray<FTransform> componentSpaceTransforms = GetComponentSpaceTransforms();
	for (auto& bone : outputBones) {
		if (bone.second.flags.accumulateOffsets) {
			spooky::NodeDescriptor boneDesc = targetNodes.count(bone.first) == 0 ? 
				spooky::NodeDescriptor(TCHAR_TO_UTF8(*(bone.first.ToString()))):
				targetNodes[bone.first];
			spooky::Transform3D currentPose = bone.second.flags.globalSpace ? 
				skeleton.getNodeGlobalPose(boneDesc):
				skeleton.getNodeLocalPose(boneDesc);
			spooky::Transform3D sensedPose = USpookyFusionPlant::convert(
				bone.second.flags.globalSpace ?
				componentSpaceTransforms[bone.second.id].ToMatrixWithScale() :
				BoneSpaceTransforms[bone.second.id].ToMatrixWithScale()
			);
			spooky::Transform3D newOffset = sensedPose.inverse() * currentPose;
			outputOffsets[bone.first] = spooky::utility::slerpTransform3D(outputOffsets[bone.first], newOffset, bone.second.offsetLearningRate * skeleton.getNodeNonOffsetConfidence(boneDesc, t));
		}
	}
}


spooky::NodeDescriptor USpookySkeletalMeshComponent::getOutputTargetNode(const FName& bone) {
	if (targetNodes.count(bone) > 0) {
		return targetNodes[bone];
	}
	else {
		return spooky::NodeDescriptor(TCHAR_TO_UTF8(*(bone.ToString())));
	}
}

FRotator USpookySkeletalMeshComponent::getOutputRetargetRotator(const FName& bone, bool* found) {
	if (retargetRotators.count(bone) > 0) {
		*found = true;
		return retargetRotators[bone];
	}
	else {
		*found = false;
		return FRotator(0,0,0);
	}
}


ESpookyFusionType USpookySkeletalMeshComponent::GetBoneInputFusionType(const FName& bone){
	if(inputBones.count(bone) == 0){
		return ESpookyFusionType::FIXED;
	} else {
		return inputBones[bone].fusion_type;
	}
}

Eigen::VectorXf USpookySkeletalMeshComponent::GetInputConstraintCentre(const FName& bone){
	if(inputBones.count(bone) == 0){
		//TODO: make this nicer
		throw ("Bone " + bone.ToString() + " doesn't exist! - tried to access fusion parameters!");
	} else {
		TArray<float> v_ = inputBones[bone].constraint_centre;
		Eigen::VectorXf v(v_.Num());
		for (int i = 0; i < v_.Num(); i++) {
			v[i] = v_[i];
		}
		return v;
	}

}

Eigen::MatrixXf USpookySkeletalMeshComponent::GetInputConstraintVariance(const FName& bone){
	if(inputBones.count(bone) == 0){
		throw ("Bone " + bone.ToString() + " doesn't exist! - tried to access fusion parameters!");
	} else {
		TArray<float> v_ = inputBones[bone].constraint_var;
		Eigen::VectorXf v(v_.Num());
		for (int i = 0; i < v_.Num(); i++) {
			v[i] = v_[i];
		}
		return v.asDiagonal(); 
	}
}

Eigen::MatrixXf USpookySkeletalMeshComponent::GetInputProcessNoise(const FName& bone){
	if(inputBones.count(bone) == 0){
		throw ("Bone " + bone.ToString() + " doesn't exist! - tried to access fusion parameters!");
	} else {		
		TArray<float> v_ = inputBones[bone].process_noise;
		Eigen::VectorXf v(v_.Num());
		for (int i = 0; i < v_.Num(); i++) {
			v[i] = v_[i];
		}
		return v.asDiagonal(); 
	}
}

bool USpookySkeletalMeshComponent::DoesBoneInputModelVelocity(const FName& bone){
	if(inputBones.count(bone) == 0){
		throw ("Bone " + bone.ToString() + " doesn't exist! - tried to access fusion parameters!");
	} else {		
		return inputBones[bone].model_velocity;
	}
}
FTransform USpookySkeletalMeshComponent::GetAccumulatedOffset(const FName& bone) {
	if (outputOffsets.count(bone) == 0) {
		outputOffsets[bone] = spooky::Transform3D::Identity();
	}
	return USpookyFusionPlant::convert(outputOffsets[bone]);
	
}


