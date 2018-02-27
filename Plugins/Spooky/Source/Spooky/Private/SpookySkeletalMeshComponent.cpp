
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

void USpookySkeletalMeshComponent::SetDefaultBoneInputParams(const FSpookyBoneInputParameters& params){
	defaultBoneInputParams = std::make_unique<FSpookyBoneInputParameters>(params);
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
			if (i < boneTargetNodes.Num() && boneTargetNodes[i].Compare("") != 0) {
				targetNodes[bones[i]] = spooky::NodeDescriptor(TCHAR_TO_UTF8(*(boneTargetNodes[i].ToString())));
			}
			if (i < boneRetargetRotators.Num()) {
				retargetRotators[bones[i]] = boneRetargetRotators[i];
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
		outputBones[info.name] = info;
	}
}

void USpookySkeletalMeshComponent::SetBoneInputParams(const FSpookyBoneInputParameters& info, ESpookyReturnStatus& branch){
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


spooky::NodeDescriptor USpookySkeletalMeshComponent::getOutputTargetNode(const FName& bone) {
	if (targetNodes.count(bone) > 0) {
		return targetNodes[bone];
	}
	else {
		return spooky::NodeDescriptor(TCHAR_TO_UTF8(*(bone.ToString())));
	}
}

FRotator USpookySkeletalMeshComponent::getOutputRetargetRotator(const FName& bone) {
	if (retargetRotators.count(bone) > 0) {
		return retargetRotators[bone];
	}
	else {
		return FRotator(0,0,0);
	}
}


ESpookyFusionType USpookySkeletalMeshComponent::GetBoneFusionType(const FName& bone){
	if(inputBones.count(bone) == 0){
		return ESpookyFusionType::FIXED;
	} else {
		return inputBones.type;
	}
}

Eigen::VectorXf USpookySkeletalMeshComponent::GetConstraintCentre(const FName& bone){
	if(inputBones.count(bone) == 0){
		//TODO: make this nicer
		throw ("Bone " + bone.ToString() + " doesn't exist! - tried to access fusion parameters!");
	} else {
		return inputBones[bone].constraint_centre;
	}

}

Eigen::MatrixXf USpookySkeletalMeshComponent::GetConstraintVariance(const FName& bone){
	if(inputBones.count(bone) == 0){
		throw std::runtime_exception("Bone " + bone.toString() + " doesn't exist! - tried to access fusion parameters!");
	} else {
		return inputBones[bone].constraint_variance.asDiagonal();
	}
}

Eigen::MatrixXf USpookySkeletalMeshComponent::GetProcessNoise(const FName& bone){
	if(inputBones.count(bone) == 0){
		throw ("Bone " + bone.ToString() + " doesn't exist! - tried to access fusion parameters!");
	} else {		
		return inputBones[bone].process_noise.asDiagonal();
	}
}

