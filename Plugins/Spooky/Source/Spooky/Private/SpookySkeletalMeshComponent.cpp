
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

void USpookySkeletalMeshComponent::SetDefaultBoneInfo(const FSpookySkeletonBoneInfo& info){
	defaultBoneInfo = std::make_unique<FSpookySkeletonBoneInfo>(info);
}


void USpookySkeletalMeshComponent::AddActiveBones(const TArray<FName>& bones, const TArray<FName>& boneTargetNodes, ESpookyReturnStatus& branch){
	bool bones_exist = true;
	if(!defaultBoneInfo){
		branch = ESpookyReturnStatus::Failure;
		SPOOKY_LOG("ERROR: NO DEFAULT BONE INFO SET SO CANNOT ADD ACTIVE BONES");
		return;	
	}
	std::vector<FName> missingBones;
	for(int i = 0; i < bones.Num(); i++){
		bool thisBoneExists = this->SkeletalMesh->RefSkeleton.FindBoneIndex(bones[i]) != INDEX_NONE;
		if (thisBoneExists) {
			activeBones[bones[i]] = *defaultBoneInfo;
			if (i < boneTargetNodes.Num() && boneTargetNodes[i].Compare("") != 0) {
				targetNodes[boneTargetNodes[i]] = spooky::NodeDescriptor(TCHAR_TO_UTF8(*(boneTargetNodes[i].ToString())));
			}
		}
		else {
			missingBones.push_back(bones[i]);
			bones_exist = false;
		}
	}
	if (!bones_exist) {
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

void USpookySkeletalMeshComponent::SetBoneInfo(const FSpookySkeletonBoneInfo& info, ESpookyReturnStatus& branch){
	if(activeBones.count(info.name) == 0){
		branch = ESpookyReturnStatus::Failure;
	} else {
		branch = ESpookyReturnStatus::Success;
		activeBones[info.name] = info;
	}
}



void USpookySkeletalMeshComponent::UpdateTimestamp(const FName& bone,const float& t_sec){
	if(!setup) throw "SpookySkeletalMeshComponent - not set up!!!!!!!!!!!";
	activeBones[bone].timestamp_sec = t_sec;
}


void USpookySkeletalMeshComponent::UpdateAllTimestamps(const float& t_sec){
	if(!setup) throw "SpookySkeletalMeshComponent - not set up!!!!!!!!!!!";
	for(auto& bone : activeBones){
		bone.second.timestamp_sec = t_sec;
	}
}


void USpookySkeletalMeshComponent::UpdateConfidence(const FName& bone,const float& confidence){
	if(!setup) throw "SpookySkeletalMeshComponent - not set up!!!!!!!!!!!";
	activeBones[bone].confidence = confidence;
}

spooky::NodeDescriptor USpookySkeletalMeshComponent::getTargetNode(const FName& bone) {
	if (targetNodes.count(bone) > 0) {
		return targetNodes[bone];
	}
	else {
		return spooky::NodeDescriptor(TCHAR_TO_UTF8(*(bone.ToString())));
	}
}

