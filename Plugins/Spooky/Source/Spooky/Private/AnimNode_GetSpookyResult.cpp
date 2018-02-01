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
#include "AnimNode_GetSpookyResult.h"

	FAnimNode_GetSpookyResult::FAnimNode_GetSpookyResult()
		: FAnimNode_Base(){

	}
	// FAnimNode_Base interface
	void FAnimNode_GetSpookyResult::Initialize(const FAnimationInitializeContext& Context) {
	}
	void FAnimNode_GetSpookyResult::CacheBones(const FAnimationCacheBonesContext& Context) {
	}
	void FAnimNode_GetSpookyResult::Update(const FAnimationUpdateContext& Context) {
		//***************************************
		// Evaluate Graph, see AnimNode_Base, AnimNodeBase.h
		EvaluateGraphExposedInputs.Execute(Context);
		//***************************************

	}
	
	void FAnimNode_GetSpookyResult::Evaluate(FPoseContext& Output){
		if (spookyFP) {
			//Copy new data in from the fusion plant
			//The root node pose is fixed by externals
			//Output.Pose[FCompactPoseBoneIndex(0)] = Output.Pose.GetRefPose(FCompactPoseBoneIndex(0));
			SPOOKY_FLOG(Output.Pose.GetRefPose(FCompactPoseBoneIndex(0)).ToString());
			for (int index = 0; index < Output.Pose.GetNumBones(); index++) {
				FString bone_name = Output.GetAnimBlueprint()->TargetSkeleton->GetReferenceSkeleton().GetBoneName(index).GetPlainNameString();
				Output.Pose[FCompactPoseBoneIndex(index)] = spookyFP->getBoneTransform(bone_name);
				SPOOKY_FLOG(bone_name);
				SPOOKY_FLOG(Output.Pose[FCompactPoseBoneIndex(0)].ToString());
				//Output.Pose[FCompactPoseBoneIndex(index)].SetRotation(FQuat(0, 0, 0, 1));
				//SPOOKY_FLOG(bone_name);
				//SPOOKY_FLOG(Output.Pose[FCompactPoseBoneIndex(index)].ToString());
				//SPOOKY_FLOG("Ref pose");
				//SPOOKY_FLOG(Output.Pose.GetRefPose(FCompactPoseBoneIndex(index)).ToString());

			}
		}

	}
	// End of FAnimNode_Base interface

