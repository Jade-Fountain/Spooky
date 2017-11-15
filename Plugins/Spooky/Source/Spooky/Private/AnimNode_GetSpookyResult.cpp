/*  This file is part of UnrealFusion, a sensor fusion plugin for VR in the Unreal Engine
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
			for (int index = 0; index < Output.Pose.GetNumBones(); index++) {
				FString bone_name = Output.GetAnimBlueprint()->TargetSkeleton->GetReferenceSkeleton().GetBoneName(index).GetPlainNameString();
				Output.Pose[FCompactPoseBoneIndex(index)] = spookyFP->getBoneTransform(bone_name);
				//Output.Pose[FCompactPoseBoneIndex(index)].SetRotation(FQuat(0, 0, 0, 1));
			}
		}

	}
	// End of FAnimNode_Base interface

