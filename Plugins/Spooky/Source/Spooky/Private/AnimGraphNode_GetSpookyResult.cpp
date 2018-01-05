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
#include "Spooky.h"

#include "AnimGraphNode_GetSpookyResult.h"

UAnimGraphNode_GetSpookyResult::UAnimGraphNode_GetSpookyResult(class FObjectInitializer const &) {

}
// Begin UEdGraphNode interface.
FLinearColor UAnimGraphNode_GetSpookyResult::GetNodeTitleColor() const {
	return FLinearColor(0,0,0,1);
}
FText UAnimGraphNode_GetSpookyResult::GetTooltipText() const {
 	return NSLOCTEXT("Spooky","AnimGraphNodeGetSpookyResult_Tooltip","Returns the result of spooky skeleton fusion.");
}
FText UAnimGraphNode_GetSpookyResult::GetNodeTitle(ENodeTitleType::Type TitleType) const {
	return NSLOCTEXT("Spooky", "AnimGraphNodeGetSpookyResult_Title","Get Spooky Result");
}
// End UEdGraphNode interface.

// UAnimGraphNode_Base interface
FString UAnimGraphNode_GetSpookyResult::GetNodeCategory() const {
	return FString("Spooky");
}
void UAnimGraphNode_GetSpookyResult::PreloadRequiredAssets() {

}
void UAnimGraphNode_GetSpookyResult::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) {

}
// End of UAnimGraphNode_Base interface

// Begin UObject Interface
void UAnimGraphNode_GetSpookyResult::PostEditChangeChainProperty(struct FPropertyChangedChainEvent& PropertyChangedEvent) {

}
// End UObject Interface

