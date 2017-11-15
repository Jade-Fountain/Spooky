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

