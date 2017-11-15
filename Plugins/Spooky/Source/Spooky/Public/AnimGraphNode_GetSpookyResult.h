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

#include "Editor/UnrealEd/Public/Editor.h"
#include "Editor/AnimGraph/Classes/AnimGraphNode_Base.h"
#include "AnimNode_GetSpookyResult.h"

#include "AnimGraphNode_GetSpookyResult.generated.h"


UCLASS(MinimalAPI)
class UAnimGraphNode_GetSpookyResult : public UAnimGraphNode_Base
{
	GENERATED_UCLASS_BODY()

	UPROPERTY(EditAnywhere, Category=Settings)
	FAnimNode_GetSpookyResult Node;

	// Begin UEdGraphNode interface.
	virtual FLinearColor GetNodeTitleColor() const override;
	virtual FText GetTooltipText() const override;
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	// End UEdGraphNode interface.

	// UAnimGraphNode_Base interface
	virtual FString GetNodeCategory() const override;
	virtual void PreloadRequiredAssets() override;
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
	// End of UAnimGraphNode_Base interface

	// Begin UObject Interface
	virtual void PostEditChangeChainProperty(struct FPropertyChangedChainEvent& PropertyChangedEvent) override;
	// End UObject Interface

private:
	UPROPERTY()
	bool bHasInit = false;

	USkeleton* TargetSkeleton = nullptr;
};
