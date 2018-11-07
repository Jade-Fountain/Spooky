// Copyright 1998-2016 Epic Games, Inc. All Rights Reserved.

#pragma once

#define _ENABLE_EXTENDED_ALIGNED_STORAGE

#include "Engine.h"

#include "ModuleManager.h"

class FSpookyModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
	
};