// Copyright 1998-2016 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class Spooky : ModuleRules
{
	public Spooky(TargetInfo Target)
	{
		
		PublicIncludePaths.AddRange(
			new string[] {
				"Spooky/Public",
               // "Spooky/Spooky"
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				"Spooky/Private",
				//"Spooky/Spooky",
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
                "AnimGraph"
				// ... add other public dependencies that you statically link with here ...
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
                "AnimGraph",
                "Core",
                "UnrealEd",         // for FAssetEditorManager
				"AssetTools",       // class FAssetTypeActions_Base
                "Slate",
                "SlateCore",
                "PropertyEditor",
                "EditorStyle",
                "DesktopWidgets",
                "DesktopPlatform",
                "Projects",
                "InputCore",
                "BlueprintGraph"
				// ... add private dependencies that you statically link with here ...	
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
