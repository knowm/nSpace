////////////////////////////////////////////////////////////////////
//
//									nSpace.Build.cs
//
//					C# build file for the Unreal engine.
//
////////////////////////////////////////////////////////////////////

using UnrealBuildTool;
using System;

public class nSpace : ModuleRules
	{
	public nSpace(TargetInfo Target)
		{
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore" });

		PrivateDependencyModuleNames.AddRange(new string[] {  });

		// Environment variable stored location of nSpace
		String strn = Environment.GetEnvironmentVariable("SDK_NSPACE");

		// Include file for the nSpace ActiveX client library.
		PublicIncludePaths.Add(strn+"/c/lib/nshxl/");

		// Libraries
		if (Target.Platform == UnrealTargetPlatform.Win64)
			{
			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/sysl.2015.lib");
			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/nshxl.2015.lib");
			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/nspcl.2015.lib");
			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/adtl.2015.lib");
			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/ccl.2015.lib");
			}	// if

		// 
		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");
		// if ((Target.Platform == UnrealTargetPlatform.Win32) || (Target.Platform == UnrealTargetPlatform.Win64))
		// {
		//		if (UEBuildConfiguration.bCompileSteamOSS == true)
		//		{
		//			DynamicallyLoadedModuleNames.Add("OnlineSubsystemSteam");
		//		}
		// }
	}
}
