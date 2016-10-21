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
//			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/sysl.lib");
//			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/nshxl.lib");
//			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/nspcl.lib");
//			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/adtl.lib");
//			PublicAdditionalLibraries.Add(strn+"/c/x64/debug/ccl.lib");

			PublicAdditionalLibraries.Add(strn + "/c/x64/release/sysl.lib");
			PublicAdditionalLibraries.Add(strn + "/c/x64/release/nshxl.lib");
			PublicAdditionalLibraries.Add(strn + "/c/x64/release/nspcl.lib");
			PublicAdditionalLibraries.Add(strn + "/c/x64/release/adtl.lib");
			PublicAdditionalLibraries.Add(strn + "/c/x64/release/ccl.lib");
		}  // if

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
