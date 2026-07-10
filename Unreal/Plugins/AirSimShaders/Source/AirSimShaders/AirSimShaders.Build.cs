// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

using UnrealBuildTool;

public class AirSimShaders : ModuleRules
{
    public AirSimShaders(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PrivateIncludePaths.Add(System.IO.Path.Combine(GetModuleDirectory("Renderer"), "Private"));

        PublicDependencyModuleNames.AddRange(new string[] { "Core" });
        PrivateDependencyModuleNames.AddRange(new string[] { "CoreUObject", "Engine", "Projects", "RenderCore", "Renderer", "RHI" });
    }
}
