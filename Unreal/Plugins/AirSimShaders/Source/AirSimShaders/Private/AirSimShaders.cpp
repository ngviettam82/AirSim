// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Modules/ModuleInterface.h"
#include "Modules/ModuleManager.h"
#include "ShaderCore.h"

class FAirSimShaders : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        if (TSharedPtr<IPlugin> plugin = IPluginManager::Get().FindPlugin(TEXT("AirSimShaders"))) {
            AddShaderSourceDirectoryMapping(
                TEXT("/Plugin/AirSimShaders"),
                FPaths::Combine(plugin->GetBaseDir(), TEXT("Shaders")));
        }
    }

    virtual void ShutdownModule() override
    {
    }
};

IMPLEMENT_MODULE(FAirSimShaders, AirSimShaders)
