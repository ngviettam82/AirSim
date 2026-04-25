// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "AirSim.h"
#include "Materials/Material.h"
#include "Materials/MaterialInterface.h"
#include "Misc/Paths.h"
#include "Misc/CoreDelegates.h"
#include "Modules/ModuleManager.h"
#include "Modules/ModuleInterface.h"

#if WITH_EDITOR
#include "Editor.h"
#include "FileHelpers.h"
#endif

namespace
{
#if WITH_EDITOR
	const TCHAR* AnnotationMaterialPath = TEXT("Material'/AirSim/HUDAssets/AnnotationMaterial.AnnotationMaterial'");

	void EnsureAnnotationMaterialSupportsInstancedStaticMeshes()
	{
		// Run before PIE so instanced segmentation mirrors never fall back to UE's default material.
		if (!GIsEditor || IsRunningCommandlet() || FApp::IsGame())
		{
			return;
		}

		UMaterialInterface* annotation_material_interface = LoadObject<UMaterialInterface>(nullptr, AnnotationMaterialPath);
		if (!IsValid(annotation_material_interface))
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Failed to load %s while preparing instanced segmentation materials."), AnnotationMaterialPath);
			return;
		}

		UMaterial* annotation_material = annotation_material_interface->GetMaterial();
		if (!IsValid(annotation_material))
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: %s did not resolve to a valid base material for instanced segmentation."), *annotation_material_interface->GetPathName());
			return;
		}

		if (!annotation_material->CheckMaterialUsage(MATUSAGE_InstancedStaticMeshes))
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: %s still does not support instanced static meshes; segmentation mirrors may render with the default material."), *annotation_material->GetPathName());
			return;
		}

		UPackage* material_package = annotation_material->GetOutermost();
		if (IsValid(material_package) && material_package->IsDirty())
		{
			const bool bSaved = UEditorLoadingAndSavingUtils::SavePackages({ material_package }, true);
			if (bSaved)
			{
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: Saved %s after enabling instanced static mesh support on %s."), *material_package->GetName(), *annotation_material->GetName());
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: %s was updated for instanced static meshes but could not be saved automatically. Please save the asset once in the editor."), *annotation_material->GetPathName());
			}
		}
	}
#endif
}

class FAirSim : public IModuleInterface
{
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

#if WITH_EDITOR
    void OnPostEngineInit();
    void OnPreBeginPIE(bool bIsSimulating);
#endif
};

IMPLEMENT_MODULE(FAirSim, AirSim)

void FAirSim::StartupModule()
{
    //plugin startup
    UE_LOG(LogTemp, Log, TEXT("StartupModule: AirSim plugin"));

#if WITH_EDITOR
    FCoreDelegates::OnPostEngineInit.AddRaw(this, &FAirSim::OnPostEngineInit);
    FEditorDelegates::PreBeginPIE.AddRaw(this, &FAirSim::OnPreBeginPIE);
#endif
}

void FAirSim::ShutdownModule()
{
    //plugin shutdown

#if WITH_EDITOR
    FCoreDelegates::OnPostEngineInit.RemoveAll(this);
    FEditorDelegates::PreBeginPIE.RemoveAll(this);
#endif
}

#if WITH_EDITOR
void FAirSim::OnPostEngineInit()
{
	EnsureAnnotationMaterialSupportsInstancedStaticMeshes();
}

void FAirSim::OnPreBeginPIE(bool bIsSimulating)
{
	EnsureAnnotationMaterialSupportsInstancedStaticMeshes();
}
#endif
