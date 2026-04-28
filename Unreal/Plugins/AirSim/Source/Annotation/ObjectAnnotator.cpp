// Weichao Qiu @ 2017
// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.
#include "ObjectAnnotator.h"
#include "Runtime/Engine/Public/EngineUtils.h"
#include "SceneInterface.h"
#include "../Private/ScenePrivate.h"
#include "Runtime/Launch/Resources/Version.h"
#include "Components/SkinnedMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "LandscapeComponent.h"
#include "LandscapeProxy.h"
#include "AnnotationComponent.h"
#include "AirBlueprintLib.h"

// For UE4 < 17
// check https://github.com/unrealcv/unrealcv/blob/1369a72be8428547318d8a52ae2d63e1eb57a001/Source/UnrealCV/Private/Controller/ObjectAnnotator.cpp#L1

namespace
{
	bool IsSourceStencilBackendName(const FString& render_backend)
	{
		const FString normalized_backend = render_backend.TrimStartAndEnd().ToLower();
		return normalized_backend == TEXT("sourcestencil") ||
			normalized_backend == TEXT("source_stencil") ||
			normalized_backend == TEXT("stencil");
	}

	bool IsProxyBackendName(const FString& render_backend)
	{
		const FString normalized_backend = render_backend.TrimStartAndEnd().ToLower();
		return normalized_backend == TEXT("proxy") ||
			normalized_backend == TEXT("annotationproxy") ||
			normalized_backend == TEXT("annotation_proxy");
	}

	FString StripRuntimeSuffixes(const FString& value)
	{
		if (value.IsEmpty())
		{
			return value;
		}

		TArray<FString> parts;
		value.ParseIntoArray(parts, TEXT("_"), true);

		while (parts.Num() > 1 && parts.Last().IsNumeric())
		{
			parts.Pop();
		}

		if (parts.Num() == 0)
		{
			return value;
		}

		return FString::Join(parts, TEXT("_"));
	}

	FString GetStableMeshLabel(UMeshComponent* component)
	{
		if (const UStaticMeshComponent* static_mesh_component = Cast<UStaticMeshComponent>(component))
		{
			if (IsValid(static_mesh_component->GetStaticMesh()))
			{
				return static_mesh_component->GetStaticMesh()->GetName();
			}
		}

		if (const USkinnedMeshComponent* skinned_mesh_component = Cast<USkinnedMeshComponent>(component))
		{
			if (UObject* skinned_asset = skinned_mesh_component->GetSkinnedAsset())
			{
				return skinned_asset->GetName();
			}
		}

		return FString();
	}

	FString GetStableLandscapeLabel(ULandscapeComponent* component)
	{
		if (!IsValid(component))
		{
			return FString();
		}

		if (ALandscapeProxy* landscape_proxy = component->GetLandscapeProxy())
		{
			return landscape_proxy->GetName();
		}

		if (AActor* owner = component->GetOwner())
		{
			return owner->GetName();
		}

		return component->GetName();
	}

	FString GetLandscapeComponentId(ULandscapeComponent* component)
	{
		if (!IsValid(component))
		{
			return FString();
		}

		const FString landscape_label = GetStableLandscapeLabel(component);
		const FIntPoint section_base = component->GetSectionBase();
		return FString::Printf(TEXT("Landscape_%s_%s_%d_%d"), *landscape_label, *component->GetName(), section_base.X, section_base.Y);
	}

	AActor* GetAnnotationOwnerForComponent(const USceneComponent* component)
	{
		if (!IsValid(component))
		{
			return nullptr;
		}

		AActor* owner = component->GetAttachmentRootActor();
		if (!IsValid(owner))
		{
			owner = component->GetOwner();
		}

		return owner;
	}

	struct FSourceStencilRestoreState
	{
		bool bRenderCustomDepth = false;
		int32 CustomDepthStencilValue = 0;
		ERendererStencilMask CustomDepthStencilWriteMask = ERendererStencilMask::ERSM_Default;
		int32 ReferenceCount = 0;
	};

	TMap<TWeakObjectPtr<UPrimitiveComponent>, FSourceStencilRestoreState> SourceStencilRestoreStates;

	void PruneInvalidSourceStencilStates()
	{
		for (auto It = SourceStencilRestoreStates.CreateIterator(); It; ++It)
		{
			if (!It.Key().IsValid())
			{
				It.RemoveCurrent();
			}
		}
	}

	void AddSourceStencilReference(UPrimitiveComponent* component)
	{
		if (!IsValid(component))
		{
			return;
		}

		PruneInvalidSourceStencilStates();
		TWeakObjectPtr<UPrimitiveComponent> component_key(component);
		FSourceStencilRestoreState* existing_state = SourceStencilRestoreStates.Find(component_key);
		if (existing_state != nullptr)
		{
			++existing_state->ReferenceCount;
			return;
		}

		FSourceStencilRestoreState restore_state;
		restore_state.bRenderCustomDepth = component->bRenderCustomDepth;
		restore_state.CustomDepthStencilValue = component->CustomDepthStencilValue;
		restore_state.CustomDepthStencilWriteMask = component->CustomDepthStencilWriteMask;
		restore_state.ReferenceCount = 1;
		SourceStencilRestoreStates.Add(component_key, restore_state);
	}

	void ReleaseSourceStencilReference(UPrimitiveComponent* component)
	{
		if (!component)
		{
			return;
		}

		TWeakObjectPtr<UPrimitiveComponent> component_key(component);
		FSourceStencilRestoreState* existing_state = SourceStencilRestoreStates.Find(component_key);
		if (existing_state == nullptr)
		{
			return;
		}

		--existing_state->ReferenceCount;
		if (existing_state->ReferenceCount > 0)
		{
			return;
		}

		if (IsValid(component))
		{
			component->SetCustomDepthStencilValue(existing_state->CustomDepthStencilValue);
			component->SetCustomDepthStencilWriteMask(existing_state->CustomDepthStencilWriteMask);
			component->SetRenderCustomDepth(existing_state->bRenderCustomDepth);
		}
		SourceStencilRestoreStates.Remove(component_key);
	}

	void ApplySourceStencilValue(UPrimitiveComponent* component, uint8 stencil_value)
	{
		if (!IsValid(component))
		{
			return;
		}

		component->SetCustomDepthStencilWriteMask(ERendererStencilMask::ERSM_255);
		component->SetCustomDepthStencilValue(static_cast<int32>(stencil_value));
		component->SetRenderCustomDepth(true);
	}
}


FObjectAnnotator::FObjectAnnotator()
{
	name_ = FString("InstanceSegmentation");
	type_ = AnnotatorType::InstanceSegmentation;
	show_by_default_ = false;
	set_direct_ = false;
	max_view_distance_ = -1.0f;
	use_source_stencil_backend_ = true;
	proxy_component_budget_ = 5000;
	proxy_component_budget_warning_logged_ = false;
}

FObjectAnnotator::FObjectAnnotator(FString name, AnnotatorType type, bool show_by_default, bool set_direct, FString texture_path, FString texture_prefix, float max_view_distance, FString render_backend, int32 proxy_component_budget)
{
	name_ = name;
	type_ = type;
	show_by_default_ = show_by_default;
	set_direct_ = set_direct;
	texture_path_ = texture_path;
	texture_prefix_ = texture_prefix;
	max_view_distance_ = max_view_distance;
	proxy_component_budget_ = proxy_component_budget;
	proxy_component_budget_warning_logged_ = false;

	const bool always_source_stencil = type_ == AnnotatorType::InstanceSegmentation || type_ == AnnotatorType::Infrared;
	const bool requested_source_stencil = IsSourceStencilBackendName(render_backend);
	const bool requested_proxy = IsProxyBackendName(render_backend);
	const bool supports_source_stencil = always_source_stencil || (type_ == AnnotatorType::RGB && !set_direct_);
	use_source_stencil_backend_ = always_source_stencil || (requested_source_stencil && supports_source_stencil);

	if (requested_source_stencil && !supports_source_stencil)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation [%s]: SourceStencil backend is only supported for RGB index, InstanceSegmentation, and Infrared annotation layers. Falling back to proxy annotation."), *name_);
	}

	if (requested_proxy && always_source_stencil)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation [%s]: Proxy backend is disabled for built-in indexed segmentation/infrared. Using SourceStencil."), *name_);
	}
}

void FObjectAnnotator::Initialize(ULevel* level) {
	switch (type_)
	{
	case AnnotatorType::RGB:
		InitializeRGB(level);
		break;
	case AnnotatorType::Greyscale:
		InitializeGreyscale(level);
		break;
	case AnnotatorType::Texture:
		InitializeTexture(level);
		break;
	case AnnotatorType::InstanceSegmentation:
		InitializeInstanceSegmentation(level);
		break;
	case AnnotatorType::Infrared:
		InitializeInfrared(level);
		break;
	}
}

bool FObjectAnnotator::IsPaintable(AActor* actor)
{
	if (!IsValid(actor))
	{
		return false;
	}
	TArray<UMeshComponent*> paintable_components;
	actor->GetComponents<UMeshComponent>(paintable_components);
	if (paintable_components.Num() > 0)
	{
		return true;
	}

	if (UsesIndexedAnnotationColors())
	{
		TArray<ULandscapeComponent*> landscape_components;
		actor->GetComponents<ULandscapeComponent>(landscape_components);
		return landscape_components.Num() > 0;
	}

	return false;
}

void FObjectAnnotator::getPaintableComponentMeshes(AActor* actor, TMap<FString, UMeshComponent*>* paintable_components_meshes)
{
	TArray<UMeshComponent*> paintable_components;
	actor->GetComponents<UMeshComponent>(paintable_components);
	int index = 0;
	for (auto component : paintable_components)
	{
		int32 PersistentPrimitiveIndex = component->GetUniqueID();
		if (const UPrimitiveComponent* PrimitiveComp = Cast<UPrimitiveComponent>(component))
		{
			if (const FPrimitiveSceneProxy* SceneProxy = PrimitiveComp->SceneProxy)
			{
				int32 PersistentPrimitiveIndexTemp = SceneProxy->GetPrimitiveSceneInfo()->GetPersistentIndex().Index;
				if (PersistentPrimitiveIndexTemp != -1)
					PersistentPrimitiveIndex = PersistentPrimitiveIndexTemp;
			}
		}
		if (paintable_components.Num() == 1) {
			if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(component)) {
				if (actor->GetParentActor()) {
					if (staticmesh_component->GetStaticMesh() != nullptr) {
						FString component_name = staticmesh_component->GetStaticMesh()->GetName();
						component_name.Append("_");
						component_name.Append(FString::FromInt(0));
						component_name.Append("_");
						if (actor->GetRootComponent()->GetAttachParent()) {
							component_name.Append(actor->GetRootComponent()->GetAttachParent()->GetName());
							component_name.Append("_");
						}
						component_name.Append(actor->GetParentActor()->GetName());					
						component_name.Append("_");
						component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
						paintable_components_meshes->Emplace(component_name, component);
					}
				}
				else {
					if (staticmesh_component->GetStaticMesh() != nullptr) {
						FString component_name = staticmesh_component->GetStaticMesh()->GetName();
						component_name.Append("_");
						component_name.Append(FString::FromInt(0));
						component_name.Append("_");
						if (actor->GetRootComponent()->GetAttachParent()) {
							component_name.Append(actor->GetRootComponent()->GetAttachParent()->GetName());
							component_name.Append("_");
						}
						component_name.Append(actor->GetName());
						component_name.Append("_");
						component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
						paintable_components_meshes->Emplace(component_name, component);
					}
					else {					
						FString component_name = actor->GetName();
						component_name.Append("_");
						component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
						paintable_components_meshes->Emplace(component_name, component);
					}					
				}
			}
			if (USkinnedMeshComponent* SkinnedMeshComponent = Cast<USkinnedMeshComponent>(component)) {
				FString component_name = actor->GetName();
				component_name.Append("_");
				component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
				paintable_components_meshes->Emplace(component_name, component);
			}
		}
		else {
			FString component_name;
			if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(component)) {
				if (staticmesh_component->GetStaticMesh() != nullptr) {
					component_name = staticmesh_component->GetStaticMesh()->GetName();
					component_name.Append("_");
					component_name.Append(FString::FromInt(index));
					component_name.Append("_");
					if (actor->GetParentActor()) {
						if (actor->GetRootComponent()->GetAttachParent()) {
							component_name.Append(actor->GetRootComponent()->GetAttachParent()->GetName());
							component_name.Append("_");
						}
						component_name.Append(actor->GetParentActor()->GetName());
					}
					else {
						component_name.Append(actor->GetName());
					}
					component_name.Append("_");
					component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
				}
			}
			if (USkinnedMeshComponent* skinnedmesh_component = Cast<USkinnedMeshComponent>(component)) {
				component_name = actor->GetName();
				component_name.Append("_");
				component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
			}
			paintable_components_meshes->Emplace(component_name, component);
			index++;
		}
	}
}

void FObjectAnnotator::getPaintableComponentMeshesAndTags(AActor* actor, TMap<FString, UMeshComponent*>* paintable_components_meshes, TMap<FString, TArray<FName>>* paintable_components_tags)
{
	TArray<UMeshComponent*> paintable_components;
	actor->GetComponents<UMeshComponent>(paintable_components);
	int index = 0;
	for (auto component : paintable_components)
	{
		int32 PersistentPrimitiveIndex = component->GetUniqueID();
		if (const UPrimitiveComponent* PrimitiveComp = Cast<UPrimitiveComponent>(component))
		{
			if (const FPrimitiveSceneProxy* SceneProxy = PrimitiveComp->SceneProxy)
			{
				int32 PersistentPrimitiveIndexTemp = SceneProxy->GetPrimitiveSceneInfo()->GetPersistentIndex().Index;
				if (PersistentPrimitiveIndexTemp != -1)
					PersistentPrimitiveIndex = PersistentPrimitiveIndexTemp;
			}
		}
		if (paintable_components.Num() == 1) {
			if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(component)) {
				if (actor->GetParentActor()) {
					if (staticmesh_component->GetStaticMesh() != nullptr) {
						FString component_name = staticmesh_component->GetStaticMesh()->GetName();
						component_name.Append("_");
						component_name.Append(FString::FromInt(0));
						component_name.Append("_");
						if (actor->GetRootComponent()->GetAttachParent()) {
							component_name.Append(actor->GetRootComponent()->GetAttachParent()->GetName());
							component_name.Append("_");
						}
						component_name.Append(actor->GetParentActor()->GetName());
						component_name.Append("_");
						component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
						paintable_components_meshes->Emplace(component_name, component);
						paintable_components_tags->Emplace(component_name, staticmesh_component->ComponentTags);
					}
				}
				else {
					if (staticmesh_component->GetStaticMesh() != nullptr) {
						FString component_name = staticmesh_component->GetStaticMesh()->GetName();
						component_name.Append("_");
						component_name.Append(FString::FromInt(0));
						component_name.Append("_");
						if (actor->GetRootComponent()->GetAttachParent()) {
							component_name.Append(actor->GetRootComponent()->GetAttachParent()->GetName());
							component_name.Append("_");
						}
						component_name.Append(actor->GetName());
						component_name.Append("_");
						component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
						paintable_components_meshes->Emplace(component_name, component);
						if (actor->Tags.Num() > 0)
							paintable_components_tags->Emplace(component_name, actor->Tags);
						else
							paintable_components_tags->Emplace(component_name, staticmesh_component->ComponentTags);
					}
					else {
						FString component_name = actor->GetName();
						component_name.Append("_");
						component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
						paintable_components_meshes->Emplace(component_name, component);
						if (actor->Tags.Num() > 0)
							paintable_components_tags->Emplace(component_name, actor->Tags);
						else
							paintable_components_tags->Emplace(component_name, staticmesh_component->ComponentTags);
					}
			
				}
			}
			if (USkinnedMeshComponent* SkinnedMeshComponent = Cast<USkinnedMeshComponent>(component)) {
				FString component_name = actor->GetName();
				component_name.Append("_");
				component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
				if (actor->Tags.Num() > 0)
					paintable_components_tags->Emplace(component_name, actor->Tags);
				else
					paintable_components_tags->Emplace(component_name, SkinnedMeshComponent->ComponentTags);
				paintable_components_meshes->Emplace(component_name, component);
			}
		}
		else {
			FString component_name;
			if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(component)) {
				if (staticmesh_component->GetStaticMesh() != nullptr) {
					component_name = staticmesh_component->GetStaticMesh()->GetName();
					component_name.Append("_");
					component_name.Append(FString::FromInt(index));
					component_name.Append("_");
					if (actor->GetParentActor()) {
						if (actor->GetRootComponent()->GetAttachParent()) {
							component_name.Append(actor->GetRootComponent()->GetAttachParent()->GetName());
							component_name.Append("_");
						}
						component_name.Append(actor->GetParentActor()->GetName());
					}
					else {
						component_name.Append(actor->GetName());
					}
				}
				component_name.Append("_");
				component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
				paintable_components_tags->Emplace(component_name, staticmesh_component->ComponentTags);
			}
			if (USkinnedMeshComponent* skinnedmesh_component = Cast<USkinnedMeshComponent>(component)) {
				component_name = actor->GetName();
				component_name.Append("_");
				component_name.Append(FString::FromInt(PersistentPrimitiveIndex));
				paintable_components_tags->Emplace(component_name, skinnedmesh_component->ComponentTags);
			}
			paintable_components_meshes->Emplace(component_name, component);
			index++;
		}
	}
}

void FObjectAnnotator::getPaintableLandscapeComponentsAndTags(AActor* actor, TMap<FString, ULandscapeComponent*>* paintable_landscape_components, TMap<FString, TArray<FName>>* paintable_component_tags)
{
	if (!IsValid(actor) || paintable_landscape_components == nullptr || paintable_component_tags == nullptr)
	{
		return;
	}

	TArray<ULandscapeComponent*> landscape_components;
	if (ALandscapeProxy* landscape_proxy = Cast<ALandscapeProxy>(actor))
	{
		for (ULandscapeComponent* landscape_component : landscape_proxy->LandscapeComponents)
		{
			if (IsValid(landscape_component))
			{
				landscape_components.Add(landscape_component);
			}
		}
	}
	else
	{
		actor->GetComponents<ULandscapeComponent>(landscape_components);
	}

	for (ULandscapeComponent* landscape_component : landscape_components)
	{
		if (!IsValid(landscape_component))
		{
			continue;
		}

		const FString component_name = GetLandscapeComponentId(landscape_component);
		if (component_name.IsEmpty())
		{
			continue;
		}

		paintable_landscape_components->Emplace(component_name, landscape_component);
		if (actor->Tags.Num() > 0)
		{
			paintable_component_tags->Emplace(component_name, actor->Tags);
		}
		else
		{
			paintable_component_tags->Emplace(component_name, landscape_component->ComponentTags);
		}
	}
}

bool FObjectAnnotator::SetComponentRGBColorByIndex(FString component_id, uint32 color_index)
{
	if (UsesIndexedAnnotationColors() && color_index > 255)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation [%s]: Ignored ID %u for %s because SourceStencil labels must be in 0..255."), *name_, color_index, *component_id);
		return false;
	}

	TArray<FString> component_ids;
	GetComponentIdsForColorUpdate(component_id, component_ids);
	if (component_ids.Num() > 0)
	{
		FColor color = GetAnnotationColorForIndex(color_index);
		bool updated_any = false;
		for (const FString& current_component_id : component_ids)
		{
			bool updated_component = false;
			if (UMeshComponent* const* component_ptr = name_to_component_map_.Find(current_component_id))
			{
				if (IsValid(*component_ptr))
				{
					updated_component = UpdatePaintRGBComponent(*component_ptr, color, current_component_id);
				}
			}
			else if (ULandscapeComponent* const* landscape_component_ptr = name_to_landscape_component_map_.Find(current_component_id))
			{
				if (IsValid(*landscape_component_ptr))
				{
					updated_component = UpdatePaintLandscapeComponent(*landscape_component_ptr, color, current_component_id);
				}
			}

			if (!updated_component)
			{
				continue;
			}

			UpdateColorMappings(current_component_id, color_index);
			updated_any = true;
			UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Adjusted indexed annotation of object %s to new ID # %s (Display: %s)"), *name_, *current_component_id, *FString::FromInt(color_index), *name_to_gammacorrected_color_map_[current_component_id]);
		}
		return updated_any;
	}
	else
	{
		return false;
	}
}

bool FObjectAnnotator::SetComponentRGBColorByColor(FString component_id, FColor color)
{
	TArray<FString> component_ids;
	GetComponentIdsForColorUpdate(component_id, component_ids);
	if (component_ids.Num() > 0)
	{
		int32 color_index = ColorGenerator_.GetIndexForColor(color);
		bool updated_any = false;
		for (const FString& current_component_id : component_ids)
		{
			bool updated_component = false;
			if (UMeshComponent* const* component_ptr = name_to_component_map_.Find(current_component_id))
			{
				if (IsValid(*component_ptr))
				{
					updated_component = UpdatePaintRGBComponent(*component_ptr, color, current_component_id);
				}
			}
			else if (ULandscapeComponent* const* landscape_component_ptr = name_to_landscape_component_map_.Find(current_component_id))
			{
				if (IsValid(*landscape_component_ptr))
				{
					updated_component = UpdatePaintLandscapeComponent(*landscape_component_ptr, color, current_component_id);
				}
			}

			if (!updated_component)
			{
				continue;
			}

			UpdateColorMappings(current_component_id, color_index);
			updated_any = true;
			UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Adjusted RGB annotation of object %s to new RGB color: %s (ID # %s)"), *name_, *current_component_id, *name_to_gammacorrected_color_map_[current_component_id], *FString::FromInt(color_index));
		}
		return updated_any;
	}
	else
	{
		return false;
	}
}

bool FObjectAnnotator::SetComponentGreyScaleColorByValue(FString component_id, float greyscale_value)
{
	if (name_to_component_map_.Contains(component_id))
	{
		if (greyscale_value >= 1) {
			greyscale_value = 1;
		}
		else if (greyscale_value <= 0) {
			greyscale_value = 0;
		}
		FLinearColor new_color_linear = FLinearColor(greyscale_value, greyscale_value, greyscale_value);
		FColor color = new_color_linear.ToFColor(true);
		UMeshComponent* component = name_to_component_map_[component_id];
		if (UpdatePaintRGBComponent(component, color, component_id))
		{
			FString color_string = FString::FromInt(color.R) + "," + FString::FromInt(color.G) + "," + FString::FromInt(color.B);
			FString color_string_gammacorrected = color_string;
			const FString* found_index_color = color_to_name_map_.FindKey(component_id);

			if (found_index_color != nullptr) {
				color_to_name_map_.Remove(*found_index_color);
			}
			color_to_name_map_.Emplace(color_string, component_id);
			const FString* found_index_color_gamma = gammacorrected_color_to_name_map_.FindKey(component_id);
			if (found_index_color != nullptr) {
				gammacorrected_color_to_name_map_.Remove(*found_index_color_gamma);
			}
			color_to_name_map_.Emplace(color_string, component_id);
			gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, component_id);
			name_to_gammacorrected_color_map_[component_id] = color_string_gammacorrected;
			name_to_value_map_[component_id] = greyscale_value;
			UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Adjusted greyscale annotation of object %s to new value %f (RGB: %s)"), *name_, *component_id, greyscale_value, *color_string_gammacorrected);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool FObjectAnnotator::SetComponentTextureByDirectPath(FString component_id, FString path)
{
	if (name_to_component_map_.Contains(component_id))
	{

		FString new_texture;

		if (set_direct_) {
			new_texture = path;
		}
		else {
			return false;
		}
		UMeshComponent* component = name_to_component_map_[component_id];
		if (UpdatePaintTextureComponent(component, path, component_id))
		{
			name_to_texture_path_map_[component_id] = new_texture;
			UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Adjusted texture annotation of object %s to new direct texture %s"), *name_, *component_id, *path);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool FObjectAnnotator::SetComponentTextureByRelativePath(FString component_id)
{
	if (name_to_component_map_.Contains(component_id))
	{
		FString new_texture;

		if (set_direct_) {
			return false;
		}
		else {
			FString component_name;
			if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(name_to_component_map_[component_id])) {
				if (staticmesh_component->GetStaticMesh() != nullptr) {
					component_name = staticmesh_component->GetStaticMesh()->GetName();
				}
			}
			else if (USkinnedMeshComponent* skinnedmesh_component = Cast<USkinnedMeshComponent>(name_to_component_map_[component_id])) {
				if (skinnedmesh_component->GetSkinnedAsset() != nullptr) {
					component_name = skinnedmesh_component->GetSkinnedAsset()->GetName();
				}
			}
			new_texture = texture_path_ + "/" + texture_prefix_ + "-" + component_name;
		}

		UMeshComponent* component = name_to_component_map_[component_id];
		if (UpdatePaintTextureComponent(component, new_texture, component_id))
		{
			name_to_texture_path_map_[component_id] = new_texture;
			UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Adjusted texture annotation of object %s to new relative texture %s"), *name_, *component_id, *new_texture);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool FObjectAnnotator::AnnotateNewActor(AActor* actor)
{
	switch (type_)
	{
	case AnnotatorType::RGB:
		return AnnotateNewActorRGB(actor);		
	case AnnotatorType::Greyscale:
		return AnnotateNewActorGreyscale(actor);
	case AnnotatorType::Texture:
		return AnnotateNewActorTexture(actor);
	case AnnotatorType::InstanceSegmentation:
		return AnnotateNewActorInstanceSegmentation(actor);
	case AnnotatorType::Infrared:
		return AnnotateNewActorInstanceSegmentation(actor);
	}
	return false;
}

bool FObjectAnnotator::AnnotateNewActorInstanceSegmentation(AActor* actor) {
	if (actor && IsPaintable(actor)) {
		bool annotated_any = false;
		TMap<FString, UMeshComponent*> paintable_components_meshes;
		TMap<FString, TArray<FName>> paintable_components_tags;
		getPaintableComponentMeshesAndTags(actor, &paintable_components_meshes, &paintable_components_tags);
		for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
		{
			if (name_to_component_map_.Contains(it.Key())) {
				name_to_component_map_[it.Key()] = it.Value();
				component_to_name_map_.FindOrAdd(it.Value()) = it.Key();
				const uint32 color_index = GetOrCreateLabelColorIndex(it.Key(), it.Value());
				FColor Color = GetAnnotationColorForIndex(color_index);
				UpdateColorMappings(it.Key(), color_index);
				check(UpdatePaintRGBComponent(it.Value(), Color, it.Key()));
				annotated_any = true;
			}
			else {
				FName* found_tag = paintable_components_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
					FString tag = tagFName.ToString();
					return tag.Contains("InstanceSegmentation_disable");
					});
				if (found_tag == nullptr) {
					name_to_component_map_.FindOrAdd(it.Key()) = it.Value();
					component_to_name_map_.FindOrAdd(it.Value()) = it.Key();
					uint32 ObjectIndex = GetOrCreateLabelColorIndex(it.Key(), it.Value());
					FColor new_color = GetAnnotationColorForIndex(ObjectIndex);
					UpdateColorMappings(it.Key(), ObjectIndex);
					check(PaintRGBComponent(it.Value(), new_color, it.Key()));
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new object %s with ID # %s (Display: %s)"), *name_, *it.Key(), *FString::FromInt(ObjectIndex), *name_to_gammacorrected_color_map_[it.Key()]);
					annotated_any = true;
				}else{
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Ignored new object %s"), *name_, *it.Key());
				}
			}
		}

		TMap<FString, ULandscapeComponent*> paintable_landscape_components;
		TMap<FString, TArray<FName>> paintable_landscape_tags;
		getPaintableLandscapeComponentsAndTags(actor, &paintable_landscape_components, &paintable_landscape_tags);
		for (auto it = paintable_landscape_components.CreateConstIterator(); it; ++it)
		{
			if (name_to_landscape_component_map_.Contains(it.Key())) {
				name_to_landscape_component_map_[it.Key()] = it.Value();
				landscape_component_to_name_map_.FindOrAdd(it.Value()) = it.Key();
				const uint32 color_index = GetOrCreateLandscapeLabelColorIndex(it.Key(), it.Value());
				FColor Color = GetAnnotationColorForIndex(color_index);
				UpdateColorMappings(it.Key(), color_index);
				check(UpdatePaintLandscapeComponent(it.Value(), Color, it.Key()));
				annotated_any = true;
			}
			else {
				FName* found_tag = paintable_landscape_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
					FString tag = tagFName.ToString();
					return tag.Contains("InstanceSegmentation_disable");
					});
				if (found_tag == nullptr) {
					name_to_landscape_component_map_.FindOrAdd(it.Key()) = it.Value();
					landscape_component_to_name_map_.FindOrAdd(it.Value()) = it.Key();
					uint32 ObjectIndex = GetOrCreateLandscapeLabelColorIndex(it.Key(), it.Value());
					FColor new_color = GetAnnotationColorForIndex(ObjectIndex);
					UpdateColorMappings(it.Key(), ObjectIndex);
					check(PaintLandscapeComponent(it.Value(), new_color, it.Key()));
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new landscape object %s with ID # %s (Display: %s)"), *name_, *it.Key(), *FString::FromInt(ObjectIndex), *name_to_gammacorrected_color_map_[it.Key()]);
					annotated_any = true;
				}
				else {
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Ignored new landscape object %s"), *name_, *it.Key());
				}
			}
		}
		return annotated_any;
	}
	else {
		return false;
	}
}

bool FObjectAnnotator::AnnotateNewActorRGB(AActor* actor) {
	if (actor && IsPaintable(actor)) {
		TMap<FString, UMeshComponent*> paintable_components_meshes;
		TMap<FString, TArray<FName>> paintable_components_tags;
		getPaintableComponentMeshesAndTags(actor, &paintable_components_meshes, &paintable_components_tags);
		for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
		{
			FName* found_tag = paintable_components_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
				FString tag = tagFName.ToString();
				return tag.Contains(name_);
				});

			if (found_tag != nullptr) {
				FString tag = found_tag->ToString();
				TArray<FString> splitTag;
				tag.ParseIntoArray(splitTag, TEXT("_"), true);

				FColor new_color;
				int32 color_index;
				if (set_direct_) {
					new_color = FColor(FCString::Atoi(*splitTag[1]), FCString::Atoi(*splitTag[2]), FCString::Atoi(*splitTag[3]));
					color_index = ColorGenerator_.GetIndexForColor(new_color);
				}
				else {
					color_index = FCString::Atoi(*splitTag[1]);
					new_color = ColorGenerator_.GetColorFromColorMap(color_index);
				}
				FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
				FString color_string_gammacorrected = FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.R)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.G)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.B));


				if (name_to_component_map_.Contains(it.Key())) {

					name_to_color_index_map_[it.Key()] = color_index;
					const FString* found_index_color = color_to_name_map_.FindKey(it.Key());
					if (found_index_color != nullptr) {
						color_to_name_map_.Remove(*found_index_color);
					}
					color_to_name_map_.Emplace(color_string, it.Key());
					const FString* found_index_color_gamma = gammacorrected_color_to_name_map_.FindKey(it.Key());
					if (found_index_color != nullptr) {
						gammacorrected_color_to_name_map_.Remove(*found_index_color_gamma);
					}
					gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
					name_to_gammacorrected_color_map_.Emplace(it.Key(), color_string_gammacorrected);
					check(UpdatePaintRGBComponent(it.Value(), new_color, it.Key()));
					if (set_direct_) {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Updated RGB annotated object %s with direct RGB color: %s (ID # %s)"), *name_, *it.Key(), *color_string_gammacorrected, *FString::FromInt(color_index));

					}
					else {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Updated RGB annotated object %s with ID # %s (RGB: %s)"), *name_, *it.Key(), *FString::FromInt(color_index), *color_string_gammacorrected);
					}
				}
				else {
					name_to_component_map_.Emplace(it.Key(), it.Value());
					component_to_name_map_.Emplace(it.Value(), it.Key());
					name_to_color_index_map_.Emplace(it.Key(), color_index);
					color_to_name_map_.Emplace(color_string, it.Key());
					gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
					name_to_gammacorrected_color_map_.Emplace(it.Key(), color_string_gammacorrected);
					check(PaintRGBComponent(it.Value(), new_color, it.Key()));
					if (set_direct_) {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new RGB annotated object %s with direct RGB color: %s (ID # %s)"), *name_, *it.Key(), *color_string_gammacorrected, *FString::FromInt(color_index));

					}
					else {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new RGB annotated object %s with ID # %s (RGB: %s)"), *name_, *it.Key(), *FString::FromInt(color_index), *color_string_gammacorrected);
					}
				}
			}else if (show_by_default_ && !it.Key().Contains("hidden_sphere") && !it.Key().Contains("AnnotationSphere")) {
				name_to_component_map_.Emplace(it.Key(), it.Value());
				component_to_name_map_.Emplace(it.Value(), it.Key());
				FColor new_color = FColor(0, 0, 0);
				name_to_color_index_map_.Emplace(it.Key(), 2744000 - 1);
				FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
				FString color_string_gammacorrected = FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.R)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.G)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.B));
				color_to_name_map_.Emplace(color_string, it.Key());
				gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
				name_to_gammacorrected_color_map_.Emplace(it.Key(), color_string_gammacorrected);
				check(PaintRGBComponent(it.Value(), new_color, it.Key()));
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added untagged RGB annotated object %s with default color (RGB: %s)"), *name_, *it.Key(), *color_string_gammacorrected);
			}
		}
		return true;
	}
	else {
		return false;
	}
}

bool FObjectAnnotator::AnnotateNewActorGreyscale(AActor* actor) {
	if (actor && IsPaintable(actor)) {
		TMap<FString, UMeshComponent*> paintable_components_meshes;
		TMap<FString, TArray<FName>> paintable_components_tags;
		getPaintableComponentMeshesAndTags(actor, &paintable_components_meshes, &paintable_components_tags);
		for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
		{
			FName* found_tag = paintable_components_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
				FString tag = tagFName.ToString();
				return tag.Contains(name_);
				});

			if (found_tag != nullptr) {
				FString tag = found_tag->ToString();
				TArray<FString> splitTag;
				tag.ParseIntoArray(splitTag, TEXT("_"), true);

				float greyscale_value = FCString::Atof(*splitTag[1]);
				if (greyscale_value >= 1) {
					greyscale_value = 1;
				}
				else if (greyscale_value <= 0) {
					greyscale_value = 0;
				}
				FLinearColor new_color_linear = FLinearColor(greyscale_value, greyscale_value, greyscale_value);
				FColor new_color = new_color_linear.ToFColor(true);
				FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
				FString color_string_gammacorrected = color_string;


				if (name_to_component_map_.Contains(it.Key())) {
					const FString* found_index_color = color_to_name_map_.FindKey(it.Key());
					if (found_index_color != nullptr) {
						color_to_name_map_.Remove(*found_index_color);
					}
					color_to_name_map_.Emplace(color_string, it.Key());
					const FString* found_index_color_gamma = gammacorrected_color_to_name_map_.FindKey(it.Key());
					if (found_index_color != nullptr) {
						gammacorrected_color_to_name_map_.Remove(*found_index_color_gamma);
					}
					name_to_value_map_[it.Key()] = greyscale_value;
					gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
					name_to_gammacorrected_color_map_[it.Key()] = color_string_gammacorrected;
					check(UpdatePaintRGBComponent(it.Value(), new_color, it.Key()));

					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Updated greyscale annotated object %s with value %f (RGB: %s)"), *name_, *it.Key(), greyscale_value, *color_string_gammacorrected);
			}
				else {
					name_to_component_map_.Emplace(it.Key(), it.Value());
					component_to_name_map_.Emplace(it.Value(), it.Key());
					color_to_name_map_.Emplace(color_string, it.Key());
					gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
					name_to_gammacorrected_color_map_.Emplace(it.Key(), color_string_gammacorrected);
					name_to_value_map_.Emplace(it.Key(), greyscale_value);
					check(PaintRGBComponent(it.Value(), new_color, it.Key()));
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new greyscale annotated object %s with value %f (RGB: %s)"), *name_, *it.Key(), greyscale_value, *color_string_gammacorrected);
				}
			}else if (show_by_default_ && !it.Key().Contains("hidden_sphere") && !it.Key().Contains("AnnotationSphere")) {
				name_to_component_map_.Emplace(it.Key(), it.Value());
				component_to_name_map_.Emplace(it.Value(), it.Key());
				FColor new_color = FColor(0, 0, 0);
				FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
				FString color_string_gammacorrected = color_string;
				color_to_name_map_.Emplace(color_string, it.Key());
				gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
				name_to_gammacorrected_color_map_.Emplace(it.Key(), color_string_gammacorrected);
				name_to_value_map_.Emplace(it.Key(), 0);
				check(PaintRGBComponent(it.Value(), new_color, it.Key()));
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added untagged greyscale annotated object %s with default color (RGB: %s)"), *name_, *it.Key(), *color_string_gammacorrected);
			}
		}
		return true;
	}
	else {
		return false;
	}
}

bool FObjectAnnotator::AnnotateNewActorTexture(AActor* actor) {
	if (actor && IsPaintable(actor)) {
		TMap<FString, UMeshComponent*> paintable_components_meshes;
		TMap<FString, TArray<FName>> paintable_components_tags;
		getPaintableComponentMeshesAndTags(actor, &paintable_components_meshes, &paintable_components_tags);
		for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
		{
			FName* found_tag = paintable_components_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
				FString tag = tagFName.ToString();
				return tag.Contains(name_);
				});

			if (found_tag != nullptr) {
				FString tag = found_tag->ToString();
				TArray<FString> splitTag;
				tag.ParseIntoArray(splitTag, TEXT("_"), true);
				
				FString new_texture;

				if (set_direct_) {
					new_texture = splitTag[1];
				} else {
					FString component_name;
					if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(it.Value())) {
						if (staticmesh_component->GetStaticMesh() != nullptr) {
							component_name = staticmesh_component->GetStaticMesh()->GetName();
						}
					} else if (USkinnedMeshComponent* skinnedmesh_component = Cast<USkinnedMeshComponent>(it.Value())) {
						if (skinnedmesh_component->GetSkinnedAsset() != nullptr) {
							component_name = skinnedmesh_component->GetSkinnedAsset()->GetName();
						}
					}
					new_texture = texture_path_ + "/" + texture_prefix_ + "-" + component_name;
				}

				if (name_to_component_map_.Contains(it.Key())) {
					name_to_texture_path_map_[it.Key()] = new_texture;
					check(UpdatePaintTextureComponent(it.Value(), new_texture, it.Key()));
					if (set_direct_) {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Updated texture annotated object %s with texture: %s"), *name_, *it.Key(), *new_texture);

					}
					else {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Updated texture annotated object %s with texture: %s"), *name_, *it.Key(), *new_texture);
					}
				}
				else {
					name_to_component_map_.Emplace(it.Key(), it.Value());
					component_to_name_map_.Emplace(it.Value(), it.Key());
					name_to_texture_path_map_.Emplace(it.Key(), new_texture);
					check(PaintTextureComponent(it.Value(), new_texture, it.Key()));
					if (set_direct_) {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new texture annotated object %s with texture: %s"), *name_, *it.Key(), *new_texture);

					}
					else {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new texture annotated object %s with texture: %s"), *name_, *it.Key(), *new_texture);
					}
				}
			}else if (show_by_default_ && !it.Key().Contains("hidden_sphere") && !it.Key().Contains("AnnotationSphere")) {
				name_to_component_map_.Emplace(it.Key(), it.Value());
				component_to_name_map_.Emplace(it.Value(), it.Key());
				FString new_texture = "/AirSim/HUDAssets/k";
				name_to_texture_path_map_.Emplace(it.Key(), new_texture);
				check(PaintTextureComponent(it.Value(), new_texture, it.Key()));
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added untagged texture annotated object %s with default texture"), *name_, *it.Key());
			}
		}
		return true;
	}
	else {
		return false;
	}
}

bool FObjectAnnotator::DeleteActor(AActor* actor)
{
	if (actor && IsPaintable(actor)) {
		bool deleted_any = false;
		TMap<FString, UMeshComponent*> paintable_components_meshes;
		getPaintableComponentMeshes(actor, &paintable_components_meshes);
		for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
		{
			if (name_to_component_map_.Contains(it.Key())) {
				check(DeleteComponent(it.Value(), it.Key()));
				RemoveTrackedComponent(it.Key());
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Deleted object %s."), *name_, *it.Key());
				deleted_any = true;

			}
			else {
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: could not delete object %s."), *name_, *it.Key());
				return false;
			}
		}

		if (UsesIndexedAnnotationColors())
		{
			TMap<FString, ULandscapeComponent*> paintable_landscape_components;
			TMap<FString, TArray<FName>> paintable_landscape_tags;
			getPaintableLandscapeComponentsAndTags(actor, &paintable_landscape_components, &paintable_landscape_tags);
			for (auto it = paintable_landscape_components.CreateConstIterator(); it; ++it)
			{
				if (name_to_landscape_component_map_.Contains(it.Key())) {
					check(DeleteLandscapeComponent(it.Value(), it.Key()));
					RemoveTrackedComponent(it.Key());
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Deleted landscape object %s."), *name_, *it.Key());
					deleted_any = true;
				}
				else {
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: could not delete landscape object %s."), *name_, *it.Key());
					return false;
				}
			}
		}
		return deleted_any;
	}
	else {
		return false;
	}
}

uint32 FObjectAnnotator::GetComponentIndex(FString component_id)
{
	if (name_to_color_index_map_.Num() == 0)
	{
		return -1;
	}
	if (name_to_color_index_map_.Contains(component_id))
	{
		return name_to_color_index_map_[component_id];
	}
	if (label_to_color_index_map_.Contains(component_id))
	{
		return label_to_color_index_map_[component_id];
	}
	else
	{
		return -1;
	}
}

FString FObjectAnnotator::GetComponentRGBColor(FString component_id)
{
	if (name_to_gammacorrected_color_map_.Num() == 0)
	{
		return FString(TEXT(""));
	}
	if (name_to_gammacorrected_color_map_.Contains(component_id))
	{
		return name_to_gammacorrected_color_map_[component_id];
	}
	else
	{
		return FString(TEXT(""));
	}
}

float FObjectAnnotator::GetComponentGreyscaleValue(FString component_id)
{
	if (name_to_value_map_.Num() == 0)
	{
		return 0.;
	}
	if (name_to_value_map_.Contains(component_id))
	{
		return name_to_value_map_[component_id];
	}
	else
	{
		return 0.;
	}
}

FString FObjectAnnotator::GetComponentTexturePath(FString component_id)
{
	if (name_to_texture_path_map_.Num() == 0)
	{
		return FString(TEXT(""));
	}
	if (name_to_texture_path_map_.Contains(component_id))
	{
		return name_to_texture_path_map_[component_id];
	}
	else
	{
		return FString(TEXT(""));
	}
}

bool FObjectAnnotator::UsesIndexedAnnotationColors() const
{
	return type_ == AnnotatorType::InstanceSegmentation ||
		type_ == AnnotatorType::Infrared ||
		use_source_stencil_backend_;
}

bool FObjectAnnotator::CanCreateProxyAnnotationComponent(const FString& component_name)
{
	if (UsesIndexedAnnotationColors() || proxy_component_budget_ < 0)
	{
		return true;
	}

	if (name_to_annotation_component_map_.Num() < proxy_component_budget_)
	{
		return true;
	}

	if (!proxy_component_budget_warning_logged_)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation [%s]: Proxy annotation budget reached (%d components). Skipping additional proxy components; first skipped component: %s. Increase ProxyComponentBudget or use Backend=SourceStencil where supported."), *name_, proxy_component_budget_, *component_name);
		proxy_component_budget_warning_logged_ = true;
	}

	return false;
}

FColor FObjectAnnotator::GetAnnotationColorForIndex(uint32 color_index) const
{
	if (type_ == AnnotatorType::Infrared)
	{
		const uint8 grayscale_value = static_cast<uint8>(color_index % 256);
		return FColor(grayscale_value, grayscale_value, grayscale_value, 255);
	}

	return ColorGenerator_.GetColorFromColorMap(color_index);
}

FString FObjectAnnotator::GetDisplayColorString(const FColor& color) const
{
	if (type_ == AnnotatorType::Infrared)
	{
		return FString::FromInt(color.R) + "," + FString::FromInt(color.G) + "," + FString::FromInt(color.B);
	}

	return FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(color.R)) + "," +
		FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(color.G)) + "," +
		FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(color.B));
}

void FObjectAnnotator::InitializeIndexedAnnotation(ULevel* InLevel, const TCHAR* annotation_mode)
{
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Starting full level %s annotation."), *name_, annotation_mode);
	for (AActor* actor : InLevel->Actors)
	{
		if (actor && IsPaintable(actor))
		{
			TMap<FString, UMeshComponent*> paintable_components_meshes;
			TMap<FString, TArray<FName>> paintable_components_tags;
			getPaintableComponentMeshesAndTags(actor, &paintable_components_meshes, &paintable_components_tags);
			for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
			{
				FName* found_tag = paintable_components_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
					FString tag = tagFName.ToString();
					return tag.Contains("InstanceSegmentation_disable");
					});
				if(!it.Key().Contains("hidden_sphere") && !it.Key().Contains("AnnotationSphere") && found_tag == nullptr) {
					name_to_component_map_.FindOrAdd(it.Key()) = it.Value();
					component_to_name_map_.FindOrAdd(it.Value()) = it.Key();
					const uint32 color_index = GetOrCreateLabelColorIndex(it.Key(), it.Value());
					FColor new_color = GetAnnotationColorForIndex(color_index);
					UpdateColorMappings(it.Key(), color_index);
					check(PaintRGBComponent(it.Value(), new_color, it.Key()));
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new object %s with ID # %s (Display: %s)"), *name_, *it.Key(), *FString::FromInt(color_index), *name_to_gammacorrected_color_map_[it.Key()]);
				}				
			}

			TMap<FString, ULandscapeComponent*> paintable_landscape_components;
			TMap<FString, TArray<FName>> paintable_landscape_tags;
			getPaintableLandscapeComponentsAndTags(actor, &paintable_landscape_components, &paintable_landscape_tags);
			for (auto it = paintable_landscape_components.CreateConstIterator(); it; ++it)
			{
				FName* found_tag = paintable_landscape_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
					FString tag = tagFName.ToString();
					return tag.Contains("InstanceSegmentation_disable");
					});
				if(!it.Key().Contains("hidden_sphere") && !it.Key().Contains("AnnotationSphere") && found_tag == nullptr) {
					name_to_landscape_component_map_.FindOrAdd(it.Key()) = it.Value();
					landscape_component_to_name_map_.FindOrAdd(it.Value()) = it.Key();
					const uint32 color_index = GetOrCreateLandscapeLabelColorIndex(it.Key(), it.Value());
					FColor new_color = GetAnnotationColorForIndex(color_index);
					UpdateColorMappings(it.Key(), color_index);
					check(PaintLandscapeComponent(it.Value(), new_color, it.Key()));
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new landscape object %s with ID # %s (Display: %s)"), *name_, *it.Key(), *FString::FromInt(color_index), *name_to_gammacorrected_color_map_[it.Key()]);
				}
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Completed full level %s annotation."), *name_, annotation_mode);
}

void FObjectAnnotator::InitializeInstanceSegmentation(ULevel* InLevel)
{
	InitializeIndexedAnnotation(InLevel, TEXT("instance segmentation"));
}

void FObjectAnnotator::InitializeInfrared(ULevel* InLevel)
{
	InitializeIndexedAnnotation(InLevel, TEXT("infrared"));
}

void FObjectAnnotator::InitializeRGB(ULevel* InLevel)
{
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Starting full level RGB annotation by searching for tags."), *name_);
	for (AActor* actor : InLevel->Actors)
	{
		if (actor && IsPaintable(actor))
		{

			TMap<FString, UMeshComponent*> paintable_components_meshes;
			TMap<FString, TArray<FName>> paintable_components_tags;
			getPaintableComponentMeshesAndTags(actor, &paintable_components_meshes, &paintable_components_tags);
			for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
			{
				FName* found_tag = paintable_components_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
					FString tag = tagFName.ToString();
					return tag.Contains(name_);
					});

				if (found_tag != nullptr) {
					FString tag = found_tag->ToString();
					TArray<FString> splitTag;
					tag.ParseIntoArray(splitTag, TEXT("_"), true);
					name_to_component_map_.Emplace(it.Key(), it.Value());
					component_to_name_map_.Emplace(it.Value(), it.Key());

					FColor new_color;
					uint32 color_index;
					if (set_direct_) {
						new_color = FColor(FCString::Atoi(*splitTag[1]), FCString::Atoi(*splitTag[2]), FCString::Atoi(*splitTag[3]));
						color_index = ColorGenerator_.GetIndexForColor(new_color);
					}
					else {
						color_index = FCString::Atoi(*splitTag[1]);
						new_color = ColorGenerator_.GetColorFromColorMap(color_index);
					}
					name_to_color_index_map_.Emplace(it.Key(), color_index);
					FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
					FString color_string_gammacorrected = FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.R)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.G)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.B));
					color_to_name_map_.Emplace(color_string, it.Key());
					gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
					name_to_gammacorrected_color_map_.Emplace(it.Key(), color_string_gammacorrected);
					check(PaintRGBComponent(it.Value(), new_color, it.Key()));
					if (set_direct_) {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new RGB annotated object %s with direct RGB color: %s (ID # %s)"), *name_, *it.Key(), *color_string_gammacorrected, *FString::FromInt(color_index));

					}
					else {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new RGB annotated object %s with ID # %s (RGB: %s)"), *name_, *it.Key(), *FString::FromInt(color_index), *color_string_gammacorrected);
					}
				}
				else if (show_by_default_ && !it.Key().Contains("hidden_sphere") && !it.Key().Contains("AnnotationSphere")) {
					name_to_component_map_.Emplace(it.Key(), it.Value());
					component_to_name_map_.Emplace(it.Value(), it.Key());
					FColor new_color = FColor(0, 0, 0);
					name_to_color_index_map_.Emplace(it.Key(), 2744000 - 1);
					FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
					FString color_string_gammacorrected = FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.R)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.G)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.B));
					color_to_name_map_.Emplace(color_string, it.Key());
					gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
					name_to_gammacorrected_color_map_.Emplace(it.Key(), color_string_gammacorrected);
					check(PaintRGBComponent(it.Value(), new_color, it.Key()));
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added untagged RGB annotated object %s with default color (RGB: %s)"), *name_, *it.Key(), *color_string_gammacorrected);
				}

			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Completed full level RGB annotation."), *name_);
}

void FObjectAnnotator::InitializeGreyscale(ULevel* InLevel)
{
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Starting full level greyscale annotation by searching for tags."), *name_);
	for (AActor* actor : InLevel->Actors)
	{
		if (actor && IsPaintable(actor))
		{

			TMap<FString, UMeshComponent*> paintable_components_meshes;
			TMap<FString, TArray<FName>> paintable_components_tags;
			getPaintableComponentMeshesAndTags(actor, &paintable_components_meshes, &paintable_components_tags);
			for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
			{
				FName* found_tag = paintable_components_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
					FString tag = tagFName.ToString();
					return tag.Contains(name_);
					});

				if (found_tag != nullptr) {
					FString tag = found_tag->ToString();
					TArray<FString> splitTag;
					tag.ParseIntoArray(splitTag, TEXT("_"), true);
					name_to_component_map_.Emplace(it.Key(), it.Value());
					component_to_name_map_.Emplace(it.Value(), it.Key());

					float greyscale_value = FCString::Atof(*splitTag[1]);
					if (greyscale_value >= 1) {
						greyscale_value = 1;
					}
					else if (greyscale_value <= 0) {
						greyscale_value = 0;
					}
					FLinearColor new_color_linear = FLinearColor(greyscale_value, greyscale_value, greyscale_value);
					FColor new_color = new_color_linear.ToFColor(true);

					FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
					FString color_string_gammacorrected = color_string;
					color_to_name_map_.Emplace(color_string, it.Key());
					gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
					name_to_gammacorrected_color_map_.Emplace(it.Key(), color_string_gammacorrected);
					name_to_value_map_.Emplace(it.Key(), greyscale_value);
					check(PaintRGBComponent(it.Value(), new_color, it.Key()));
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new greyscale annotated object %s with direct greyscale value %f (RGB: %s)"), *name_, *it.Key(), greyscale_value , *color_string_gammacorrected);
				}
				else if (show_by_default_ && !it.Key().Contains("hidden_sphere") && !it.Key().Contains("AnnotationSphere")) {
					name_to_component_map_.Emplace(it.Key(), it.Value());
					component_to_name_map_.Emplace(it.Value(), it.Key());
					FColor new_color = FColor(0, 0, 0);
					FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
					FString color_string_gammacorrected = color_string;
					color_to_name_map_.Emplace(color_string, it.Key());
					gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, it.Key());
					name_to_gammacorrected_color_map_.Emplace(it.Key(), color_string_gammacorrected);
					name_to_value_map_.Emplace(it.Key(), 0);
					check(PaintRGBComponent(it.Value(), new_color, it.Key()));
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added untagged greyscale annotated object %s with default color (RGB: %s)"), *name_, *it.Key(), *color_string_gammacorrected);
				}
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Completed full level greyscale annotation."), *name_);
}

void FObjectAnnotator::InitializeTexture(ULevel* InLevel)
{
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Starting full level texture annotation by searching for tags."), *name_);
	for (AActor* actor : InLevel->Actors)
	{
		if (actor && IsPaintable(actor))
		{

			TMap<FString, UMeshComponent*> paintable_components_meshes;
			TMap<FString, TArray<FName>> paintable_components_tags;
			getPaintableComponentMeshesAndTags(actor, &paintable_components_meshes, &paintable_components_tags);
			for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
			{
				FName* found_tag = paintable_components_tags[it.Key()].FindByPredicate([this](const FName& tagFName) {
					FString tag = tagFName.ToString();
					return tag.Contains(name_);
					});

				if (found_tag != nullptr) {
					FString tag = found_tag->ToString();
					TArray<FString> splitTag;
					tag.ParseIntoArray(splitTag, TEXT("_"), true);
					name_to_component_map_.Emplace(it.Key(), it.Value());
					component_to_name_map_.Emplace(it.Value(), it.Key());

					FString new_texture;

					if (set_direct_) {
						new_texture = splitTag[1];
					}
					else {
						FString component_name;
						if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(it.Value())) {
							if (staticmesh_component->GetStaticMesh() != nullptr) {
								component_name = staticmesh_component->GetStaticMesh()->GetName();
							}
						}
						else if (USkinnedMeshComponent* skinnedmesh_component = Cast<USkinnedMeshComponent>(it.Value())) {
							if (skinnedmesh_component->GetSkinnedAsset() != nullptr) {
								component_name = skinnedmesh_component->GetSkinnedAsset()->GetName();
							}
						}
						new_texture = texture_path_ + "/" + texture_prefix_ + "-" + component_name;
					}
					name_to_texture_path_map_.Emplace(it.Key(), new_texture);
					check(PaintTextureComponent(it.Value(), new_texture, it.Key()));
					if (set_direct_) {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new texture annotated object %s with texture: %s"), *name_, *it.Key(), *new_texture);
					}
					else {
						UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added new texture annotated object %s with texture: %s"), *name_, *it.Key(), *new_texture);
					}
				}
				else if (show_by_default_ && !it.Key().Contains("hidden_sphere") && !it.Key().Contains("AnnotationSphere")) {
					name_to_component_map_.Emplace(it.Key(), it.Value());
					component_to_name_map_.Emplace(it.Value(), it.Key());
					FString new_texture = "/AirSim/HUDAssets/k";
					name_to_texture_path_map_.Emplace(it.Key(), new_texture);
					check(PaintTextureComponent(it.Value(), new_texture, it.Key()));
					UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Added untagged texture annotated object %s with default texture"), *name_, *it.Key());
				}

			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Completed full level RGB annotation."), *name_);
}

bool FObjectAnnotator::IsRGBColorValid(FColor color) {
	if (ColorGenerator_.GetIndexForColor(color) != INDEX_NONE)
		return true;
	return false;
}

FString FObjectAnnotator::GetLabelKey(const FString& component_name, UMeshComponent* component) const
{
	if (const FString* cached_label_key = name_to_label_key_map_.Find(component_name))
	{
		return *cached_label_key;
	}

	const FString stable_mesh_label = GetStableMeshLabel(component);
	if (!stable_mesh_label.IsEmpty())
	{
		return stable_mesh_label;
	}

	const FString normalized_component_name = StripRuntimeSuffixes(component_name);
	if (!normalized_component_name.IsEmpty())
	{
		return normalized_component_name;
	}

	if (IsValid(component) && IsValid(component->GetOwner()))
	{
		return StripRuntimeSuffixes(component->GetOwner()->GetName());
	}

	return StripRuntimeSuffixes(component_name);
}

FString FObjectAnnotator::GetLandscapeLabelKey(const FString& component_name, ULandscapeComponent* component) const
{
	if (const FString* cached_label_key = name_to_label_key_map_.Find(component_name))
	{
		return *cached_label_key;
	}

	const FString stable_landscape_label = GetStableLandscapeLabel(component);
	if (!stable_landscape_label.IsEmpty())
	{
		return stable_landscape_label;
	}

	return StripRuntimeSuffixes(component_name);
}

uint32 FObjectAnnotator::GetNextAvailableColorIndex() const
{
	uint32 next_color_index = 0;
	for (const auto& element : name_to_color_index_map_)
	{
		next_color_index = FMath::Max(next_color_index, element.Value + 1);
	}
	return next_color_index;
}

uint32 FObjectAnnotator::GetOrCreateLabelColorIndex(const FString& component_name, UMeshComponent* component)
{
	const FString label_key = GetLabelKey(component_name, component);
	name_to_label_key_map_.FindOrAdd(component_name) = label_key;
	if (const uint32* color_index = label_to_color_index_map_.Find(label_key))
	{
		return *color_index;
	}

	const uint32 color_index = GetDefaultIndexedColorIndex(label_key);
	label_to_color_index_map_.Add(label_key, color_index);
	return color_index;
}

uint32 FObjectAnnotator::GetOrCreateLandscapeLabelColorIndex(const FString& component_name, ULandscapeComponent* component)
{
	const FString label_key = GetLandscapeLabelKey(component_name, component);
	name_to_label_key_map_.FindOrAdd(component_name) = label_key;
	if (const uint32* color_index = label_to_color_index_map_.Find(label_key))
	{
		return *color_index;
	}

	const uint32 color_index = GetDefaultIndexedColorIndex(label_key);
	label_to_color_index_map_.Add(label_key, color_index);
	return color_index;
}

uint32 FObjectAnnotator::GetDefaultIndexedColorIndex(const FString& label_key) const
{
	if (!UsesIndexedAnnotationColors())
	{
		return GetNextAvailableColorIndex();
	}

	return (GetTypeHash(label_key) % 255) + 1;
}

void FObjectAnnotator::UpdateColorMappings(const FString& component_name, uint32 color_index)
{
	const FColor color = GetAnnotationColorForIndex(color_index);
	const FString color_string = FString::FromInt(color.R) + "," + FString::FromInt(color.G) + "," + FString::FromInt(color.B);
	const FString color_string_gammacorrected = GetDisplayColorString(color);

	if (const FString* found_index_color = color_to_name_map_.FindKey(component_name))
	{
		color_to_name_map_.Remove(*found_index_color);
	}
	if (const FString* found_index_color_gamma = gammacorrected_color_to_name_map_.FindKey(component_name))
	{
		gammacorrected_color_to_name_map_.Remove(*found_index_color_gamma);
	}

	color_to_name_map_.Emplace(color_string, component_name);
	gammacorrected_color_to_name_map_.Emplace(color_string_gammacorrected, component_name);
	name_to_color_index_map_.FindOrAdd(component_name) = color_index;
	name_to_gammacorrected_color_map_.FindOrAdd(component_name) = color_string_gammacorrected;

	if (const FString* label_key = name_to_label_key_map_.Find(component_name))
	{
		label_to_color_index_map_.FindOrAdd(*label_key) = color_index;
	}
}

void FObjectAnnotator::GetComponentIdsForColorUpdate(const FString& component_id, TArray<FString>& component_ids) const
{
	if (name_to_component_map_.Contains(component_id) || name_to_landscape_component_map_.Contains(component_id))
	{
		if (const FString* label_key = name_to_label_key_map_.Find(component_id))
		{
			for (const auto& label_entry : name_to_label_key_map_)
			{
				if (label_entry.Value == *label_key)
				{
					component_ids.Add(label_entry.Key);
				}
			}
		}

		if (component_ids.Num() == 0)
		{
			component_ids.Add(component_id);
		}
		return;
	}

	if (label_to_color_index_map_.Contains(component_id))
	{
		for (const auto& label_entry : name_to_label_key_map_)
		{
			if (label_entry.Value == component_id)
			{
				component_ids.Add(label_entry.Key);
			}
		}
	}
}

uint8 FObjectAnnotator::GetStencilValueForAnnotationColor(const FColor& color, const FString& component_name) const
{
	if (type_ == AnnotatorType::Infrared)
	{
		return color.R;
	}

	const int32 color_index = ColorGenerator_.GetIndexForColor(color);
	if (color_index != INDEX_NONE)
	{
		return static_cast<uint8>(color_index & 0xFF);
	}

	if (const uint32* mapped_color_index = name_to_color_index_map_.Find(component_name))
	{
		return static_cast<uint8>(*mapped_color_index & 0xFF);
	}

	return color.R;
}

void FObjectAnnotator::TrackSourceStencilComponent(UPrimitiveComponent* component)
{
	if (!IsValid(component))
	{
		return;
	}

	for (auto It = source_stencil_components_.CreateIterator(); It; ++It)
	{
		if (!It->IsValid())
		{
			It.RemoveCurrent();
		}
	}

	const TWeakObjectPtr<UPrimitiveComponent> component_key(component);
	if (!source_stencil_components_.Contains(component_key))
	{
		AddSourceStencilReference(component);
		source_stencil_components_.Add(component_key);
	}
}

void FObjectAnnotator::ReleaseSourceStencilComponent(UPrimitiveComponent* component)
{
	if (!component)
	{
		return;
	}

	const TWeakObjectPtr<UPrimitiveComponent> component_key(component);
	if (source_stencil_components_.Remove(component_key) > 0)
	{
		ReleaseSourceStencilReference(component);
	}
}

void FObjectAnnotator::ReleaseAllSourceStencilComponents()
{
	for (const TWeakObjectPtr<UPrimitiveComponent>& component_ptr : source_stencil_components_)
	{
		if (UPrimitiveComponent* component = component_ptr.Get())
		{
			ReleaseSourceStencilReference(component);
		}
	}
	source_stencil_components_.Empty();
}

bool FObjectAnnotator::PaintSourceStencilComponent(UPrimitiveComponent* component, const FColor& color, const FString& component_name)
{
	if (!IsValid(component))
	{
		return false;
	}

	TrackSourceStencilComponent(component);
	ApplySourceStencilValue(component, GetStencilValueForAnnotationColor(color, component_name));
	return true;
}

bool FObjectAnnotator::PaintRGBComponent(UMeshComponent* component, const FColor& color, const FString& component_name)
{
	if (!component) return false;
	if (UsesIndexedAnnotationColors())
	{
		return PaintSourceStencilComponent(component, color, component_name);
	}

	FLinearColor LinearColor = FLinearColor(color);
	const FColor NewColor = LinearColor.ToFColor(false);

	if (!CanCreateProxyAnnotationComponent(component_name))
	{
		return false;
	}

	FString newName = name_ + "_" + component_name;
	UAnnotationComponent* AnnotationComponent = NewObject<UAnnotationComponent>(component, FName(*newName));
	AnnotationComponent->SetupAttachment(component);
	AnnotationComponent->RegisterComponent();
	AnnotationComponent->SetAnnotationColor(NewColor);
	AnnotationComponent->SetVisibleInSceneCaptureOnly(true);
	AnnotationComponent->SetVisibleInRayTracing(false);
	AnnotationComponent->bVisibleInReflectionCaptures = false;
	AnnotationComponent->bAffectDynamicIndirectLighting = false;
	AnnotationComponent->bAffectDistanceFieldLighting = false;
	AnnotationComponent->bVisibleInRealTimeSkyCaptures = false;
	AnnotationComponent->bRenderInMainPass = false;
	AnnotationComponent->MarkRenderStateDirty();
	TrackAnnotationComponent(component_name, AnnotationComponent);
	return true;
}

bool FObjectAnnotator::UpdatePaintRGBComponent(UMeshComponent* component, const FColor& color, const FString& component_name)
{
	if (!component) return false;
	if (UsesIndexedAnnotationColors())
	{
		return PaintSourceStencilComponent(component, color, component_name);
	}

	FLinearColor LinearColor = FLinearColor(color);
	const FColor NewColor = LinearColor.ToFColor(false);
	if (UAnnotationComponent* tracked_annotation_component = FindTrackedAnnotationComponent(component_name, component))
	{
		tracked_annotation_component->SetAnnotationColor(NewColor);
		tracked_annotation_component->MarkRenderStateDirty();
		return true;
	}

	AActor* owner = GetAnnotationOwnerForComponent(component);
	if (!IsValid(owner))
	{
		return false;
	}

	TArray<UActorComponent*> AnnotationComponents = owner->K2_GetComponentsByClass(UAnnotationComponent::StaticClass());
	for (UActorComponent* Component : AnnotationComponents)
	{
		UAnnotationComponent* AnnotationComponent = Cast<UAnnotationComponent>(Component);
		if (!IsValid(AnnotationComponent))
		{
			continue;
		}
		FName componentFName = *AnnotationComponent->GetName();
		FString componentName = componentFName.ToString();
		if (componentName == name_ + "_" + component_name) {
			AnnotationComponent->SetAnnotationColor(NewColor);
			AnnotationComponent->MarkRenderStateDirty();
			TrackAnnotationComponent(component_name, AnnotationComponent);
			return true;
		}		
	}
	return PaintRGBComponent(component, color, component_name);
}

bool FObjectAnnotator::PaintLandscapeComponent(ULandscapeComponent* component, const FColor& color, const FString& component_name)
{
	if (!IsValid(component)) return false;
	if (UsesIndexedAnnotationColors())
	{
		return PaintSourceStencilComponent(component, color, component_name);
	}

	FLinearColor LinearColor = FLinearColor(color);
	const FColor NewColor = LinearColor.ToFColor(false);

	if (!CanCreateProxyAnnotationComponent(component_name))
	{
		return false;
	}

	FString newName = name_ + "_" + component_name;
	UAnnotationComponent* AnnotationComponent = NewObject<UAnnotationComponent>(component, FName(*newName));
	if (!IsValid(AnnotationComponent))
	{
		return false;
	}

	AnnotationComponent->SetupAttachment(component);
	AnnotationComponent->RegisterComponent();
	AnnotationComponent->SetAnnotationColor(NewColor);
	AnnotationComponent->SetVisibleInSceneCaptureOnly(true);
	AnnotationComponent->SetVisibleInRayTracing(false);
	AnnotationComponent->bVisibleInReflectionCaptures = false;
	AnnotationComponent->bAffectDynamicIndirectLighting = false;
	AnnotationComponent->bAffectDistanceFieldLighting = false;
	AnnotationComponent->bVisibleInRealTimeSkyCaptures = false;
	AnnotationComponent->bRenderInMainPass = false;
	AnnotationComponent->SetReceivesDecals(false);
	AnnotationComponent->SetCastShadow(false);
	AnnotationComponent->MarkRenderStateDirty();
	TrackAnnotationComponent(component_name, AnnotationComponent);
	return true;
}

bool FObjectAnnotator::UpdatePaintLandscapeComponent(ULandscapeComponent* component, const FColor& color, const FString& component_name)
{
	if (!IsValid(component)) return false;
	if (UsesIndexedAnnotationColors())
	{
		return PaintSourceStencilComponent(component, color, component_name);
	}

	FLinearColor LinearColor = FLinearColor(color);
	const FColor NewColor = LinearColor.ToFColor(false);
	const FString expected_component_name = name_ + "_" + component_name;
	if (UAnnotationComponent* tracked_annotation_component = FindTrackedAnnotationComponent(component_name, component))
	{
		tracked_annotation_component->SetAnnotationColor(NewColor);
		tracked_annotation_component->MarkRenderStateDirty();
		return true;
	}

	AActor* owner = GetAnnotationOwnerForComponent(component);
	if (!IsValid(owner))
	{
		return false;
	}

	TArray<UActorComponent*> AnnotationComponents = owner->K2_GetComponentsByClass(UAnnotationComponent::StaticClass());
	for (UActorComponent* Component : AnnotationComponents)
	{
		UAnnotationComponent* AnnotationComponent = Cast<UAnnotationComponent>(Component);
		if (!IsValid(AnnotationComponent))
		{
			continue;
		}

		if (AnnotationComponent->GetName() == expected_component_name) {
			AnnotationComponent->SetAnnotationColor(NewColor);
			AnnotationComponent->MarkRenderStateDirty();
			TrackAnnotationComponent(component_name, AnnotationComponent);
			return true;
		}
	}

	return PaintLandscapeComponent(component, color, component_name);
}

bool FObjectAnnotator::PaintTextureComponent(UMeshComponent* component, const FString& texture_path, const FString& component_name)
{
	if (!component) return false;

	if (!CanCreateProxyAnnotationComponent(component_name))
	{
		return false;
	}

	FString newName = name_ + "_" + component_name;
	UAnnotationComponent* AnnotationComponent = NewObject<UAnnotationComponent>(component, FName(*newName));
	AnnotationComponent->SetupAttachment(component);
	AnnotationComponent->RegisterComponent();
	AnnotationComponent->SetAnnotationTexture(texture_path);
	AnnotationComponent->SetVisibleInSceneCaptureOnly(true);
	AnnotationComponent->SetVisibleInRayTracing(false);
	AnnotationComponent->bVisibleInReflectionCaptures = false;
	AnnotationComponent->bAffectDynamicIndirectLighting = false;
	AnnotationComponent->bAffectDistanceFieldLighting = false;
	AnnotationComponent->bRenderInMainPass = false;
	AnnotationComponent->bVisibleInRealTimeSkyCaptures = false;
	AnnotationComponent->MarkRenderStateDirty();
	TrackAnnotationComponent(component_name, AnnotationComponent);
	return true;
}

bool FObjectAnnotator::UpdatePaintTextureComponent(UMeshComponent* component, const FString& texture_path, const FString& component_name)
{
	if (!component) return false;

	if (UAnnotationComponent* tracked_annotation_component = FindTrackedAnnotationComponent(component_name, component))
	{
		tracked_annotation_component->SetAnnotationTexture(texture_path);
		tracked_annotation_component->MarkRenderStateDirty();
		return true;
	}

	AActor* owner = GetAnnotationOwnerForComponent(component);
	if (!IsValid(owner))
	{
		return false;
	}

	TArray<UActorComponent*> AnnotationComponents = owner->K2_GetComponentsByClass(UAnnotationComponent::StaticClass());
	for (UActorComponent* Component : AnnotationComponents)
	{
		UAnnotationComponent* AnnotationComponent = Cast<UAnnotationComponent>(Component);
		if (!IsValid(AnnotationComponent))
		{
			continue;
		}
		FName componentFName = *AnnotationComponent->GetName();
		FString componentName = componentFName.ToString();
		if (componentName == name_ + "_" + component_name) {
			AnnotationComponent->SetAnnotationTexture(texture_path);
			AnnotationComponent->MarkRenderStateDirty();
			TrackAnnotationComponent(component_name, AnnotationComponent);
			return true;
		}
	}
	return PaintTextureComponent(component, texture_path, component_name);
}

bool FObjectAnnotator::DeleteComponent(UMeshComponent* component, const FString& component_name)
{
	if (!component) return false;
	if (UsesIndexedAnnotationColors())
	{
		ReleaseSourceStencilComponent(component);
		return true;
	}

	const FString expected_component_name = name_ + "_" + component_name;
	UAnnotationComponent* deleted_tracked_component = nullptr;
	if (UAnnotationComponent* tracked_annotation_component = FindTrackedAnnotationComponent(component_name, component))
	{
		annotation_component_list_.Remove(tracked_annotation_component);
		name_to_annotation_component_map_.Remove(component_name);
		tracked_annotation_component->DestroyComponent();
		deleted_tracked_component = tracked_annotation_component;
	}

	AActor* owner = GetAnnotationOwnerForComponent(component);
	if (!IsValid(owner))
	{
		return false;
	}

	TArray<UActorComponent*> AnnotationComponents = owner->K2_GetComponentsByClass(UAnnotationComponent::StaticClass());
	for (UActorComponent* Component : AnnotationComponents)
	{
		UAnnotationComponent* AnnotationComponent = Cast<UAnnotationComponent>(Component);
		if (!IsValid(AnnotationComponent))
		{
			continue;
		}
		if (AnnotationComponent == deleted_tracked_component)
		{
			continue;
		}
		FName componentFName = *AnnotationComponent->GetName();
		FString componentName = componentFName.ToString();
		if (componentName == expected_component_name) {
			annotation_component_list_.Remove(AnnotationComponent);
			name_to_annotation_component_map_.Remove(component_name);
			AnnotationComponent->DestroyComponent();
		}				
	}
	return true;
}

bool FObjectAnnotator::DeleteLandscapeComponent(ULandscapeComponent* component, const FString& component_name)
{
	if (!IsValid(component)) return false;
	if (UsesIndexedAnnotationColors())
	{
		ReleaseSourceStencilComponent(component);
		return true;
	}

	const FString expected_component_name = name_ + "_" + component_name;
	UAnnotationComponent* deleted_tracked_component = nullptr;
	if (UAnnotationComponent* tracked_annotation_component = FindTrackedAnnotationComponent(component_name, component))
	{
		annotation_component_list_.Remove(tracked_annotation_component);
		name_to_annotation_component_map_.Remove(component_name);
		tracked_annotation_component->DestroyComponent();
		deleted_tracked_component = tracked_annotation_component;
	}

	AActor* owner = GetAnnotationOwnerForComponent(component);
	if (!IsValid(owner))
	{
		return false;
	}

	TArray<UActorComponent*> AnnotationComponents = owner->K2_GetComponentsByClass(UAnnotationComponent::StaticClass());
	for (UActorComponent* Component : AnnotationComponents)
	{
		UAnnotationComponent* AnnotationComponent = Cast<UAnnotationComponent>(Component);
		if (!IsValid(AnnotationComponent))
		{
			continue;
		}
		if (AnnotationComponent == deleted_tracked_component)
		{
			continue;
		}

		if (AnnotationComponent->GetName() == expected_component_name) {
			annotation_component_list_.Remove(AnnotationComponent);
			name_to_annotation_component_map_.Remove(component_name);
			AnnotationComponent->DestroyComponent();
		}
	}
	return true;
}

UAnnotationComponent* FObjectAnnotator::FindTrackedAnnotationComponent(const FString& component_name, const USceneComponent* attach_parent)
{
	UAnnotationComponent** annotation_component_ptr = name_to_annotation_component_map_.Find(component_name);
	if (annotation_component_ptr == nullptr)
	{
		return nullptr;
	}

	UAnnotationComponent* annotation_component = *annotation_component_ptr;
	if (IsValid(annotation_component) && (attach_parent == nullptr || annotation_component->GetAttachParent() == attach_parent))
	{
		return annotation_component;
	}

	annotation_component_list_.Remove(annotation_component);
	name_to_annotation_component_map_.Remove(component_name);
	return nullptr;
}

void FObjectAnnotator::TrackAnnotationComponent(const FString& component_name, UAnnotationComponent* component)
{
	if (!IsValid(component))
	{
		return;
	}

	name_to_annotation_component_map_.FindOrAdd(component_name) = component;
	annotation_component_list_.AddUnique(component);
}

void FObjectAnnotator::RemoveTrackedComponent(const FString& component_name)
{
	if (UMeshComponent** component_ptr = name_to_component_map_.Find(component_name))
	{
		if (UsesIndexedAnnotationColors())
		{
			ReleaseSourceStencilComponent(Cast<UPrimitiveComponent>(*component_ptr));
		}
		component_to_name_map_.Remove(*component_ptr);
	}
	if (ULandscapeComponent** landscape_component_ptr = name_to_landscape_component_map_.Find(component_name))
	{
		if (IsValid(*landscape_component_ptr))
		{
			DeleteLandscapeComponent(*landscape_component_ptr, component_name);
		}
		landscape_component_to_name_map_.Remove(*landscape_component_ptr);
	}

	if (const FString* color_key = color_to_name_map_.FindKey(component_name))
	{
		color_to_name_map_.Remove(*color_key);
	}
	if (const FString* gamma_color_key = gammacorrected_color_to_name_map_.FindKey(component_name))
	{
		gammacorrected_color_to_name_map_.Remove(*gamma_color_key);
	}

	FString label_key;
	const bool had_label_key = name_to_label_key_map_.RemoveAndCopyValue(component_name, label_key);
	name_to_annotation_component_map_.Remove(component_name);
	name_to_component_map_.Remove(component_name);
	name_to_landscape_component_map_.Remove(component_name);
	name_to_color_index_map_.Remove(component_name);
	name_to_gammacorrected_color_map_.Remove(component_name);
	name_to_value_map_.Remove(component_name);
	name_to_texture_path_map_.Remove(component_name);

	if (had_label_key)
	{
		bool label_key_still_used = false;
		for (const auto& label_entry : name_to_label_key_map_)
		{
			if (label_entry.Value == label_key)
			{
				label_key_still_used = true;
				break;
			}
		}

		if (!label_key_still_used)
		{
			label_to_color_index_map_.Remove(label_key);
		}
	}
}

void FObjectAnnotator::SetViewForAnnotationRender(FEngineShowFlags& show_flags)
{
	// Annotation components and lightweight instanced mirrors both rely on the annotation material.
	show_flags.SetMaterials(true);
	show_flags.SetLighting(false);
	show_flags.SetBSPTriangles(true);
	show_flags.SetPostProcessing(false);
	show_flags.SetHMDDistortion(false);
	show_flags.SetTonemapper(false);
	show_flags.SetEyeAdaptation(false);
	show_flags.SetFog(false);
	show_flags.SetPaper2DSprites(false);
	show_flags.SetBloom(false);
	show_flags.SetMotionBlur(false);
	show_flags.SetSkyLighting(false);
	show_flags.SetVisualizeSkyAtmosphere(false);
	show_flags.SetAmbientOcclusion(false);
	show_flags.SetAtmosphere(false);
	show_flags.SetInstancedFoliage(true);
	show_flags.SetInstancedGrass(true);
	show_flags.SetLandscape(true);
	show_flags.SetNaniteMeshes(false);
	show_flags.SetTextRender(false);
	show_flags.SetTemporalAA(false);
	show_flags.SetDecals(false);
}

void FObjectAnnotator::SetViewForSourceStencilAnnotationRender(FEngineShowFlags& show_flags)
{
	show_flags.SetMaterials(false);
	show_flags.SetLighting(false);
	show_flags.SetBSPTriangles(true);
	show_flags.SetPostProcessing(true);
	show_flags.SetPostProcessMaterial(true);
	show_flags.SetHMDDistortion(false);
	show_flags.SetTonemapper(false);
	show_flags.SetEyeAdaptation(false);
	show_flags.SetFog(false);
	show_flags.SetPaper2DSprites(false);
	show_flags.SetBloom(false);
	show_flags.SetMotionBlur(false);
	show_flags.SetSkyLighting(false);
	show_flags.SetVisualizeSkyAtmosphere(false);
	show_flags.SetAmbientOcclusion(false);
	show_flags.SetAtmosphere(false);
	show_flags.SetInstancedFoliage(true);
	show_flags.SetInstancedGrass(true);
	show_flags.SetLandscape(true);
	show_flags.SetNaniteMeshes(true);
	show_flags.SetTextRender(false);
	show_flags.SetTemporalAA(false);
	show_flags.SetDecals(false);
}

bool FObjectAnnotator::IsDirect()
{
	return set_direct_;
}

FObjectAnnotator::AnnotatorType FObjectAnnotator::GetType()
{
	return type_;
}

bool FObjectAnnotator::UsesSourceStencilBackend() const
{
	return use_source_stencil_backend_;
}

void FObjectAnnotator::UpdateAnnotationComponents(UWorld* World)
{
	if (!IsValid(World))
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation [%s]: Can not get AnnotationComponents, World is invalid."), *name_);
		return;
	}

	annotation_component_list_.RemoveAll([](const TWeakObjectPtr<UPrimitiveComponent>& Component) {
		return !Component.IsValid();
		});
	for (auto It = name_to_annotation_component_map_.CreateIterator(); It; ++It)
	{
		if (!IsValid(It.Value()))
		{
			It.RemoveCurrent();
		}
	}

	if (UsesIndexedAnnotationColors())
	{
		TArray<FString> stale_component_names;
		for (const auto& component_entry : name_to_component_map_)
		{
			if (!IsValid(component_entry.Value))
			{
				stale_component_names.Add(component_entry.Key);
				continue;
			}

			const FColor color = GetAnnotationColorForIndex(name_to_color_index_map_.FindRef(component_entry.Key));
			if (!UpdatePaintRGBComponent(component_entry.Value, color, component_entry.Key))
			{
				stale_component_names.Add(component_entry.Key);
			}
		}

		for (const auto& component_entry : name_to_landscape_component_map_)
		{
			if (!IsValid(component_entry.Value))
			{
				stale_component_names.Add(component_entry.Key);
				continue;
			}

			const FColor color = GetAnnotationColorForIndex(name_to_color_index_map_.FindRef(component_entry.Key));
			if (!UpdatePaintLandscapeComponent(component_entry.Value, color, component_entry.Key))
			{
				stale_component_names.Add(component_entry.Key);
			}
		}

		for (const FString& stale_component_name : stale_component_names)
		{
			RemoveTrackedComponent(stale_component_name);
		}
		return;
	}

	if (annotation_component_list_.Num() > 0)
	{
		return;
	}

	// Check how much time is spent here!
	TArray<UObject*> UObjectList;
	bool bIncludeDerivedClasses = false;
	EObjectFlags ExclusionFlags = EObjectFlags::RF_ClassDefaultObject;
	EInternalObjectFlags ExclusionInternalFlags = EInternalObjectFlags::None;
	GetObjectsOfClass(UAnnotationComponent::StaticClass(), UObjectList, bIncludeDerivedClasses, ExclusionFlags, ExclusionInternalFlags);

	for (UObject* Object : UObjectList)
	{
		UAnnotationComponent* Component = Cast<UAnnotationComponent>(Object);
		if (!IsValid(Component))
		{
			continue;
		}
		FName componentFName = *Component->GetName();
		FString componentName = componentFName.ToString();
		const FString component_prefix = name_ + "_";
		if (Component->GetWorld() == World
			&& !annotation_component_list_.Contains(Component)
			&& componentName.StartsWith(component_prefix)
			&& !componentName.Contains("annotation_sphere"))
		{
			annotation_component_list_.Add(Component);
			const FString tracked_component_name = componentName.RightChop(component_prefix.Len());
			name_to_annotation_component_map_.FindOrAdd(tracked_component_name) = Component;
		}
	}

	if (annotation_component_list_.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation [%s]: No annotation in the scene to show."), *name_);
	}
}

TArray<TWeakObjectPtr<UPrimitiveComponent>>  FObjectAnnotator::GetAnnotationComponents() {
	return annotation_component_list_;
}

std::vector<std::string> FObjectAnnotator::GetAllComponentNames() {
	std::vector<std::string> retval;
	TMap<FString, uint32> nameToColorIndexMapTemp = name_to_color_index_map_;
	for (auto const& element : nameToColorIndexMapTemp) {
		retval.emplace_back(std::string(TCHAR_TO_UTF8(*element.Key)));
	}
	return retval;
}


TMap<FString, UMeshComponent*> FObjectAnnotator::GetNameToComponentMap() {
	return name_to_component_map_;
}

TMap<FString, UPrimitiveComponent*> FObjectAnnotator::GetNameToPrimitiveComponentMap() {
	TMap<FString, UPrimitiveComponent*> primitive_component_map;
	for (const auto& element : name_to_component_map_)
	{
		if (IsValid(element.Value))
		{
			primitive_component_map.Emplace(element.Key, Cast<UPrimitiveComponent>(element.Value));
		}
	}
	for (const auto& element : name_to_landscape_component_map_)
	{
		if (IsValid(element.Value))
		{
			primitive_component_map.Emplace(element.Key, Cast<UPrimitiveComponent>(element.Value));
		}
	}
	return primitive_component_map;
}

TMap<UMeshComponent*, FString> FObjectAnnotator::GetComponentToNameMap() {
	return component_to_name_map_;
}


TMap<FString, FString> FObjectAnnotator::GetColorToComponentNameMap() {
	return gammacorrected_color_to_name_map_;
}

TMap<FString, float> FObjectAnnotator::GetComponentToValueMap() {
	return name_to_value_map_;
}

TArray<FColor> FObjectAnnotator::GetColorMap(){
	return ColorGenerator_.GetColorMap();
}


void FObjectAnnotator::EndPlay() {

	ReleaseAllSourceStencilComponents();

	if (!UsesIndexedAnnotationColors())
	{
		TSet<TWeakObjectPtr<UPrimitiveComponent>> components_to_destroy;
		for (const TWeakObjectPtr<UPrimitiveComponent>& component_ptr : annotation_component_list_)
		{
			components_to_destroy.Add(component_ptr);
		}
		for (const auto& component_entry : name_to_annotation_component_map_)
		{
			components_to_destroy.Add(component_entry.Value);
		}
		for (const TWeakObjectPtr<UPrimitiveComponent>& component_ptr : components_to_destroy)
		{
			if (UPrimitiveComponent* component = component_ptr.Get())
			{
				component->DestroyComponent();
			}
		}
	}
	name_to_color_index_map_.Empty();
	color_to_name_map_.Empty();
	gammacorrected_color_to_name_map_.Empty();
	name_to_component_map_.Empty();
	name_to_landscape_component_map_.Empty();
	name_to_annotation_component_map_.Empty();
	name_to_label_key_map_.Empty();
	label_to_color_index_map_.Empty();
	annotation_component_list_.Empty();
	name_to_gammacorrected_color_map_.Empty();
	name_to_value_map_.Empty();
	name_to_texture_path_map_.Empty();
	component_to_name_map_.Empty();
	landscape_component_to_name_map_.Empty();
}

int32 FColorGenerator::GetChannelValue(uint32 index) const
{
	static int32 values[256] = { 0 };
	static bool init = false;
	if (!init)
	{
		float step = 256;
		uint32 iter = 0;
		values[0] = 0;
		while (step >= 1)
		{
			for (uint32 value = step - 1; value <= 256; value += step * 2)
			{
				iter++;
				values[iter] = value;
			}
			step /= 2;
		}
		init = true;
	}
	if (index >= 0 && index <= 255)
	{
		return values[index];
	}
	else
	{
		check(false);
		return -1;
	}
}

void FColorGenerator::GetColors(int32 max_val, bool enable_1, bool enable_2, bool enable_3, TArray<FColor>& color_map, TArray<int32>& ok_values) const
{

	for (int32 I = 0; I <= (enable_1 ? 0 : max_val - 1); I++)
	{
		for (int32 J = 0; J <= (enable_2 ? 0 : max_val - 1); J++)
		{
			for (int32 K = 0; K <= (enable_3 ? 0 : max_val - 1); K++)
			{
				uint8 R = GetChannelValue(enable_1 ? max_val : I);
				uint8 G = GetChannelValue(enable_2 ? max_val : J);
				uint8 B = GetChannelValue(enable_3 ? max_val : K);
				if (ok_values.Contains(R) && ok_values.Contains(G) && ok_values.Contains(B) && R != 149 && B != 149 && G != 149) {
					FColor color(R, G, B, 255);
					color_map.Add(color);
				}
			}
		}
	}
}

TArray<FColor> FColorGenerator::color_map_;

FColor FColorGenerator::GetColorFromColorMap(int32 color_index) const
{
	static TArray<int32> ok_values_;
	int num_per_channel = 256;
	int uneven_start = 79;
	int full_start = 149;
	int uneven_count = FMath::FloorToInt((full_start - uneven_start + 2) / 2.0f);
	if (color_map_.Num() == 0) {

		for (int32 i = uneven_start; i <= full_start; i += 2) {
			ok_values_.Emplace(i);
		}
		for (int32 i = full_start + 1; i < num_per_channel; i++) {
			ok_values_.Emplace(i);
		}
		for (int32 max_channel_index = 0; max_channel_index < num_per_channel; max_channel_index++)
		{
			GetColors(max_channel_index, false, false, true, color_map_, ok_values_);
			GetColors(max_channel_index, false, true, false, color_map_, ok_values_);
			GetColors(max_channel_index, false, true, true, color_map_, ok_values_);
			GetColors(max_channel_index, true, false, false, color_map_, ok_values_);
			GetColors(max_channel_index, true, false, true, color_map_, ok_values_);
			GetColors(max_channel_index, true, true, false, color_map_, ok_values_);
			GetColors(max_channel_index, true, true, true, color_map_, ok_values_);
		}
	}
	if (color_index < 0 || color_index >= pow((num_per_channel - full_start) + uneven_count - 3, 3))
	{
		UE_LOG(LogTemp, Error, TEXT("AirSim Annotation: Object index %i is out of the available color map boundary [0, %s]"), color_index, *FString::SanitizeFloat(pow((num_per_channel - full_start) + uneven_count - 3, 3)));
		return FColor(0, 0, 0);
	}
	else {
		return color_map_[color_index];
	}
}

int FColorGenerator::GetIndexForColor(FColor color) const {
	return color_map_.Find(color);
}

int FColorGenerator::GetGammaCorrectedColor(int color_index) const {
	return GammaCorrectionTable_[color_index];
}

 TArray<FColor> FColorGenerator::GetColorMap() const{
	return color_map_;
}

int32 FColorGenerator::GammaCorrectionTable_[256] =
{
	0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 79, 0,
	81, 0, 83, 0, 85, 0, 86, 0, 88, 0,
	90, 0, 93, 0, 95, 0, 96, 0, 98, 0,
	101, 0, 102, 0, 105, 0, 106, 0, 109, 0,
	110, 0, 113, 0, 114, 0, 117, 0, 119, 0,
	120, 0, 122, 0, 125, 0, 127, 0, 129, 0,
	131, 0, 133, 0, 135, 0, 137, 0, 139, 0,
	141, 0, 143, 0, 145, 0, 147, 147, 148, 150,
	151, 152, 153, 154, 155, 156, 157, 158, 159, 160,
	161, 162, 163, 164, 165, 166, 167, 168, 169, 170,
	171, 172, 173, 174, 175, 176, 177, 178, 179, 180,
	181, 182, 183, 184, 185, 186, 187, 188, 189, 190,
	191, 192, 193, 194, 195, 196, 197, 198, 199, 200,
	201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
	211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
	221, 222, 223, 224, 225, 226, 227, 228, 229, 230,
	231, 232, 233, 234, 235, 236, 237, 238, 239, 240,
	241, 242, 243, 244, 245, 246, 247, 248, 249, 250,
	251, 252, 253, 254, 255
};
