# Instance Segmentation in Cosys-AirSim

An instance segmentation system is implemented into Cosys-AirSim. The current built-in `ImageType::Segmentation` path uses Unreal CustomDepth/CustomStencil on the original scene primitives, then converts the 8-bit stencil value to the AirSim segmentation color map in the capture post-process.

Current indexed instance segmentation supports regular static meshes, skeletal meshes, `UInstancedStaticMeshComponent` content, and Unreal Landscape components by labeling the source components directly. It does not create generated annotation mirror components for segmentation.

## Limitations
* Built-in source-stencil segmentation IDs are limited to `0..255`. IDs outside this range are rejected by `simSetSegmentationObjectID()` and `simSetSegmentationObjectIDs()`.
* Startup auto-labeling uses stable 8-bit label values. In large scenes, different objects can share a label because the stencil domain is intentionally compact.
* Static meshes, skeletal meshes, instanced static mesh components, and `ULandscapeComponent` terrain are supported by the built-in instance segmentation path.
  * One `UInstancedStaticMeshComponent` receives one segmentation ID/color. Per-instance IDs inside the same instanced component are not supported yet.
  * One `ALandscapeProxy` receives one segmentation ID/color by default. Internally, each `ULandscapeComponent` has its own listed component ID, but the owning landscape proxy name is used as the shared label key so `simSetSegmentationObjectID("<LandscapeProxyName>", ...)` updates all components of that landscape together.
  * UE foliage painted as instanced static mesh foliage is covered by the instanced static mesh path because `UFoliageInstancedStaticMeshComponent` derives from `UInstancedStaticMeshComponent`. It still receives one ID/color per foliage component, not per individual tree/grass instance.
  * Brush objects aren't supported. In Unreal Engine 5.5, `UBrushComponent` derives from `UPrimitiveComponent`, not `UMeshComponent`, so brushes are skipped by the current object discovery path. As a work-around, convert them to StaticMesh assets.
  * Other unsupported primitive types, such as decals, text, non-instanced foliage systems, and non-mesh custom primitives, generally will not render in segmentation/infrared unless they are added to the supported object discovery path and receive a stencil label.

## Usage
By default, AirSim does not run the startup segmentation scan. Set the root setting `InitialInstanceSegmentation` to `true` in `settings.json` when you want AirSim to assign an object ID/color index to each supported object label at simulation startup:

```json
{
  "SettingsVersion": 2.0,
  "InitialInstanceSegmentation": true
}
```

When `InitialInstanceSegmentation` is omitted or set to `false`, segmentation/infrared startup labeling and the first forced refresh are skipped. This is useful for projects that do not need segmentation masks. In that mode, the segmentation object list starts empty; ID update APIs can only update objects that have already been added to the annotator, for example by `ASimModeBase::AddNewActorToInstanceSegmentation(AActor)`.
Please see the [Image API documentation](image_apis.md#segmentation) on how to set or get the color information after objects are registered.

For an example of the Instance Segmentation API, please see the script _segmentation_test.py_ (Cosys-Airsim/PythonClient/segmentation/segmentation_test.py).

For many ID changes, prefer `simSetSegmentationObjectIDs(mesh_names, object_ids, is_name_regex=False)` over calling `simSetSegmentationObjectID()` in a loop. The batch API applies all ID changes in one RPC/game-thread update and requests one segmentation/infrared camera refresh at the end if anything changed. The `mesh_names` and `object_ids` lists must have the same length, and it returns a boolean result for each input name in the same order as `mesh_names`. This is important for large UE 5.5 environments with landscape or foliage annotation components.

For a script that generates a full list of objects and their associated color, please see the script _segmentation_generate_list.py_ (Cosys-Airsim/PythonClient/segmentation/segmentation_generate_list.py).

For a script that randomizes or restores many object IDs, please see the script _segmentation_randomize_ids.py_ (Cosys-Airsim/PythonClient/segmentation/segmentation_randomize_ids.py). It uses `simSetSegmentationObjectIDs()` when the connected simulator supports it, and falls back to the single-object API for older builds.

When a new object is spawned in your environment by for example a c++ or blueprint extension you made,
and you want it to work with the instance segmentation system, you can use the extended function `ASimModeBase::AddNewActorToInstanceSegmentation(AActor)` which is also available in blueprints.

Make sure to provide human-readable names to your objects in your environment as the ground truth tables that the AirSim API can provide will use your object naming to create the table.

If you want to not label specific components/meshes of an actor, you can add the Unreal Tag `InstanceSegmentation_disable` to the components/meshes you want to ignore.

## Credits
The original proxy segmentation method was a derivative of and inspired by the work of [UnrealCV](https://unrealcv.org/). Their work is licensed under the MIT License. The current built-in segmentation path uses source CustomStencil labels instead of generated proxy geometry.
It is made by students from Johns Hopkins University and Peking University under the supervision of Prof. Alan Yuille and Prof. Yizhou Wang.
You can read the paper on their work [here](https://dl.acm.org/doi/10.1145/3123266.3129396).
