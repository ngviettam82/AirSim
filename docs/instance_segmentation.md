# Instance Segmentation in Cosys-AirSim

An Instance segmentation system is implemented into Cosys-AirSim. It uses annotation-only proxy rendering to allow each supported object in the world to get its own color.

Current indexed instance segmentation supports regular static meshes, skeletal meshes, and `UInstancedStaticMeshComponent` content. Instanced static meshes are rendered through generated annotation mirror components that copy the source mesh and instance transforms, so the segmentation image stays aligned without changing the original scene materials.

## Limitations
* 2744000 different colors are currently available to be assigned to unique objects. If your environment during a run requires more colors, you will generate errors and new objects will be assigned color [0,0,0].
* Static meshes, skeletal meshes, and instanced static mesh components are supported by the built-in instance segmentation path.
  * One `UInstancedStaticMeshComponent` receives one segmentation ID/color. Per-instance IDs inside the same instanced component are not supported yet.
  * Landscape objects aren't supported yet. In Unreal Engine 5.5, `ULandscapeComponent` derives from `UPrimitiveComponent`, not `UMeshComponent`, while the current object discovery path only gathers `UMeshComponent` instances and `UAnnotationComponent` only creates static/skeletal mesh proxies. As a work-around, StaticMesh terrain must be used.
  * UE foliage painted as instanced static mesh foliage is covered by the instanced static mesh path because `UFoliageInstancedStaticMeshComponent` derives from `UInstancedStaticMeshComponent`. It still receives one ID/color per foliage component, not per individual tree/grass instance, and annotation materials do not reproduce wind or other source-material world-position-offset deformation.
  * Brush objects aren't supported. In Unreal Engine 5.5, `UBrushComponent` derives from `UPrimitiveComponent`, not `UMeshComponent`, so brushes are skipped by the current object discovery path. As a work-around, convert them to StaticMesh assets.
  * Other unsupported primitive types, such as decals, text, non-instanced foliage systems, and non-mesh custom primitives, generally will not render in segmentation/infrared because the captures render only generated annotation components in their show-only lists.

## Landscape follow-up plan

The preferred lightweight path for future landscape support is to add a landscape-specific annotation proxy instead of converting terrain to meshes or overriding the original landscape material. The implementation should:

1. Add a landscape annotation component/proxy that renders `ULandscapeComponent` geometry with a constant annotation material or color.
2. Extend object discovery to include `ALandscapeProxy::LandscapeComponents` in addition to `UMeshComponent`.
3. Add the generated landscape annotation components to the same segmentation and infrared show-only lists.
4. Keep one label/color for the whole landscape by default, with optional per-landscape-component IDs only if needed.
5. Avoid per-frame terrain mesh conversion, source material mutation, and full-scene material overrides because those are expensive and fragile in large environments.

## Usage
By default, at the start of the simulation, it assigns an object ID/color index to each supported object label found by the annotator. You can disable this by setting the main parameter `InitialInstanceSegmentation` to false in the settings.json file.
Please see the [Image API documentation](image_apis.md#segmentation) on how to manually set or get the color information.

For an example of the Instance Segmentation API, please see the script _segmentation_test.py_ (Cosys-Airsim/PythonClient/segmentation/segmentation_test.py).

For a script that generates a full list of objects and their associated color, please see the script _segmentation_generate_list.py_ (Cosys-Airsim/PythonClient/segmentation/segmentation_generate_list.py).

When a new object is spawned in your environment by for example a c++ or blueprint extension you made,
and you want it to work with the instance segmentation system, you can use the extended function `ASimModeBase::AddNewActorToInstanceSegmentation(AActor)` which is also available in blueprints.

Make sure to provide human-readable names to your objects in your environment as the ground truth tables that the AirSim API can provide will use your object naming to create the table.

If you want to not label specific components/meshes of an actor, you can add the Unreal Tag `InstanceSegmentation_disable` to the components/meshes you want to ignore.

## Credits
The method used to use Proxy meshes to segment object is a derivative of and inspired by the work of [UnrealCV](https://unrealcv.org/). Their work is licensed under the MIT License.
It is made by students from Johns Hopkins University and Peking University under the supervision of Prof. Alan Yuille and Prof. Yizhou Wang.
You can read the paper on their work [here](https://dl.acm.org/doi/10.1145/3123266.3129396).
