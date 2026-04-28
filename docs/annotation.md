# Annotation in Cosys-AirSim

A multi-layer annotation system is implemented into Cosys-AirSim. Annotation can render custom RGB, greyscale, or texture layers for actors and components in the world.

There are two rendering backends:

* `SourceStencil` writes labels directly to the original Unreal source primitives through CustomDepth/CustomStencil. It is lightweight and does not create proxy mesh components, but it is limited to 8-bit labels (`0..255`) and is only supported for indexed RGB-style labels.
* `Proxy` creates AirSim annotation components that render with annotation materials. This supports direct RGB colors, greyscale values, and textures, but it can be expensive in very dense environments.

Built-in `InstanceSegmentation` and `Infrared` use the source-stencil backend. Optional custom annotation layers are only created when they are listed in `settings.json`.
An annotation layer allows the user to tag individual actors and/or their child-components with a certain annotation value. This can be used to create ground truth data for machine learning models or to create a visual representation of the environment.

Let's say you want to train a model to detect cars or pedestrians, you create an RGB annotation layer where  you can tag all the cars and pedestrians in the environment with a certain RGB color respectively.
Through the API you can then get the image of this RGB annotation layer (GPU LiDAR is also supported next to cameras).
Or you want to assign a ripeness value to all the apples in your environment, you can create a greyscale annotation layer where you can tag all the apples with a certain greyscale value between 0 and 1.
Similarly, you can also load a texture to a specific mesh component only visible in the annotation layer. For example when trying to show where defects are in a mesh.
The annotation system uses actor and/or component tags to set these values for the 3 modes (greyscale, RGB, texture). You can add these manually or use the APIs (RPC API, Unreal Blueprint, Unreal c++).

## Limitations
* Source-stencil annotation labels are 8-bit values (`0..255`). Use this backend for class IDs, compact instance IDs, or EVN-style masks where 256 labels are enough. It does not support arbitrary direct RGB colors or textures.
* Proxy RGB index layers can still use the larger AirSim RGB colormap, but they create proxy annotation components and should be used with a practical `ProxyComponentBudget`.
* Custom annotation layers support mesh components, including regular static meshes, skeletal meshes, and instanced static mesh components. Built-in instance segmentation and infrared additionally support Unreal Landscape components because their indexed startup path gathers landscapes separately.
  * For instanced static mesh components, one source component receives one annotation ID/color; per-instance IDs inside the same component are not supported yet.
  * For landscapes in built-in segmentation/infrared, each `ULandscapeComponent` is listed separately, while the owning `ALandscapeProxy` name is used as a shared label key for grouped ID updates.
  * UE foliage painted as instanced static mesh foliage is covered by source-stencil indexed paths because `UFoliageInstancedStaticMeshComponent` derives from `UInstancedStaticMeshComponent`. It still receives one ID/color per foliage component, not per individual tree/grass instance.
  * Brush objects aren't supported. In Unreal Engine 5.5, `UBrushComponent` derives from `UPrimitiveComponent`, not `UMeshComponent`, so brushes are skipped by the current object discovery path. As a work-around, convert them to StaticMesh assets.
  * Other unsupported primitive types, such as decals, text, non-instanced foliage systems, and non-mesh custom primitives, generally will not render in annotation captures unless they are added to a supported backend.

## Usage

### Settings JSON definition of layers
To use custom annotation layers, define them in the `Annotation` array in `settings.json`. You can define as many as you want and use them simultaneously. Each layer is addressed by its `Name`.
Here you define each layer with a name, the type and some other settings, often specific to the type.
For example:
```json
{
  ...
    "Annotation": [
    {
        "Name": "RGBTestDirect",
        "Type": 0,
        "Default": true,
        "SetDirect": true,
        "Backend": "Proxy",
        "ViewDistance": 10
    },
    {
        "Name": "RGBTestIndex",
        "Type": 0,
        "Default": false,
        "SetDirect": false,
        "Backend": "SourceStencil",
        "ProxyComponentBudget": 0
    },
    {
        "Name": "GreyscaleTest",
        "Type": 1,
        "Default": true,
        "Backend": "Proxy",
        "ProxyComponentBudget": 5000,
        "ViewDistance": 5
    },
    {
        "Name": "TextureTestDirect",
        "Type": 2,
        "Default": true,
        "SetDirect": true,
        "Backend": "Proxy",
        "ProxyComponentBudget": 5000
    },
    {
        "Name": "TextureTestRelativePath",
        "Type": 2,
        "Default": false,
        "SetDirect": false,
        "TexturePath": "/Game/AnnotationTest",
        "TexturePrefix": "Test1"
    }
    ],
  ...
}
```
The types are:
```cpp
  RGB = 0,
  Greyscale = 1,
  Texture = 2
```

Common settings:

* `Default` applies to all types and controls what happens when no tag is set for an actor/component. If omitted, it defaults to `false`. When set to `false`, the mesh is not rendered in the custom annotation layer. When set to `true`, the mesh is rendered with the default value of the layer.
* `Backend` chooses the render backend. Supported values are `Auto`, `SourceStencil`, and `Proxy`. If omitted, it defaults to `Auto`. `SourceStencil` is supported for RGB index layers (`Type: 0`, `SetDirect: false`) and for built-in segmentation/infrared. Direct RGB, greyscale, and texture layers fall back to `Proxy`.
* `ProxyComponentBudget` limits how many proxy annotation components one layer may create. If omitted, it defaults to `5000`. Use `0` to prevent proxy creation for that layer, or `-1` for no budget limit.

The `ViewDistance` setting applies to all types and allows you to set the maximum distance in meters at which the annotation layer is rendered.
This only applies to the camera sensor output as for LiDAR you can set the maximum range distance of the sensor differently.
This value is by default set to -1 which means infinite draw distance.

### Type 0: RGB
Similar to [instance segmentation](instance_segmentation.md), you can use the RGB annotation layer to tag objects in the environment with an RGB label.
You can do this by directly setting the color yourself (direct mode), or by assigning the object an index that will be linked to the colormap.
To use direct mode, set the settings of this layer with `SetDirect` to `true`. For index mode, set to `false`.
Actor/component tags have the following format: `annotationName_R_G_B` for direct mode or `annotationName_ID` for index mode.
So if for example your RGB annotation layer is called `RGBTestDirect`, you can tag an actor with the tag `RGBTestDirect_255_0_0` to give it a red color.
Or for index mode, `RGBTest_5` to give it the fifth color in the colormap.

RGB direct mode uses the proxy backend. RGB index mode can use either backend:

* `Backend: "SourceStencil"` is the lightweight option. It labels source primitives directly and supports IDs `0..255`.
* `Backend: "Proxy"` keeps the larger RGB colormap behavior, but creates proxy annotation components and is controlled by `ProxyComponentBudget`.

When `Default` is set to `true`, all objects without a tag for this layer will be rendered with the layer default. When `Default` is omitted or set to `false`, untagged objects are not included in this custom annotation layer.

The instance segmentation API function to get the colormap also applies to the RGB index mode. For example in Python you can use:
```python
colorMap = client.simGetSegmentationColorMap()
```

Several RPC API functions are available to influence or retrieve the RGB annotation layer.  Currently, it is not possible to use the RPC API to add new actors or components to the annotation system, you can only update their values. For example in Python:

* `simSetAnnotationObjectID(annotation_name, mesh_name, object_id, is_name_regex=False/True)` to update the color of an object in index mode (regex allows to set multiple with wildcards for example) when it already exists in the annotation system
* `simSetAnnotationObjectColor(annotation_name, mesh_name, r, g, b, is_name_regex=False/True)` to update the color of an object in direct mode  (regex allows to set multiple with wildcards for example) when it already exists in the annotation system
* `simGetAnnotationObjectID(annotation_name, mesh_name)` to get the ID of an object in index mode
* `simGetAnnotationObjectColor(annotation_name, mesh_name)` to get the color of an object in direct mode
* `simIsValidColor(r,g,b)` You can check if a color is valid using this function

The same is available in Unreal Blueprint and Unreal c++. You can find  the functions in the `Annotation` category.

* `Add RGBDirect Annotation Tag to Component/Actor(annotation_name, component/actor, color, update_annotation=true/false)` to set the color of an object in direct mode
* `Update RGBDirect Annotation Tag to Component/Actor(annotation_name, component/actor, color, update_annotation=true/false)` to update the color of an object in direct mode already in the system
* `Add RGBIndex Annotation Tag to Component/Actor(annotation_name, component/actor, object_id, update_annotation=true/false)` to set the index of an object in index mode
* `Update RGBIndex Annotation Tag to Component/Actor(annotation_name, component/actor, object_id, update_annotation=true/false)` to update the index of an object in index mode already in the system
* `Is Annotation RGB Valid(color)`You can check if a color is valid using this function

Note that enabling _update_annotation_ is a relatively slow process, specially on actors with lots of annotated components.
Ideally set _update_annotation_ to false during the process of adding tags to the actor and only turn on update_annotation for the last component or actor you want to update.
Alternatively, you can use the `Add New Actor To Annotation()` blueprint function to update the annotation layer for this actor after you have added all tags.

### Type 1: Greyscale
You can use the greyscale annotation layer to tag objects in the environment with a float value between 0 and 1. Note that this has the precision of uint8.
Actor/component tags have the following format: `annotationName_value`.
So if for example your RGB annotation layer is called `GreyscaleTest`, you can tag an actor with the tag `GreyscaleTest_0.76` to give it a value of 0.76 which would result in a color of (194, 194, 194).

Greyscale annotation uses the proxy backend. When `Default` is set to `true`, all objects without a tag for this layer will be rendered in black. When `Default` is omitted or set to `false`, untagged objects are not included in this custom annotation layer.

Several RPC API functions are available to influence or retrieve the RGB annotation layer. Currently, it is not possible to use the RPC API to add new actors or components to the annotation system, you can only update their values. For example in Python:

* `simSetAnnotationObjectValue(annotation_name, mesh_name, greyscale_value, is_name_regex=False/True)` to update the value of an object (regex allows to set multiple with wildcards for example) when it already exists in the annotation system
* `simGetAnnotationObjectValue(annotation_name, mesh_name)` to get the value of an object

The same is available in Unreal Blueprint and Unreal c++. You can find  the functions in the `Annotation` category.

* `Add Greyscale Annotation Tag to Component/Actor(annotation_name, component/actor, value, update_annotation=true/false)` to update the value of an object when it already exists in the annotation system
* `Update Greyscale Annotation Tag to Component/Actor(annotation_name, component/actor, value, update_annotation=true/false)` to update the value of an object

Note that enabling _update_annotation_ is a relatively slow process, specially on actors with lots of annotated components.
Ideally set _update_annotation_ to false during the process of adding tags to the actor and only turn on update_annotation for the last component or actor you want to update.
Alternatively, you can use the `Add New Actor To Annotation()` blueprint function to update the annotation layer for this actor after you have added all tags.

### Type 2: Texture
You can use the texture annotation layer to tag objects in the environment with a specific texture. This can be a color or greyscale texture, or you can mix them. Choice is up to you.
You can do this by directly setting the texture yourself (direct mode), or by assigning a texture that is loaded based on a set path and the name of the mesh.
To use direct mode, set the settings of this layer with `SetDirect` to `true`. For path reference mode, set to `false`.

Actor/component tags have the following format: `annotationName_texturepath` for direct mode.
The Unreal texture path name has to be rather specific:
 - If your texture is in the environment content folder, you must add `/Game/` in front of the path.
 - If it is in the Cosys-AirSim plugin content folder, you must add `/AirSim/` in front of the path.
 - For Engine textures, you must add `/Engine/` in front of the path.
So if for example your texture annotation layer is called `TextureTestDirect`, and your texture *TestTexture* is in the game content folder under a subfolder *AnnotationTest* you can tag an actor with the tag `TextureTest_/Game/AnnotationTest/TestTexture` to give it this texture.

For path reference mod, the content of the tag is not really important as long as it contains the name of the annotation layer and an underscore, for example `annotationName_enable`.
What is important is in reference mode is that you have a texture in the content folder with the name of the mesh if you do enable this object by setting a tag.
You must place your textures in the folder defined by the `TexturePath` setting in the settings.json file for this layer. And the texture must have the same name as the mesh and start with the prefix set by the `TexturePrefix` setting in the settings.json file for this layer followed by a hyphen.
So for example if you have a static mesh called *Cylinder* and your texture layer is called `TextureTestDirect` with the settings `TexturePath` set to `/Game/AnnotationTest` and `TexturePrefix` set to `Test1`, you must have a texture called `Test1-Cylinder` in the folder `/Game/AnnotationTest`.

Texture annotation uses the proxy backend. When `Default` is set to `true`, all objects without a tag for this layer will be rendered with the layer default. When `Default` is omitted or set to `false`, untagged objects are not included in this custom annotation layer.

Several RPC API functions are available to influence or retrieve the RGB annotation layer.  Currently, it is not possible to use the RPC API to add new actors or components to the annotation system, you can only update their values. For example in Python:

* `simSetAnnotationObjectTextureByPath(annotation_name, mesh_name, texture_path, is_name_regex=False/True)` to set the texture of an object in direct mode, the texture path should be same format as described above, for example `/Game/MyTextures/TestTexture1` (regex allows to set multiple with wildcards for example)
* `simEnableAnnotationObjectTextureByPath(annotation_name, mesh_name, is_name_regex=False/True)` to enable the texture of an object in relative path mode, this does require a texture in the relative path as described above!  (regex allows to set multiple with wildcards for example)
* `simGetAnnotationObjectTexturePath(annotation_name, mesh_name)` to get the texture path of an object

The same is available in Unreal Blueprint and Unreal c++. You can find  the functions in the `Annotation` category.

* `Add Texture Direct Annotation Tag to Component/Actor By Path(annotation_name, component/actor, texture_path, update_annotation=true/false)` to set the texture of an object in direct mode, the texture path should be same format as described above, for example `/Game/MyTextures/TestTexture1`
* `Update Texture Direct Annotation Tag to Component/Actor By Path(annotation_name, component/actor, texture_path, update_annotation=true/false)` to update texture of an object in direct mode that is already in the system, the texture path should be same format as described above, for example `/Game/MyTextures/TestTexture1`
* `Add Texture Direct Annotation Tag to Component/Actor(annotation_name, component/actor, texture, update_annotation=true/false)` to set the texture of an object in direct mode, the texture can be directly referenced as UTexture* Object
* `Update Texture Direct Annotation Tag to Component/Actor(annotation_name, component/actor, texture, update_annotation=true/false)` to update texture of an object in direct mode that is already in the system, the texture can be directly referenced as UTexture* Object
* `Enable Texture By Path Annotation Tag to Component/Actor(annotation_name, component/actor, update_annotation=true/false)` to enable the texture of an object in relative path mode, this does require a texture in the relative path as described above!

Note that enabling _update_annotation_ is a relatively slow process, specially on actors with lots of annotated components.
Ideally set _update_annotation_ to false during the process of adding tags to the actor and only turn on update_annotation for the last component or actor you want to update.
Alternatively, you can use the `Add New Actor To Annotation()` blueprint function to update the annotation layer for this actor after you have added all tags.

### Common functionality
When the world loads, only the custom annotation layers listed in `settings.json` are initialized. For each configured layer, meshes are checked for matching tags. Untagged meshes are only added when that layer explicitly sets `"Default": true`.
With the unreal blueprint and c++ functions however, you can also decide to update the annotation layer only when you want to with the `update_annotation` argument.
If you have many objects to update, this can save a lot of time by doing it only for the last object.

Some API functions exist for all types, for example in Python:

* `simListAnnotationObjects(annotation_name)` to get a list of all objects within this annotation layer.
* `simListAnnotationPoses(annotation_name, ned=True/False, only_visible=False/True)` to get the 3D poses of all objects in this annotation layer. The returned pose is in NED coordinates in SI units with its origin at Player Start by default or in Unreal NED frame if the `ned` boolean argument is set to `talse`.

Similarly, for Unreal Blueprint and Unreal c++. You can find  the functions in the `Annotation` category.

* `Does Annotation Layer Exist(annotation_name)` to figure out if a layer exists or not
* `Add New Actor To Annotation(annotation_name, actor, update_annotation=true/false)` if you manually added a tag, and want to update the annotation layer with this actor. This is useful to run after adding multiple tags to the actor and its components with the other api calls, and you want to update the annotation layer only once, otherwise it will be much slower.
* `Delete Actor From Annotation(annotation_name, actor, update_annotation=true/false)` if you manually remove all tags from an actor for this layer and remove it from the annotation layer
* `Force Update Annotation(annotation_name)` to force an update of the annotation layer.

### Getting annotation data from sensors
The easiest way to get the images from annotation cameras, is through the image API. See the [Image API documentation](image_apis.md#annotation) for more information.
GPU LiDAR is also supported, but each GPU Lidar can only render one annotation layer. See the [GPU LiDAR documentation](gpulidar.md) for more information.

You can also display the annotation layers in the subwindows. See the [Settings documentation](settings.md#subwindows) for more information.
For example:
```json
{
  ...
    "SubWindows": [
        {
            "WindowID": 0,
            "CameraName": "front_center",
            "ImageType": 11,
            "VehicleName": "robot1",
            "Annotation": "GreyscaleTest",
            "Visible": false
        },
  ...
```
## Credits
The proxy annotation method is a derivative of and inspired by the work of [UnrealCV](https://unrealcv.org/). Their work is licensed under the MIT License.
It is made by students from Johns Hopkins University and Peking University under the supervision of Prof. Alan Yuille and Prof. Yizhou Wang.
You can read the paper on their work [here](https://dl.acm.org/doi/10.1145/3123266.3129396).
