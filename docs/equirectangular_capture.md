# Equirectangular Captures

Cosys-AirSim can capture existing image types as 360 degree equirectangular images by setting `ProjectionMode` to `"Equirectangular"` in `settings.json`. This is a projection mode, not a new `ImageType`; the image request still uses the normal image type values such as `Scene`, `DepthPerspective`, `Segmentation`, `Infrared`, `Lighting`, or `Annotation`.

## Settings

Add the projection mode to the `CaptureSettings` entry for each image type that should be returned as an equirectangular image:

```json
{
  "CameraDefaults": {
    "CaptureSettings": [
      {
        "ImageType": 0,
        "Height": 720,
        "ProjectionMode": "Equirectangular"
      },
      {
        "ImageType": 2,
        "Height": 720,
        "ProjectionMode": "Equirectangular",
        "MaxDepthMeters": 20.0
      },
      {
        "ImageType": 5,
        "Height": 720,
        "ProjectionMode": "Equirectangular"
      }
    ]
  }
}
```

For equirectangular captures:

- `Height` is the cube-face size and the output image height.
- Output width is always `2 * Height`.
- `Width`, `FOV_Degrees`, and `OrthoWidth` are ignored.
- The same `simGetImage` and `simGetImages` requests are used; the response projection is selected by settings.

For example, `Height: 720` returns `1440x720`. To capture scene, depth, and segmentation from the same configured camera:

```python
import cosysairsim as airsim

client = airsim.MultirotorClient()
client.confirmConnection()

responses = client.simGetImages(
    [
        airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False, float_as_bytes=True),
        airsim.ImageRequest("front_center", airsim.ImageType.Segmentation, False, False),
    ],
    vehicle_name="drone_1",
)
```

`simGetImage()` also works for compressed non-float image types. Float depth output requires `simGetImages()` with `pixels_as_float=True`.

For exact depth, request `pixels_as_float=True, float_as_bytes=True`. This returns the same exact `float32` depth values as little-endian raw bytes in `image_data_uint8` and leaves `image_data_float` empty:

```python
depth_response = responses[1]
depth = airsim.response_to_2d_float_array(depth_response)
```

## Image Types

Equirectangular capture is supported through the existing image types. It does not introduce a separate image type.

Continuous visual outputs use bilinear sampling during cube-to-equirectangular unwrap:

- `Scene`
- `DepthVis`
- `SurfaceNormals`
- `Lighting`
- `OpticalFlowVis`

Discrete or metric outputs use nearest sampling to preserve exact labels and numeric values:

- `DepthPlanar`
- `DepthPerspective`
- `Segmentation`
- `Infrared`
- `Annotation`
- `DisparityNormalized`
- `OpticalFlow`

`DepthPerspective` is the recommended 360 depth image because each pixel stores distance along that pixel's viewing ray. `DepthPlanar` is still available, but it remains face-local planar depth; a single global planar camera plane does not exist for a full 360 degree equirectangular image.

`MaxDepthMeters` can be set on `DepthPlanar` and `DepthPerspective` capture settings. A positive value clamps returned float depth pixels for both normal and equirectangular projections. Omitting it leaves the depth output unchanged.

## Scene And Lighting Exposure

Unreal's eye adaptation, local exposure, bloom, depth of field, and some other view-local post effects are evaluated per rendered view. If those effects are applied independently to six cube faces, seams can appear after unwrap.

For equirectangular `Scene` and `Lighting`, Cosys-AirSim captures the six cube faces as HDR scene color, unwraps them, then applies one global exposure and tonemap to the full equirectangular image. This is why these captures intentionally do not use the exact same per-face post-processing path as normal perspective or orthographic captures.

The global equirectangular exposure can be adjusted with:

- `EquirectangularExposureCompensation`: log2 compensation. `1` is 2x brighter, `-1` is 2x darker. If omitted, `AutoExposureCompensation` is used.
- `EquirectangularExposureMin`: minimum global exposure multiplier. Default is `0.02`.
- `EquirectangularExposureMax`: maximum global exposure multiplier. Default is `20.0`.

Normal perspective and orthographic captures continue to use their existing Unreal camera/post-process path.

## Camera Info

When the scene image type for a camera is configured as equirectangular, `simGetCameraInfo` still returns the camera pose. The projection matrix is returned as `NaN` values because a 360 degree equirectangular image has no single perspective projection matrix.

## Subwindow Preview

Unreal subwindows can display equirectangular capture settings. Normal perspective and orthographic subwindows use the existing 2D render target. Equirectangular subwindows use a GPU preview target generated from the cube capture. The HUD sizes each subwindow from the actual render target attached to it, so normal subwindows keep their configured 2D camera aspect ratio and equirectangular subwindows display as a 2:1 view without routing through `simGetImage` or `simGetImages`.

The subwindow output is a visual preview. Exact float depth values, exact segmentation IDs, exact annotation labels, and exact infrared values remain the responsibility of the image APIs. Depth subwindows are displayed as a range-mapped grayscale preview using `MaxDepthMeters` when it is set, or 100 meters when it is omitted.

## Platform Notes

The validated production baseline is UE 5.5 on Windows with D3D11 or D3D12. UE 5.5 Vulkan has known cube-face readback limitations in this path, so equirectangular capture reports an error on Vulkan instead of treating it as supported. UE 5.7 support remains provisional until the migration validation matrix passes.

## Operational Notes

Each equirectangular image type owns a cube render target. High resolutions can therefore consume much more GPU memory than a normal perspective camera, especially when several equirectangular image types are enabled on the same camera. If memory pressure is a concern, configure only the image types needed for that run or split captures across separate camera/settings profiles.

When copying Cosys-AirSim into another Unreal project, copy the complete `Unreal/Plugins` plugin set. Equirectangular subwindow previews depend on the `AirSimShaders` plugin in addition to the main `AirSim` plugin.

Exact `DepthPlanar` and `DepthPerspective` responses should use `float_as_bytes=True` for high-resolution captures. This keeps the same `float32` values while avoiding the very slow RPC path that serializes one float object per pixel.

Annotation output depends on the configured annotation layer. Untagged objects may share the layer default color, so an all-black annotation image usually means the requested layer exists but no object has a non-default annotation value for that layer.
