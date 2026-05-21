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
        airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False),
        airsim.ImageRequest("front_center", airsim.ImageType.Segmentation, False, False),
    ],
    vehicle_name="drone_1",
)
```

`simGetImage()` also works for compressed non-float image types. Float depth output requires `simGetImages()` with `pixels_as_float=True`.

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

When a camera image type is configured as equirectangular, `simGetCameraInfo` still returns the camera pose. The projection matrix is returned as `NaN` values because a 360 degree equirectangular image has no single perspective projection matrix.

## Platform Notes

The validated runtime target is Windows with D3D11 or D3D12. UE 5.5 Vulkan has known cube-face readback limitations in this path, so equirectangular capture reports an error on Vulkan instead of treating it as supported.

## Validation And Debug Tools

Useful local tools:

```powershell
python tools\validate_equirectangular_projection.py
python tools\validate_equirectangular_runtime.py --vehicle drone_1 --equirectangular-camera front_center --ref-camera front_center_ref --expected-height 720
python tools\capture_equirectangular_three_types.py --vehicle drone_1 --camera front_center
python tools\stream_camera_fps.py --vehicle drone_1 --camera front_center --image-type Scene --display-width 1440 --display-height 720
```

The runtime validation script expects one configured equirectangular camera and one normal reference camera. The capture script writes `Scene`, `DepthPerspective`, and `Segmentation` artifacts plus a summary JSON file under `Documents/AirSim`.
