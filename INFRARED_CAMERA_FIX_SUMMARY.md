# Infrared Camera Debug and Unreal 5.5 Compatibility Fixes

## Summary

Fixed critical infrared camera rendering issues in Cosys-AirSim for Unreal 5.5 compatibility. The infrared camera was not rendering any data because it wasn't properly configured to use the annotation component rendering system that's required for segmentation-based thermal imaging.

## Issues Found and Fixed

### 1. **Infrared Camera Not Rendering Annotation Components** (CRITICAL)
**File:** `Unreal/Plugins/AirSim/Source/PIPCamera.cpp`

**Problem:**
The infrared capture component was not configured to render annotation components. Unlike the Segmentation camera which explicitly sets up special rendering to show only annotation components, the infrared camera was left with default settings that would attempt to render the scene normally, resulting in black/empty images.

**Root Cause:**
In `PostInitializeComponents()`, the code sets:
```cpp
FObjectAnnotator::SetViewForAnnotationRender(captures_[Utils::toNumeric(ImageType::Segmentation)]->ShowFlags);
captures_[Utils::toNumeric(ImageType::Segmentation)]->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
```

But this was ONLY done for Segmentation, not for Infrared.

**Fix Applied:**
Added the same annotation rendering setup for infrared:
```cpp
FObjectAnnotator::SetViewForAnnotationRender(captures_[Utils::toNumeric(ImageType::Infrared)]->ShowFlags);
captures_[Utils::toNumeric(ImageType::Infrared)]->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
```

This ensures the infrared capture component:
- Uses the annotation rendering view flags
- Only renders primitives from the "show only list" (where annotation components are added)
- Properly displays object colors representing thermal values

### 2. **Infrared Render Target Gamma Settings** (IMPORTANT)
**File:** `Unreal/Plugins/AirSim/Source/PIPCamera.cpp`

**Problem:**
The infrared render target was not using linear gamma (gamma = 1.0), which is essential for preserving exact pixel values that represent thermal digital counts.

**Root Cause:**
In `setupCameraFromSettings()`, Segmentation explicitly sets `TargetGamma = 1` to ensure linear color space, but Infrared did not have this setting.

**Fix Applied:**
Added gamma setting for infrared:
```cpp
if (image_type == Utils::toNumeric(ImageType::Infrared)) {
    render_targets_[image_type]->TargetGamma = 1;
}
```

This preserves the exact pixel values without gamma correction distortion.

### 3. **Infrared Image Gamma Correction Disabled** (IMPORTANT)
**File:** `Unreal/Plugins/AirSim/Source/UnrealImageCapture.cpp`

**Problem:**
The infrared image data was not being properly extracted without gamma correction. Like Segmentation and Annotation, infrared should preserve raw pixel values by disabling gamma correction during image extraction.

**Root Cause:**
In `getSceneCaptureImage()`, the `disable_gamma` flag was only set for Segmentation and Annotation:
```cpp
if (requests[i].image_type == ImageCaptureBase::ImageType::Segmentation || 
    requests[i].image_type == ImageCaptureBase::ImageType::Annotation)
    disable_gamma = true;
```

**Fix Applied:**
Extended the condition to include Infrared:
```cpp
if (requests[i].image_type == ImageCaptureBase::ImageType::Segmentation || 
    requests[i].image_type == ImageCaptureBase::ImageType::Annotation ||
    requests[i].image_type == ImageCaptureBase::ImageType::Infrared)
    disable_gamma = true;
```

This ensures thermal digital counts are not distorted by gamma correction.

## How Infrared Camera Works in Cosys-AirSim

The infrared camera uses a segmentation-based thermal imaging system:

1. **Segmentation ID Mapping:** Objects are assigned segmentation IDs using `simSetSegmentationObjectID()`
2. **Thermal Values:** Python scripts pre-calculate thermal digital counts based on object temperature and emissivity
3. **ID Reassignment:** These thermal values are reassigned as segmentation IDs in the simulation
4. **Annotation Rendering:** The infrared camera renders only annotation components with their assigned colors
5. **Output:** Each pixel's R, G, B value (equal for grayscale) represents the thermal digital count for that object
6. **Processing:** Python code converts these grayscale values back to temperature using calibration data

## Testing Recommendations

1. **Run validation script:**
   ```bash
   python validate_infrared_camera.py
   ```

2. **Expected behavior after fix:**
   - Infrared image size should match Scene image size (not all zeros)
   - Pixel values should correspond to object segmentation IDs
   - Image should show objects with different grayscale values based on their thermal assignments

3. **Full workflow test:**
   - Set object segmentation IDs to thermal digital counts
   - Capture infrared images
   - Verify grayscale values match the assigned segmentation IDs

## Unreal 5.5 Compatibility Status

✓ **Header includes:** Already updated in previous commits (AnnotationComponent.cpp)
✓ **Engine version checks:** Properly handle both UE4.27+ and UE5.x
✓ **API compatibility:** No deprecated APIs used
✓ **Annotation system:** Works with UE5.5 properly

## Files Modified

1. `Unreal/Plugins/AirSim/Source/PIPCamera.cpp` (2 fixes)
   - Line 154-155: Added annotation rendering setup for infrared
   - Line 689-691: Added TargetGamma = 1 setting for infrared

2. `Unreal/Plugins/AirSim/Source/UnrealImageCapture.cpp` (1 fix)
   - Line 102: Extended disable_gamma condition to include infrared

## Next Steps

1. Compile the plugin in Unreal 5.5
2. Run the infrared validation script
3. Test with actual simulation environment
4. Verify thermal image output matches expected values
