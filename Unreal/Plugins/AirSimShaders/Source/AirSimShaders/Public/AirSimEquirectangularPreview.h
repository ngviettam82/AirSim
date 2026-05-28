#pragma once

#include "CoreMinimal.h"

class UTextureRenderTarget2D;
class UTextureRenderTargetCube;

namespace AirSimEquirectangularPreview
{
    enum class EPreviewMode : uint8
    {
        ColorNearest = 0,
        ColorBilinear = 1,
        SceneColor = 2,
        DepthMeters = 3
    };

    AIRSIMSHADERS_API void Draw(
        UTextureRenderTargetCube* source_cube,
        UTextureRenderTarget2D* output_target,
        EPreviewMode mode,
        float max_depth_meters,
        float exposure_compensation);
}
