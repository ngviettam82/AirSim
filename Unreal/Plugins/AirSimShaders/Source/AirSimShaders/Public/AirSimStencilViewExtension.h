#pragma once

#include "CoreMinimal.h"

class ISceneViewExtension;

namespace AirSimStencilViewExtension
{
    enum class EOutputMode : uint8
    {
        Segmentation,
        Infrared
    };

    AIRSIMSHADERS_API TSharedPtr<ISceneViewExtension, ESPMode::ThreadSafe> Create(EOutputMode output_mode);
}
