#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureRenderTargetCube.h"
#include "DynamicRHI.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"

#include "AirBlueprintLib.h"
#include "Async/Async.h"

namespace
{
    using ImageType = msr::airlib::ImageCaptureBase::ImageType;

    struct FCubeFaceBasis
    {
        ECubeFace Face;
        FVector Dir;
        FVector Up;
        FVector Right;
    };

    struct FEquirectangularCubeSample
    {
        ECubeFace Face;
        float PixelX;
        float PixelY;
    };

    struct FEquirectangularSamplingMap
    {
        int32 CubeSize = 0;
        int32 OutputWidth = 0;
        int32 OutputHeight = 0;
        // The mapping depends only on cube size, not camera pose or image data.
        // It is cached so each frame reuses the same face/texel references.
        TArray<uint8> NearestFaces;
        TArray<int32> NearestPixelIndices;
        TArray<uint8> BilinearFaces;
        TArray<int32> BilinearPixelIndices;
        TArray<float> BilinearWeights;
    };

    const FCubeFaceBasis& GetCubeFaceBasis(ECubeFace face)
    {
        // Matches UE's CalcCubeFaceTransform: right = up ^ dir.
        static const FCubeFaceBasis bases[6] = {
            { CubeFace_PosX, FVector(1, 0, 0), FVector(0, 1, 0), FVector(0, 0, -1) },
            { CubeFace_NegX, FVector(-1, 0, 0), FVector(0, 1, 0), FVector(0, 0, 1) },
            { CubeFace_PosY, FVector(0, 1, 0), FVector(0, 0, -1), FVector(1, 0, 0) },
            { CubeFace_NegY, FVector(0, -1, 0), FVector(0, 0, 1), FVector(1, 0, 0) },
            { CubeFace_PosZ, FVector(0, 0, 1), FVector(0, 1, 0), FVector(1, 0, 0) },
            { CubeFace_NegZ, FVector(0, 0, -1), FVector(0, 1, 0), FVector(-1, 0, 0) }
        };
        return bases[FMath::Clamp<int32>(static_cast<int32>(face), 0, 5)];
    }

    bool UseBilinearEquirectangularSampling(ImageType image_type)
    {
        switch (image_type) {
        case ImageType::Scene:
        case ImageType::DepthVis:
        case ImageType::SurfaceNormals:
        case ImageType::Lighting:
        case ImageType::OpticalFlowVis:
            return true;
        default:
            return false;
        }
    }

    bool UseGlobalEquirectangularScenePipeline(ImageType image_type)
    {
        return image_type == ImageType::Scene ||
               image_type == ImageType::Lighting;
    }

    void CaptureSceneForRequest(const RenderRequest::RenderParams* params)
    {
        if (params == nullptr) {
            return;
        }

        const bool capture_immediately =
            params->image_type == ImageType::Segmentation ||
            params->image_type == ImageType::Infrared;

        if (params->isEquirectangular() &&
            params->render_target_cube != nullptr &&
            params->render_component_cube != nullptr) {
            if (capture_immediately) {
                params->render_component_cube->CaptureScene();
            }
            else {
                params->render_component_cube->CaptureSceneDeferred();
            }
        }
        else if (params->render_target != nullptr && params->render_component != nullptr) {
            if (capture_immediately) {
                params->render_component->CaptureScene();
            }
            else {
                params->render_component->CaptureSceneDeferred();
            }
        }
    }

    bool UsesSameCaptureTarget(
        const RenderRequest::RenderParams* lhs,
        const RenderRequest::RenderParams* rhs)
    {
        if (lhs == nullptr || rhs == nullptr || lhs->isEquirectangular() != rhs->isEquirectangular()) {
            return false;
        }

        if (lhs->isEquirectangular()) {
            return lhs->render_component_cube == rhs->render_component_cube &&
                   lhs->render_target_cube == rhs->render_target_cube;
        }

        return lhs->render_component == rhs->render_component &&
               lhs->render_target == rhs->render_target;
    }

    void CaptureScenesForRequests(
        std::shared_ptr<RenderRequest::RenderParams> params[],
        unsigned int request_count)
    {
        for (unsigned int index = 0; index < request_count; ++index) {
            bool already_captured = false;
            for (unsigned int previous_index = 0; previous_index < index; ++previous_index) {
                if (UsesSameCaptureTarget(params[index].get(), params[previous_index].get())) {
                    already_captured = true;
                    break;
                }
            }

            if (!already_captured) {
                CaptureSceneForRequest(params[index].get());
            }
        }
    }

    bool IsDepthMetersImage(ImageType image_type)
    {
        return image_type == ImageType::DepthPlanar ||
               image_type == ImageType::DepthPerspective;
    }

    bool ShouldPackFloatDepthAsBytes(const RenderRequest::RenderParams* params)
    {
        // Opt-in float depth bytes avoid serializing one msgpack float
        // object per pixel through RPC.
        return params != nullptr &&
               params->pixels_as_float &&
               params->float_as_bytes &&
               IsDepthMetersImage(params->image_type);
    }

    float ApplyMaxDepthMeters(float value, const RenderRequest::RenderParams* params)
    {
        if (params != nullptr &&
            params->max_depth_meters > 0.0f &&
            IsDepthMetersImage(params->image_type) &&
            FMath::IsFinite(value)) {
            return FMath::Min(value, params->max_depth_meters);
        }
        return value;
    }

    bool IsCubeFaceReadbackSupported()
    {
        if (GDynamicRHI == nullptr) {
            return true;
        }

        const TCHAR* rhi_name = GDynamicRHI->GetName();
        return rhi_name == nullptr || FCString::Strifind(rhi_name, TEXT("Vulkan")) == nullptr;
    }

    FVector EquirectangularPixelToDirection(int32 x, int32 y, int32 width, int32 height)
    {
        const double u = (static_cast<double>(x) + 0.5) / static_cast<double>(width);
        const double v = (static_cast<double>(y) + 0.5) / static_cast<double>(height);
        const double yaw = (u - 0.5) * 2.0 * PI;
        const double pitch = (0.5 - v) * PI;
        const double cos_pitch = FMath::Cos(pitch);

        return FVector(
            cos_pitch * FMath::Cos(yaw),
            cos_pitch * FMath::Sin(yaw),
            FMath::Sin(pitch));
    }

    FEquirectangularCubeSample DirectionToCubeSample(const FVector& direction, int32 cube_size)
    {
        ECubeFace best_face = CubeFace_PosX;
        double best_dot = -2.0;

        for (int32 face_index = 0; face_index < 6; ++face_index) {
            const FCubeFaceBasis& basis = GetCubeFaceBasis(static_cast<ECubeFace>(face_index));
            const double face_dot = FVector::DotProduct(direction, basis.Dir);
            if (face_dot > best_dot) {
                best_dot = face_dot;
                best_face = basis.Face;
            }
        }

        const FCubeFaceBasis& basis = GetCubeFaceBasis(best_face);
        const double denom = FMath::Max(best_dot, static_cast<double>(KINDA_SMALL_NUMBER));
        const double s = FVector::DotProduct(direction, basis.Right) / denom;
        const double t = FVector::DotProduct(direction, basis.Up) / denom;

        FEquirectangularCubeSample sample;
        sample.Face = best_face;
        sample.PixelX = static_cast<float>(((s + 1.0) * 0.5) * cube_size - 0.5);
        sample.PixelY = static_cast<float>(((1.0 - t) * 0.5) * cube_size - 0.5);
        return sample;
    }

    FVector FaceTexelToDirection(ECubeFace face, int32 texel_x, int32 texel_y, int32 cube_size)
    {
        const FCubeFaceBasis& basis = GetCubeFaceBasis(face);
        const double u = (static_cast<double>(texel_x) + 0.5) / static_cast<double>(cube_size);
        const double v = (static_cast<double>(texel_y) + 0.5) / static_cast<double>(cube_size);
        const double s = u * 2.0 - 1.0;
        const double t = 1.0 - v * 2.0;

        return (basis.Dir + basis.Right * s + basis.Up * t).GetSafeNormal();
    }

    int32 FaceIndex(ECubeFace face)
    {
        return FMath::Clamp<int32>(static_cast<int32>(face), 0, 5);
    }

    template <typename PixelType>
    bool HasCubeFacePixels(const TArray<PixelType>* cube_faces, int32 face_index, int32 cube_size)
    {
        const int64 expected_pixel_count = static_cast<int64>(cube_size) * static_cast<int64>(cube_size);
        return cube_faces != nullptr &&
               face_index >= 0 &&
               face_index < 6 &&
               expected_pixel_count > 0 &&
               cube_faces[face_index].Num() >= expected_pixel_count;
    }

    void SetNearestCubePixelRef(const FEquirectangularCubeSample& sample, int32 cube_size, uint8& out_face, int32& out_pixel_index)
    {
        const int32 x = FMath::Clamp(FMath::RoundToInt(sample.PixelX), 0, cube_size - 1);
        const int32 y = FMath::Clamp(FMath::RoundToInt(sample.PixelY), 0, cube_size - 1);
        out_face = static_cast<uint8>(FaceIndex(sample.Face));
        out_pixel_index = y * cube_size + x;
    }

    void SetCubeTapPixelRef(ECubeFace face, int32 x, int32 y, int32 cube_size, uint8& out_face, int32& out_pixel_index)
    {
        if (x >= 0 && x < cube_size && y >= 0 && y < cube_size) {
            out_face = static_cast<uint8>(FaceIndex(face));
            out_pixel_index = y * cube_size + x;
            return;
        }

        const FVector tap_direction = FaceTexelToDirection(face, x, y, cube_size);
        const FEquirectangularCubeSample remapped = DirectionToCubeSample(tap_direction, cube_size);
        SetNearestCubePixelRef(remapped, cube_size, out_face, out_pixel_index);
    }

    void BuildEquirectangularSamplingMap(int32 cube_size, FEquirectangularSamplingMap& map)
    {
        map.CubeSize = cube_size;
        map.OutputWidth = cube_size * 2;
        map.OutputHeight = cube_size;

        const int32 output_pixel_count = map.OutputWidth * map.OutputHeight;
        map.NearestFaces.SetNumUninitialized(output_pixel_count);
        map.NearestPixelIndices.SetNumUninitialized(output_pixel_count);
        map.BilinearFaces.SetNumUninitialized(output_pixel_count * 4);
        map.BilinearPixelIndices.SetNumUninitialized(output_pixel_count * 4);
        map.BilinearWeights.SetNumUninitialized(output_pixel_count * 4);

        for (int32 y = 0; y < map.OutputHeight; ++y) {
            for (int32 x = 0; x < map.OutputWidth; ++x) {
                const int32 output_index = y * map.OutputWidth + x;
                const FVector direction = EquirectangularPixelToDirection(x, y, map.OutputWidth, map.OutputHeight);
                const FEquirectangularCubeSample sample = DirectionToCubeSample(direction, cube_size);
                SetNearestCubePixelRef(
                    sample,
                    cube_size,
                    map.NearestFaces[output_index],
                    map.NearestPixelIndices[output_index]);

                const int32 x0 = FMath::FloorToInt(sample.PixelX);
                const int32 y0 = FMath::FloorToInt(sample.PixelY);
                const float tx = sample.PixelX - x0;
                const float ty = sample.PixelY - y0;
                const float weights[4] = {
                    (1.0f - tx) * (1.0f - ty),
                    tx * (1.0f - ty),
                    (1.0f - tx) * ty,
                    tx * ty
                };
                const int32 tap_x[4] = { x0, x0 + 1, x0, x0 + 1 };
                const int32 tap_y[4] = { y0, y0, y0 + 1, y0 + 1 };
                const int32 tap_base = output_index * 4;
                for (int32 tap = 0; tap < 4; ++tap) {
                    SetCubeTapPixelRef(
                        sample.Face,
                        tap_x[tap],
                        tap_y[tap],
                        cube_size,
                        map.BilinearFaces[tap_base + tap],
                        map.BilinearPixelIndices[tap_base + tap]);
                    map.BilinearWeights[tap_base + tap] = weights[tap];
                }
            }
        }
    }

    const FEquirectangularSamplingMap& GetEquirectangularSamplingMap(int32 cube_size)
    {
        static FCriticalSection sampling_map_lock;
        static TMap<int32, TUniquePtr<FEquirectangularSamplingMap>> sampling_maps;

        FScopeLock lock(&sampling_map_lock);
        TUniquePtr<FEquirectangularSamplingMap>& map = sampling_maps.FindOrAdd(cube_size);
        if (!map.IsValid()) {
            map = MakeUnique<FEquirectangularSamplingMap>();
            BuildEquirectangularSamplingMap(cube_size, *map);
        }
        return *map;
    }

    FColor ReadMappedCubeColor(const TArray<FColor>* cube_faces, const FEquirectangularSamplingMap& map, int32 output_index)
    {
        const int32 face_index = static_cast<int32>(map.NearestFaces[output_index]);
        if (!HasCubeFacePixels(cube_faces, face_index, map.CubeSize)) {
            return FColor::Black;
        }

        FColor color = cube_faces[face_index][map.NearestPixelIndices[output_index]];
        color.A = 255;
        return color;
    }

    FColor SampleMappedCubeColorBilinear(const TArray<FColor>* cube_faces, const FEquirectangularSamplingMap& map, int32 output_index)
    {
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
        const int32 tap_base = output_index * 4;
        for (int32 tap = 0; tap < 4; ++tap) {
            const int32 tap_index = tap_base + tap;
            const int32 face_index = static_cast<int32>(map.BilinearFaces[tap_index]);
            if (!HasCubeFacePixels(cube_faces, face_index, map.CubeSize)) {
                continue;
            }

            const FColor color = cube_faces[face_index][map.BilinearPixelIndices[tap_index]];
            const float weight = map.BilinearWeights[tap_index];
            r += static_cast<float>(color.R) * weight;
            g += static_cast<float>(color.G) * weight;
            b += static_cast<float>(color.B) * weight;
        }

        return FColor(
            static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(r), 0, 255)),
            static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(g), 0, 255)),
            static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(b), 0, 255)),
            255);
    }

    FLinearColor ReadMappedCubeLinear(const TArray<FFloat16Color>* cube_faces, const FEquirectangularSamplingMap& map, int32 face_index, int32 pixel_index)
    {
        if (!HasCubeFacePixels(cube_faces, face_index, map.CubeSize)) {
            return FLinearColor::Black;
        }

        const FFloat16Color color = cube_faces[face_index][pixel_index];
        return FLinearColor(
            FMath::Max(0.0f, color.R.GetFloat()),
            FMath::Max(0.0f, color.G.GetFloat()),
            FMath::Max(0.0f, color.B.GetFloat()),
            1.0f);
    }

    FLinearColor SampleMappedCubeLinearBilinear(const TArray<FFloat16Color>* cube_faces, const FEquirectangularSamplingMap& map, int32 output_index)
    {
        FLinearColor result = FLinearColor::Black;
        const int32 tap_base = output_index * 4;
        for (int32 tap = 0; tap < 4; ++tap) {
            const int32 tap_index = tap_base + tap;
            result += ReadMappedCubeLinear(
                cube_faces,
                map,
                static_cast<int32>(map.BilinearFaces[tap_index]),
                map.BilinearPixelIndices[tap_index]) * map.BilinearWeights[tap_index];
        }
        result.A = 1.0f;
        return result;
    }

    float ReadMappedCubeFloat(const TArray<FFloat16Color>* cube_faces, const FEquirectangularSamplingMap& map, int32 output_index)
    {
        const int32 face_index = static_cast<int32>(map.NearestFaces[output_index]);
        if (!HasCubeFacePixels(cube_faces, face_index, map.CubeSize)) {
            return 0.0f;
        }

        return cube_faces[face_index][map.NearestPixelIndices[output_index]].R.GetFloat();
    }

    float Luminance(const FLinearColor& color)
    {
        return color.R * 0.2126f + color.G * 0.7152f + color.B * 0.0722f;
    }

    float GetFinitePositiveOrDefault(float value, float default_value)
    {
        return FMath::IsFinite(value) && value > 0.0f ? value : default_value;
    }

    float ComputeEquirectangularExposure(const TArray<FLinearColor>& linear_pixels, const RenderRequest::RenderParams* params)
    {
        double log_luminance_sum = 0.0;
        int32 sample_count = 0;
        for (const FLinearColor& color : linear_pixels) {
            const float luminance = Luminance(color);
            if (FMath::IsFinite(luminance) && luminance > 0.0f) {
                log_luminance_sum += FMath::Loge(static_cast<double>(luminance) + 1.0e-4);
                ++sample_count;
            }
        }

        if (sample_count == 0) {
            return 1.0f;
        }

        const float log_average_luminance = static_cast<float>(FMath::Exp(log_luminance_sum / sample_count));
        float exposure_min = 0.02f;
        float exposure_max = 20.0f;
        if (params != nullptr) {
            exposure_min = GetFinitePositiveOrDefault(params->equirectangular_exposure_min, exposure_min);
            exposure_max = GetFinitePositiveOrDefault(params->equirectangular_exposure_max, exposure_max);
        }
        if (exposure_min > exposure_max) {
            Swap(exposure_min, exposure_max);
        }

        const float compensation = params != nullptr && FMath::IsFinite(params->equirectangular_exposure_compensation)
            ? params->equirectangular_exposure_compensation
            : 0.0f;
        const float compensated_exposure =
            (0.18f / FMath::Max(log_average_luminance, 1.0e-4f)) * FMath::Pow(2.0f, compensation);
        return FMath::Clamp(compensated_exposure, exposure_min, exposure_max);
    }

    float TonemapAces(float value)
    {
        const float a = 2.51f;
        const float b = 0.03f;
        const float c = 2.43f;
        const float d = 0.59f;
        const float e = 0.14f;
        return FMath::Clamp((value * (a * value + b)) / (value * (c * value + d) + e), 0.0f, 1.0f);
    }

    uint8 LinearToSrgbByte(float value)
    {
        const float srgb = FMath::Pow(FMath::Clamp(value, 0.0f, 1.0f), 1.0f / 2.2f);
        return static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(srgb * 255.0f), 0, 255));
    }

    FColor TonemapEquirectangularColor(const FLinearColor& color, float exposure)
    {
        const FLinearColor exposed = color * exposure;
        return FColor(
            LinearToSrgbByte(TonemapAces(exposed.R)),
            LinearToSrgbByte(TonemapAces(exposed.G)),
            LinearToSrgbByte(TonemapAces(exposed.B)),
            255);
    }

    void ConvertCubeFacesToEquirectangularColor(const TArray<FColor>* cube_faces, int32 cube_size, bool use_bilinear, RenderRequest::RenderResult* result)
    {
        const FEquirectangularSamplingMap& map = GetEquirectangularSamplingMap(cube_size);
        const int32 output_pixel_count = map.OutputWidth * map.OutputHeight;
        result->width = map.OutputWidth;
        result->height = map.OutputHeight;
        result->bmp.SetNum(output_pixel_count);

        for (int32 index = 0; index < output_pixel_count; ++index) {
            result->bmp[index] = use_bilinear
                ? SampleMappedCubeColorBilinear(cube_faces, map, index)
                : ReadMappedCubeColor(cube_faces, map, index);
        }
    }

    void ConvertCubeFacesToEquirectangularSceneColor(const TArray<FFloat16Color>* cube_faces, int32 cube_size, const RenderRequest::RenderParams* params, RenderRequest::RenderResult* result)
    {
        const FEquirectangularSamplingMap& map = GetEquirectangularSamplingMap(cube_size);
        const int32 output_pixel_count = map.OutputWidth * map.OutputHeight;
        TArray<FLinearColor> linear_pixels;
        linear_pixels.SetNum(output_pixel_count);

        result->width = map.OutputWidth;
        result->height = map.OutputHeight;
        result->bmp.SetNum(output_pixel_count);

        for (int32 index = 0; index < output_pixel_count; ++index) {
            linear_pixels[index] = SampleMappedCubeLinearBilinear(cube_faces, map, index);
        }

        const float exposure = ComputeEquirectangularExposure(linear_pixels, params);
        for (int32 index = 0; index < linear_pixels.Num(); ++index) {
            result->bmp[index] = TonemapEquirectangularColor(linear_pixels[index], exposure);
        }
    }

    void ConvertCubeFacesToEquirectangularFloat(const TArray<FFloat16Color>* cube_faces, int32 cube_size, const RenderRequest::RenderParams* params, RenderRequest::RenderResult* result)
    {
        const FEquirectangularSamplingMap& map = GetEquirectangularSamplingMap(cube_size);
        const int32 output_pixel_count = map.OutputWidth * map.OutputHeight;
        result->width = map.OutputWidth;
        result->height = map.OutputHeight;
        // Float equirectangular data is already in final API layout, so bypass
        // bmp_float and avoid the normal 2D post-packaging conversion pass.
        result->image_data_float.SetNumUninitialized(output_pixel_count);

        for (int32 index = 0; index < output_pixel_count; ++index) {
            result->image_data_float[index] = ApplyMaxDepthMeters(ReadMappedCubeFloat(cube_faces, map, index), params);
        }
    }

    void PackFloatDepthAsBytes(RenderRequest::RenderResult* result)
    {
        if (result == nullptr || result->image_data_float.Num() <= 0) {
            return;
        }

        const int32 byte_count = result->image_data_float.Num() * static_cast<int32>(sizeof(float));
        result->image_data_uint8.SetNumUninitialized(byte_count, false);
        FMemory::Memcpy(
            result->image_data_uint8.GetData(),
            result->image_data_float.GetData(),
            byte_count);
        result->image_data_float.Reset();
    }
}

RenderRequest::RenderRequest(UGameViewportClient* game_viewport, std::function<void()>&& query_camera_pose_cb)
    : params_(nullptr), results_(nullptr), req_size_(0), wait_signal_(new msr::airlib::WorkerThreadSignal), game_viewport_(game_viewport), query_camera_pose_cb_(std::move(query_camera_pose_cb))
{
}

RenderRequest::~RenderRequest()
{
}

struct RenderRequest::FCancellableRequestState
{
    std::mutex Mutex;
    std::mutex WaitMutex;
    std::condition_variable Condition;
    RenderRequest* Request = nullptr;
    std::shared_ptr<std::atomic<bool>> Cancellation;
    std::atomic<bool> Complete{ false };
    std::atomic<bool> CleanupQueued{ false };
    bool RenderCommandQueued = false;
    bool ViewportModified = false;
};

// Read pixels from render target using render thread, then package the result
// on the thread that calls this method.
void RenderRequest::getScreenshot(
    std::shared_ptr<RenderParams> params[],
    std::vector<std::shared_ptr<RenderResult>>& results,
    unsigned int req_size,
    bool use_safe_method,
    const std::shared_ptr<std::atomic<bool>>& cancellation)
{
    //TODO: is below really needed?
    for (unsigned int i = 0; i < req_size; ++i) {
        results.push_back(std::make_shared<RenderResult>());

        if (!params[i]->pixels_as_float)
            results[i]->bmp.Reset();
        else
            results[i]->bmp_float.Reset();
        results[i]->width = 0;
        results[i]->height = 0;
        results[i]->time_stamp = 0;
    }

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();

    if (use_safe_method) {
        for (unsigned int i = 0; i < req_size; ++i) {
            if (params[i]->isEquirectangular() && params[i]->render_target_cube != nullptr && params[i]->render_component_cube != nullptr) {
                FIntPoint cube_size;
                setupEquirectangularRenderResource(params[i]->render_target_cube, cube_size);
                const int32 cube_width = cube_size.X;
                if (!IsCubeFaceReadbackSupported()) {
                    UE_LOG(LogTemp, Error, TEXT("AirSim equirectangular readback is not supported on the active Vulkan RHI. Use D3D11 or D3D12 for equirectangular capture."));
                }
                else if (cube_width > 0) {
                    FTextureRenderTargetCubeResource* cube_resource =
                        static_cast<FTextureRenderTargetCubeResource*>(params[i]->render_target_cube->GameThread_GetRenderTargetResource());
                    if (cube_resource != nullptr) {
                        if (!params[i]->pixels_as_float && UseGlobalEquirectangularScenePipeline(params[i]->image_type)) {
                            TArray<FFloat16Color> cube_faces[6];
                            for (int32 face = 0; face < 6; ++face) {
                                FReadSurfaceDataFlags flags(RCM_MinMax, static_cast<ECubeFace>(face));
                                cube_resource->ReadPixels(cube_faces[face], flags);
                            }
                            ConvertCubeFacesToEquirectangularSceneColor(cube_faces, cube_width, params[i].get(), results[i].get());
                        }
                        else if (!params[i]->pixels_as_float) {
                            TArray<FColor> cube_faces[6];
                            for (int32 face = 0; face < 6; ++face) {
                                FReadSurfaceDataFlags flags(RCM_UNorm, static_cast<ECubeFace>(face));
                                flags.SetLinearToGamma(false);
                                cube_resource->ReadPixels(cube_faces[face], flags);
                            }
                            ConvertCubeFacesToEquirectangularColor(
                                cube_faces,
                                cube_width,
                                UseBilinearEquirectangularSampling(params[i]->image_type),
                                results[i].get());
                        }
                        else {
                            TArray<FFloat16Color> cube_faces[6];
                            for (int32 face = 0; face < 6; ++face) {
                                FReadSurfaceDataFlags flags(RCM_MinMax, static_cast<ECubeFace>(face));
                                cube_resource->ReadPixels(cube_faces[face], flags);
                            }
                            ConvertCubeFacesToEquirectangularFloat(cube_faces, cube_width, params[i].get(), results[i].get());
                        }
                    }
                }
            }
            else if (params[i]->render_target != nullptr && params[i]->render_component != nullptr) {
                //TODO: below doesn't work right now because it must be running in game thread
                FIntPoint img_size;
                if (!params[i]->pixels_as_float) {
                    //below is documented method but more expensive because it forces flush
                    FTextureRenderTargetResource* rt_resource = params[i]->render_target->GameThread_GetRenderTargetResource();
                    auto flags = setupRenderResource(rt_resource, params[i].get(), results[i].get(), img_size);
                    if (params[i]->disable_gamma)flags.SetLinearToGamma(false);
                    rt_resource->ReadPixels(results[i]->bmp, flags);
                }
                else {
                    FTextureRenderTargetResource* rt_resource = params[i]->render_target->GetRenderTargetResource();
                    setupRenderResource(rt_resource, params[i].get(), results[i].get(), img_size);
                    rt_resource->ReadFloat16Pixels(results[i]->bmp_float);
                }
            }
        }
    }
    else if (cancellation == nullptr) {
        //wait for render thread to pick up our task
        params_ = params;
        results_ = results.data();
        req_size_ = req_size;

        // Queue up the task of querying camera pose in the game thread and synchronizing render thread with camera pose
        AsyncTask(ENamedThreads::GameThread, [this]() {
            check(IsInGameThread());

            saved_DisableWorldRendering_ = game_viewport_->bDisableWorldRendering;
            game_viewport_->bDisableWorldRendering = 0;
            end_draw_handle_ = game_viewport_->OnEndDraw().AddLambda([this] {
                check(IsInGameThread());

                // capture CameraPose for this frame
                query_camera_pose_cb_();

                // The completion is called immeidately after GameThread sends the
                // rendering commands to RenderThread. Hence, our ExecuteTask will
                // execute *immediately* after RenderThread renders the scene!
                RenderRequest* This = this;
                ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
                (
                    [This](FRHICommandListImmediate& RHICmdList) {
                        This->ExecuteTask();
                    });

                game_viewport_->bDisableWorldRendering = saved_DisableWorldRendering_;

                assert(end_draw_handle_.IsValid());
                game_viewport_->OnEndDraw().Remove(end_draw_handle_);
            });

            // while we're still on GameThread, enqueue request for capture the scene!
            CaptureScenesForRequests(params_, req_size_);
        });

        // wait for this task to complete
        while (!wait_signal_->waitFor(5)) {
            // log a message and continue wait
            // lamda function still references a few objects for which there is no refcount.
            // Walking away will cause memory corruption, which is much more difficult to debug.
            UE_LOG(LogTemp, Warning, TEXT("Failed: timeout waiting for screenshot"));
        }
    }
    else {
        params_ = params;
        results_ = results.data();
        req_size_ = req_size;

        const std::shared_ptr<FCancellableRequestState> state = std::make_shared<FCancellableRequestState>();
        state->Request = this;
        state->Cancellation = cancellation;

        AsyncTask(ENamedThreads::GameThread, [state]() {
            std::lock_guard<std::mutex> state_lock(state->Mutex);
            RenderRequest* request = state->Request;
            if (request == nullptr)
                return;
            if (state->Cancellation->load()) {
                state->Request = nullptr;
                state->Complete = true;
                state->Condition.notify_all();
                return;
            }

            request->saved_DisableWorldRendering_ = request->game_viewport_->bDisableWorldRendering;
            request->game_viewport_->bDisableWorldRendering = 0;
            state->ViewportModified = true;
            request->end_draw_handle_ = request->game_viewport_->OnEndDraw().AddLambda([state]() {
                std::lock_guard<std::mutex> end_draw_lock(state->Mutex);
                RenderRequest* active_request = state->Request;
                if (active_request == nullptr)
                    return;

                if (state->Cancellation->load()) {
                    if (state->ViewportModified) {
                        active_request->game_viewport_->bDisableWorldRendering = active_request->saved_DisableWorldRendering_;
                        state->ViewportModified = false;
                    }
                    if (active_request->end_draw_handle_.IsValid()) {
                        active_request->game_viewport_->OnEndDraw().Remove(active_request->end_draw_handle_);
                        active_request->end_draw_handle_.Reset();
                    }
                    state->Request = nullptr;
                    state->Complete = true;
                    state->Condition.notify_all();
                    return;
                }

                active_request->query_camera_pose_cb_();
                state->RenderCommandQueued = true;
                ENQUEUE_RENDER_COMMAND(SceneDrawCompletionCancellable)
                (
                    [state](FRHICommandListImmediate& RHICmdList) {
                        std::lock_guard<std::mutex> render_lock(state->Mutex);
                        RenderRequest* render_request = state->Request;
                        if (render_request != nullptr) {
                            render_request->ExecuteTask(false);
                            state->Request = nullptr;
                        }
                        state->Complete = true;
                        state->Condition.notify_all();
                    });

                if (state->ViewportModified) {
                    active_request->game_viewport_->bDisableWorldRendering = active_request->saved_DisableWorldRendering_;
                    state->ViewportModified = false;
                }
                if (active_request->end_draw_handle_.IsValid()) {
                    active_request->game_viewport_->OnEndDraw().Remove(active_request->end_draw_handle_);
                    active_request->end_draw_handle_.Reset();
                }
            });

            CaptureScenesForRequests(request->params_, request->req_size_);
        });

        double next_warning_time = FPlatformTime::Seconds() + 5.0;
        while (!state->Complete.load()) {
            if (cancellation->load() && !state->CleanupQueued.exchange(true)) {
                AsyncTask(ENamedThreads::GameThread, [state]() {
                    std::lock_guard<std::mutex> cleanup_lock(state->Mutex);
                    RenderRequest* active_request = state->Request;
                    if (active_request == nullptr || state->Complete.load() || state->RenderCommandQueued)
                        return;

                    if (state->ViewportModified) {
                        active_request->game_viewport_->bDisableWorldRendering = active_request->saved_DisableWorldRendering_;
                        state->ViewportModified = false;
                    }
                    if (active_request->end_draw_handle_.IsValid()) {
                        active_request->game_viewport_->OnEndDraw().Remove(active_request->end_draw_handle_);
                        active_request->end_draw_handle_.Reset();
                    }
                    state->Request = nullptr;
                    state->Complete = true;
                    state->Condition.notify_all();
                });
            }

            const double now = FPlatformTime::Seconds();
            if (!cancellation->load() && now >= next_warning_time) {
                UE_LOG(LogTemp, Warning, TEXT("Failed: timeout waiting for cancellable screenshot"));
                next_warning_time = now + 5.0;
            }
            std::unique_lock<std::mutex> wait_lock(state->WaitMutex);
            state->Condition.wait_for(wait_lock, std::chrono::milliseconds(1), [state]() {
                return state->Complete.load();
            });
        }
    }

    for (unsigned int i = 0; i < req_size; ++i) {
        const bool has_result_target = params[i]->isEquirectangular()
            ? (params[i]->render_target_cube != nullptr && params[i]->render_component_cube != nullptr)
            : (params[i]->render_target != nullptr && params[i]->render_component != nullptr);
        if (has_result_target) {
            const int32 pixel_count = (results[i]->width > 0 && results[i]->height > 0)
                ? results[i]->width * results[i]->height
                : 0;
            if (!params[i]->pixels_as_float) {
                if (pixel_count > 0 && results[i]->bmp.Num() == pixel_count) {
                    results[i]->image_data_uint8.SetNumUninitialized(pixel_count * 3, false);
                    if (params[i]->compress)
                        UAirBlueprintLib::CompressImageArray(results[i]->width, results[i]->height, results[i]->bmp, results[i]->image_data_uint8);
                    else {
                        uint8* ptr = results[i]->image_data_uint8.GetData();
                        for (const auto& item : results[i]->bmp) {
                            *ptr++ = item.R;
                            *ptr++ = item.G;
                            *ptr++ = item.B;
                        }
                    }
                }
                else {
                    results[i]->width = 0;
                    results[i]->height = 0;
                }
            }
            else {
                const bool has_final_float_data = pixel_count > 0 && results[i]->image_data_float.Num() == pixel_count;
                if (!has_final_float_data && pixel_count > 0 && results[i]->bmp_float.Num() == pixel_count) {
                    results[i]->image_data_float.SetNumUninitialized(pixel_count);
                    float* ptr = results[i]->image_data_float.GetData();
                    for (const auto& item : results[i]->bmp_float) {
                        *ptr++ = ApplyMaxDepthMeters(item.R.GetFloat(), params[i].get());
                    }
                }
                else if (!has_final_float_data) {
                    results[i]->width = 0;
                    results[i]->height = 0;
                }

                if (ShouldPackFloatDepthAsBytes(params[i].get()) &&
                    pixel_count > 0 &&
                    results[i]->image_data_float.Num() == pixel_count) {
                    PackFloatDepthAsBytes(results[i].get());
                }
            }
        }
    }
}

FReadSurfaceDataFlags RenderRequest::setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size)
{
    size = rt_resource->GetSizeXY();
    result->width = size.X;
    result->height = size.Y;
    FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
    flags.SetLinearToGamma(false);

    return flags;
}

void RenderRequest::setupEquirectangularRenderResource(const UTextureRenderTargetCube* render_target, FIntPoint& cube_size)
{
    const int32 size = render_target != nullptr ? static_cast<int32>(render_target->GetSurfaceWidth()) : 0;
    cube_size = FIntPoint(size, size);
}

void RenderRequest::ExecuteTask(bool signal_completion)
{
    if (params_ != nullptr && req_size_ > 0) {
        for (unsigned int i = 0; i < req_size_; ++i) {
            if (params_[i]->isEquirectangular() && params_[i]->render_target_cube != nullptr && params_[i]->render_component_cube != nullptr) {
                FIntPoint cube_size;
                setupEquirectangularRenderResource(params_[i]->render_target_cube, cube_size);
                const int32 cube_width = cube_size.X;
                if (!IsCubeFaceReadbackSupported()) {
                    UE_LOG(LogTemp, Error, TEXT("AirSim equirectangular readback is not supported on the active Vulkan RHI. Use D3D11 or D3D12 for equirectangular capture."));
                }
                else if (cube_width > 0) {
                    FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
                    FTextureRenderTargetCubeResource* cube_resource =
                        static_cast<FTextureRenderTargetCubeResource*>(params_[i]->render_target_cube->GetRenderTargetResource());

                    if (cube_resource != nullptr && cube_resource->TextureRHI.IsValid()) {
                        if (!params_[i]->pixels_as_float && UseGlobalEquirectangularScenePipeline(params_[i]->image_type)) {
                            TArray<FFloat16Color> cube_faces[6];
                            for (int32 face = 0; face < 6; ++face) {
                                FReadSurfaceDataFlags flags(RCM_MinMax, static_cast<ECubeFace>(face));
                                RHICmdList.ReadSurfaceFloatData(
                                    cube_resource->TextureRHI,
                                    FIntRect(0, 0, cube_width, cube_width),
                                    cube_faces[face],
                                    flags);
                            }

                            ConvertCubeFacesToEquirectangularSceneColor(cube_faces, cube_width, params_[i].get(), results_[i].get());
                        }
                        else if (!params_[i]->pixels_as_float) {
                            TArray<FColor> cube_faces[6];
                            for (int32 face = 0; face < 6; ++face) {
                                FReadSurfaceDataFlags flags(RCM_UNorm, static_cast<ECubeFace>(face));
                                flags.SetLinearToGamma(false);
                                RHICmdList.ReadSurfaceData(
                                    cube_resource->TextureRHI,
                                    FIntRect(0, 0, cube_width, cube_width),
                                    cube_faces[face],
                                    flags);
                            }

                            ConvertCubeFacesToEquirectangularColor(
                                cube_faces,
                                cube_width,
                                UseBilinearEquirectangularSampling(params_[i]->image_type),
                                results_[i].get());
                        }
                        else {
                            TArray<FFloat16Color> cube_faces[6];
                            for (int32 face = 0; face < 6; ++face) {
                                FReadSurfaceDataFlags flags(RCM_MinMax, static_cast<ECubeFace>(face));
                                RHICmdList.ReadSurfaceFloatData(
                                    cube_resource->TextureRHI,
                                    FIntRect(0, 0, cube_width, cube_width),
                                    cube_faces[face],
                                    flags);
                            }

                            ConvertCubeFacesToEquirectangularFloat(cube_faces, cube_width, params_[i].get(), results_[i].get());
                        }
                    }
                }
            }
            else if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
                FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
                auto rt_resource = params_[i]->render_target->GetRenderTargetResource();
                if (rt_resource != nullptr) {
                    const FTextureRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
                    FIntPoint size;
                    auto flags = setupRenderResource(rt_resource, params_[i].get(), results_[i].get(), size);

                    //should we be using ENQUEUE_UNIQUE_RENDER_COMMAND_ONEPARAMETER which was in original commit by @saihv
                    //https://github.com/Microsoft/AirSim/pull/162/commits/63e80c43812300a8570b04ed42714a3f6949e63f#diff-56b790f9394f7ca1949ddbb320d8456fR64
                    if (!params_[i]->pixels_as_float) {
                        //below is undocumented method that avoids flushing, but it seems to segfault every 2000 or so calls
                        RHICmdList.ReadSurfaceData(
                            rhi_texture,
                            FIntRect(0, 0, size.X, size.Y),
                            results_[i]->bmp,
                            flags);
                    }
                    else {
                        RHICmdList.ReadSurfaceFloatData(
                            rhi_texture,
                            FIntRect(0, 0, size.X, size.Y),
                            results_[i]->bmp_float,
                            CubeFace_PosX,
                            0,
                            0);
                    }
                }
            }
            results_[i]->time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
        }

        req_size_ = 0;
        params_ = nullptr;
        results_ = nullptr;

        if (signal_completion)
            wait_signal_->signal();
    }
}
