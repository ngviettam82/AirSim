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

    bool IsDepthMetersImage(ImageType image_type)
    {
        return image_type == ImageType::DepthPlanar ||
               image_type == ImageType::DepthPerspective;
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

    FColor ReadCubeColorNearest(const TArray<FColor>* cube_faces, ECubeFace face, int32 x, int32 y, int32 cube_size)
    {
        const int32 face_index = FaceIndex(face);
        if (!HasCubeFacePixels(cube_faces, face_index, cube_size)) {
            return FColor::Black;
        }

        const int32 clamped_x = FMath::Clamp(x, 0, cube_size - 1);
        const int32 clamped_y = FMath::Clamp(y, 0, cube_size - 1);
        FColor color = cube_faces[face_index][clamped_y * cube_size + clamped_x];
        color.A = 255;
        return color;
    }

    FColor ReadCubeColorTap(const TArray<FColor>* cube_faces, ECubeFace face, int32 x, int32 y, int32 cube_size)
    {
        if (x >= 0 && x < cube_size && y >= 0 && y < cube_size) {
            return ReadCubeColorNearest(cube_faces, face, x, y, cube_size);
        }

        const FVector tap_direction = FaceTexelToDirection(face, x, y, cube_size);
        const FEquirectangularCubeSample remapped = DirectionToCubeSample(tap_direction, cube_size);
        return ReadCubeColorNearest(
            cube_faces,
            remapped.Face,
            FMath::RoundToInt(remapped.PixelX),
            FMath::RoundToInt(remapped.PixelY),
            cube_size);
    }

    FColor SampleCubeColorNearest(const TArray<FColor>* cube_faces, const FEquirectangularCubeSample& sample, int32 cube_size)
    {
        return ReadCubeColorNearest(
            cube_faces,
            sample.Face,
            FMath::RoundToInt(sample.PixelX),
            FMath::RoundToInt(sample.PixelY),
            cube_size);
    }

    FColor SampleCubeColorBilinear(const TArray<FColor>* cube_faces, const FEquirectangularCubeSample& sample, int32 cube_size)
    {
        const int32 x0 = FMath::FloorToInt(sample.PixelX);
        const int32 y0 = FMath::FloorToInt(sample.PixelY);
        const float tx = sample.PixelX - x0;
        const float ty = sample.PixelY - y0;

        const FColor c00 = ReadCubeColorTap(cube_faces, sample.Face, x0, y0, cube_size);
        const FColor c10 = ReadCubeColorTap(cube_faces, sample.Face, x0 + 1, y0, cube_size);
        const FColor c01 = ReadCubeColorTap(cube_faces, sample.Face, x0, y0 + 1, cube_size);
        const FColor c11 = ReadCubeColorTap(cube_faces, sample.Face, x0 + 1, y0 + 1, cube_size);

        auto bilerp = [tx, ty](uint8 v00, uint8 v10, uint8 v01, uint8 v11) -> uint8 {
            const float top = FMath::Lerp(static_cast<float>(v00), static_cast<float>(v10), tx);
            const float bottom = FMath::Lerp(static_cast<float>(v01), static_cast<float>(v11), tx);
            return static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(FMath::Lerp(top, bottom, ty)), 0, 255));
        };

        return FColor(
            bilerp(c00.R, c10.R, c01.R, c11.R),
            bilerp(c00.G, c10.G, c01.G, c11.G),
            bilerp(c00.B, c10.B, c01.B, c11.B),
            255);
    }

    FFloat16Color SampleCubeFloatNearest(const TArray<FFloat16Color>* cube_faces, const FEquirectangularCubeSample& sample, int32 cube_size)
    {
        const int32 face_index = FaceIndex(sample.Face);
        if (!HasCubeFacePixels(cube_faces, face_index, cube_size)) {
            return FFloat16Color();
        }

        const int32 x = FMath::Clamp(FMath::RoundToInt(sample.PixelX), 0, cube_size - 1);
        const int32 y = FMath::Clamp(FMath::RoundToInt(sample.PixelY), 0, cube_size - 1);
        return cube_faces[face_index][y * cube_size + x];
    }

    FLinearColor ReadCubeLinearNearest(const TArray<FFloat16Color>* cube_faces, ECubeFace face, int32 x, int32 y, int32 cube_size)
    {
        const int32 face_index = FaceIndex(face);
        if (!HasCubeFacePixels(cube_faces, face_index, cube_size)) {
            return FLinearColor::Black;
        }

        const int32 clamped_x = FMath::Clamp(x, 0, cube_size - 1);
        const int32 clamped_y = FMath::Clamp(y, 0, cube_size - 1);
        const FFloat16Color color = cube_faces[face_index][clamped_y * cube_size + clamped_x];
        return FLinearColor(
            FMath::Max(0.0f, color.R.GetFloat()),
            FMath::Max(0.0f, color.G.GetFloat()),
            FMath::Max(0.0f, color.B.GetFloat()),
            1.0f);
    }

    FLinearColor ReadCubeLinearTap(const TArray<FFloat16Color>* cube_faces, ECubeFace face, int32 x, int32 y, int32 cube_size)
    {
        if (x >= 0 && x < cube_size && y >= 0 && y < cube_size) {
            return ReadCubeLinearNearest(cube_faces, face, x, y, cube_size);
        }

        const FVector tap_direction = FaceTexelToDirection(face, x, y, cube_size);
        const FEquirectangularCubeSample remapped = DirectionToCubeSample(tap_direction, cube_size);
        return ReadCubeLinearNearest(
            cube_faces,
            remapped.Face,
            FMath::RoundToInt(remapped.PixelX),
            FMath::RoundToInt(remapped.PixelY),
            cube_size);
    }

    FLinearColor SampleCubeLinearBilinear(const TArray<FFloat16Color>* cube_faces, const FEquirectangularCubeSample& sample, int32 cube_size)
    {
        const int32 x0 = FMath::FloorToInt(sample.PixelX);
        const int32 y0 = FMath::FloorToInt(sample.PixelY);
        const float tx = sample.PixelX - x0;
        const float ty = sample.PixelY - y0;

        const FLinearColor c00 = ReadCubeLinearTap(cube_faces, sample.Face, x0, y0, cube_size);
        const FLinearColor c10 = ReadCubeLinearTap(cube_faces, sample.Face, x0 + 1, y0, cube_size);
        const FLinearColor c01 = ReadCubeLinearTap(cube_faces, sample.Face, x0, y0 + 1, cube_size);
        const FLinearColor c11 = ReadCubeLinearTap(cube_faces, sample.Face, x0 + 1, y0 + 1, cube_size);

        const FLinearColor top = FMath::Lerp(c00, c10, tx);
        const FLinearColor bottom = FMath::Lerp(c01, c11, tx);
        return FMath::Lerp(top, bottom, ty);
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
        const int32 output_width = cube_size * 2;
        const int32 output_height = cube_size;
        result->width = output_width;
        result->height = output_height;
        result->bmp.SetNum(output_width * output_height);

        for (int32 y = 0; y < output_height; ++y) {
            for (int32 x = 0; x < output_width; ++x) {
                const FVector direction = EquirectangularPixelToDirection(x, y, output_width, output_height);
                const FEquirectangularCubeSample sample = DirectionToCubeSample(direction, cube_size);
                result->bmp[y * output_width + x] = use_bilinear
                    ? SampleCubeColorBilinear(cube_faces, sample, cube_size)
                    : SampleCubeColorNearest(cube_faces, sample, cube_size);
            }
        }
    }

    void ConvertCubeFacesToEquirectangularSceneColor(const TArray<FFloat16Color>* cube_faces, int32 cube_size, const RenderRequest::RenderParams* params, RenderRequest::RenderResult* result)
    {
        const int32 output_width = cube_size * 2;
        const int32 output_height = cube_size;
        TArray<FLinearColor> linear_pixels;
        linear_pixels.SetNum(output_width * output_height);

        result->width = output_width;
        result->height = output_height;
        result->bmp.SetNum(output_width * output_height);

        for (int32 y = 0; y < output_height; ++y) {
            for (int32 x = 0; x < output_width; ++x) {
                const FVector direction = EquirectangularPixelToDirection(x, y, output_width, output_height);
                const FEquirectangularCubeSample sample = DirectionToCubeSample(direction, cube_size);
                linear_pixels[y * output_width + x] = SampleCubeLinearBilinear(cube_faces, sample, cube_size);
            }
        }

        const float exposure = ComputeEquirectangularExposure(linear_pixels, params);
        for (int32 index = 0; index < linear_pixels.Num(); ++index) {
            result->bmp[index] = TonemapEquirectangularColor(linear_pixels[index], exposure);
        }
    }

    void ConvertCubeFacesToEquirectangularFloat(const TArray<FFloat16Color>* cube_faces, int32 cube_size, RenderRequest::RenderResult* result)
    {
        const int32 output_width = cube_size * 2;
        const int32 output_height = cube_size;
        result->width = output_width;
        result->height = output_height;
        result->bmp_float.SetNum(output_width * output_height);

        for (int32 y = 0; y < output_height; ++y) {
            for (int32 x = 0; x < output_width; ++x) {
                const FVector direction = EquirectangularPixelToDirection(x, y, output_width, output_height);
                const FEquirectangularCubeSample sample = DirectionToCubeSample(direction, cube_size);
                result->bmp_float[y * output_width + x] = SampleCubeFloatNearest(cube_faces, sample, cube_size);
            }
        }
    }
}

RenderRequest::RenderRequest(UGameViewportClient* game_viewport, std::function<void()>&& query_camera_pose_cb)
    : params_(nullptr), results_(nullptr), req_size_(0), wait_signal_(new msr::airlib::WorkerThreadSignal), game_viewport_(game_viewport), query_camera_pose_cb_(std::move(query_camera_pose_cb))
{
}

RenderRequest::~RenderRequest()
{
}

// read pixels from render target using render thread, then compress the result into PNG
// argument on the thread that calls this method.
void RenderRequest::getScreenshot(std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size, bool use_safe_method)
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
                            ConvertCubeFacesToEquirectangularFloat(cube_faces, cube_width, results[i].get());
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
    else {
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
            for (unsigned int i = 0; i < req_size_; ++i) {
                if (params_[i]->isEquirectangular() && params_[i]->render_target_cube != nullptr && params_[i]->render_component_cube != nullptr) {
                    params_[i]->render_component_cube->CaptureSceneDeferred();
                }
                else if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
                    params_[i]->render_component->CaptureSceneDeferred();
                }
            }
        });

        // wait for this task to complete
        while (!wait_signal_->waitFor(5)) {
            // log a message and continue wait
            // lamda function still references a few objects for which there is no refcount.
            // Walking away will cause memory corruption, which is much more difficult to debug.
            UE_LOG(LogTemp, Warning, TEXT("Failed: timeout waiting for screenshot"));
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
                if (pixel_count > 0 && results[i]->bmp_float.Num() == pixel_count) {
                    results[i]->image_data_float.SetNumUninitialized(pixel_count);
                    float* ptr = results[i]->image_data_float.GetData();
                    for (const auto& item : results[i]->bmp_float) {
                        *ptr++ = ApplyMaxDepthMeters(item.R.GetFloat(), params[i].get());
                    }
                }
                else {
                    results[i]->width = 0;
                    results[i]->height = 0;
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

void RenderRequest::ExecuteTask()
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

                            ConvertCubeFacesToEquirectangularFloat(cube_faces, cube_width, results_[i].get());
                        }
                    }
                }
            }
            else if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
                FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
                auto rt_resource = params_[i]->render_target->GetRenderTargetResource();
                if (rt_resource != nullptr) {
                    const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
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

        wait_signal_->signal();
    }
}
