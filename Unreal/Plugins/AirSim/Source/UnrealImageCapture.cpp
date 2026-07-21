#include "UnrealImageCapture.h"
#include "AirBlueprintLib.h"
#include "Engine/World.h"
#include "ImageUtils.h"

#include "RenderRequest.h"
#include "common/ClockFactory.hpp"
#include "Misc/ScopeLock.h"
#include <chrono>
#include <exception>
#include <limits>
#include <thread>
#include <utility>

namespace
{
    using ImageType = msr::airlib::ImageCaptureBase::ImageType;
    using AirSimSettings = msr::airlib::AirSimSettings;

    FCriticalSection LabelCubeWarmupExecutionCriticalSection;
    FCriticalSection LabelCubeWarmupCriticalSection;
    TSet<TWeakObjectPtr<USceneCaptureComponentCube>> WarmedLabelCubeCaptures;

    struct EquirectangularExposureSettings
    {
        float compensation = std::numeric_limits<float>::quiet_NaN();
        float min = std::numeric_limits<float>::quiet_NaN();
        float max = std::numeric_limits<float>::quiet_NaN();
    };

    bool tryGetCaptureSetting(APIPCamera* camera, ImageType image_type, AirSimSettings::CaptureSetting& out)
    {
        if (camera == nullptr) {
            return false;
        }

        const auto camera_params = camera->getParams();
        const auto capture_setting = camera_params.capture_settings.find(static_cast<int>(image_type));
        if (capture_setting == camera_params.capture_settings.end()) {
            return false;
        }

        out = capture_setting->second;
        return true;
    }

    float getMaxDepthMeters(APIPCamera* camera, ImageType image_type)
    {
        if (image_type != ImageType::DepthPlanar && image_type != ImageType::DepthPerspective) {
            return 0.0f;
        }

        AirSimSettings::CaptureSetting capture_setting;
        if (!tryGetCaptureSetting(camera, image_type, capture_setting)) {
            return 0.0f;
        }

        const float max_depth_meters = capture_setting.max_depth_meters;
        return FMath::IsFinite(max_depth_meters) && max_depth_meters > 0.0f ? max_depth_meters : 0.0f;
    }

    EquirectangularExposureSettings getEquirectangularExposureSettings(APIPCamera* camera, ImageType image_type)
    {
        EquirectangularExposureSettings result;
        AirSimSettings::CaptureSetting capture_setting;
        if (!tryGetCaptureSetting(camera, image_type, capture_setting)) {
            return result;
        }

        result.compensation = FMath::IsFinite(capture_setting.equirectangular_exposure_compensation)
            ? capture_setting.equirectangular_exposure_compensation
            : capture_setting.auto_exposure_bias;
        result.min = capture_setting.equirectangular_exposure_min;
        result.max = capture_setting.equirectangular_exposure_max;
        return result;
    }

    bool MarkLabelCubeCaptureForWarmup(const RenderRequest::RenderParams* params)
    {
        if (params == nullptr ||
            !params->isEquirectangular() ||
            params->render_component_cube == nullptr ||
            params->render_target_cube == nullptr ||
            (params->image_type != ImageType::Segmentation &&
             params->image_type != ImageType::Infrared)) {
            return false;
        }

        FScopeLock lock(&LabelCubeWarmupCriticalSection);
        for (auto It = WarmedLabelCubeCaptures.CreateIterator(); It; ++It) {
            if (!It->IsValid()) {
                It.RemoveCurrent();
            }
        }

        const TWeakObjectPtr<USceneCaptureComponentCube> capture_key(
            params->render_component_cube);
        if (WarmedLabelCubeCaptures.Contains(capture_key)) {
            return false;
        }

        WarmedLabelCubeCaptures.Add(capture_key);
        return true;
    }

    void ResetLabelCubeWarmup(const RenderRequest::RenderParams* params)
    {
        if (params == nullptr || params->render_component_cube == nullptr) {
            return;
        }

        FScopeLock lock(&LabelCubeWarmupCriticalSection);
        WarmedLabelCubeCaptures.Remove(
            TWeakObjectPtr<USceneCaptureComponentCube>(params->render_component_cube));
    }

    std::shared_ptr<RenderRequest::RenderParams> MakeLabelCubeWarmupParams(
        const RenderRequest::RenderParams* params)
    {
        if (params == nullptr ||
            params->render_component_cube == nullptr ||
            params->render_target_cube == nullptr) {
            return nullptr;
        }

        return std::make_shared<RenderRequest::RenderParams>(
            params->render_component_cube,
            params->render_target_cube,
            false,
            false,
            params->disable_gamma,
            params->image_type,
            params->max_depth_meters,
            params->equirectangular_exposure_compensation,
            params->equirectangular_exposure_min,
            params->equirectangular_exposure_max,
            false);
    }

}

UnrealImageCapture::UnrealImageCapture(const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras)
    : cameras_(cameras)
{
    //TODO: explore screenshot option
    //addScreenCaptureHandler(camera->GetWorld());
}

UnrealImageCapture::~UnrealImageCapture()
{
}

namespace
{
    // Serialize scene captures across Recording / RPC / CameraHost.
    FCriticalSection GAirSimImageCaptureMutex;
}

void UnrealImageCapture::getImages(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                                   std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses) const
{
    FScopeLock capture_lock(&GAirSimImageCaptureMutex);
    if (cameras_->valsSize() == 0) {
        for (unsigned int i = 0; i < requests.size(); ++i) {
            responses.push_back(ImageResponse());
            responses[responses.size() - 1].message = "camera is not set";
        }
    }
    else
        getSceneCaptureImage(requests, responses, false, nullptr, std::function<void()>(), true);
}

void UnrealImageCapture::getImages(
    const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
    const std::shared_ptr<std::atomic<bool>>& cancellation) const
{
    FScopeLock capture_lock(&GAirSimImageCaptureMutex);
    if (cameras_->valsSize() == 0) {
        for (size_t index = 0; index < requests.size(); ++index) {
            responses.emplace_back();
            responses.back().message = "camera is not set";
        }
    }
    else {
        getSceneCaptureImage(requests, responses, false, cancellation, std::function<void()>(), false);
    }
}

void UnrealImageCapture::getImagesForRecording(
    const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
    const std::shared_ptr<std::atomic<bool>>& cancellation,
    std::function<void()>&& prepare_capture) const
{
    FScopeLock capture_lock(&GAirSimImageCaptureMutex);
    if (cancellation && cancellation->load())
        return;
    if (cameras_->valsSize() == 0) {
        for (size_t index = 0; index < requests.size(); ++index) {
            responses.emplace_back();
            responses.back().message = "camera is not set";
        }
    }
    else {
        getSceneCaptureImage(requests, responses, false, cancellation, std::move(prepare_capture), true);
    }
}

void UnrealImageCapture::getSceneCaptureImage(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                                              std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
                                              bool use_safe_method,
                                              const std::shared_ptr<std::atomic<bool>>& cancellation,
                                              std::function<void()> prepare_capture,
                                              bool activate_camera_types) const
{
    std::vector<std::shared_ptr<RenderRequest::RenderParams>> render_params;
    std::vector<std::shared_ptr<RenderRequest::RenderResult>> render_results;

    if (cancellation && cancellation->load())
        return;

    bool visibilityChanged = false;
    UGameViewportClient* gameViewport = nullptr;
    std::exception_ptr preparation_exception;
    // Camera lookup, component activation, and render-target access all touch
    // UObjects. Keep that preparation on the game thread; only the completed
    // CPU image buffers cross back to the caller's worker thread.
    UAirBlueprintLib::RunCommandOnGameThread(
        [this, &requests, &responses, &render_params, &visibilityChanged, &gameViewport, &preparation_exception, cancellation, activate_camera_types]() {
            try {
                if (cancellation && cancellation->load())
                    return;
                for (unsigned int i = 0; i < requests.size(); ++i) {
                APIPCamera* camera = cameras_->at(requests.at(i).camera_name);
                const bool has_annotation = requests[i].image_type != ImageType::Annotation ||
                    camera->GetAnnotationNameExist(requests[i].annotation_name);
                if (activate_camera_types && has_annotation) {
                    visibilityChanged = const_cast<UnrealImageCapture*>(this)->updateCameraVisibility(camera, requests[i]) || visibilityChanged;
                }

                if (gameViewport == nullptr) {
                    gameViewport = camera->GetWorld()->GetGameViewport();
                }

                responses.emplace_back();
                ImageResponse& response = responses.back();
                UTextureRenderTarget2D* textureTarget = nullptr;
                USceneCaptureComponent2D* capture = nullptr;
                UTextureRenderTargetCube* equirectangularTextureTarget = nullptr;
                USceneCaptureComponentCube* equirectangularCapture = nullptr;
                bool useEquirectangular = false;
                if (!has_annotation) {
                    response.message = "Can't take screenshot because the annotation name does not exist for this camera";
                }
                else {
                    useEquirectangular = camera->isEquirectangularCapture(requests[i].image_type, requests[i].annotation_name);
                    if (useEquirectangular) {
                        equirectangularCapture = camera->getEquirectangularCaptureComponent(requests[i].image_type, false, requests[i].annotation_name);
                        if (equirectangularCapture == nullptr) {
                            response.message = "Can't take equirectangular screenshot because cube camera type is not active";
                        }
                        else if (equirectangularCapture->TextureTarget == nullptr) {
                            response.message = "Can't take equirectangular screenshot because cube texture target is null";
                        }
                        else {
                            equirectangularTextureTarget = equirectangularCapture->TextureTarget;
                        }
                    }
                    else {
                        capture = camera->getCaptureComponent(requests[i].image_type, false, requests[i].annotation_name);
                        if (capture == nullptr) {
                            response.message = "Can't take screenshot because camera type is not active";
                        }
                        else if (capture->TextureTarget == nullptr) {
                            response.message = "Can't take screenshot because texture target is null";
                        }
                        else {
                            textureTarget = capture->TextureTarget;
                        }
                    }
                }

                const bool disable_gamma = requests[i].image_type == ImageCaptureBase::ImageType::Segmentation ||
                    requests[i].image_type == ImageCaptureBase::ImageType::Annotation ||
                    requests[i].image_type == ImageCaptureBase::ImageType::Infrared;
                const float max_depth_meters = getMaxDepthMeters(camera, requests[i].image_type);
                const EquirectangularExposureSettings equirectangular_exposure =
                    getEquirectangularExposureSettings(camera, requests[i].image_type);
                if (useEquirectangular) {
                    render_params.push_back(std::make_shared<RenderRequest::RenderParams>(
                        equirectangularCapture,
                        equirectangularTextureTarget,
                        requests[i].pixels_as_float,
                        requests[i].compress,
                        disable_gamma,
                        requests[i].image_type,
                        max_depth_meters,
                        equirectangular_exposure.compensation,
                        equirectangular_exposure.min,
                        equirectangular_exposure.max,
                        requests[i].float_as_bytes));
                }
                else {
                    render_params.push_back(std::make_shared<RenderRequest::RenderParams>(
                        capture,
                        textureTarget,
                        requests[i].pixels_as_float,
                        requests[i].compress,
                        disable_gamma,
                        requests[i].image_type,
                        max_depth_meters,
                        equirectangular_exposure.compensation,
                        equirectangular_exposure.min,
                        equirectangular_exposure.max,
                        requests[i].float_as_bytes));
                }
                }
            }
            catch (...) {
                preparation_exception = std::current_exception();
            }
        },
        true);

    if (preparation_exception)
        std::rethrow_exception(preparation_exception);

    if (use_safe_method && visibilityChanged) {
        // We don't do game/render thread synchronization for safe method.
        // We just blindly sleep for 200ms (the old way)
        std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    }

    if (nullptr == gameViewport) {
        return;
    }

    {
        // Serialize first-use priming so a concurrent request cannot read the
        // same cube between marking it warm and completing both prime frames.
        FScopeLock warmup_execution_lock(&LabelCubeWarmupExecutionCriticalSection);
        std::vector<std::shared_ptr<RenderRequest::RenderParams>> label_cube_warmup_params;
        for (const std::shared_ptr<RenderRequest::RenderParams>& params : render_params) {
            if (MarkLabelCubeCaptureForWarmup(params.get())) {
                std::shared_ptr<RenderRequest::RenderParams> warmup_params =
                    MakeLabelCubeWarmupParams(params.get());
                if (warmup_params != nullptr) {
                    label_cube_warmup_params.push_back(std::move(warmup_params));
                }
                else {
                    ResetLabelCubeWarmup(params.get());
                }
            }
        }
        if (!label_cube_warmup_params.empty()) {
            // UE 5.5 exposes a newly activated cube capture's replacing-tonemapper
            // material two completed capture frames later. Prime each label cube
            // on first use so every user-visible response is labeled.
            bool warmup_complete = false;
            for (int32 warmup_index = 0; warmup_index < 2; ++warmup_index) {
                std::vector<std::shared_ptr<RenderRequest::RenderResult>> warmup_results;
                RenderRequest warmup_request{ gameViewport, []() {} };
                warmup_request.getScreenshot(
                    label_cube_warmup_params.data(),
                    warmup_results,
                    label_cube_warmup_params.size(),
                    use_safe_method,
                    cancellation);

                warmup_complete = warmup_results.size() == label_cube_warmup_params.size();
                for (const std::shared_ptr<RenderRequest::RenderResult>& result : warmup_results) {
                    warmup_complete = warmup_complete &&
                        result != nullptr &&
                        result->width > 0 &&
                        result->height > 0;
                }
                if (!warmup_complete) {
                    break;
                }
            }

            if (!warmup_complete) {
                for (const std::shared_ptr<RenderRequest::RenderParams>& params : label_cube_warmup_params) {
                    ResetLabelCubeWarmup(params.get());
                }
            }
        }
    }

    auto query_camera_pose_cb = [this, &requests, &responses]() {
        size_t count = requests.size();
        for (size_t i = 0; i < count; i++) {
            const ImageRequest& request = requests.at(i);
            APIPCamera* camera = cameras_->at(request.camera_name);
            ImageResponse& response = responses.at(i);
            auto camera_pose = camera->getPose();
            response.camera_position = camera_pose.position;
            response.camera_orientation = camera_pose.orientation;
        }
    };
    RenderRequest render_request{ gameViewport, std::move(query_camera_pose_cb), std::move(prepare_capture) };

    render_request.getScreenshot(render_params.data(), render_results, render_params.size(), use_safe_method, cancellation);

    for (unsigned int i = 0; i < requests.size(); ++i) {
        const ImageRequest& request = requests.at(i);
        ImageResponse& response = responses.at(i);

        response.camera_name = request.camera_name;
        response.request_time_stamp = render_results[i]->request_time_stamp;
        response.time_stamp = render_results[i]->time_stamp;
        response.render_frame_number = render_results[i]->render_frame_number;
        response.image_data_uint8 = std::vector<uint8_t>(render_results[i]->image_data_uint8.GetData(), render_results[i]->image_data_uint8.GetData() + render_results[i]->image_data_uint8.Num());
        response.image_data_float = std::vector<float>(render_results[i]->image_data_float.GetData(), render_results[i]->image_data_float.GetData() + render_results[i]->image_data_float.Num());
        if (use_safe_method) {
            // Currently, we don't have a way to synthronize image capturing and camera pose when safe method is used,
            msr::airlib::Pose pose;
            UAirBlueprintLib::RunCommandOnGameThread([this, &request, &pose]() {
                APIPCamera* camera = cameras_->at(request.camera_name);
                pose = camera->getPose();
            }, true);
            response.camera_position = pose.position;
            response.camera_orientation = pose.orientation;
        }
        response.pixels_as_float = request.pixels_as_float;
        response.compress = request.compress;
        response.width = render_results[i]->width;
        response.height = render_results[i]->height;
        response.image_type = request.image_type;
		response.annotation_name = request.annotation_name;
    }
}

bool UnrealImageCapture::updateCameraVisibility(APIPCamera* camera, const msr::airlib::ImageCaptureBase::ImageRequest& request)
{
    bool visibilityChanged = false;
    if (!camera->getCameraTypeEnabled(request.image_type, request.annotation_name)) {
        camera->setCameraTypeEnabled(request.image_type, true, request.annotation_name);
        visibilityChanged = true;
    }

    return visibilityChanged;
}

bool UnrealImageCapture::getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng)
{
    FScreenshotRequest::RequestScreenshot(false); // This is an async operation
    return true;
}

void UnrealImageCapture::addScreenCaptureHandler(UWorld* world)
{
    static bool is_installed = false;

    if (!is_installed) {
        UGameViewportClient* ViewportClient = world->GetGameViewport();
        ViewportClient->OnScreenshotCaptured().Clear();
        ViewportClient->OnScreenshotCaptured().AddLambda(
            [this](int32 SizeX, int32 SizeY, const TArray<FColor>& Bitmap) {
                // Make sure that all alpha values are opaque.
                TArray<FColor>& RefBitmap = const_cast<TArray<FColor>&>(Bitmap);
                for (auto& Color : RefBitmap)
                    Color.A = 255;

                TArray<uint8_t> last_compressed_png;
                FImageUtils::CompressImageArray(SizeX, SizeY, RefBitmap, last_compressed_png);
                last_compressed_png_ = std::vector<uint8_t>(last_compressed_png.GetData(), last_compressed_png.GetData() + last_compressed_png.Num());
            });

        is_installed = true;
    }
}
