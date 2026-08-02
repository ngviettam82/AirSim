#pragma once

#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureRenderTargetCube.h"
#include "common/WorkerThread.hpp"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SceneCaptureComponentCube.h"
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "common/Common.hpp"
#include "common/ImageCaptureBase.hpp"


class RenderRequest : public FRenderCommand
{
    struct FCancellableRequestState;

public:
    struct RenderParams {
        USceneCaptureComponent2D * const render_component;
        UTextureRenderTarget2D* render_target;
        USceneCaptureComponentCube* const render_component_cube;
        UTextureRenderTargetCube* render_target_cube;
        bool pixels_as_float;
        bool compress;
        bool float_as_bytes;
        bool disable_gamma;
        msr::airlib::ImageCaptureBase::ImageType image_type;
        float max_depth_meters;
        float equirectangular_exposure_compensation;
        float equirectangular_exposure_min;
        float equirectangular_exposure_max;
        // Game-thread snapshot filled immediately before CaptureScene.  It
        // travels with the CPU result so downstream writers need not access
        // the capture component from a worker thread.
        bool camera_info_is_perspective = false;
        float camera_horizontal_fov_degrees = 0.0f;

        // A capture transaction records its immutable source-time metadata on
        // the game thread immediately before the explicit CaptureScene call.
        // Readback copies this metadata only after the matching render command
        // has completed, so a timestamp is never inferred from callback or
        // GPU-readback completion time.
        msr::airlib::TTimePoint capture_time_stamp = 0;
        uint64_t capture_frame_number = 0;
        uint64_t capture_generation = 0;
        bool has_capture_provenance = false;
        bool capture_transaction_rejected = false;
        std::string capture_error;

        RenderParams(USceneCaptureComponent2D * render_component_val, UTextureRenderTarget2D* render_target_val,
                     bool pixels_as_float_val, bool compress_val, bool disable_gamma_val,
                     msr::airlib::ImageCaptureBase::ImageType image_type_val,
                     float max_depth_meters_val,
                     float equirectangular_exposure_compensation_val,
                     float equirectangular_exposure_min_val,
                     float equirectangular_exposure_max_val,
                     bool float_as_bytes_val)
            : render_component(render_component_val), render_target(render_target_val),
              render_component_cube(nullptr), render_target_cube(nullptr),
              pixels_as_float(pixels_as_float_val), compress(compress_val), float_as_bytes(float_as_bytes_val),
              disable_gamma(disable_gamma_val), image_type(image_type_val),
              max_depth_meters(max_depth_meters_val),
              equirectangular_exposure_compensation(equirectangular_exposure_compensation_val),
              equirectangular_exposure_min(equirectangular_exposure_min_val),
              equirectangular_exposure_max(equirectangular_exposure_max_val)
        {
        }

        RenderParams(USceneCaptureComponentCube* render_component_val, UTextureRenderTargetCube* render_target_val,
                     bool pixels_as_float_val, bool compress_val, bool disable_gamma_val,
                     msr::airlib::ImageCaptureBase::ImageType image_type_val,
                     float max_depth_meters_val,
                     float equirectangular_exposure_compensation_val,
                     float equirectangular_exposure_min_val,
                     float equirectangular_exposure_max_val,
                     bool float_as_bytes_val)
            : render_component(nullptr), render_target(nullptr),
              render_component_cube(render_component_val), render_target_cube(render_target_val),
              pixels_as_float(pixels_as_float_val), compress(compress_val), float_as_bytes(float_as_bytes_val),
              disable_gamma(disable_gamma_val), image_type(image_type_val),
              max_depth_meters(max_depth_meters_val),
              equirectangular_exposure_compensation(equirectangular_exposure_compensation_val),
              equirectangular_exposure_min(equirectangular_exposure_min_val),
              equirectangular_exposure_max(equirectangular_exposure_max_val)
        {
        }

        bool isEquirectangular() const
        {
            return render_component_cube != nullptr || render_target_cube != nullptr;
        }
    };
    struct RenderResult {
        TArray<uint8> image_data_uint8;
        TArray<float> image_data_float;
        std::string message;

        TArray<FColor> bmp;
        TArray<FFloat16Color> bmp_float;

        int width;
        int height;

        msr::airlib::TTimePoint request_time_stamp;
        msr::airlib::TTimePoint time_stamp;
        uint64_t render_frame_number;
        uint64_t capture_generation;
        bool has_render_frame_timestamp;
    };

private:
    static FReadSurfaceDataFlags setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size);
    static void setupEquirectangularRenderResource(const UTextureRenderTargetCube* render_target, FIntPoint& cube_size);

    std::shared_ptr<RenderParams>* params_;
    std::shared_ptr<RenderResult>* results_;
    unsigned int req_size_;

    std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;

    std::function<void()> query_camera_pose_cb_;
    std::function<void()> prepare_capture_cb_;
    msr::airlib::TTimePoint request_time_stamp_ = 0;
    msr::airlib::TTimePoint render_time_stamp_ = 0;
    uint64_t render_frame_number_ = 0;

    // Must be called on the game thread. It snapshots source-time metadata,
    // captures explicitly, and leaves a readback command to be queued
    // immediately after it.
    void beginCaptureTransaction();

public:
    RenderRequest(std::function<void()>&& query_camera_pose_cb,
                  std::function<void()>&& prepare_capture_cb = std::function<void()>());
    ~RenderRequest();

    void DoTask(ENamedThreads::Type CurrentThread, const FGraphEventRef& MyCompletionGraphEvent)
    {
        ExecuteTask();
    } 

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(RenderRequest, STATGROUP_RenderThreadCommands);
    }

    // Read pixels from render target using render thread, then package the result
    // on the thread that calls this method.
    void getScreenshot(
        std::shared_ptr<RenderParams> params[],
        std::vector<std::shared_ptr<RenderResult>>& results,
        unsigned int req_size,
        bool use_safe_method,
        const std::shared_ptr<std::atomic<bool>>& cancellation = nullptr);

    void ExecuteTask(bool signal_completion = true);
};
