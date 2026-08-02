// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ImageCaptureBase_hpp
#define air_ImageCaptureBase_hpp

#include "common/Common.hpp"
#include "common/common_utils/EnumFlags.hpp"
#include <limits>

namespace msr
{
namespace airlib
{

    // This is an abstraction for cameras associated with a vehicle.  Each camera has a unique id.
    class ImageCaptureBase
    {
    public: //types
        enum class ImageType : int
        { //this indexes to array, -1 is special to indicate main camera
            Scene = 0,
            DepthPlanar,
            DepthPerspective,
            DepthVis,
            DisparityNormalized,
            Segmentation,
            SurfaceNormals,
            Infrared,
            OpticalFlow,
            OpticalFlowVis,
            Lighting,
            Annotation,
            Count //must be last
        };

        struct ImageRequest
        {
            std::string camera_name;
            ImageCaptureBase::ImageType image_type = ImageCaptureBase::ImageType::Scene;
            bool pixels_as_float = false;
            // For non-float images, true returns JPEG bytes and false returns
            // uncompressed RGB bytes.
            bool compress = true;
            std::string annotation_name;
            bool float_as_bytes = false;
            // Settings-driven recording can defer JPEG encoding until after
            // raw render readback. These fields remain recorder-private so
            // the public RPC request remains unchanged.
            bool recording_jpeg = false;
            int recording_jpeg_quality = 85;

            ImageRequest()
            {
            }

            ImageRequest(const std::string& camera_name_val,
                         ImageCaptureBase::ImageType image_type_val,
                         bool pixels_as_float_val = false,
                         bool compress_val = true,
                         const std::string& annotation_name_val = "",
                         bool float_as_bytes_val = false)
                : camera_name(camera_name_val)
                , image_type(image_type_val)
                , pixels_as_float(pixels_as_float_val)
                , compress(compress_val)
                , annotation_name(annotation_name_val)
                , float_as_bytes(float_as_bytes_val)
            {
            }
        };

        struct ImageResponse
        {
            vector<uint8_t> image_data_uint8;
            vector<float> image_data_float;

            std::string camera_name;
            Vector3r camera_position = Vector3r::Zero();
            Quaternionr camera_orientation = Quaternionr::Identity();
            // Sim-clock time immediately before the render request is submitted.
            TTimePoint request_time_stamp = 0;
            // Sim-clock time sampled on the game thread immediately before the
            // explicit CaptureScene transaction. The ordered render-thread
            // readback below is bound to that capture command, not to GPU
            // completion or response delivery.
            TTimePoint time_stamp = 0;
            uint64_t render_frame_number = 0;
            // Process-local monotonically increasing identifier for the exact
            // capture/readback transaction that produced this payload. It is
            // zero when no frame provenance was established.
            uint64_t capture_generation = 0;
            // True only when an explicit CaptureScene command and its ordered
            // readback establish that time_stamp identifies these pixels. A
            // request, readback-completion, or transport fallback must never
            // be used as a bag-image stamp.
            bool has_render_frame_timestamp = false;
            // These are snapped on the game thread immediately before the
            // scene capture is requested. They describe that capture request
            // without requiring a writer worker to touch Unreal objects.
            bool camera_info_is_perspective = false;
            float camera_horizontal_fov_degrees = 0.0f;
            std::string message;
            bool pixels_as_float = false;
            // For non-float images, true means JPEG bytes and false means raw
            // RGB bytes.
            bool compress = true;
            int width = 0, height = 0;
            ImageType image_type = ImageType::Scene;
			std::string annotation_name;
            // Set only by the settings-driven recorder when JPEG encoding is
            // deferred to the recorder writer worker.
            bool recording_jpeg = false;
            int recording_jpeg_quality = 85;

            // Image responses cross the Unreal/RPC boundary and may later be
            // written by a worker.  Validate their shape before copying or
            // serializing them so a corrupted container header cannot turn
            // into an unbounded allocation.  Empty payloads are retained for
            // ordinary capture-error responses; populated payloads must match
            // the requested layout exactly (except compressed JPEG data, whose
            // bounded encoded size varies with image content).
            static bool hasValidPayloadLayout(int width,
                                              int height,
                                              bool pixels_as_float,
                                              bool compress,
                                              size_t uint8_count,
                                              size_t float_count)
            {
                if (width < 0 || height < 0)
                    return false;

                if (width == 0 || height == 0)
                    return uint8_count == 0 && float_count == 0;

                constexpr uint64_t kMaxImagePixels = 16384ULL * 16384ULL;
                constexpr uint64_t kMaxCompressedOverheadBytes = 64ULL * 1024ULL;
                const uint64_t pixel_count = static_cast<uint64_t>(width) *
                    static_cast<uint64_t>(height);
                if (pixel_count == 0 || pixel_count > kMaxImagePixels)
                    return false;

                if (uint8_count == 0 && float_count == 0)
                    return true;

                if (pixels_as_float) {
                    const uint64_t expected_float_count = pixel_count;
                    const uint64_t expected_byte_count = pixel_count * sizeof(float);
                    return (float_count == expected_float_count && uint8_count == 0) ||
                           (uint8_count == expected_byte_count && float_count == 0);
                }

                if (!compress) {
                    const uint64_t expected_byte_count = pixel_count * 3ULL;
                    return float_count == 0 && uint8_count == expected_byte_count;
                }

                // JPEG payloads vary with image content. Five bytes per pixel
                // plus fixed overhead is deliberately generous for the
                // supported encoder while still bounding a malformed response
                // before network copy.
                const uint64_t maximum_byte_count = pixel_count * 5ULL +
                    kMaxCompressedOverheadBytes;
                return float_count == 0 && uint8_count <= maximum_byte_count;
            }
        };

    public: //methods
        virtual void getImages(const std::vector<ImageRequest>& requests, std::vector<ImageResponse>& responses) const = 0;
        virtual ~ImageCaptureBase() = default;
    };
}
} //namespace
#endif
