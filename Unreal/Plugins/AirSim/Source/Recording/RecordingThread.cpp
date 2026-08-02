#include "RecordingThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/RunnableThread.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <limits>
#include <thread>
#include <utility>
#include "common/ClockFactory.hpp"
#include "common/common_utils/FileSystem.hpp"

std::unique_ptr<FRecordingThread> FRecordingThread::running_instance_;
std::unique_ptr<FRecordingThread> FRecordingThread::finishing_instance_;
msr::airlib::WorkerThreadSignal FRecordingThread::finishing_signal_;
bool FRecordingThread::first_ = true;
std::atomic<uint64_t> FRecordingThread::next_sequence_id_{ 1 };

FRecordingThread::FRecordingThread()
    : stop_task_counter_(0), recording_file_(nullptr)
{
}

void FRecordingThread::startRecording(const RecordingSetting& settings,
                                      const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& vehicle_sim_apis,
                                      PhysicsLockFn physics_lock)
{
    stopRecording();
    // Wait for previous worker to fully exit before starting a new session.
    waitForFinishingInstance();

    running_instance_.reset(new FRecordingThread());
    running_instance_->settings_ = settings;
    running_instance_->vehicle_sim_apis_ = vehicle_sim_apis;
    running_instance_->image_wrapper_module_ = UAirBlueprintLib::getImageWrapperModule();
    // Optional short lock for consistent kinematics reads. Recording never pauses physics.
    running_instance_->physics_lock_ = physics_lock
        ? std::move(physics_lock)
        : PhysicsLockFn([](const std::function<void()>& work) { work(); });

    for (const auto& vehicle_sim_api : vehicle_sim_apis) {
        auto vehicle_name = vehicle_sim_api->getVehicleName();
        running_instance_->image_captures_[vehicle_name] =
            static_cast<const UnrealImageCapture*>(vehicle_sim_api->getImageCapture());
        running_instance_->last_sensor_poses_[vehicle_name] = msr::airlib::Pose();
        running_instance_->last_image_poses_[vehicle_name] = msr::airlib::Pose();
    }

    running_instance_->last_sensor_on_ = 0;
    running_instance_->last_image_on_ = 0;
    running_instance_->next_sequence_to_write_ = 1;
    running_instance_->pending_records_.clear();
    running_instance_->producers_done_.store(false);
    running_instance_->image_cancellation_ = std::make_shared<std::atomic<bool>>(false);
    next_sequence_id_.store(1);

    if (settings.isRosbagOutput()) {
        try {
            const std::string session_path = common_utils::FileSystem::getLogFolderPath(true, settings.folder);
            running_instance_->rosbag_writer_.reset(new RosbagWriter());
            if (!running_instance_->rosbag_writer_->startRecording(
                    session_path, settings.rosbag.file_name)) {
                UAirBlueprintLib::LogMessage(
                    TEXT("Recording Error"), TEXT("Failed to create ROS bag recording session"),
                    LogDebugLevel::Failure);
                running_instance_.reset();
                return;
            }

            const std::string output_path = running_instance_->rosbag_writer_->outputPath();
            UE_LOG(LogTemp, Log, TEXT("ROS bag recording: %s"),
                   UTF8_TO_TCHAR(output_path.c_str()));
        }
        catch (...) {
            UAirBlueprintLib::LogMessage(
                TEXT("Recording Error"), TEXT("Failed to create ROS bag recording session"),
                LogDebugLevel::Failure);
            running_instance_.reset();
            return;
        }

        running_instance_->rosbag_active_.store(true);
        running_instance_->rosbag_history_active_.store(true);
        const size_t imu_count = running_instance_->startRosbagImuHistory();
        UAirBlueprintLib::LogMessage(
            TEXT("ROS bag native IMU history"),
            FString::Printf(TEXT("Enabled for %llu IMU sensor(s)"), static_cast<uint64>(imu_count)),
            LogDebugLevel::Informational);
    }
    else {
        running_instance_->recording_file_.reset(new RecordingFile());
        msr::airlib::RecordingCapture header_template;
        header_template.schema_sensor_names = settings.sensor_schema;
        header_template.schema_tokens = msr::airlib::RecordingCapture::buildSchemaTokens(settings.sensor_schema);
        if (vehicle_sim_apis.valsSize() > 0) {
            auto* first = *vehicle_sim_apis.begin();
            auto cap = first->createRecordingCapture(0, {}, header_template.schema_tokens);
            header_template.vehicle_extra_header = cap.vehicle_extra_header;
            header_template.vehicle_extra_values = cap.vehicle_extra_values;
        }
        if (!running_instance_->recording_file_->startRecording(header_template, settings.folder)) {
            UAirBlueprintLib::LogMessage(
                TEXT("Recording Error"), TEXT("Failed to create recording session"),
                LogDebugLevel::Failure);
            running_instance_.reset();
            return;
        }
    }
    running_instance_->is_ready_.store(true);
    if (!running_instance_->startThread()) {
        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("Failed to create recording worker"), LogDebugLevel::Failure);
        std::unique_ptr<FRecordingThread> failed_instance = std::move(running_instance_);
        failed_instance->is_ready_.store(false);
        failed_instance->stopAllRosbagImuHistory(false);
        if (failed_instance->rosbag_writer_)
            failed_instance->rosbag_writer_->abortRecording();
    }
}

FRecordingThread::~FRecordingThread()
{
    Stop();
    if (thread_)
        thread_->WaitForCompletion();
    if (image_cancellation_)
        image_cancellation_->store(true);
    pending_records_cv_.notify_all();
    if (image_thread_.joinable())
        image_thread_.join();
    producers_done_.store(true);
    pending_records_cv_.notify_all();
    if (writer_thread_.joinable())
        writer_thread_.join();
    stopAllRosbagImuHistory(false);
    if (rosbag_writer_)
        rosbag_writer_->stopRecording();
}

bool FRecordingThread::startThread()
{
    thread_.reset(FRunnableThread::Create(this, TEXT("FRecordingThread"), 0, TPri_BelowNormal));
    return thread_ != nullptr;
}

void FRecordingThread::waitForFinishingInstance()
{
    if (!finishing_instance_)
        return;

    double next_warning_time = FPlatformTime::Seconds() + 5.0;
    while (!finishing_signal_.waitForRetry(0.001, 1)) {
        // EndPlay runs on the game thread. A cancelled render request may need
        // one final game-thread cleanup task before the image worker can exit.
        if (IsInGameThread())
            FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);

        const double now = FPlatformTime::Seconds();
        if (now >= next_warning_time) {
            UE_LOG(LogTemp, Warning, TEXT("Waiting for recording workers to stop safely"));
            next_warning_time = now + 5.0;
        }
        FPlatformProcess::SleepNoStats(0.001f);
    }
    finishing_instance_.reset();
}

void FRecordingThread::init()
{
    first_ = true;
    next_sequence_id_.store(1);
}

bool FRecordingThread::isRecording()
{
    return running_instance_ != nullptr;
}

void FRecordingThread::stopRecording()
{
    if (!running_instance_)
        return;
    waitForFinishingInstance();
    finishing_instance_ = std::move(running_instance_);
    finishing_instance_->is_ready_.store(false);
    finishing_instance_->Stop();
}

void FRecordingThread::killRecording()
{
    stopRecording();
    waitForFinishingInstance();
}

bool FRecordingThread::Init()
{
    // The session is fully configured before FRunnableThread::Create, so Init
    // must not wait on session state or on the previous worker's finish signal.
    first_ = false;
    if (recording_file_) {
        UE_LOG(LogTemp, Log, TEXT("Initiated default recording thread (freerun dual-rate)"));
    }
    else if (rosbag_writer_) {
        UE_LOG(LogTemp, Log, TEXT("Initiated ROS bag recording thread (freerun dual-rate)"));
    }
    return true;
}

uint32 FRecordingThread::Run()
{
    const float image_interval = std::isfinite(settings_.record_interval) && settings_.record_interval > 0
        ? settings_.record_interval
        : 0.05f;
    const float sensor_interval = std::isfinite(settings_.sensor_record_interval) && settings_.sensor_record_interval > 0
        ? settings_.sensor_record_interval
        : image_interval;

    while (stop_task_counter_.GetValue() == 0 && !is_ready_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (stop_task_counter_.GetValue() != 0) {
        stopAllRosbagImuHistory(false);
        if (rosbag_writer_)
            rosbag_writer_->stopRecording();
        recording_file_.reset();
        return 0;
    }

    writer_thread_ = std::thread(&FRecordingThread::runWriterLoop, this);
    image_thread_ = std::thread(&FRecordingThread::runImageLoop, this, image_interval);
    runSensorLoop(sensor_interval);

    if (image_cancellation_)
        image_cancellation_->store(true);
    if (image_thread_.joinable())
        image_thread_.join();

    // No image producer remains. Atomically disable each source ring and
    // enqueue its final samples before allowing the ordered writer to finish.
    stopAllRosbagImuHistory(true);

    producers_done_.store(true);
    pending_records_cv_.notify_all();
    if (writer_thread_.joinable())
        writer_thread_.join();

    if (rosbag_writer_)
        rosbag_writer_->stopRecording();

    recording_file_.reset();
    return 0;
}

size_t FRecordingThread::startRosbagImuHistory()
{
    size_t enabled = 0;
    for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
        const auto selected_sensors = settings_.sensors.find(vehicle_sim_api->getVehicleName());
        if (selected_sensors == settings_.sensors.end())
            continue;
        try {
            enabled += vehicle_sim_api->startRecordingImuHistory(
                static_cast<size_t>(settings_.rosbag.max_imu_buffer_samples),
                selected_sensors->second);
        }
        catch (...) {
            UAirBlueprintLib::LogMessageString(
                "ROS bag IMU history setup failed",
                vehicle_sim_api->getVehicleName(),
                LogDebugLevel::Failure);
        }
    }
    return enabled;
}

std::vector<msr::airlib::RecordingImuBatch> FRecordingThread::drainRosbagImuHistory(
    VehicleSimApiBase* vehicle_sim_api)
{
    if (!rosbag_history_active_.load() || vehicle_sim_api == nullptr)
        return {};
    try {
        return vehicle_sim_api->drainRecordingImuHistory();
    }
    catch (...) {
        const std::string vehicle_name = vehicle_sim_api->getVehicleName();
        UE_LOG(LogTemp, Error, TEXT("ROS bag IMU history drain failed for %s"),
               UTF8_TO_TCHAR(vehicle_name.c_str()));
        return {};
    }
}

std::vector<msr::airlib::RecordingImuBatch> FRecordingThread::stopRosbagImuHistory(
    VehicleSimApiBase* vehicle_sim_api)
{
    if (vehicle_sim_api == nullptr)
        return {};
    try {
        return vehicle_sim_api->stopRecordingImuHistory();
    }
    catch (...) {
        const std::string vehicle_name = vehicle_sim_api->getVehicleName();
        UE_LOG(LogTemp, Error, TEXT("ROS bag IMU history shutdown failed for %s"),
               UTF8_TO_TCHAR(vehicle_name.c_str()));
        return {};
    }
}

void FRecordingThread::stopAllRosbagImuHistory(bool enqueue_final_records)
{
    const bool was_active = rosbag_history_active_.exchange(false);
    if (!was_active && !rosbag_writer_)
        return;

    for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
        auto imu_batches = stopRosbagImuHistory(vehicle_sim_api);
        if (!enqueue_final_records || imu_batches.empty())
            continue;

        msr::airlib::RecordingCapture capture;
        capture.vehicle_name = vehicle_sim_api->getVehicleName();
        capture.sequence_id = next_sequence_id_.fetch_add(1);
        capture.frame_time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
        capture.association_mode = "rosbag_imu_final_drain";
        auto record = std::make_shared<PendingRecord>();
        record->capture = std::move(capture);
        record->imu_batches = std::move(imu_batches);
        record->write_recording_file = false;
        record->ready = true;
        enqueueRecord(record);
    }
}

int64_t FRecordingThread::imageSyncToleranceNanos() const
{
    const double tolerance_ns = static_cast<double>(settings_.image_sync_tolerance_ms) * 1.0E6;
    if (!std::isfinite(tolerance_ns) || tolerance_ns < 0)
        return 5000000;
    if (tolerance_ns == 0)
        return 0;

    const double max_nanos = static_cast<double>(std::numeric_limits<int64_t>::max());
    if (tolerance_ns >= max_nanos)
        return std::numeric_limits<int64_t>::max();
    return static_cast<int64_t>(tolerance_ns);
}

void FRecordingThread::enqueueRecord(const PendingRecordPtr& record)
{
    {
        std::lock_guard<std::mutex> lock(pending_records_mutex_);
        pending_records_[record->capture.sequence_id] = record;
    }
    pending_records_cv_.notify_all();
}

void FRecordingThread::completeRecord(const PendingRecordPtr& record,
                                      std::vector<ImageCaptureBase::ImageResponse>&& responses,
                                      bool capture_failed)
{
    {
        std::lock_guard<std::mutex> lock(pending_records_mutex_);
        record->responses = std::move(responses);
        record->capture.image_request_time_stamps.clear();
        record->capture.image_time_stamps.clear();
        record->capture.image_delays_ns.clear();
        record->capture.image_sync_within_tolerance.clear();

        bool has_saved_image = false;
        for (const auto& response : record->responses) {
            record->capture.image_request_time_stamps.push_back(response.request_time_stamp);
            const bool has_render_frame_time =
                response.has_render_frame_timestamp && response.capture_generation != 0 &&
                response.time_stamp != 0;
            record->capture.image_time_stamps.push_back(
                has_render_frame_time ? response.time_stamp : 0);
            const bool has_timing = has_render_frame_time && record->capture.frame_time_stamp != 0;
            const int64_t delay_ns = has_timing
                ? static_cast<int64_t>(response.time_stamp) - static_cast<int64_t>(record->capture.frame_time_stamp)
                : 0;
            record->capture.image_delays_ns.push_back(delay_ns);
            record->capture.image_sync_within_tolerance.push_back(
                has_timing && std::llabs(delay_ns) <= record->capture.image_sync_tolerance_ns);
            if (record->capture.render_frame_number == 0 && response.render_frame_number != 0)
                record->capture.render_frame_number = response.render_frame_number;

            const bool response_has_data =
                (response.message.empty() || response.width != 0 || response.height != 0) &&
                (!response.image_data_uint8.empty() || !response.image_data_float.empty());
            has_saved_image = has_saved_image || response_has_data;
        }

        if (capture_failed || !has_saved_image)
            record->capture.association_mode = "frame_latched_capture_failed";
        record->ready = true;
    }
    pending_records_cv_.notify_all();
}

bool FRecordingThread::encodeJpegResponse(ImageCaptureBase::ImageResponse& response) const
{
    if (!response.recording_jpeg)
        return true;

    try {
        if (image_wrapper_module_ == nullptr || response.pixels_as_float ||
            response.width <= 0 || response.height <= 0 ||
            response.recording_jpeg_quality < 40 || response.recording_jpeg_quality > 100) {
            return false;
        }

        const uint64 pixel_count = static_cast<uint64>(response.width) *
            static_cast<uint64>(response.height);
        if (pixel_count == 0 ||
            pixel_count > static_cast<uint64>(std::numeric_limits<size_t>::max() / 4u) ||
            pixel_count > static_cast<uint64>(std::numeric_limits<int64>::max() / 4u)) {
            return false;
        }

        const size_t raw_byte_count = static_cast<size_t>(pixel_count) * 3u;
        if (response.image_data_uint8.size() != raw_byte_count)
            return false;

        std::vector<uint8> rgba(static_cast<size_t>(pixel_count) * 4u);
        for (size_t pixel = 0; pixel < static_cast<size_t>(pixel_count); ++pixel) {
            const size_t rgb_offset = pixel * 3u;
            const size_t rgba_offset = pixel * 4u;
            rgba[rgba_offset] = response.image_data_uint8[rgb_offset];
            rgba[rgba_offset + 1u] = response.image_data_uint8[rgb_offset + 1u];
            rgba[rgba_offset + 2u] = response.image_data_uint8[rgb_offset + 2u];
            rgba[rgba_offset + 3u] = 255;
        }

        const TSharedPtr<IImageWrapper> wrapper = image_wrapper_module_->CreateImageWrapper(
            EImageFormat::JPEG, TEXT("AirSimRosbagRecording"));
        if (!wrapper.IsValid() ||
            !wrapper->SetRaw(rgba.data(), static_cast<int64>(rgba.size()),
                             response.width, response.height, ERGBFormat::RGBA, 8)) {
            return false;
        }

        const TArray64<uint8> jpeg = wrapper->GetCompressed(response.recording_jpeg_quality);
        if (jpeg.Num() <= 0)
            return false;

        response.image_data_uint8.assign(jpeg.GetData(), jpeg.GetData() + jpeg.Num());
        response.image_data_float.clear();
        response.compress = true;
        return true;
    }
    catch (const std::exception&) {
        return false;
    }
    catch (...) {
        return false;
    }
}

bool FRecordingThread::encodeJpegResponses(std::vector<ImageCaptureBase::ImageResponse>& responses) const
{
    bool succeeded = true;
    for (auto& response : responses) {
        if (!response.recording_jpeg || encodeJpegResponse(response))
            continue;

        succeeded = false;
        UE_LOG(LogTemp, Error, TEXT("Recording JPEG encode failed for %s"),
               UTF8_TO_TCHAR(response.camera_name.c_str()));
        response.message = "JPEG recording encode failed";
        response.image_data_uint8.clear();
        response.image_data_float.clear();
        response.width = 0;
        response.height = 0;
        response.compress = false;
    }
    return succeeded;
}

void FRecordingThread::runSensorLoop(float sensor_interval)
{
    const auto schema_tokens =
        msr::airlib::RecordingCapture::buildSchemaTokens(settings_.sensor_schema);

    while (stop_task_counter_.GetValue() == 0) {
        if (msr::airlib::ClockFactory::get()->elapsedSince(last_sensor_on_) <= sensor_interval) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        last_sensor_on_ = msr::airlib::ClockFactory::get()->nowNanos();

        for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
            if (stop_task_counter_.GetValue() != 0)
                break;

            const auto& vehicle_name = vehicle_sim_api->getVehicleName();
            std::vector<std::string> sensor_names;
            const auto sensor_it = settings_.sensors.find(vehicle_name);
            if (sensor_it != settings_.sensors.end())
                sensor_names = sensor_it->second;

            try {
                msr::airlib::RecordingCapture capture;
                physics_lock_([&]() {
                    capture = vehicle_sim_api->createRecordingCapture(0, sensor_names, schema_tokens);
                });

                // Native IMU samples are drained after the short physics lock.
                // The source ring is independent of TSV sampling and does not
                // require world, game-thread, or render-thread access.
                auto imu_batches = drainRosbagImuHistory(vehicle_sim_api);
                const bool write_recording_file = !settings_.record_on_move ||
                    last_sensor_poses_[vehicle_name] != capture.pose;
                const auto is_rosbag_auxiliary_sensor = [](const msr::airlib::RecordingSensorSample& sample) {
                    using SensorType = msr::airlib::SensorBase::SensorType;
                    return sample.present && sample.sensor_time_stamp != 0 &&
                        (sample.sensor_type == SensorType::Gps ||
                         sample.sensor_type == SensorType::Barometer ||
                         sample.sensor_type == SensorType::Magnetometer);
                };
                const auto auxiliary_sensor_identity = [&vehicle_name](
                                                     const msr::airlib::RecordingSensorSample& sample) {
                    return vehicle_name + std::string(1, '\x1f') +
                        std::to_string(static_cast<int>(sample.sensor_type)) +
                        std::string(1, '\x1f') + sample.sensor_name;
                };
                const bool has_new_rosbag_auxiliary_sensor = settings_.record_on_move &&
                    rosbag_active_.load() &&
                    std::any_of(capture.sensors.begin(), capture.sensors.end(),
                                [this, &is_rosbag_auxiliary_sensor, &auxiliary_sensor_identity](
                                    const msr::airlib::RecordingSensorSample& sample) {
                                    if (!is_rosbag_auxiliary_sensor(sample))
                                        return false;
                                    const auto previous = last_rosbag_auxiliary_sensor_time_stamps_.find(
                                        auxiliary_sensor_identity(sample));
                                    return previous == last_rosbag_auxiliary_sensor_time_stamps_.end() ||
                                        previous->second != sample.sensor_time_stamp;
                                });
                if (!write_recording_file && imu_batches.empty() && !has_new_rosbag_auxiliary_sensor)
                    continue;
                if (settings_.record_on_move && rosbag_active_.load()) {
                    for (const auto& sample : capture.sensors) {
                        if (is_rosbag_auxiliary_sensor(sample)) {
                            last_rosbag_auxiliary_sensor_time_stamps_[auxiliary_sensor_identity(sample)] =
                                sample.sensor_time_stamp;
                        }
                    }
                }
                last_sensor_poses_[vehicle_name] = capture.pose;

                capture.sequence_id = next_sequence_id_.fetch_add(1);
                capture.association_mode = "sensor_only";
                capture.image_sync_tolerance_ns = imageSyncToleranceNanos();
                auto record = std::make_shared<PendingRecord>();
                record->capture = std::move(capture);
                record->imu_batches = std::move(imu_batches);
                record->write_recording_file = write_recording_file;
                record->ready = true;
                enqueueRecord(record);
            }
            catch (...) {
                auto imu_batches = drainRosbagImuHistory(vehicle_sim_api);
                if (!imu_batches.empty()) {
                    msr::airlib::RecordingCapture capture;
                    capture.vehicle_name = vehicle_name;
                    capture.sequence_id = next_sequence_id_.fetch_add(1);
                    capture.frame_time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
                    capture.association_mode = "rosbag_imu_snapshot_failed";
                    auto record = std::make_shared<PendingRecord>();
                    record->capture = std::move(capture);
                    record->imu_batches = std::move(imu_batches);
                    record->write_recording_file = false;
                    record->ready = true;
                    enqueueRecord(record);
                }
                UE_LOG(LogTemp, Error, TEXT("Recording sensor sample failed for %s"),
                       UTF8_TO_TCHAR(vehicle_name.c_str()));
            }
        }
    }
}

void FRecordingThread::runImageLoop(float image_interval)
{
    struct PreparedRecordState
    {
        std::mutex mutex;
        PendingRecordPtr record;
    };

    const auto schema_tokens =
        msr::airlib::RecordingCapture::buildSchemaTokens(settings_.sensor_schema);

    while (stop_task_counter_.GetValue() == 0 &&
           image_cancellation_ && !image_cancellation_->load()) {
        if (msr::airlib::ClockFactory::get()->elapsedSince(last_image_on_) <= image_interval) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        last_image_on_ = msr::airlib::ClockFactory::get()->nowNanos();

        for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
            if (stop_task_counter_.GetValue() != 0 || image_cancellation_->load())
                break;

            const auto& vehicle_name = vehicle_sim_api->getVehicleName();
            const auto request_it = settings_.requests.find(vehicle_name);
            const auto capture_it = image_captures_.find(vehicle_name);
            if (request_it == settings_.requests.end() || request_it->second.empty() ||
                capture_it == image_captures_.end() || capture_it->second == nullptr) {
                continue;
            }

            std::vector<std::string> sensor_names;
            const auto sensor_it = settings_.sensors.find(vehicle_name);
            if (sensor_it != settings_.sensors.end())
                sensor_names = sensor_it->second;

            auto prepared = std::make_shared<PreparedRecordState>();
            std::vector<ImageCaptureBase::ImageResponse> responses;
            bool capture_failed = false;

            auto prepare_capture = [this, prepared, vehicle_sim_api, vehicle_name, sensor_names, schema_tokens]() {
                try {
                    msr::airlib::RecordingCapture capture;
                    physics_lock_([&]() {
                        capture = vehicle_sim_api->createRecordingCapture(0, sensor_names, schema_tokens);
                        vehicle_sim_api->updateRenderedState(0);
                    });

                    // This callback is invoked on the game thread immediately before
                    // CaptureScene, so rendered vehicle state is latched to the sample.
                    vehicle_sim_api->updateRendering(0);

                    if (settings_.record_on_move && last_image_poses_[vehicle_name] == capture.pose)
                        return;
                    last_image_poses_[vehicle_name] = capture.pose;

                    capture.sequence_id = next_sequence_id_.fetch_add(1);
                    capture.association_mode = "frame_latched_freerun";
                    capture.image_sync_tolerance_ns = imageSyncToleranceNanos();

                    auto record = std::make_shared<PendingRecord>();
                    record->capture = std::move(capture);
                    record->ready = false;
                    {
                        std::lock_guard<std::mutex> state_lock(prepared->mutex);
                        prepared->record = record;
                    }
                    enqueueRecord(record);
                }
                catch (...) {
                    UE_LOG(LogTemp, Error, TEXT("Recording image snapshot failed for %s"),
                           UTF8_TO_TCHAR(vehicle_name.c_str()));
                }
            };

            try {
                capture_it->second->getImagesForRecording(
                    request_it->second, responses, image_cancellation_, std::move(prepare_capture));
            }
            catch (...) {
                capture_failed = true;
                UE_LOG(LogTemp, Error, TEXT("Recording image capture failed for %s"),
                       UTF8_TO_TCHAR(vehicle_name.c_str()));
            }

            PendingRecordPtr record;
            {
                std::lock_guard<std::mutex> state_lock(prepared->mutex);
                record = prepared->record;
            }
            if (record)
                completeRecord(record, std::move(responses), capture_failed);
        }
    }
}

void FRecordingThread::runWriterLoop()
{
    while (true) {
        PendingRecordPtr record;
        {
            std::unique_lock<std::mutex> lock(pending_records_mutex_);
            pending_records_cv_.wait(lock, [this]() {
                const auto it = pending_records_.find(next_sequence_to_write_);
                return (it != pending_records_.end() && it->second->ready) ||
                       producers_done_.load();
            });

            if (producers_done_.load() && pending_records_.empty())
                break;

            auto it = pending_records_.find(next_sequence_to_write_);
            if (it == pending_records_.end() && producers_done_.load()) {
                it = pending_records_.begin();
                next_sequence_to_write_ = it->first;
            }
            if (it == pending_records_.end())
                continue;
            if (!it->second->ready && producers_done_.load()) {
                it->second->capture.association_mode = "frame_latched_capture_incomplete";
                it->second->ready = true;
            }
            if (!it->second->ready)
                continue;
            record = it->second;
            pending_records_.erase(it);
            ++next_sequence_to_write_;
        }

        if (!encodeJpegResponses(record->responses) &&
            record->capture.association_mode == "frame_latched_freerun") {
            record->capture.association_mode = "frame_latched_encode_failed";
        }

        if (recording_file_ && record->write_recording_file)
            recording_file_->appendRecord(record->responses, record->capture);

        if (rosbag_active_.load() && rosbag_writer_ && rosbag_writer_->isRecording()) {
            rosbag_writer_->appendRecord(record->responses, record->capture, record->imu_batches);
            if (!rosbag_writer_->isRecording()) {
                rosbag_active_.store(false);
                rosbag_history_active_.store(false);
                stopAllRosbagImuHistory(false);
            }
        }
    }

    if (rosbag_writer_)
        rosbag_writer_->stopRecording();
}

void FRecordingThread::Stop()
{
    stop_task_counter_.Increment();
    is_ready_.store(false);
    if (image_cancellation_)
        image_cancellation_->store(true);
    pending_records_cv_.notify_all();
}

void FRecordingThread::Exit()
{
    stopAllRosbagImuHistory(false);
    if (rosbag_writer_)
        rosbag_writer_->stopRecording();
    if (recording_file_)
        recording_file_.reset();
    finishing_signal_.signal();
}
