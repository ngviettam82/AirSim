#include "RecordingThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/RunnableThread.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <thread>
#include <utility>
#include "common/ClockFactory.hpp"

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
    running_instance_->recording_file_->startRecording(header_template, settings.folder);
    running_instance_->is_ready_.store(true);
    if (!running_instance_->startThread()) {
        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("Failed to create recording worker"), LogDebugLevel::Failure);
        std::unique_ptr<FRecordingThread> failed_instance = std::move(running_instance_);
        failed_instance->is_ready_.store(false);
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
        UAirBlueprintLib::LogMessage(TEXT("Initiated recording thread (freerun dual-rate)"), TEXT(""), LogDebugLevel::Success);
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

    producers_done_.store(true);
    pending_records_cv_.notify_all();
    if (writer_thread_.joinable())
        writer_thread_.join();

    recording_file_.reset();
    return 0;
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
            record->capture.image_time_stamps.push_back(response.time_stamp);
            const bool has_timing = response.time_stamp != 0 && record->capture.frame_time_stamp != 0;
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

                if (settings_.record_on_move && last_sensor_poses_[vehicle_name] == capture.pose)
                    continue;
                last_sensor_poses_[vehicle_name] = capture.pose;

                capture.sequence_id = next_sequence_id_.fetch_add(1);
                capture.association_mode = "sensor_only";
                capture.image_sync_tolerance_ns = imageSyncToleranceNanos();
                auto record = std::make_shared<PendingRecord>();
                record->capture = std::move(capture);
                record->ready = true;
                enqueueRecord(record);
            }
            catch (...) {
                UAirBlueprintLib::LogMessageString("Recording sensor sample failed", vehicle_name, LogDebugLevel::Failure);
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
                    UAirBlueprintLib::LogMessageString("Recording image snapshot failed", vehicle_name, LogDebugLevel::Failure);
                }
            };

            try {
                capture_it->second->getImagesForRecording(
                    request_it->second, responses, image_cancellation_, std::move(prepare_capture));
            }
            catch (...) {
                capture_failed = true;
                UAirBlueprintLib::LogMessageString("Recording image capture failed", vehicle_name, LogDebugLevel::Failure);
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

        if (recording_file_)
            recording_file_->appendRecord(record->responses, record->capture);
    }
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
    if (recording_file_)
        recording_file_.reset();
    finishing_signal_.signal();
}
