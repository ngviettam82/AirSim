#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "Recording/RecordingFile.h"
#include "Recording/RosbagWriter.h"
#include "UnrealImageCapture.h"
#include "physics/Kinematics.hpp"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include "common/ClockFactory.hpp"
#include "common/AirSimSettings.hpp"
#include "common/WorkerThread.hpp"
#include "common/RecordingCapture.hpp"
#include "common/common_utils/UniqueValueMap.hpp"

class IImageWrapperModule;

class FRecordingThread : public FRunnable
{
public:
    typedef msr::airlib::AirSimSettings::RecordingSetting RecordingSetting;
    typedef msr::airlib::VehicleSimApiBase VehicleSimApiBase;
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    using PhysicsLockFn = std::function<void(const std::function<void()>&)>;

public:
    FRecordingThread();
    virtual ~FRecordingThread();

    static void init();
    static void startRecording(const RecordingSetting& settings,
                               const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& vehicle_sim_apis,
                               PhysicsLockFn physics_lock = nullptr);
    static void stopRecording();
    static void killRecording();
    static bool isRecording();

protected:
    virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;
    virtual void Exit() override;

private:
    struct PendingRecord
    {
        msr::airlib::RecordingCapture capture;
        std::vector<ImageCaptureBase::ImageResponse> responses;
        std::vector<msr::airlib::RecordingImuBatch> imu_batches;
        bool write_recording_file = true;
        bool ready = false;
    };

    using PendingRecordPtr = std::shared_ptr<PendingRecord>;

    void runSensorLoop(float sensor_interval);
    void runImageLoop(float image_interval);
    void runWriterLoop();
    void enqueueRecord(const PendingRecordPtr& record);
    void completeRecord(const PendingRecordPtr& record,
                        std::vector<ImageCaptureBase::ImageResponse>&& responses,
                        bool capture_failed);
    bool encodeJpegResponses(std::vector<ImageCaptureBase::ImageResponse>& responses) const;
    bool encodeJpegResponse(ImageCaptureBase::ImageResponse& response) const;
    size_t startRosbagImuHistory();
    std::vector<msr::airlib::RecordingImuBatch> drainRosbagImuHistory(VehicleSimApiBase* vehicle_sim_api);
    std::vector<msr::airlib::RecordingImuBatch> stopRosbagImuHistory(VehicleSimApiBase* vehicle_sim_api);
    void stopAllRosbagImuHistory(bool enqueue_final_records);
    bool startThread();
    static void waitForFinishingInstance();
    int64_t imageSyncToleranceNanos() const;

    FThreadSafeCounter stop_task_counter_;

    static std::unique_ptr<FRecordingThread> running_instance_;
    static std::unique_ptr<FRecordingThread> finishing_instance_;
    static msr::airlib::WorkerThreadSignal finishing_signal_;
    static bool first_;
    static std::atomic<uint64_t> next_sequence_id_;

    std::unique_ptr<FRunnableThread> thread_;

    RecordingSetting settings_;
    std::unique_ptr<RecordingFile> recording_file_;
    std::unique_ptr<RosbagWriter> rosbag_writer_;
    IImageWrapperModule* image_wrapper_module_ = nullptr;
    common_utils::UniqueValueMap<std::string, VehicleSimApiBase*> vehicle_sim_apis_;
    std::unordered_map<std::string, const UnrealImageCapture*> image_captures_;
    std::unordered_map<std::string, msr::airlib::Pose> last_sensor_poses_;
    std::unordered_map<std::string, msr::airlib::Pose> last_image_poses_;
    std::unordered_map<std::string, msr::airlib::TTimePoint> last_rosbag_auxiliary_sensor_time_stamps_;
    PhysicsLockFn physics_lock_;

    std::thread image_thread_;
    std::thread writer_thread_;
    std::mutex pending_records_mutex_;
    std::condition_variable pending_records_cv_;
    std::map<uint64_t, PendingRecordPtr> pending_records_;
    uint64_t next_sequence_to_write_ = 1;
    std::atomic<bool> producers_done_{ false };
    std::shared_ptr<std::atomic<bool>> image_cancellation_;
    // Bag output and source history have separate lifetimes: history closes
    // before producer shutdown, while the writer remains active long enough
    // to flush that final native IMU batch.
    std::atomic<bool> rosbag_active_{ false };
    std::atomic<bool> rosbag_history_active_{ false };

    // Dual-rate free-run: sensors at sensor_record_interval, images at record_interval.
    msr::airlib::TTimePoint last_sensor_on_;
    msr::airlib::TTimePoint last_image_on_;
    std::atomic<bool> is_ready_{ false };
};
