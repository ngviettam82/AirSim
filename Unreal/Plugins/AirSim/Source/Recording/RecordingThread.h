#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "Recording/RecordingFile.h"
#include "physics/Kinematics.hpp"
#include <memory>
#include <functional>
#include <atomic>
#include "common/ClockFactory.hpp"
#include "common/AirSimSettings.hpp"
#include "common/WorkerThread.hpp"
#include "common/RecordingCapture.hpp"
#include "common/common_utils/UniqueValueMap.hpp"

class FRecordingThread : public FRunnable
{
public:
    typedef msr::airlib::AirSimSettings::RecordingSetting RecordingSetting;
    typedef msr::airlib::VehicleSimApiBase VehicleSimApiBase;
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    using PhysicsLockFn = std::function<void(const std::function<void()>&)>;
    using PhysicsPauseFn = std::function<void(bool /*pause*/)>;

public:
    FRecordingThread();
    virtual ~FRecordingThread();

    static void init();
    static void startRecording(const RecordingSetting& settings,
                               const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& vehicle_sim_apis,
                               PhysicsLockFn physics_lock = nullptr,
                               PhysicsPauseFn physics_pause = nullptr);
    static void stopRecording();
    static void killRecording();
    static bool isRecording();

protected:
    virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;
    virtual void Exit() override;

private:
    FThreadSafeCounter stop_task_counter_;

    static std::unique_ptr<FRecordingThread> running_instance_;
    static std::unique_ptr<FRecordingThread> finishing_instance_;
    static msr::airlib::WorkerThreadSignal finishing_signal_;
    static bool first_;
    static std::atomic<uint64_t> next_sequence_id_;

    std::unique_ptr<FRunnableThread> thread_;

    RecordingSetting settings_;
    std::unique_ptr<RecordingFile> recording_file_;
    common_utils::UniqueValueMap<std::string, VehicleSimApiBase*> vehicle_sim_apis_;
    std::unordered_map<std::string, const ImageCaptureBase*> image_captures_;
    std::unordered_map<std::string, msr::airlib::Pose> last_poses_;
    PhysicsLockFn physics_lock_;
    PhysicsPauseFn physics_pause_;

    msr::airlib::TTimePoint last_screenshot_on_;
    std::atomic<bool> is_ready_{ false };
};
