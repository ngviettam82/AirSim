#include "RecordingThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "HAL/RunnableThread.h"
#include "Kismet/KismetSystemLibrary.h"

#include <thread>
#include "common/ClockFactory.hpp"

std::unique_ptr<FRecordingThread> FRecordingThread::running_instance_;
std::unique_ptr<FRecordingThread> FRecordingThread::finishing_instance_;
msr::airlib::WorkerThreadSignal FRecordingThread::finishing_signal_;
bool FRecordingThread::first_ = true;
std::atomic<uint64_t> FRecordingThread::next_sequence_id_{ 1 };

FRecordingThread::FRecordingThread()
    : stop_task_counter_(0), recording_file_(nullptr)
{
    thread_.reset(FRunnableThread::Create(this, TEXT("FRecordingThread"), 0, TPri_BelowNormal));
}

void FRecordingThread::startRecording(const RecordingSetting& settings,
                                      const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& vehicle_sim_apis,
                                      PhysicsLockFn physics_lock,
                                      PhysicsPauseFn physics_pause)
{
    stopRecording();
    // Wait for previous worker to fully exit before starting a new session.
    if (finishing_instance_) {
        finishing_signal_.waitForRetry(1, 30);
        finishing_instance_.reset();
    }

    running_instance_.reset(new FRecordingThread());
    running_instance_->settings_ = settings;
    running_instance_->vehicle_sim_apis_ = vehicle_sim_apis;
    running_instance_->physics_lock_ = physics_lock
        ? std::move(physics_lock)
        : PhysicsLockFn([](const std::function<void()>& work) { work(); });
    running_instance_->physics_pause_ = physics_pause
        ? std::move(physics_pause)
        : PhysicsPauseFn([](bool) {});

    for (const auto& vehicle_sim_api : vehicle_sim_apis) {
        auto vehicle_name = vehicle_sim_api->getVehicleName();
        running_instance_->image_captures_[vehicle_name] = vehicle_sim_api->getImageCapture();
        running_instance_->last_poses_[vehicle_name] = msr::airlib::Pose();
    }

    running_instance_->last_screenshot_on_ = 0;
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
}

FRecordingThread::~FRecordingThread()
{
    if (this == running_instance_.get())
        stopRecording();
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
    if (finishing_instance_) {
        finishing_signal_.waitForRetry(1, 30);
        finishing_instance_.reset();
    }
    finishing_instance_ = std::move(running_instance_);
    finishing_instance_->is_ready_.store(false);
    finishing_instance_->Stop();
}

void FRecordingThread::killRecording()
{
    stopRecording();
    if (!finishing_instance_)
        return;

    bool finished = finishing_signal_.waitForRetry(1, 5);
    if (!finished && finishing_instance_->thread_) {
        UE_LOG(LogTemp, Log, TEXT("killing recording thread"));
        finishing_instance_->thread_->Kill(false);
    }
    finishing_instance_.reset();
}

bool FRecordingThread::Init()
{
    // Previous worker is fully joined in startRecording/stopRecording before a
    // new instance is created. Do NOT wait on finishing_signal_ here: that signal
    // is already consumed by startRecording's waitForRetry, and a second wait
    // deadlocks restart (start/stop/start).
    first_ = false;
    if (recording_file_) {
        UAirBlueprintLib::LogMessage(TEXT("Initiated recording thread"), TEXT(""), LogDebugLevel::Success);
    }
    return true;
}

uint32 FRecordingThread::Run()
{
    while (stop_task_counter_.GetValue() == 0) {
        if (!is_ready_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const bool interval_elapsed =
            msr::airlib::ClockFactory::get()->elapsedSince(last_screenshot_on_) > settings_.record_interval;
        if (!interval_elapsed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        last_screenshot_on_ = msr::airlib::ClockFactory::get()->nowNanos();

        for (const auto& vehicle_sim_api : vehicle_sim_apis_) {
            if (stop_task_counter_.GetValue() != 0)
                break;

            const auto& vehicle_name = vehicle_sim_api->getVehicleName();
            std::vector<std::string> sensor_names;
            auto sensor_it = settings_.sensors.find(vehicle_name);
            if (sensor_it != settings_.sensors.end())
                sensor_names = sensor_it->second;

            const auto schema_tokens =
                msr::airlib::RecordingCapture::buildSchemaTokens(settings_.sensor_schema);

            // Capture transaction: pause physics so snapshot + images share one
            // frozen discrete state. Do NOT hold the physics lock across getImages
            // or RunCommandOnGameThread (would deadlock with game-thread Tick).
            bool paused = false;
            msr::airlib::RecordingCapture capture;
            try {
                physics_pause_(true);
                paused = true;

                const uint64_t sequence_id = next_sequence_id_.fetch_add(1);
                physics_lock_([&]() {
                    capture = vehicle_sim_api->createRecordingCapture(
                        sequence_id, sensor_names, schema_tokens);
                    // Copy physics → last_phys_pose_ under lock while world is frozen.
                    // Do not touch UObject/AActor state here (worker thread).
                    vehicle_sim_api->updateRenderedState(0);
                    capture.association_mode = "physics_paused_snapshot";
                });

                if (settings_.record_on_move && last_poses_[vehicle_name] == capture.pose) {
                    physics_pause_(false);
                    paused = false;
                    continue;
                }
                last_poses_[vehicle_name] = capture.pose;

                // Snap pawn/camera actors to the frozen physics pose on the game
                // thread so CaptureScene matches kinematics/sensors.
                UAirBlueprintLib::RunCommandOnGameThread([vehicle_sim_api]() {
                    vehicle_sim_api->updateRendering(0);
                }, true);

                std::vector<ImageCaptureBase::ImageResponse> responses;
                auto req_it = settings_.requests.find(vehicle_name);
                if (req_it != settings_.requests.end() && !req_it->second.empty()) {
                    image_captures_[vehicle_name]->getImages(req_it->second, responses);
                    for (const auto& r : responses)
                        capture.image_time_stamps.push_back(r.time_stamp);
                    capture.render_frame_number =
                        static_cast<uint64_t>(UKismetSystemLibrary::GetFrameCount());
                }

                physics_pause_(false);
                paused = false;

                if (stop_task_counter_.GetValue() == 0 && recording_file_)
                    recording_file_->appendRecord(responses, capture);
            }
            catch (...) {
                if (paused)
                    physics_pause_(false);
                UAirBlueprintLib::LogMessageString("Recording sample failed", vehicle_name, LogDebugLevel::Failure);
            }
        }
    }

    recording_file_.reset();
    return 0;
}

void FRecordingThread::Stop()
{
    stop_task_counter_.Increment();
    is_ready_.store(false);
}

void FRecordingThread::Exit()
{
    if (recording_file_)
        recording_file_.reset();
    finishing_signal_.signal();
}
