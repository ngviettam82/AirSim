// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ImuBase_hpp
#define msr_airlib_ImuBase_hpp

#include "sensors/SensorBase.hpp"
#include "common/RecordingCapture.hpp"
#include <algorithm>
#include <atomic>
#include <deque>
#include <mutex>
#include <utility>

namespace msr
{
namespace airlib
{

    class ImuBase : public SensorBase
    {
    public:
        ImuBase(const std::string& sensor_name = "")
            : SensorBase(sensor_name)
        {
        }

    public: //types
        struct Output
        { //structure is same as ROS IMU message
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            TTimePoint time_stamp;
            Quaternionr orientation;
            Vector3r angular_velocity;
            Vector3r linear_acceleration;
        };

    public:
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            UpdatableObject::reportState(reporter);

            reporter.writeValue("IMU-Ang", output_.angular_velocity);
            reporter.writeValue("IMU-Lin", output_.linear_acceleration);
        }

        const Output& getOutput() const
        {
            return output_;
        }

        // Recording history is opt-in. When disabled, update-time overhead is
        // a single atomic branch and no mutex, allocation, or copy occurs.
        void enableRecordingHistory(size_t max_samples) const
        {
            const size_t bounded_capacity = std::max<size_t>(1, max_samples);
            std::lock_guard<std::mutex> lock(recording_history_mutex_);
            recording_history_.clear();
            recording_history_dropped_ = 0;
            recording_history_capacity_ = bounded_capacity;
            recording_history_enabled_.store(true, std::memory_order_release);
        }

        RecordingImuBatch drainRecordingHistory() const
        {
            return takeRecordingHistory(false);
        }

        // Atomically stops future history appends and returns all samples that
        // raced with recorder shutdown. This is used before MCAP finalization.
        RecordingImuBatch disableAndDrainRecordingHistory() const
        {
            recording_history_enabled_.store(false, std::memory_order_release);
            return takeRecordingHistory(true);
        }

        bool isRecordingHistoryEnabled() const
        {
            return recording_history_enabled_.load(std::memory_order_acquire);
        }

    protected:
        void setOutput(const Output& output)
        {
            output_ = output;
            appendRecordingHistory(output);
        }

    private:
        void appendRecordingHistory(const Output& output) const
        {
            if (!recording_history_enabled_.load(std::memory_order_relaxed))
                return;

            RecordingImuSample sample;
            sample.time_stamp = output.time_stamp;
            sample.physics_step_id = clock()->getStepCount();
            sample.orientation = output.orientation;
            sample.angular_velocity = output.angular_velocity;
            sample.linear_acceleration = output.linear_acceleration;

            std::lock_guard<std::mutex> lock(recording_history_mutex_);
            if (!recording_history_enabled_.load(std::memory_order_acquire))
                return;

            if (recording_history_.size() >= recording_history_capacity_) {
                recording_history_.pop_front();
                ++recording_history_dropped_;
            }
            recording_history_.push_back(std::move(sample));
        }

        RecordingImuBatch takeRecordingHistory(bool clear_capacity) const
        {
            std::deque<RecordingImuSample> samples;
            RecordingImuBatch batch;
            batch.sensor_name = getName();
            {
                std::lock_guard<std::mutex> lock(recording_history_mutex_);
                samples.swap(recording_history_);
                batch.dropped_samples = recording_history_dropped_;
                recording_history_dropped_ = 0;
                if (clear_capacity)
                    recording_history_capacity_ = 0;
            }

            batch.samples.reserve(samples.size());
            while (!samples.empty()) {
                batch.samples.emplace_back(std::move(samples.front()));
                samples.pop_front();
            }
            return batch;
        }

    private:
        Output output_{};
        mutable std::atomic<bool> recording_history_enabled_{ false };
        mutable std::mutex recording_history_mutex_;
        mutable std::deque<RecordingImuSample> recording_history_;
        mutable size_t recording_history_capacity_ = 0;
        mutable uint64_t recording_history_dropped_ = 0;
    };
}
} //namespace
#endif
