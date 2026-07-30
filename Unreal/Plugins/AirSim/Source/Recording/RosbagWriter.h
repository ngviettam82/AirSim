// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/ImageCaptureBase.hpp"
#include "common/RecordingCapture.hpp"

// Writes a self-contained ROS 2-compatible MCAP stream. Ownership is limited
// to FRecordingThread's ordered writer worker; no Unreal objects are accessed
// here, only immutable CPU data handed off by the recorder.
class RosbagWriter
{
public:
    RosbagWriter();
    ~RosbagWriter();

    RosbagWriter(const RosbagWriter&) = delete;
    RosbagWriter& operator=(const RosbagWriter&) = delete;

    bool startRecording(const std::string& session_folder, const std::string& configured_file_name);
    void appendRecord(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
                      const msr::airlib::RecordingCapture& capture,
                      const std::vector<msr::airlib::RecordingImuBatch>& imu_batches);
    void stopRecording();
    // Discards an active, not-yet-finalized recording without publishing MCAP.
    void abortRecording();

    bool isRecording() const;
    std::string outputPath() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
