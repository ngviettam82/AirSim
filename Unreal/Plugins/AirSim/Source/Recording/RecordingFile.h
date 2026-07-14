#pragma once

#include "CoreMinimal.h"
#include <string>
#include <vector>
#include "AirBlueprintLib.h"
#include "physics/Kinematics.hpp"
#include "HAL/FileManager.h"
#include "api/VehicleSimApiBase.hpp"
#include "common/RecordingCapture.hpp"
#include "common/ImageCaptureBase.hpp"

class RecordingFile
{
public:
    ~RecordingFile();

    // Write a request-local capture transaction. Image I/O does not re-sample pose/sensors.
    void appendRecord(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
                      const msr::airlib::RecordingCapture& capture) const;

    void appendColumnHeader(const std::string& header_columns);
    void startRecording(const msr::airlib::RecordingCapture& header_template,
                        const std::string& folder = "");
    void stopRecording(bool ignore_if_stopped);
    bool isRecording() const;

private:
    void createFile(const std::string& file_path, const std::string& header_columns);
    void closeFile();
    void writeString(const std::string& line) const;
    bool isFileOpen() const;

private:
    std::string record_filename = "airsim_rec";
    std::string image_path_;
    bool is_recording_ = false;
    IFileHandle* log_file_handle_ = nullptr;
};
