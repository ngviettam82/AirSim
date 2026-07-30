#include "RecordingFile.h"
#include "HAL/PlatformFileManager.h"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "common/common_utils/FileSystem.hpp"

void RecordingFile::appendRecord(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
                                 const msr::airlib::RecordingCapture& capture) const
{
    std::ostringstream image_file_names;
    std::ostringstream image_request_time_stamps;
    std::ostringstream image_time_stamps;
    std::ostringstream image_delays_ns;
    std::ostringstream image_delays_ms;
    std::ostringstream image_sync_flags;
    int saved = 0;

    for (size_t i = 0; i < responses.size(); ++i) {
        const auto& response = responses[i];
        if ((!response.message.empty() && response.width == 0 && response.height == 0) ||
            (response.image_data_uint8.empty() && response.image_data_float.empty())) {
            continue; // do not list failed captures
        }
        if (response.recording_jpeg && !response.pixels_as_float && !response.compress) {
            UE_LOG(LogTemp, Error, TEXT("Skipping unencoded JPEG recording image from %s"),
                   UTF8_TO_TCHAR(response.camera_name.c_str()));
            continue;
        }

        std::ostringstream image_file_name;
        image_file_name << "img_" << capture.vehicle_name << "_"
                        << response.camera_name << "_"
                        << common_utils::Utils::toNumeric(response.image_type);
        if (!response.annotation_name.empty())
            image_file_name << "_" << response.annotation_name;
        image_file_name << "_s" << capture.sequence_id
                        << "_p" << capture.physics_step_id
                        << "_fs" << capture.frame_time_stamp;

        const bool has_render_frame_time =
            response.has_render_frame_timestamp && response.time_stamp != 0;
        const bool has_timing = has_render_frame_time && capture.frame_time_stamp != 0;
        const int64_t delay_ns = has_timing
            ? static_cast<int64_t>(response.time_stamp) - static_cast<int64_t>(capture.frame_time_stamp)
            : 0;
        const int64_t delay_us = delay_ns / 1000;
        const bool within_tolerance = has_timing &&
            std::llabs(delay_ns) <= capture.image_sync_tolerance_ns;
        if (has_render_frame_time)
            image_file_name << "_it" << response.time_stamp;
        if (has_timing) {
            image_file_name << "_dt" << (delay_ns < 0 ? "m" : "p")
                            << std::llabs(delay_us) << "us_"
                            << (within_tolerance ? "ok" : "over");
        }
        else {
            image_file_name << "_dt_unknown";
        }

        std::string extension = ".jpg";
        if (response.pixels_as_float)
            extension = ".pfm";
        else if (!response.compress)
            extension = ".ppm";
        image_file_name << extension;

        const std::string fname = image_file_name.str();
        const std::string full = common_utils::FileSystem::combine(image_path_, fname);
        try {
            if (extension == ".pfm") {
                common_utils::Utils::writePFMfile(response.image_data_float.data(), response.width, response.height, full);
            }
            else if (extension == ".ppm") {
                common_utils::Utils::writePPMfile(response.image_data_uint8.data(), response.width, response.height, full);
            }
            else {
                std::ofstream file(full, std::ios::binary);
                file.exceptions(std::ios::failbit | std::ios::badbit);
                file.write(reinterpret_cast<const char*>(response.image_data_uint8.data()),
                           static_cast<std::streamsize>(response.image_data_uint8.size()));
                file.close();
            }
            if (saved > 0)
            {
                image_file_names << ";";
                image_request_time_stamps << ";";
                image_time_stamps << ";";
                image_delays_ns << ";";
                image_delays_ms << ";";
                image_sync_flags << ";";
            }
            image_file_names << fname;
            image_request_time_stamps << response.request_time_stamp;
            if (has_render_frame_time)
                image_time_stamps << response.time_stamp;
            if (has_timing) {
                image_delays_ns << delay_ns;
                image_delays_ms << std::fixed << std::setprecision(6)
                                << static_cast<double>(delay_ns) / 1.0E6;
                image_sync_flags << (within_tolerance ? "1" : "0");
            }
            ++saved;
        }
        catch (std::exception& ex) {
            UE_LOG(LogTemp, Error, TEXT("Image file save failed: %s"), UTF8_TO_TCHAR(ex.what()));
        }
    }

    std::ostringstream metadata;
    metadata << capture.association_mode << "\t"
             << image_request_time_stamps.str() << "\t"
             << image_time_stamps.str() << "\t"
             << image_delays_ns.str() << "\t"
             << image_delays_ms.str() << "\t"
             << image_sync_flags.str() << "\t"
             << image_file_names.str() << "\n";
    writeString(capture.toRecordLine().append(metadata.str()));
}

void RecordingFile::appendColumnHeader(const std::string& header_columns)
{
    writeString(header_columns + "\n");
}

void RecordingFile::createFile(const std::string& file_path, const std::string& header_columns)
{
    try {
        closeFile();
        IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
        log_file_handle_ = platform_file.OpenWrite(*FString(file_path.c_str()));
        appendColumnHeader(header_columns);
    }
    catch (std::exception& ex) {
        UE_LOG(LogTemp, Error, TEXT("Failed to create recording file %s: %s"),
               UTF8_TO_TCHAR(file_path.c_str()), UTF8_TO_TCHAR(ex.what()));
    }
}

bool RecordingFile::isFileOpen() const
{
    return log_file_handle_ != nullptr;
}

void RecordingFile::closeFile()
{
    if (isFileOpen())
        delete log_file_handle_;
    log_file_handle_ = nullptr;
}

void RecordingFile::writeString(const std::string& str) const
{
    try {
        if (log_file_handle_) {
            FString line_f(str.c_str());
            log_file_handle_->Write((const uint8*)TCHAR_TO_ANSI(*line_f), line_f.Len());
        }
        else {
            UE_LOG(LogTemp, Error, TEXT("Attempted to write to a recording log file that was not open"));
        }
    }
    catch (std::exception& ex) {
        UE_LOG(LogTemp, Error, TEXT("Recording file write failed: %s"), UTF8_TO_TCHAR(ex.what()));
    }
}

RecordingFile::~RecordingFile()
{
    stopRecording(true);
}

bool RecordingFile::startRecording(const msr::airlib::RecordingCapture& header_template, const std::string& folder)
{
    try {
        session_path_ = common_utils::FileSystem::getLogFolderPath(true, folder);
        image_path_ = common_utils::FileSystem::ensureFolder(session_path_, "images");
        std::string log_filepath = common_utils::FileSystem::getLogFileNamePath(session_path_, record_filename, "", ".txt", false);
        if (log_filepath != "")
            createFile(log_filepath, header_template.fullHeaderLine());
        else {
            UE_LOG(LogTemp, Error, TEXT("Cannot start recording because a log-file path is not available"));
            return false;
        }
        if (isFileOpen()) {
            is_recording_ = true;
            UE_LOG(LogTemp, Log, TEXT("Recording started"));
            return true;
        }
        else {
            UE_LOG(LogTemp, Error, TEXT("Error creating recording log file: %s"),
                   UTF8_TO_TCHAR(log_filepath.c_str()));
            return false;
        }
    }
    catch (...) {
        UE_LOG(LogTemp, Error, TEXT("Error starting recording"));
        return false;
    }
}

void RecordingFile::stopRecording(bool ignore_if_stopped)
{
    is_recording_ = false;
    if (!isFileOpen()) {
        if (ignore_if_stopped)
            return;
        UE_LOG(LogTemp, Error, TEXT("Recording file was not open during stop"));
    }
    else {
        closeFile();
    }
    UE_LOG(LogTemp, Log, TEXT("Recording stopped; data saved to %s"), UTF8_TO_TCHAR(image_path_.c_str()));
}

bool RecordingFile::isRecording() const
{
    return is_recording_;
}

const std::string& RecordingFile::sessionPath() const
{
    return session_path_;
}
