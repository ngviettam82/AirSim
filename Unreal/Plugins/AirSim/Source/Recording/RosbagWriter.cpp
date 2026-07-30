// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "Recording/RosbagWriter.h"

#include "CoreMinimal.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/Paths.h"

#include "common/Ros2Cdr.hpp"
#include "common/Ros2TopicName.hpp"
#include "sensors/gps/GpsBase.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
    constexpr uint64_t kMcapMagicSize = 8;
    // DDS reserves 256 bytes including the NUL terminator for a topic name.
    // Keep generated ROS names below that limit so a collision suffix remains
    // publishable by ROS 2 implementations.
    constexpr size_t kMaxRosTopicLength = 255;
    const uint8 kMcapMagic[kMcapMagicSize] = { 137, 77, 67, 65, 80, 48, 13, 10 }; // \x89MCAP0\r\n

    enum class McapOpcode : uint8
    {
        Header = 0x01,
        Footer = 0x02,
        Schema = 0x03,
        Channel = 0x04,
        Message = 0x05,
        Statistics = 0x0B,
        SummaryOffset = 0x0E,
        DataEnd = 0x0F
    };

    enum class Ros2QosProfile : uint8
    {
        SensorData,
        ReliableMetadata
    };

    // Matches ROS 2 SensorDataQoS used by typical camera and IMU drivers.
    constexpr char kRos2SensorDataQos[] =
        "- history: 1\n"
        "  depth: 5\n"
        "  reliability: 2\n"
        "  durability: 2\n"
        "  deadline:\n"
        "    sec: 0\n"
        "    nsec: 0\n"
        "  lifespan:\n"
        "    sec: 0\n"
        "    nsec: 0\n"
        "  liveliness: 0\n"
        "  liveliness_lease_duration:\n"
        "    sec: 0\n"
        "    nsec: 0\n"
        "  avoid_ros_namespace_conventions: false\n";

    // Recorder diagnostic metadata is low-rate and should be retained by a
    // normal reliable ROS subscriber during replay.
    constexpr char kRos2ReliableMetadataQos[] =
        "- history: 1\n"
        "  depth: 10\n"
        "  reliability: 1\n"
        "  durability: 2\n"
        "  deadline:\n"
        "    sec: 0\n"
        "    nsec: 0\n"
        "  lifespan:\n"
        "    sec: 0\n"
        "    nsec: 0\n"
        "  liveliness: 0\n"
        "  liveliness_lease_duration:\n"
        "    sec: 0\n"
        "    nsec: 0\n"
        "  avoid_ros_namespace_conventions: false\n";

    const char* ros2QosProfile(Ros2QosProfile profile)
    {
        return profile == Ros2QosProfile::ReliableMetadata
            ? kRos2ReliableMetadataQos
            : kRos2SensorDataQos;
    }

    constexpr char kStringSchema[] =
        "string data\n";

    constexpr char kImageSchema[] =
        "std_msgs/Header header\n"
        "uint32 height\n"
        "uint32 width\n"
        "string encoding\n"
        "uint8 is_bigendian\n"
        "uint32 step\n"
        "uint8[] data\n"
        "================================================================================\n"
        "MSG: std_msgs/msg/Header\n"
        "builtin_interfaces/Time stamp\n"
        "string frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/msg/Time\n"
        "int32 sec\n"
        "uint32 nanosec\n";

    constexpr char kCompressedImageSchema[] =
        "std_msgs/Header header\n"
        "string format\n"
        "uint8[] data\n"
        "================================================================================\n"
        "MSG: std_msgs/msg/Header\n"
        "builtin_interfaces/Time stamp\n"
        "string frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/msg/Time\n"
        "int32 sec\n"
        "uint32 nanosec\n";

    constexpr char kCameraInfoSchema[] =
        "std_msgs/Header header\n"
        "uint32 height\n"
        "uint32 width\n"
        "string distortion_model\n"
        "float64[] d\n"
        "float64[9] k\n"
        "float64[9] r\n"
        "float64[12] p\n"
        "uint32 binning_x\n"
        "uint32 binning_y\n"
        "sensor_msgs/RegionOfInterest roi\n"
        "================================================================================\n"
        "MSG: std_msgs/msg/Header\n"
        "builtin_interfaces/Time stamp\n"
        "string frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/msg/Time\n"
        "int32 sec\n"
        "uint32 nanosec\n"
        "================================================================================\n"
        "MSG: sensor_msgs/msg/RegionOfInterest\n"
        "uint32 x_offset\n"
        "uint32 y_offset\n"
        "uint32 height\n"
        "uint32 width\n"
        "bool do_rectify\n";

    constexpr char kImuSchema[] =
        "std_msgs/Header header\n"
        "geometry_msgs/Quaternion orientation\n"
        "float64[9] orientation_covariance\n"
        "geometry_msgs/Vector3 angular_velocity\n"
        "float64[9] angular_velocity_covariance\n"
        "geometry_msgs/Vector3 linear_acceleration\n"
        "float64[9] linear_acceleration_covariance\n"
        "================================================================================\n"
        "MSG: std_msgs/msg/Header\n"
        "builtin_interfaces/Time stamp\n"
        "string frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/msg/Time\n"
        "int32 sec\n"
        "uint32 nanosec\n"
        "================================================================================\n"
        "MSG: geometry_msgs/msg/Quaternion\n"
        "float64 x\n"
        "float64 y\n"
        "float64 z\n"
        "float64 w\n"
        "================================================================================\n"
        "MSG: geometry_msgs/msg/Vector3\n"
        "float64 x\n"
        "float64 y\n"
        "float64 z\n";

    constexpr char kNavSatFixSchema[] =
        "std_msgs/Header header\n"
        "sensor_msgs/NavSatStatus status\n"
        "float64 latitude\n"
        "float64 longitude\n"
        "float64 altitude\n"
        "float64[9] position_covariance\n"
        "uint8 position_covariance_type\n"
        "================================================================================\n"
        "MSG: std_msgs/msg/Header\n"
        "builtin_interfaces/Time stamp\n"
        "string frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/msg/Time\n"
        "int32 sec\n"
        "uint32 nanosec\n"
        "================================================================================\n"
        "MSG: sensor_msgs/msg/NavSatStatus\n"
        "int8 status\n"
        "uint16 service\n";

    constexpr char kMagneticFieldSchema[] =
        "std_msgs/Header header\n"
        "geometry_msgs/Vector3 magnetic_field\n"
        "float64[9] magnetic_field_covariance\n"
        "================================================================================\n"
        "MSG: std_msgs/msg/Header\n"
        "builtin_interfaces/Time stamp\n"
        "string frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/msg/Time\n"
        "int32 sec\n"
        "uint32 nanosec\n"
        "================================================================================\n"
        "MSG: geometry_msgs/msg/Vector3\n"
        "float64 x\n"
        "float64 y\n"
        "float64 z\n";

    constexpr char kAltimeterSchema[] =
        "std_msgs/Header header\n"
        "float32 altitude\n"
        "float32 pressure\n"
        "float32 qnh\n"
        "================================================================================\n"
        "MSG: std_msgs/msg/Header\n"
        "builtin_interfaces/Time stamp\n"
        "string frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/msg/Time\n"
        "int32 sec\n"
        "uint32 nanosec\n";

    const char* imageTypeName(msr::airlib::ImageCaptureBase::ImageType image_type)
    {
        using ImageType = msr::airlib::ImageCaptureBase::ImageType;
        switch (image_type) {
        case ImageType::Scene:
            return "Scene";
        case ImageType::DepthPlanar:
            return "DepthPlanar";
        case ImageType::DepthPerspective:
            return "DepthPerspective";
        case ImageType::DepthVis:
            return "DepthVis";
        case ImageType::DisparityNormalized:
            return "DisparityNormalized";
        case ImageType::Segmentation:
            return "Segmentation";
        case ImageType::SurfaceNormals:
            return "SurfaceNormals";
        case ImageType::Infrared:
            return "Infrared";
        case ImageType::OpticalFlow:
            return "OpticalFlow";
        case ImageType::OpticalFlowVis:
            return "OpticalFlowVis";
        case ImageType::Lighting:
            return "Lighting";
        case ImageType::Annotation:
            return "Annotation";
        default:
            return "Unknown";
        }
    }

    bool hasImageData(const msr::airlib::ImageCaptureBase::ImageResponse& response)
    {
        return (response.message.empty() || response.width != 0 || response.height != 0) &&
               response.width > 0 && response.height > 0 &&
               (!response.image_data_uint8.empty() || !response.image_data_float.empty());
    }

    std::string escapeJson(const std::string& value)
    {
        std::ostringstream escaped;
        for (const unsigned char ch : value) {
            switch (ch) {
            case '\\':
                escaped << "\\\\";
                break;
            case '"':
                escaped << "\\\"";
                break;
            case '\b':
                escaped << "\\b";
                break;
            case '\f':
                escaped << "\\f";
                break;
            case '\n':
                escaped << "\\n";
                break;
            case '\r':
                escaped << "\\r";
                break;
            case '\t':
                escaped << "\\t";
                break;
            default:
                if (ch < 0x20) {
                    escaped << "\\u" << std::hex << std::uppercase << std::setw(4)
                            << std::setfill('0') << static_cast<int>(ch) << std::dec
                            << std::nouppercase << std::setfill(' ');
                }
                else {
                    escaped << static_cast<char>(ch);
                }
                break;
            }
        }
        return escaped.str();
    }

    std::string imageTopicPrefix(const msr::airlib::RecordingCapture& capture,
                                 const msr::airlib::ImageCaptureBase::ImageResponse& response)
    {
        std::ostringstream topic;
        topic << "/airsim_node/" << msr::airlib::normalizeRos2TopicToken(capture.vehicle_name, "vehicle")
              << "/" << msr::airlib::normalizeRos2TopicToken(response.camera_name, "camera")
              << "_" << imageTypeName(response.image_type);
        if (!response.annotation_name.empty())
            topic << "_" << msr::airlib::normalizeRos2TopicToken(response.annotation_name, "annotation");
        return topic.str();
    }

    std::string topicIdentity(const std::string& kind, const std::vector<std::string>& fields)
    {
        std::string identity = kind;
        for (const std::string& field : fields) {
            identity.push_back('\0');
            identity += field;
        }
        return identity;
    }
}

class RosbagWriter::Impl
{
public:
    struct Buffer
    {
        void addUint8(uint8 value)
        {
            data.push_back(value);
        }

        void addUint16(uint16 value)
        {
            addLittleEndian(value);
        }

        void addUint32(uint32 value)
        {
            addLittleEndian(value);
        }

        void addUint64(uint64 value)
        {
            addLittleEndian(value);
        }

        void addString(const std::string& value)
        {
            if (value.size() > static_cast<size_t>(std::numeric_limits<uint32>::max()))
                throw std::length_error("MCAP string exceeds uint32 length");
            addUint32(static_cast<uint32>(value.size()));
            data.insert(data.end(), value.begin(), value.end());
        }

        void addByteArray(const uint8* value, size_t size)
        {
            if (size > static_cast<size_t>(std::numeric_limits<uint32>::max()))
                throw std::length_error("MCAP byte array exceeds uint32 length");
            addUint32(static_cast<uint32>(size));
            if (size != 0) {
                if (value == nullptr)
                    throw std::invalid_argument("MCAP byte array has null data");
                data.insert(data.end(), value, value + size);
            }
        }

    private:
        template <typename T>
        void addLittleEndian(T value)
        {
            for (size_t i = 0; i < sizeof(T); ++i)
                data.push_back(static_cast<uint8>((value >> (i * 8)) & 0xFF));
        }

    public:
        std::vector<uint8> data;
    };

    struct Schema
    {
        uint16 id = 0;
        std::string name;
        std::string encoding;
        std::string definition;
    };

    struct Channel
    {
        uint16 id = 0;
        uint16 schema_id = 0;
        std::string topic;
        std::string message_encoding;
        std::map<std::string, std::string> metadata;
    };

    // Payload bytes are staged on disk while a recording is active.  The
    // small in-memory index lets finalization write every MCAP Message in
    // source-timestamp order without retaining camera payloads in RAM.
    struct PendingMessage
    {
        uint16 channel_id = 0;
        msr::airlib::TTimePoint time_stamp = 0;
        uint64 spool_offset = 0;
        uint64 payload_size = 0;
        uint64 insertion_order = 0;
    };

    bool start(const std::string& session_folder, const std::string& configured_file_name)
    {
        stop();
        try {
            const FString requested_name = UTF8_TO_TCHAR(configured_file_name.c_str());
            FString file_name = FPaths::GetCleanFilename(requested_name);
            if (file_name.IsEmpty())
                file_name = TEXT("airsim_rec.mcap");
            if (!file_name.EndsWith(TEXT(".mcap"), ESearchCase::IgnoreCase))
                file_name += TEXT(".mcap");

            const FString folder = UTF8_TO_TCHAR(session_folder.c_str());
            output_path_ = FPaths::Combine(folder, file_name);
            temporary_output_path_ = output_path_ + TEXT(".tmp");
            temporary_spool_path_ = output_path_ + TEXT(".tmp.spool");
            IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
            spool_file_.reset(platform_file.OpenWrite(*temporary_spool_path_));
            if (!spool_file_)
                throw std::runtime_error("could not open MCAP payload spool");

            offset_ = 0;
            spool_size_ = 0;
            next_schema_id_ = 1;
            next_channel_id_ = 1;
            next_message_sequence_ = 0;
            next_pending_message_order_ = 0;
            schemas_.clear();
            channels_.clear();
            schema_ids_.clear();
            channel_ids_.clear();
            channel_message_counts_.clear();
            topic_identities_.clear();
            last_sensor_time_stamps_.clear();
            pending_messages_.clear();
            message_count_ = 0;
            message_start_time_ = 0;
            message_end_time_ = 0;
            finalized_ = false;

            is_recording_ = true;
            return true;
        }
        catch (const std::exception& ex) {
            fail("Cannot start ROS bag", ex.what());
            return false;
        }
    }

    void append(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
                const msr::airlib::RecordingCapture& capture,
                const std::vector<msr::airlib::RecordingImuBatch>& imu_batches)
    {
        if (!is_recording_)
            return;

        try {
            writeSensorSamples(capture);
            writeImuBatches(capture, imu_batches);
            writeImages(capture, responses);
        }
        catch (const std::exception& ex) {
            fail("ROS bag write failed", ex.what());
        }
    }

    void stop()
    {
        if (!is_recording_)
            return;

        try {
            if (!spool_file_ || !spool_file_->Flush(true))
                throw std::runtime_error("could not flush MCAP payload spool");

            IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
            spool_file_.reset();
            spool_file_.reset(platform_file.OpenRead(*temporary_spool_path_));
            if (!spool_file_)
                throw std::runtime_error("could not reopen MCAP payload spool");
            const int64 spool_size = spool_file_->Size();
            if (spool_size < 0)
                throw std::runtime_error("could not determine MCAP payload spool size");
            spool_size_ = static_cast<uint64>(spool_size);

            file_.reset(platform_file.OpenWrite(*temporary_output_path_));
            if (!file_)
                throw std::runtime_error("could not open finalized MCAP output");

            offset_ = 0;
            next_message_sequence_ = 0;
            channel_message_counts_.clear();
            message_count_ = 0;
            message_start_time_ = 0;
            message_end_time_ = 0;

            writeRaw(kMcapMagic, kMcapMagicSize);
            Buffer header;
            header.addString("ros2");
            header.addString("AirSim Unreal RosbagWriter");
            writeRecord(McapOpcode::Header, header.data);
            for (const auto& schema : schemas_)
                writeSchema(schema);
            for (const auto& channel : channels_)
                writeChannel(channel);

            std::sort(pending_messages_.begin(), pending_messages_.end(),
                      [](const PendingMessage& left, const PendingMessage& right) {
                          if (left.time_stamp != right.time_stamp)
                              return left.time_stamp < right.time_stamp;
                          return left.insertion_order < right.insertion_order;
                      });
            std::vector<uint8> spool_read_buffer(64 * 1024);
            for (const PendingMessage& message : pending_messages_)
                writeMessage(message, spool_read_buffer);

            Buffer data_end;
            data_end.addUint32(0); // Data-section CRC intentionally disabled.
            writeRecord(McapOpcode::DataEnd, data_end.data);

            const uint64 summary_start = offset_;
            const uint64 schema_start = offset_;
            for (const auto& schema : schemas_)
                writeSchema(schema);

            const uint64 channel_start = offset_;
            for (const auto& channel : channels_)
                writeChannel(channel);

            const uint64 statistics_start = offset_;
            writeStatistics();

            const uint64 summary_offset_start = offset_;
            if (!schemas_.empty())
                writeSummaryOffset(McapOpcode::Schema, schema_start, channel_start - schema_start);
            if (!channels_.empty())
                writeSummaryOffset(McapOpcode::Channel, channel_start, statistics_start - channel_start);
            writeSummaryOffset(McapOpcode::Statistics, statistics_start, summary_offset_start - statistics_start);

            Buffer footer;
            footer.addUint64(summary_start);
            footer.addUint64(summary_offset_start);
            footer.addUint32(0); // Summary CRC intentionally disabled.
            writeRecord(McapOpcode::Footer, footer.data);
            writeRaw(kMcapMagic, kMcapMagicSize);

            if (!file_->Flush(true))
                throw std::runtime_error("could not flush finalized MCAP output");
            file_.reset();
            spool_file_.reset();
            discardTemporarySpool();
            if (!platform_file.MoveFile(*output_path_, *temporary_output_path_))
                throw std::runtime_error("could not publish finalized MCAP output");
            finalized_ = true;
            pending_messages_.clear();
        }
        catch (const std::exception& ex) {
            file_.reset();
            spool_file_.reset();
            discardTemporaryOutput();
            discardTemporarySpool();
            UE_LOG(LogTemp, Error, TEXT("ROS bag finalization failed: %s"), UTF8_TO_TCHAR(ex.what()));
        }

        file_.reset();
        spool_file_.reset();
        is_recording_ = false;
    }

    void abort()
    {
        file_.reset();
        spool_file_.reset();
        discardTemporaryOutput();
        discardTemporarySpool();
        pending_messages_.clear();
        is_recording_ = false;
        finalized_ = false;
    }

    bool isRecording() const
    {
        return is_recording_;
    }

    std::string outputPath() const
    {
        return (is_recording_ || finalized_) && !output_path_.IsEmpty()
            ? std::string(TCHAR_TO_UTF8(*output_path_))
            : std::string();
    }

private:
    void writeRaw(const uint8* data, size_t size)
    {
        if (!file_)
            throw std::runtime_error("MCAP output file is not open");
        if (size > static_cast<size_t>(std::numeric_limits<int64>::max()))
            throw std::length_error("MCAP write exceeds int64 length");
        if (size != 0 && !file_->Write(data, static_cast<int64>(size)))
            throw std::runtime_error("MCAP output write failed");
        offset_ += static_cast<uint64>(size);
    }

    void writeUint8(uint8 value)
    {
        writeRaw(&value, sizeof(value));
    }

    void writeUint16(uint16 value)
    {
        const uint8 bytes[2] = {
            static_cast<uint8>(value & 0xFF),
            static_cast<uint8>((value >> 8) & 0xFF)
        };
        writeRaw(bytes, sizeof(bytes));
    }

    void writeUint32(uint32 value)
    {
        const uint8 bytes[4] = {
            static_cast<uint8>(value & 0xFF),
            static_cast<uint8>((value >> 8) & 0xFF),
            static_cast<uint8>((value >> 16) & 0xFF),
            static_cast<uint8>((value >> 24) & 0xFF)
        };
        writeRaw(bytes, sizeof(bytes));
    }

    void writeUint64(uint64 value)
    {
        uint8 bytes[8];
        for (size_t i = 0; i < sizeof(bytes); ++i)
            bytes[i] = static_cast<uint8>((value >> (i * 8)) & 0xFF);
        writeRaw(bytes, sizeof(bytes));
    }

    void writeRecord(McapOpcode opcode, const std::vector<uint8>& payload)
    {
        writeUint8(static_cast<uint8>(opcode));
        writeUint64(static_cast<uint64>(payload.size()));
        if (!payload.empty())
            writeRaw(payload.data(), payload.size());
    }

    void writeSchema(const Schema& schema)
    {
        Buffer payload;
        payload.addUint16(schema.id);
        payload.addString(schema.name);
        payload.addString(schema.encoding);
        payload.addByteArray(reinterpret_cast<const uint8*>(schema.definition.data()), schema.definition.size());
        writeRecord(McapOpcode::Schema, payload.data);
    }

    void writeChannel(const Channel& channel)
    {
        Buffer metadata;
        for (const auto& entry : channel.metadata) {
            metadata.addString(entry.first);
            metadata.addString(entry.second);
        }

        Buffer payload;
        payload.addUint16(channel.id);
        payload.addUint16(channel.schema_id);
        payload.addString(channel.topic);
        payload.addString(channel.message_encoding);
        payload.addUint32(static_cast<uint32>(metadata.data.size()));
        payload.data.insert(payload.data.end(), metadata.data.begin(), metadata.data.end());
        writeRecord(McapOpcode::Channel, payload.data);
    }

    void writeMessage(const PendingMessage& message, std::vector<uint8>& spool_read_buffer)
    {
        if (!spool_file_)
            throw std::runtime_error("MCAP payload spool is not open for reading");
        if (spool_read_buffer.empty())
            throw std::invalid_argument("MCAP payload read buffer is empty");
        if (message.spool_offset > spool_size_ ||
            message.payload_size > spool_size_ - message.spool_offset)
            throw std::runtime_error("MCAP payload spool range is invalid");
        if (message.spool_offset > static_cast<uint64>(std::numeric_limits<int64>::max()) ||
            message.payload_size > static_cast<uint64>(std::numeric_limits<int64>::max()))
            throw std::length_error("MCAP payload spool range exceeds int64 length");

        constexpr uint64 kMessagePayloadPrefixSize = 2 + 4 + 8 + 8;
        if (message.payload_size > std::numeric_limits<uint64>::max() - kMessagePayloadPrefixSize)
            throw std::length_error("MCAP message payload length overflow");
        writeUint8(static_cast<uint8>(McapOpcode::Message));
        writeUint64(kMessagePayloadPrefixSize + message.payload_size);
        writeUint16(message.channel_id);
        writeUint32(next_message_sequence_++);
        writeUint64(message.time_stamp);
        writeUint64(message.time_stamp);

        uint64 read_offset = message.spool_offset;
        uint64 bytes_remaining = message.payload_size;
        while (bytes_remaining != 0) {
            const uint64 bytes_this_read = std::min<uint64>(
                bytes_remaining, static_cast<uint64>(spool_read_buffer.size()));
            if (!spool_file_->ReadAt(spool_read_buffer.data(),
                                     static_cast<int64>(bytes_this_read),
                                     static_cast<int64>(read_offset))) {
                throw std::runtime_error("MCAP payload spool read failed");
            }
            writeRaw(spool_read_buffer.data(), static_cast<size_t>(bytes_this_read));
            read_offset += bytes_this_read;
            bytes_remaining -= bytes_this_read;
        }

        ++message_count_;
        if (message_start_time_ == 0 || message.time_stamp < message_start_time_)
            message_start_time_ = message.time_stamp;
        if (message.time_stamp > message_end_time_)
            message_end_time_ = message.time_stamp;
        ++channel_message_counts_[message.channel_id];
    }

    void writeStatistics()
    {
        Buffer payload;
        payload.addUint64(message_count_);
        payload.addUint16(static_cast<uint16>(schemas_.size()));
        payload.addUint32(static_cast<uint32>(channels_.size()));
        payload.addUint32(0); // attachments
        payload.addUint32(0); // metadata records
        payload.addUint32(0); // chunks (messages are intentionally unchunked)
        payload.addUint64(message_start_time_);
        payload.addUint64(message_end_time_);
        payload.addUint32(static_cast<uint32>(channel_message_counts_.size() * 10));
        for (const auto& entry : channel_message_counts_) {
            payload.addUint16(entry.first);
            payload.addUint64(entry.second);
        }
        writeRecord(McapOpcode::Statistics, payload.data);
    }

    void writeSummaryOffset(McapOpcode group_opcode, uint64 group_start, uint64 group_length)
    {
        Buffer payload;
        payload.addUint8(static_cast<uint8>(group_opcode));
        payload.addUint64(group_start);
        payload.addUint64(group_length);
        writeRecord(McapOpcode::SummaryOffset, payload.data);
    }

    const char* schemaDefinition(const std::string& type) const
    {
        if (type == "std_msgs/msg/String")
            return kStringSchema;
        if (type == "sensor_msgs/msg/Image")
            return kImageSchema;
        if (type == "sensor_msgs/msg/CompressedImage")
            return kCompressedImageSchema;
        if (type == "sensor_msgs/msg/CameraInfo")
            return kCameraInfoSchema;
        if (type == "sensor_msgs/msg/Imu")
            return kImuSchema;
        if (type == "sensor_msgs/msg/NavSatFix")
            return kNavSatFixSchema;
        if (type == "sensor_msgs/msg/MagneticField")
            return kMagneticFieldSchema;
        if (type == "airsim_interfaces/msg/Altimeter")
            return kAltimeterSchema;
        throw std::invalid_argument("unsupported ROS 2 schema: " + type);
    }

    uint16 ensureSchema(const std::string& type)
    {
        const auto existing = schema_ids_.find(type);
        if (existing != schema_ids_.end())
            return existing->second;
        if (next_schema_id_ == 0)
            throw std::overflow_error("MCAP schema id space exhausted");

        Schema schema;
        schema.id = next_schema_id_++;
        schema.name = type;
        schema.encoding = "ros2msg";
        schema.definition = schemaDefinition(type);
        schemas_.push_back(schema);
        schema_ids_[type] = schema.id;
        return schema.id;
    }

    uint16 ensureChannel(const std::string& topic,
                         const std::string& type,
                         Ros2QosProfile qos_profile)
    {
        const auto existing = channel_ids_.find(topic);
        if (existing != channel_ids_.end())
            return existing->second;
        if (next_channel_id_ == 0)
            throw std::overflow_error("MCAP channel id space exhausted");

        Channel channel;
        channel.id = next_channel_id_++;
        channel.schema_id = ensureSchema(type);
        channel.topic = topic;
        channel.message_encoding = "cdr";
        channel.metadata.emplace("offered_qos_profiles", ros2QosProfile(qos_profile));
        channel.metadata.emplace("topic_type_hash", "");
        channels_.push_back(channel);
        channel_ids_[topic] = channel.id;
        return channel.id;
    }

    std::string uniqueTopic(const std::string& canonical_topic, const std::string& identity)
    {
        const auto existing = topic_identities_.find(canonical_topic);
        if (existing == topic_identities_.end()) {
            topic_identities_[canonical_topic] = identity;
            return canonical_topic;
        }
        if (existing->second == identity)
            return canonical_topic;

        std::ostringstream suffix;
        suffix << "_collision_" << std::hex << msr::airlib::stableRos2TopicHash(identity);
        const std::string collision_suffix = suffix.str();
        auto makeCandidate = [&canonical_topic, &collision_suffix](uint32 collision_index) {
            std::string suffix_with_index = collision_suffix;
            if (collision_index > 1)
                suffix_with_index += "_" + std::to_string(collision_index);

            size_t prefix_length = canonical_topic.size();
            if (prefix_length + suffix_with_index.size() > kMaxRosTopicLength) {
                prefix_length = kMaxRosTopicLength - suffix_with_index.size();
                while (prefix_length > 0 &&
                       !msr::airlib::isRos2TopicAsciiAlphaNumeric(
                           static_cast<unsigned char>(canonical_topic[prefix_length - 1]))) {
                    --prefix_length;
                }
            }
            if (prefix_length == 0)
                throw std::length_error("ROS topic prefix is too long for collision suffix");
            return canonical_topic.substr(0, prefix_length) + suffix_with_index;
        };

        std::string candidate = makeCandidate(1);
        for (uint32 collision_index = 2;; ++collision_index) {
            const auto collision = topic_identities_.find(candidate);
            if (collision == topic_identities_.end()) {
                topic_identities_[candidate] = identity;
                UE_LOG(LogTemp, Warning,
                       TEXT("ROS bag topic collision for %s; recording the later source as %s"),
                       UTF8_TO_TCHAR(canonical_topic.c_str()), UTF8_TO_TCHAR(candidate.c_str()));
                return candidate;
            }
            if (collision->second == identity)
                return candidate;
            candidate = makeCandidate(collision_index);
        }
    }

    void writeRosMessage(const std::string& canonical_topic,
                         const std::string& topic_identity,
                         const std::string& type,
                         msr::airlib::TTimePoint time_stamp,
                         const std::vector<uint8_t>& payload,
                         Ros2QosProfile qos_profile = Ros2QosProfile::SensorData)
    {
        if (time_stamp == 0)
            return;
        if (!spool_file_)
            throw std::runtime_error("MCAP payload spool is not open for writing");
        if (payload.size() > static_cast<size_t>(std::numeric_limits<int64>::max()))
            throw std::length_error("MCAP payload exceeds int64 length");

        const uint16 channel_id = ensureChannel(
            uniqueTopic(canonical_topic, topic_identity), type, qos_profile);

        const int64 spool_offset = spool_file_->Tell();
        if (spool_offset < 0)
            throw std::runtime_error("could not determine MCAP payload spool offset");
        const int64 payload_size = static_cast<int64>(payload.size());
        if (payload_size > std::numeric_limits<int64>::max() - spool_offset)
            throw std::length_error("MCAP payload spool size overflow");
        if (!payload.empty() && !spool_file_->Write(payload.data(), payload_size))
            throw std::runtime_error("MCAP payload spool write failed");
        if (next_pending_message_order_ == std::numeric_limits<uint64>::max())
            throw std::overflow_error("MCAP message ordering space exhausted");

        PendingMessage message;
        message.channel_id = channel_id;
        message.time_stamp = time_stamp;
        message.spool_offset = static_cast<uint64>(spool_offset);
        message.payload_size = static_cast<uint64>(payload_size);
        message.insertion_order = next_pending_message_order_++;
        pending_messages_.push_back(message);
    }

    std::string imageMetadata(const msr::airlib::RecordingCapture& capture,
                              const msr::airlib::ImageCaptureBase::ImageResponse& response,
                              msr::airlib::TTimePoint image_header_time_stamp,
                              bool camera_info_written) const
    {
        const bool has_render_time = response.has_render_frame_timestamp &&
            response.time_stamp != 0 && capture.frame_time_stamp != 0;
        const int64_t delay_ns = has_render_time
            ? static_cast<int64_t>(response.time_stamp) - static_cast<int64_t>(capture.frame_time_stamp)
            : 0;
        const bool within_tolerance = has_render_time &&
            std::llabs(delay_ns) <= capture.image_sync_tolerance_ns;

        std::ostringstream json;
        json << std::setprecision(17);
        json << "{\"sequence_id\":" << capture.sequence_id
             << ",\"physics_step_id\":" << capture.physics_step_id
             << ",\"association_mode\":\"" << escapeJson(capture.association_mode) << "\""
             << ",\"frame_time_stamp\":" << capture.frame_time_stamp
             << ",\"image_header_time_stamp\":" << image_header_time_stamp
             << ",\"image_timestamp_source\":\"rendered_frame\""
             << ",\"render_frame_timestamp_valid\":" <<
                    (response.has_render_frame_timestamp ? "true" : "false")
             << ",\"render_frame_number\":" << response.render_frame_number
             << ",\"image_request_time_stamp\":" << response.request_time_stamp
             << ",\"image_time_stamp\":" << response.time_stamp
             << ",\"image_delay_ns\":" << delay_ns
             << ",\"image_sync_within_tolerance\":" << (within_tolerance ? "true" : "false")
             << ",\"camera_info_written\":" << (camera_info_written ? "true" : "false")
             << ",\"camera_name\":\"" << escapeJson(response.camera_name) << "\""
             << ",\"image_type\":\"" << imageTypeName(response.image_type) << "\""
             << ",\"annotation_name\":\"" << escapeJson(response.annotation_name) << "\""
             << ",\"latched_pose_ned\":{\"x\":" << capture.pose.position.x()
             << ",\"y\":" << capture.pose.position.y()
             << ",\"z\":" << capture.pose.position.z()
             << ",\"qw\":" << capture.pose.orientation.w()
             << ",\"qx\":" << capture.pose.orientation.x()
             << ",\"qy\":" << capture.pose.orientation.y()
             << ",\"qz\":" << capture.pose.orientation.z() << "}}";
        return json.str();
    }

    void writeSensorSamples(const msr::airlib::RecordingCapture& capture)
    {
        // Image captures carry a snapshot of these same latest-value sensors.
        // Emit only sensor-only rows so a camera rate cannot multiply GPS,
        // barometer, or magnetometer messages.
        if (capture.association_mode != "sensor_only")
            return;

        constexpr double kGaussToTesla = 1.0e-4;
        const std::array<double, 9> unknown_covariance{};
        std::set<std::string> emitted_identities;

        for (const auto& sample : capture.sensors) {
            if (!sample.present || sample.sensor_time_stamp == 0 ||
                sample.sensor_type == msr::airlib::SensorBase::SensorType::Imu) {
                continue;
            }

            const std::string vehicle_token =
                msr::airlib::normalizeRos2TopicToken(capture.vehicle_name, "vehicle");
            const std::string sensor_token =
                msr::airlib::normalizeRos2TopicToken(sample.sensor_name, "sensor");
            const std::string frame_id = vehicle_token + "/" + sensor_token;

            std::string topic;
            std::string identity;
            bool wrote = false;
            switch (sample.sensor_type) {
            case msr::airlib::SensorBase::SensorType::Gps: {
                topic = "/airsim_node/" + vehicle_token + "/gps/" + sensor_token;
                identity = topicIdentity("gps", { capture.vehicle_name, sample.sensor_name });
                if (!emitted_identities.insert(identity).second)
                    continue;
                const auto previous = last_sensor_time_stamps_.find(identity);
                if (previous != last_sensor_time_stamps_.end() &&
                    previous->second == sample.sensor_time_stamp) {
                    continue;
                }

                // AirSim's GNSS enum describes a fix dimension rather than a
                // ROS NavSatStatus. A 2D/3D valid position is an unaugmented
                // ROS fix; no-fix and time-only outputs are not positional fixes.
                const int8_t status = sample.gps_valid &&
                        sample.gps_fix >= static_cast<int>(msr::airlib::GpsBase::GNSS_FIX_2D_FIX)
                    ? 0 // sensor_msgs/NavSatStatus::STATUS_FIX
                    : -1; // sensor_msgs/NavSatStatus::STATUS_NO_FIX
                writeRosMessage(topic,
                                identity,
                                "sensor_msgs/msg/NavSatFix",
                                sample.sensor_time_stamp,
                                msr::airlib::serializeRos2NavSatFix(
                                    sample.sensor_time_stamp,
                                    frame_id,
                                    status,
                                    1, // sensor_msgs/NavSatStatus::SERVICE_GPS
                                    sample.gps_lat,
                                    sample.gps_lon,
                                    static_cast<double>(sample.gps_alt),
                                    unknown_covariance,
                                    0)); // COVARIANCE_TYPE_UNKNOWN
                wrote = true;
                break;
            }
            case msr::airlib::SensorBase::SensorType::Barometer: {
                topic = "/airsim_node/" + vehicle_token + "/altimeter/" + sensor_token;
                identity = topicIdentity("altimeter", { capture.vehicle_name, sample.sensor_name });
                if (!emitted_identities.insert(identity).second)
                    continue;
                const auto previous = last_sensor_time_stamps_.find(identity);
                if (previous != last_sensor_time_stamps_.end() &&
                    previous->second == sample.sensor_time_stamp) {
                    continue;
                }
                writeRosMessage(topic,
                                identity,
                                "airsim_interfaces/msg/Altimeter",
                                sample.sensor_time_stamp,
                                msr::airlib::serializeRos2Altimeter(
                                    sample.sensor_time_stamp,
                                    frame_id,
                                    static_cast<float>(sample.baro_altitude),
                                    static_cast<float>(sample.baro_pressure),
                                    static_cast<float>(sample.baro_qnh)));
                wrote = true;
                break;
            }
            case msr::airlib::SensorBase::SensorType::Magnetometer: {
                topic = "/airsim_node/" + vehicle_token + "/magnetometer/" + sensor_token;
                identity = topicIdentity("magnetometer", { capture.vehicle_name, sample.sensor_name });
                if (!emitted_identities.insert(identity).second)
                    continue;
                const auto previous = last_sensor_time_stamps_.find(identity);
                if (previous != last_sensor_time_stamps_.end() &&
                    previous->second == sample.sensor_time_stamp) {
                    continue;
                }
                // AirSim's simple sensor is Gauss in the NED body convention;
                // sensor_msgs/MagneticField is Tesla in the ROS body convention.
                std::array<double, 9> magnetic_field_covariance{};
                if (sample.mag_field_covariance_known) {
                    constexpr double kGaussSquaredToTeslaSquared = kGaussToTesla * kGaussToTesla;
                    const double axis_sign[] = { 1.0, -1.0, -1.0 };
                    for (size_t row = 0; row < 3; ++row) {
                        for (size_t column = 0; column < 3; ++column) {
                            magnetic_field_covariance[row * 3 + column] =
                                static_cast<double>(sample.mag_field_covariance[row * 3 + column]) *
                                axis_sign[row] * axis_sign[column] * kGaussSquaredToTeslaSquared;
                        }
                    }
                }
                writeRosMessage(topic,
                                identity,
                                "sensor_msgs/msg/MagneticField",
                                sample.sensor_time_stamp,
                                msr::airlib::serializeRos2MagneticField(
                                    sample.sensor_time_stamp,
                                    frame_id,
                                    static_cast<double>(sample.mag_field_body.x()) * kGaussToTesla,
                                    -static_cast<double>(sample.mag_field_body.y()) * kGaussToTesla,
                                    -static_cast<double>(sample.mag_field_body.z()) * kGaussToTesla,
                                    magnetic_field_covariance));
                wrote = true;
                break;
            }
            default:
                // The native bag does not claim a ROS schema for unsupported
                // recorder samples (for example Distance) yet.
                continue;
            }

            if (wrote)
                last_sensor_time_stamps_[identity] = sample.sensor_time_stamp;
        }
    }

    void writeImuBatches(const msr::airlib::RecordingCapture& capture,
                         const std::vector<msr::airlib::RecordingImuBatch>& imu_batches)
    {
        struct SampleReference
        {
            const msr::airlib::RecordingImuBatch* batch = nullptr;
            const msr::airlib::RecordingImuSample* sample = nullptr;
        };

        std::vector<SampleReference> samples;
        for (const auto& batch : imu_batches) {
            for (const auto& sample : batch.samples)
                samples.push_back({ &batch, &sample });
        }
        std::sort(samples.begin(), samples.end(), [](const SampleReference& left, const SampleReference& right) {
            if (left.sample->time_stamp != right.sample->time_stamp)
                return left.sample->time_stamp < right.sample->time_stamp;
            if (left.batch->vehicle_name != right.batch->vehicle_name)
                return left.batch->vehicle_name < right.batch->vehicle_name;
            return left.batch->sensor_name < right.batch->sensor_name;
        });

        for (const auto& reference : samples) {
            const auto& batch = *reference.batch;
            const auto& sample = *reference.sample;
            if (sample.time_stamp == 0)
                continue;

            // Match AirSim's ROS 2 wrapper convention for NED -> ROS values.
            const msr::airlib::Quaternionr orientation = sample.orientation.inverse();
            const msr::airlib::Vector3r angular_velocity(
                sample.angular_velocity.x(), -sample.angular_velocity.y(), -sample.angular_velocity.z());
            const msr::airlib::Vector3r linear_acceleration(
                sample.linear_acceleration.x(), -sample.linear_acceleration.y(), -sample.linear_acceleration.z());
            const std::string vehicle_token = msr::airlib::normalizeRos2TopicToken(batch.vehicle_name, "vehicle");
            const std::string sensor_token = msr::airlib::normalizeRos2TopicToken(batch.sensor_name, "imu");
            const std::string frame_id = vehicle_token + "/" + sensor_token;
            const std::string topic = "/airsim_node/" + vehicle_token + "/imu/" + sensor_token;
            writeRosMessage(topic,
                            topicIdentity("imu", { batch.vehicle_name, batch.sensor_name }),
                            "sensor_msgs/msg/Imu",
                            sample.time_stamp,
                            msr::airlib::serializeRos2Imu(
                                sample.time_stamp,
                                frame_id,
                                orientation,
                                angular_velocity,
                                linear_acceleration));
        }

        for (const auto& batch : imu_batches) {
            if (batch.dropped_samples == 0)
                continue;

            msr::airlib::TTimePoint time_stamp = capture.frame_time_stamp;
            if (!batch.samples.empty())
                time_stamp = batch.samples.back().time_stamp;
            if (time_stamp == 0)
                continue;

            std::ostringstream metadata;
            metadata << "{\"vehicle_name\":\"" << escapeJson(batch.vehicle_name)
                     << "\",\"sensor_name\":\"" << escapeJson(batch.sensor_name)
                     << "\",\"dropped_samples\":" << batch.dropped_samples
                     << ",\"sequence_id\":" << capture.sequence_id
                     << ",\"physics_step_id\":" << capture.physics_step_id << "}";
            const std::string topic = "/airsim_node/" + msr::airlib::normalizeRos2TopicToken(batch.vehicle_name, "vehicle") +
                "/recording/imu_drops";
            writeRosMessage(topic,
                            topicIdentity("imu_drops", { batch.vehicle_name }),
                            "std_msgs/msg/String",
                            time_stamp,
                            msr::airlib::serializeRos2String(metadata.str()),
                            Ros2QosProfile::ReliableMetadata);
            const std::string overflow_message = batch.vehicle_name + "/" + batch.sensor_name + ": " +
                std::to_string(batch.dropped_samples) + " samples dropped";
            UE_LOG(LogTemp, Warning, TEXT("ROS bag IMU history overflow: %s"),
                   UTF8_TO_TCHAR(overflow_message.c_str()));
        }
    }

    void writeImages(const msr::airlib::RecordingCapture& capture,
                     const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses)
    {
        for (const auto& response : responses) {
            if (!hasImageData(response))
                continue;

            // Never label image bytes with the earlier physics latch or with
            // recorder/write time.  A rendered-frame stamp is the only time
            // that is bound to this RenderResult's pixels.  If the render path
            // cannot provide it, omit the image rather than writing a bag
            // message whose Header is false.
            const msr::airlib::TTimePoint image_header_time_stamp = response.time_stamp;
            if (!response.has_render_frame_timestamp || image_header_time_stamp == 0) {
                UE_LOG(LogTemp, Warning,
                       TEXT("Skipping ROS bag image from %s because its rendered-frame timestamp is unavailable"),
                       UTF8_TO_TCHAR(response.camera_name.c_str()));
                continue;
            }

            const std::string prefix = imageTopicPrefix(capture, response);
            const std::string frame_id = msr::airlib::normalizeRos2TopicToken(capture.vehicle_name, "vehicle") + "/" +
                msr::airlib::normalizeRos2TopicToken(response.camera_name, "camera") + "_optical";
            const std::vector<std::string> image_identity_fields = {
                capture.vehicle_name,
                response.camera_name,
                std::to_string(static_cast<int>(response.image_type)),
                response.annotation_name
            };
            const double horizontal_fov_degrees =
                static_cast<double>(response.camera_horizontal_fov_degrees);
            double focal_length = 0.0;
            if (response.camera_info_is_perspective &&
                std::isfinite(horizontal_fov_degrees) &&
                horizontal_fov_degrees > 0.0 && horizontal_fov_degrees < 180.0) {
                constexpr double kPi = 3.14159265358979323846;
                focal_length = static_cast<double>(response.width) /
                    (2.0 * std::tan(horizontal_fov_degrees * kPi / 360.0));
            }
            const bool has_pinhole_camera_info =
                std::isfinite(focal_length) && focal_length > 0.0;
            const auto write_camera_info = [&]() {
                if (!has_pinhole_camera_info)
                    return false;

                writeRosMessage(prefix + "/camera_info",
                                topicIdentity("camera_info", image_identity_fields),
                                "sensor_msgs/msg/CameraInfo",
                                image_header_time_stamp,
                                msr::airlib::serializeRos2CameraInfo(
                                    image_header_time_stamp,
                                    frame_id,
                                    static_cast<uint32>(response.height),
                                    static_cast<uint32>(response.width),
                                    focal_length,
                                    focal_length,
                                    static_cast<double>(response.width) / 2.0,
                                    static_cast<double>(response.height) / 2.0));
                return true;
            };

            const uint64 pixel_count = static_cast<uint64>(response.width) *
                static_cast<uint64>(response.height);
            bool camera_info_written = false;
            if (response.pixels_as_float) {
                if (pixel_count > static_cast<uint64>(std::numeric_limits<size_t>::max() / sizeof(float)) ||
                    static_cast<uint64>(response.width) >
                        static_cast<uint64>(std::numeric_limits<uint32>::max() / sizeof(float))) {
                    UE_LOG(LogTemp, Warning,
                           TEXT("Skipping oversized float recording image from %s"),
                           UTF8_TO_TCHAR(response.camera_name.c_str()));
                    continue;
                }
                const size_t expected_byte_count = static_cast<size_t>(pixel_count) * sizeof(float);
                const uint8_t* float_bytes = nullptr;
                size_t byte_count = 0;
                if (!response.image_data_float.empty()) {
                    byte_count = response.image_data_float.size() * sizeof(float);
                    float_bytes = reinterpret_cast<const uint8_t*>(response.image_data_float.data());
                }
                else if (response.image_data_uint8.size() == expected_byte_count) {
                    // FloatAsBytes depth requests retain IEEE-754 little-endian
                    // samples in the byte vector instead of the float vector.
                    byte_count = response.image_data_uint8.size();
                    float_bytes = response.image_data_uint8.data();
                }
                if (byte_count != expected_byte_count || float_bytes == nullptr) {
                    UE_LOG(LogTemp, Warning,
                           TEXT("Skipping malformed float recording image from %s: expected %llu bytes, got %llu"),
                           UTF8_TO_TCHAR(response.camera_name.c_str()),
                           static_cast<uint64>(expected_byte_count), static_cast<uint64>(byte_count));
                    continue;
                }
                const uint32 step = static_cast<uint32>(static_cast<size_t>(response.width) * sizeof(float));
                // Validate before emitting CameraInfo, so a companion message
                // is never left behind for an image we reject.
                camera_info_written = write_camera_info();
                writeRosMessage(prefix + "/image",
                                topicIdentity("image_raw", image_identity_fields),
                                "sensor_msgs/msg/Image",
                                image_header_time_stamp,
                                msr::airlib::serializeRos2Image(
                                    image_header_time_stamp,
                                    frame_id,
                                    static_cast<uint32>(response.height),
                                    static_cast<uint32>(response.width),
                                    "32FC1",
                                    step,
                                    float_bytes,
                                    byte_count));
            }
            else if (response.compress) {
                if (response.image_data_uint8.empty()) {
                    UE_LOG(LogTemp, Warning,
                           TEXT("Skipping malformed compressed recording image from %s: no byte data"),
                           UTF8_TO_TCHAR(response.camera_name.c_str()));
                    continue;
                }
                constexpr const char* compressed_format = "jpeg";
                std::vector<std::string> compressed_identity_fields = image_identity_fields;
                compressed_identity_fields.push_back(compressed_format);
                camera_info_written = write_camera_info();
                writeRosMessage(prefix + "/image/compressed",
                                topicIdentity("image_compressed", compressed_identity_fields),
                                "sensor_msgs/msg/CompressedImage",
                                image_header_time_stamp,
                                msr::airlib::serializeRos2CompressedImage(
                                    image_header_time_stamp,
                                    frame_id,
                                    compressed_format,
                                    response.image_data_uint8.data(),
                                    response.image_data_uint8.size()));
            }
            else {
                if (response.recording_jpeg) {
                    UE_LOG(LogTemp, Warning,
                           TEXT("Skipping unencoded JPEG recording image from %s"),
                           UTF8_TO_TCHAR(response.camera_name.c_str()));
                    continue;
                }
                if (pixel_count > static_cast<uint64>(std::numeric_limits<size_t>::max() / 3u) ||
                    static_cast<uint64>(response.width) >
                        static_cast<uint64>(std::numeric_limits<uint32>::max() / 3u)) {
                    UE_LOG(LogTemp, Warning,
                           TEXT("Skipping oversized raw recording image from %s"),
                           UTF8_TO_TCHAR(response.camera_name.c_str()));
                    continue;
                }
                const size_t expected_byte_count = static_cast<size_t>(pixel_count) * 3u;
                if (response.image_data_uint8.size() != expected_byte_count) {
                    UE_LOG(LogTemp, Warning,
                           TEXT("Skipping malformed raw recording image from %s: expected %llu bytes, got %llu"),
                           UTF8_TO_TCHAR(response.camera_name.c_str()),
                           static_cast<uint64>(expected_byte_count),
                           static_cast<uint64>(response.image_data_uint8.size()));
                    continue;
                }
                const uint32 step = static_cast<uint32>(static_cast<size_t>(response.width) * 3u);
                camera_info_written = write_camera_info();
                writeRosMessage(prefix + "/image",
                                topicIdentity("image_raw", image_identity_fields),
                                "sensor_msgs/msg/Image",
                                image_header_time_stamp,
                                msr::airlib::serializeRos2Image(
                                    image_header_time_stamp,
                                    frame_id,
                                    static_cast<uint32>(response.height),
                                    static_cast<uint32>(response.width),
                                    "rgb8",
                                    step,
                                    response.image_data_uint8.data(),
                                    response.image_data_uint8.size()));
            }

            const std::string metadata_topic = "/airsim_node/" +
                msr::airlib::normalizeRos2TopicToken(capture.vehicle_name, "vehicle") + "/recording/image_metadata";
            writeRosMessage(metadata_topic,
                            topicIdentity("image_metadata", { capture.vehicle_name }),
                            "std_msgs/msg/String",
                            image_header_time_stamp,
                            msr::airlib::serializeRos2String(
                                imageMetadata(capture, response, image_header_time_stamp,
                                              camera_info_written)),
                            Ros2QosProfile::ReliableMetadata);
        }
    }

    void fail(const std::string& prefix, const std::string& detail)
    {
        file_.reset();
        spool_file_.reset();
        discardTemporaryOutput();
        discardTemporarySpool();
        pending_messages_.clear();
        is_recording_ = false;
        finalized_ = false;
        UE_LOG(LogTemp, Error, TEXT("%s: %s"), UTF8_TO_TCHAR(prefix.c_str()), UTF8_TO_TCHAR(detail.c_str()));
    }

    void discardTemporaryOutput()
    {
        if (temporary_output_path_.IsEmpty())
            return;
        FPlatformFileManager::Get().GetPlatformFile().DeleteFile(*temporary_output_path_);
    }

    void discardTemporarySpool()
    {
        if (temporary_spool_path_.IsEmpty())
            return;
        FPlatformFileManager::Get().GetPlatformFile().DeleteFile(*temporary_spool_path_);
    }

private:
    std::unique_ptr<IFileHandle> file_;
    std::unique_ptr<IFileHandle> spool_file_;
    FString output_path_;
    FString temporary_output_path_;
    FString temporary_spool_path_;
    bool is_recording_ = false;
    bool finalized_ = false;
    uint64 offset_ = 0;
    uint64 spool_size_ = 0;
    uint16 next_schema_id_ = 1;
    uint16 next_channel_id_ = 1;
    uint32 next_message_sequence_ = 0;
    uint64 next_pending_message_order_ = 0;
    std::vector<Schema> schemas_;
    std::vector<Channel> channels_;
    std::vector<PendingMessage> pending_messages_;
    std::map<std::string, uint16> schema_ids_;
    std::map<std::string, uint16> channel_ids_;
    std::map<std::string, std::string> topic_identities_;
    std::map<std::string, msr::airlib::TTimePoint> last_sensor_time_stamps_;
    std::map<uint16, uint64> channel_message_counts_;
    uint64 message_count_ = 0;
    uint64 message_start_time_ = 0;
    uint64 message_end_time_ = 0;
};

RosbagWriter::RosbagWriter()
    : impl_(new Impl())
{
}

RosbagWriter::~RosbagWriter()
{
    stopRecording();
}

bool RosbagWriter::startRecording(const std::string& session_folder, const std::string& configured_file_name)
{
    return impl_->start(session_folder, configured_file_name);
}

void RosbagWriter::appendRecord(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
                                const msr::airlib::RecordingCapture& capture,
                                const std::vector<msr::airlib::RecordingImuBatch>& imu_batches)
{
    impl_->append(responses, capture, imu_batches);
}

void RosbagWriter::stopRecording()
{
    impl_->stop();
}

void RosbagWriter::abortRecording()
{
    impl_->abort();
}

bool RosbagWriter::isRecording() const
{
    return impl_->isRecording();
}

std::string RosbagWriter::outputPath() const
{
    return impl_->outputPath();
}
