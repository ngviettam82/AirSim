// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Ros2Cdr_hpp
#define msr_airlib_Ros2Cdr_hpp

#include "common/Common.hpp"
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace msr
{
namespace airlib
{

    // Minimal ROS 2 CDR serializer for the fixed message set emitted by the
    // Unreal recording writer. CDR alignment starts immediately after the
    // four-byte encapsulation header.
    class Ros2CdrWriter
    {
    public:
        Ros2CdrWriter()
            : data_{ 0x00, 0x01, 0x00, 0x00 } // CDR_LE encapsulation
        {
        }

        void writeUint8(uint8_t value)
        {
            data_.push_back(value);
        }

        void writeInt8(int8_t value)
        {
            writeUint8(static_cast<uint8_t>(value));
        }

        void writeUint16(uint16_t value)
        {
            align(2);
            writeLittleEndian(value);
        }

        void writeInt32(int32_t value)
        {
            writeUint32(static_cast<uint32_t>(value));
        }

        void writeUint32(uint32_t value)
        {
            align(4);
            writeLittleEndian(value);
        }

        void writeFloat64(double value)
        {
            align(8);
            uint64_t bits = 0;
            static_assert(sizeof(bits) == sizeof(value), "unexpected double size");
            std::memcpy(&bits, &value, sizeof(bits));
            writeLittleEndian(bits);
        }

        void writeFloat32(float value)
        {
            align(4);
            uint32_t bits = 0;
            static_assert(sizeof(bits) == sizeof(value), "unexpected float size");
            std::memcpy(&bits, &value, sizeof(bits));
            writeLittleEndian(bits);
        }

        void writeString(const std::string& value)
        {
            if (value.size() >= static_cast<size_t>(std::numeric_limits<uint32_t>::max()))
                throw std::length_error("ROS 2 CDR string exceeds uint32 length");

            writeUint32(static_cast<uint32_t>(value.size() + 1)); // ROS 2 CDR includes the NUL
            data_.insert(data_.end(), value.begin(), value.end());
            data_.push_back(0);
        }

        void writeByteSequence(const uint8_t* data, size_t size)
        {
            if (size > static_cast<size_t>(std::numeric_limits<uint32_t>::max()))
                throw std::length_error("ROS 2 CDR byte sequence exceeds uint32 length");

            writeUint32(static_cast<uint32_t>(size));
            if (size != 0) {
                if (data == nullptr)
                    throw std::invalid_argument("ROS 2 CDR byte sequence has null data");
                data_.insert(data_.end(), data, data + size);
            }
        }

        const std::vector<uint8_t>& data() const
        {
            return data_;
        }

        std::vector<uint8_t>&& takeData()
        {
            return std::move(data_);
        }

    private:
        void align(size_t alignment)
        {
            const size_t payload_offset = data_.size() - kEncapsulationSize;
            const size_t padding = (alignment - (payload_offset % alignment)) % alignment;
            data_.insert(data_.end(), padding, 0);
        }

        template <typename T>
        void writeLittleEndian(T value)
        {
            for (size_t i = 0; i < sizeof(T); ++i) {
                data_.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xFF));
            }
        }

    private:
        static constexpr size_t kEncapsulationSize = 4;
        std::vector<uint8_t> data_;
    };

    inline void writeRos2Header(Ros2CdrWriter& writer, TTimePoint time_stamp, const std::string& frame_id)
    {
        constexpr uint64_t kNanosPerSecond = 1000000000ULL;
        const uint64_t seconds = time_stamp / kNanosPerSecond;
        if (seconds > static_cast<uint64_t>(std::numeric_limits<int32_t>::max()))
            throw std::out_of_range("ROS 2 Time seconds exceed int32 range");

        writer.writeInt32(static_cast<int32_t>(seconds));
        writer.writeUint32(static_cast<uint32_t>(time_stamp % kNanosPerSecond));
        writer.writeString(frame_id);
    }

    inline std::vector<uint8_t> serializeRos2String(const std::string& value)
    {
        Ros2CdrWriter writer;
        writer.writeString(value);
        return writer.takeData();
    }

    inline std::vector<uint8_t> serializeRos2Image(TTimePoint time_stamp,
                                                    const std::string& frame_id,
                                                    uint32_t height,
                                                    uint32_t width,
                                                    const std::string& encoding,
                                                    uint32_t step,
                                                    const uint8_t* data,
                                                    size_t size)
    {
        Ros2CdrWriter writer;
        writeRos2Header(writer, time_stamp, frame_id);
        writer.writeUint32(height);
        writer.writeUint32(width);
        writer.writeString(encoding);
        writer.writeUint8(0); // AirSim writes little-endian scalar pixels.
        writer.writeUint32(step);
        writer.writeByteSequence(data, size);
        return writer.takeData();
    }

    inline std::vector<uint8_t> serializeRos2CompressedImage(TTimePoint time_stamp,
                                                              const std::string& frame_id,
                                                              const std::string& format,
                                                              const uint8_t* data,
                                                              size_t size)
    {
        Ros2CdrWriter writer;
        writeRos2Header(writer, time_stamp, frame_id);
        writer.writeString(format);
        writer.writeByteSequence(data, size);
        return writer.takeData();
    }

    inline std::vector<uint8_t> serializeRos2NavSatFix(
        TTimePoint time_stamp,
        const std::string& frame_id,
        int8_t status,
        uint16_t service,
        double latitude,
        double longitude,
        double altitude,
        const std::array<double, 9>& position_covariance,
        uint8_t position_covariance_type)
    {
        Ros2CdrWriter writer;
        writeRos2Header(writer, time_stamp, frame_id);
        writer.writeInt8(status);
        writer.writeUint16(service);
        writer.writeFloat64(latitude);
        writer.writeFloat64(longitude);
        writer.writeFloat64(altitude);
        for (double value : position_covariance)
            writer.writeFloat64(value);
        writer.writeUint8(position_covariance_type);
        return writer.takeData();
    }

    inline std::vector<uint8_t> serializeRos2MagneticField(
        TTimePoint time_stamp,
        const std::string& frame_id,
        double magnetic_field_x,
        double magnetic_field_y,
        double magnetic_field_z,
        const std::array<double, 9>& magnetic_field_covariance)
    {
        Ros2CdrWriter writer;
        writeRos2Header(writer, time_stamp, frame_id);
        writer.writeFloat64(magnetic_field_x);
        writer.writeFloat64(magnetic_field_y);
        writer.writeFloat64(magnetic_field_z);
        for (double value : magnetic_field_covariance)
            writer.writeFloat64(value);
        return writer.takeData();
    }

    inline std::vector<uint8_t> serializeRos2Altimeter(TTimePoint time_stamp,
                                                        const std::string& frame_id,
                                                        float altitude,
                                                        float pressure,
                                                        float qnh)
    {
        Ros2CdrWriter writer;
        writeRos2Header(writer, time_stamp, frame_id);
        writer.writeFloat32(altitude);
        writer.writeFloat32(pressure);
        writer.writeFloat32(qnh);
        return writer.takeData();
    }

    // AirSim only writes CameraInfo for a perspective scene capture. Its
    // post-process distortion is not proven to match a ROS distortion model,
    // so leave distortion_model and D empty instead of fabricating plumb_bob
    // coefficients. The focal length comes from Unreal's horizontal FOV.
    inline std::vector<uint8_t> serializeRos2CameraInfo(TTimePoint time_stamp,
                                                         const std::string& frame_id,
                                                         uint32_t height,
                                                         uint32_t width,
                                                         double focal_length_x,
                                                         double focal_length_y,
                                                         double principal_point_x,
                                                         double principal_point_y)
    {
        Ros2CdrWriter writer;
        writeRos2Header(writer, time_stamp, frame_id);
        writer.writeUint32(height);
        writer.writeUint32(width);
        writer.writeString(""); // distortion model is intentionally unknown
        writer.writeUint32(0); // D: no known distortion coefficients

        const double k[9] = {
            focal_length_x, 0.0, principal_point_x,
            0.0, focal_length_y, principal_point_y,
            0.0, 0.0, 1.0
        };
        for (double value : k)
            writer.writeFloat64(value);

        const double r[9] = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
        for (double value : r)
            writer.writeFloat64(value);

        const double p[12] = {
            focal_length_x, 0.0, principal_point_x, 0.0,
            0.0, focal_length_y, principal_point_y, 0.0,
            0.0, 0.0, 1.0, 0.0
        };
        for (double value : p)
            writer.writeFloat64(value);

        writer.writeUint32(0); // binning_x
        writer.writeUint32(0); // binning_y
        writer.writeUint32(0); // roi.x_offset
        writer.writeUint32(0); // roi.y_offset
        writer.writeUint32(0); // roi.height
        writer.writeUint32(0); // roi.width
        writer.writeUint8(0); // roi.do_rectify
        return writer.takeData();
    }

    inline std::vector<uint8_t> serializeRos2Imu(TTimePoint time_stamp,
                                                  const std::string& frame_id,
                                                  const Quaternionr& orientation,
                                                  const Vector3r& angular_velocity,
                                                  const Vector3r& linear_acceleration)
    {
        Ros2CdrWriter writer;
        writeRos2Header(writer, time_stamp, frame_id);

        writer.writeFloat64(static_cast<double>(orientation.x()));
        writer.writeFloat64(static_cast<double>(orientation.y()));
        writer.writeFloat64(static_cast<double>(orientation.z()));
        writer.writeFloat64(static_cast<double>(orientation.w()));
        for (int i = 0; i < 9; ++i)
            writer.writeFloat64(0.0);

        writer.writeFloat64(static_cast<double>(angular_velocity.x()));
        writer.writeFloat64(static_cast<double>(angular_velocity.y()));
        writer.writeFloat64(static_cast<double>(angular_velocity.z()));
        for (int i = 0; i < 9; ++i)
            writer.writeFloat64(0.0);

        writer.writeFloat64(static_cast<double>(linear_acceleration.x()));
        writer.writeFloat64(static_cast<double>(linear_acceleration.y()));
        writer.writeFloat64(static_cast<double>(linear_acceleration.z()));
        for (int i = 0; i < 9; ++i)
            writer.writeFloat64(0.0);

        return writer.takeData();
    }

}
} // namespace msr::airlib

#endif
