// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_RecordingCapture_hpp
#define msr_airlib_RecordingCapture_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "sensors/SensorBase.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace msr
{
namespace airlib
{

    // Immutable request-local capture transaction for one discrete sample.
    // Image rows use timestamped, frame-latched free-run association. Sensor-only
    // rows and image rows may be produced by independent recording workers.
    struct RecordingSensorSample
    {
        std::string sensor_name;
        std::string schema_token;
        SensorBase::SensorType sensor_type = SensorBase::SensorType::Imu;
        bool present = false; // found and produced a real output (time_stamp != 0)
        std::string error;

        TTimePoint sensor_time_stamp = 0; // native; never rewritten to frame time
        int64_t age_ns = 0; // FrameTimeStamp - SensorTimeStamp when present

        Vector3r imu_linear_acceleration = Vector3r::Zero();
        Vector3r imu_angular_velocity = Vector3r::Zero();
        Quaternionr imu_orientation = Quaternionr::Identity();

        double gps_lat = 0, gps_lon = 0;
        float gps_alt = 0;
        Vector3r gps_velocity = Vector3r::Zero();
        float gps_eph = 0, gps_epv = 0;
        int gps_fix = 0;
        bool gps_valid = false;

        real_T baro_altitude = 0, baro_pressure = 0, baro_qnh = 0;
        Vector3r mag_field_body = Vector3r::Zero();
        std::array<real_T, 9> mag_field_covariance{};
        bool mag_field_covariance_known = false;
        real_T distance = 0, distance_min = 0, distance_max = 0;
    };

    // A native IMU update retained by the source sensor while ROS bag recording
    // is active. These samples are deliberately independent of the recorder's
    // polling interval so a nominal 333 Hz IMU is not reduced to 200 Hz.
    struct RecordingImuSample
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        TTimePoint time_stamp = 0;
        uint64_t physics_step_id = 0;
        Quaternionr orientation = Quaternionr::Identity();
        Vector3r angular_velocity = Vector3r::Zero();
        Vector3r linear_acceleration = Vector3r::Zero();
    };

    struct RecordingImuBatch
    {
        std::string vehicle_name;
        std::string sensor_name;
        std::vector<RecordingImuSample> samples;
        // Number overwritten in this sensor's bounded source ring before this drain.
        uint64_t dropped_samples = 0;
    };

    struct RecordingCapture
    {
        std::string vehicle_name;
        uint64_t sequence_id = 0; // session-local monotonic (restarts each session)
        uint64_t physics_step_id = 0; // ClockFactory step at snapshot
        uint64_t render_frame_number = 0; // UE game-frame snapshot for the image (0 for sensor-only rows)
        TTimePoint frame_time_stamp = 0; // sim-clock ns at pose/sensor snapshot
        std::vector<TTimePoint> image_request_time_stamps;
        std::vector<TTimePoint> image_time_stamps; // frame-proven capture-state times
        std::vector<int64_t> image_delays_ns; // ImageTimeStamp - FrameTimeStamp
        std::vector<bool> image_sync_within_tolerance;
        int64_t image_sync_tolerance_ns = 5000000;

        Pose pose;
        std::vector<RecordingSensorSample> sensors;
        std::vector<std::string> schema_tokens;
        std::vector<std::string> schema_sensor_names;

        // Car/Skid extras (header/values fragments ending with trailing tab if non-empty)
        std::string vehicle_extra_header;
        std::string vehicle_extra_values;

        std::string association_mode = "sensor_only";

        // Present+TS+Age(3)+IMU(10)+GPS(10)+Baro(3)+Mag(3)+Dist(3)=32
        static constexpr int kBaseFieldCount = 12;
        static constexpr int kFieldsPerSensor = 32;
        static constexpr int kImageMetadataFieldCount = 7;

        static std::string toLower(const std::string& s)
        {
            std::string o = s;
            std::transform(o.begin(), o.end(), o.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return o;
        }

        static bool namesEqualIgnoreCase(const std::string& a, const std::string& b)
        {
            return toLower(a) == toLower(b);
        }

        static std::string sanitizeColumnToken(const std::string& name)
        {
            std::string out;
            for (char ch : name) {
                if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                    (ch >= '0' && ch <= '9') || ch == '_')
                    out.push_back(ch);
                else
                    out.push_back('_');
            }
            return out.empty() ? "Sensor" : out;
        }

        static std::vector<std::string> buildSchemaTokens(const std::vector<std::string>& names)
        {
            std::vector<std::string> tokens;
            std::unordered_map<std::string, int> used;
            for (const auto& name : names) {
                std::string base = sanitizeColumnToken(name);
                std::string token = base;
                auto it = used.find(base);
                if (it == used.end()) {
                    used[base] = 1;
                }
                else {
                    int n = ++(it->second);
                    token = base + "_" + std::to_string(n);
                    used[token] = 1;
                }
                tokens.push_back(token);
            }
            return tokens;
        }

        static size_t countFields(const std::string& line)
        {
            if (line.empty())
                return 0;
            size_t end = line.size();
            while (end > 0 && (line[end - 1] == '\n' || line[end - 1] == '\r'))
                --end;
            if (end == 0)
                return 0;
            size_t tabs = 0;
            for (size_t i = 0; i < end; ++i)
                if (line[i] == '\t')
                    ++tabs;
            return tabs + 1;
        }

        static size_t expectedFieldCount(size_t sensor_count,
                                         size_t vehicle_extra_fields = 0,
                                         bool image_metadata = true)
        {
            return static_cast<size_t>(kBaseFieldCount) +
                   sensor_count * static_cast<size_t>(kFieldsPerSensor) +
                   vehicle_extra_fields +
                   (image_metadata ? static_cast<size_t>(kImageMetadataFieldCount) : 0u);
        }

        static std::string headerColumns(const std::vector<std::string>& schema_tokens)
        {
            std::ostringstream ss;
            ss << "VehicleName\tSequenceID\tPhysicsStepID\tRenderFrameNumber\tFrameTimeStamp\t"
               << "POS_X\tPOS_Y\tPOS_Z\tQ_W\tQ_X\tQ_Y\tQ_Z\t";
            for (const auto& n : schema_tokens) {
                ss << n << "_Present\t" << n << "_TimeStamp\t" << n << "_Age\t";
                ss << n << "_IMU_AX\t" << n << "_IMU_AY\t" << n << "_IMU_AZ\t"
                   << n << "_IMU_WX\t" << n << "_IMU_WY\t" << n << "_IMU_WZ\t"
                   << n << "_IMU_QW\t" << n << "_IMU_QX\t" << n << "_IMU_QY\t" << n << "_IMU_QZ\t";
                ss << n << "_GPS_Lat\t" << n << "_GPS_Lon\t" << n << "_GPS_Alt\t"
                   << n << "_GPS_VX\t" << n << "_GPS_VY\t" << n << "_GPS_VZ\t"
                   << n << "_GPS_Eph\t" << n << "_GPS_Epv\t" << n << "_GPS_Fix\t" << n << "_GPS_Valid\t";
                ss << n << "_Baro_Alt\t" << n << "_Baro_Pressure\t" << n << "_Baro_Qnh\t";
                ss << n << "_Mag_X\t" << n << "_Mag_Y\t" << n << "_Mag_Z\t";
                ss << n << "_Distance\t" << n << "_DistanceMin\t" << n << "_DistanceMax\t";
            }
            return ss.str();
        }

        // Always emits exactly kFieldsPerSensor fields, each followed by a tab.
        static void appendSensorFields(std::ostringstream& ss, const RecordingSensorSample* sample)
        {
            std::vector<std::string> f(static_cast<size_t>(kFieldsPerSensor), "");
            auto num = [](double v) {
                std::ostringstream o;
                o << v;
                return o.str();
            };
            if (sample && sample->present) {
                f[0] = "1";
                f[1] = std::to_string(sample->sensor_time_stamp);
                f[2] = std::to_string(sample->age_ns);
                if (sample->sensor_type == SensorBase::SensorType::Imu) {
                    f[3] = num(sample->imu_linear_acceleration.x());
                    f[4] = num(sample->imu_linear_acceleration.y());
                    f[5] = num(sample->imu_linear_acceleration.z());
                    f[6] = num(sample->imu_angular_velocity.x());
                    f[7] = num(sample->imu_angular_velocity.y());
                    f[8] = num(sample->imu_angular_velocity.z());
                    f[9] = num(sample->imu_orientation.w());
                    f[10] = num(sample->imu_orientation.x());
                    f[11] = num(sample->imu_orientation.y());
                    f[12] = num(sample->imu_orientation.z());
                }
                else if (sample->sensor_type == SensorBase::SensorType::Gps) {
                    f[13] = num(sample->gps_lat);
                    f[14] = num(sample->gps_lon);
                    f[15] = num(sample->gps_alt);
                    f[16] = num(sample->gps_velocity.x());
                    f[17] = num(sample->gps_velocity.y());
                    f[18] = num(sample->gps_velocity.z());
                    f[19] = num(sample->gps_eph);
                    f[20] = num(sample->gps_epv);
                    f[21] = std::to_string(sample->gps_fix);
                    f[22] = sample->gps_valid ? "1" : "0";
                }
                else if (sample->sensor_type == SensorBase::SensorType::Barometer) {
                    f[23] = num(sample->baro_altitude);
                    f[24] = num(sample->baro_pressure);
                    f[25] = num(sample->baro_qnh);
                }
                else if (sample->sensor_type == SensorBase::SensorType::Magnetometer) {
                    f[26] = num(sample->mag_field_body.x());
                    f[27] = num(sample->mag_field_body.y());
                    f[28] = num(sample->mag_field_body.z());
                }
                else if (sample->sensor_type == SensorBase::SensorType::Distance) {
                    f[29] = num(sample->distance);
                    f[30] = num(sample->distance_min);
                    f[31] = num(sample->distance_max);
                }
            }
            else {
                f[0] = "0";
            }
            for (int i = 0; i < kFieldsPerSensor; ++i)
                ss << f[static_cast<size_t>(i)] << "\t";
        }

        std::string toRecordLine() const
        {
            std::ostringstream ss;
            ss << vehicle_name << "\t" << sequence_id << "\t" << physics_step_id << "\t"
               << render_frame_number << "\t" << frame_time_stamp << "\t";
            ss << pose.position.x() << "\t" << pose.position.y() << "\t" << pose.position.z() << "\t";
            ss << pose.orientation.w() << "\t" << pose.orientation.x() << "\t"
               << pose.orientation.y() << "\t" << pose.orientation.z() << "\t";

            for (size_t i = 0; i < schema_tokens.size(); ++i) {
                const RecordingSensorSample* found = nullptr;
                for (const auto& s : sensors) {
                    if (s.schema_token == schema_tokens[i] ||
                        (s.schema_token.empty() && i < schema_sensor_names.size() &&
                         namesEqualIgnoreCase(s.sensor_name, schema_sensor_names[i]))) {
                        found = &s;
                        break;
                    }
                }
                appendSensorFields(ss, found);
            }
            ss << vehicle_extra_values;
            return ss.str();
        }

        std::string fullHeaderLine() const
        {
            return headerColumns(schema_tokens) + vehicle_extra_header +
                   "AssociationMode\tImageRequestTimeStamp\tImageTimeStamp\t"
                   "ImageDelayNs\tImageDelayMs\tImageSyncWithinTolerance\tImageFile";
        }
    };

    inline int64_t recordingSensorAgeNs(TTimePoint frame_ts, TTimePoint sensor_ts)
    {
        return static_cast<int64_t>(frame_ts) - static_cast<int64_t>(sensor_ts);
    }

}
} //namespace
#endif
