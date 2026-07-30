#ifndef msr_AirLibUnitTests_RecordingCaptureTest_hpp
#define msr_AirLibUnitTests_RecordingCaptureTest_hpp

#include "TestBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "common/RecordingCapture.hpp"
#include "common/Ros2Cdr.hpp"
#include "common/Ros2TopicName.hpp"
#include "sensors/imu/ImuBase.hpp"
#include <array>
#include <iostream>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{

    class RecordingCaptureTest : public TestBase
    {
    private:
        class HistoryImu : public ImuBase
        {
        public:
            void emit(TTimePoint time_stamp)
            {
                Output output;
                output.time_stamp = time_stamp;
                output.orientation = Quaternionr::Identity();
                output.angular_velocity = Vector3r(1, 2, 3);
                output.linear_acceleration = Vector3r(4, 5, 6);
                setOutput(output);
            }

        protected:
            virtual void resetImplementation() override
            {
            }
        };

    public:
        virtual void run() override
        {
            testAssert(recordingSensorAgeNs(1000, 800) == 200, "age+");
            testAssert(recordingSensorAgeNs(800, 1000) == -200, "age-");
            testAssert(RecordingCapture::namesEqualIgnoreCase("Imu", "imu"), "case");
            testAssert(normalizeRos2TopicToken("front_camera", "camera") == "front_camera",
                       "ROS 2 topic token preserves valid name");
            testAssert(normalizeRos2TopicToken("0", "camera") == "camera_0_haf63ad4c86019caf",
                       "ROS 2 topic token normalizes default camera");
            testAssert(ImageCaptureBase::ImageResponse::hasValidPayloadLayout(
                           1280, 720, false, true, 4096, 0),
                       "compressed image payload layout");
            testAssert(ImageCaptureBase::ImageResponse::hasValidPayloadLayout(
                           1280, 720, false, false, 1280u * 720u * 3u, 0),
                       "raw image payload layout");
            testAssert(ImageCaptureBase::ImageResponse::hasValidPayloadLayout(
                           1280, 720, true, false, 0, 1280u * 720u),
                       "float image payload layout");
            testAssert(!ImageCaptureBase::ImageResponse::hasValidPayloadLayout(
                            1280, 720, false, false, 1280u * 720u * 3u + 1u, 0),
                       "reject malformed raw payload");
            testAssert(!ImageCaptureBase::ImageResponse::hasValidPayloadLayout(
                            1280, 720, false, true,
                            static_cast<size_t>(std::numeric_limits<uint32_t>::max()), 0),
                       "reject oversized compressed payload");
            auto tokens = RecordingCapture::buildSchemaTokens({ "A-B", "A_B", "imu" });
            testAssert(tokens.size() == 3 && tokens[0] != tokens[1], "token disambig");

            RecordingSensorSample imu;
            imu.sensor_name = "imu";
            imu.schema_token = "imu";
            imu.sensor_type = SensorBase::SensorType::Imu;
            imu.present = true;
            imu.sensor_time_stamp = 900;
            imu.age_ns = 100;
            imu.imu_linear_acceleration = Vector3r(0.1f, 0.2f, -9.8f);
            imu.imu_orientation = Quaternionr(1, 0, 0, 0);

            RecordingCapture c;
            c.vehicle_name = "drone1";
            c.sequence_id = 42;
            c.physics_step_id = 7;
            c.render_frame_number = 3;
            c.frame_time_stamp = 1000;
            c.schema_sensor_names = { "imu", "gps" };
            c.schema_tokens = RecordingCapture::buildSchemaTokens(c.schema_sensor_names);
            imu.schema_token = c.schema_tokens[0];
            c.sensors.push_back(imu);

            const std::string header = c.fullHeaderLine();
            const std::string row = c.toRecordLine() +
                                    "frame_latched_freerun\t950\t1000\t0\t0.000000\t1\timg.jpg";
            const size_t hf = RecordingCapture::countFields(header);
            const size_t rf = RecordingCapture::countFields(row);
            testAssert(hf == rf, "header/row field count");
            testAssert(hf == RecordingCapture::expectedFieldCount(2, 0), "expected count");
            testAssert(header.find("ImageDelayMs") != std::string::npos, "image timing header");

            RecordingCapture missing = c;
            missing.sensors.clear();
            testAssert(RecordingCapture::countFields(
                           missing.toRecordLine() + "sensor_only\t\t\t\t\t\t") == hf,
                       "missing cols");

            testAssert(row.find("\t900\t") != std::string::npos, "native ts");
            testAssert(row.find("\t100\t") != std::string::npos, "age");

            c.vehicle_extra_header = "Throttle\tSteering\tBrake\tGear\tHandbrake\tRPM\tSpeed\t";
            c.vehicle_extra_values = "0.1\t0\t0\t1\t0\t1000\t5\t";
            testAssert(RecordingCapture::countFields(c.fullHeaderLine()) ==
                           RecordingCapture::countFields(
                               c.toRecordLine() + "sensor_only\t\t\t\t\t\t"),
                       "car fields");

            // restart sequence independent
            RecordingCapture r2;
            r2.sequence_id = 1;
            testAssert(r2.sequence_id == 1, "restart seq");

            // History keeps every native source update while recording, even
            // when the consumer drains less frequently than the IMU updates.
            HistoryImu history_imu;
            history_imu.enableRecordingHistory(2);
            history_imu.emit(10);
            history_imu.emit(20);
            history_imu.emit(30);
            const RecordingImuBatch history = history_imu.drainRecordingHistory();
            testAssert(history.samples.size() == 2, "imu history capacity");
            testAssert(history.samples[0].time_stamp == 20 && history.samples[1].time_stamp == 30,
                       "imu history newest samples");
            testAssert(history.dropped_samples == 1, "imu history dropped count");
            const RecordingImuBatch final_history = history_imu.disableAndDrainRecordingHistory();
            testAssert(final_history.samples.empty() && !history_imu.isRecordingHistoryEnabled(),
                       "imu history shutdown");

            const std::vector<uint8_t> string_cdr = serializeRos2String("abc");
            const std::vector<uint8_t> expected_string_cdr = {
                0x00, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
                0x61, 0x62, 0x63, 0x00
            };
            testAssert(string_cdr == expected_string_cdr, "ROS 2 String CDR");

            const TTimePoint cdr_stamp = 0x01020304ULL * 1000000000ULL + 0x11121314ULL;
            const uint8_t image_bytes[] = { 0xDE, 0xAD, 0xBE };
            const std::vector<uint8_t> image_cdr = serializeRos2Image(
                cdr_stamp, "f", 0x11223344, 0x55667788, "rgb8", 0x99AABBCC,
                image_bytes, sizeof(image_bytes));
            const std::vector<uint8_t> expected_image_cdr = {
                0x00, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01,
                0x14, 0x13, 0x12, 0x11, 0x02, 0x00, 0x00, 0x00,
                0x66, 0x00, 0x00, 0x00, 0x44, 0x33, 0x22, 0x11,
                0x88, 0x77, 0x66, 0x55, 0x05, 0x00, 0x00, 0x00,
                0x72, 0x67, 0x62, 0x38, 0x00, 0x00, 0x00, 0x00,
                0xCC, 0xBB, 0xAA, 0x99, 0x03, 0x00, 0x00, 0x00,
                0xDE, 0xAD, 0xBE
            };
            testAssert(image_cdr == expected_image_cdr, "ROS 2 Image CDR alignment");

            const std::vector<uint8_t> compressed_image_cdr = serializeRos2CompressedImage(
                cdr_stamp, "f", "jpeg", image_bytes, sizeof(image_bytes));
            const std::vector<uint8_t> expected_compressed_image_cdr = {
                0x00, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01,
                0x14, 0x13, 0x12, 0x11, 0x02, 0x00, 0x00, 0x00,
                0x66, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
                0x6A, 0x70, 0x65, 0x67, 0x00, 0x00, 0x00, 0x00,
                0x03, 0x00, 0x00, 0x00, 0xDE, 0xAD, 0xBE
            };
            testAssert(compressed_image_cdr == expected_compressed_image_cdr,
                       "ROS 2 CompressedImage CDR alignment");

            const auto imu_cdr = serializeRos2Imu(
                cdr_stamp, "f", Quaternionr::Identity(), Vector3r::Zero(), Vector3r::Zero());
            testAssert(imu_cdr.size() == 316, "ROS 2 IMU CDR size");

            const std::array<double, 9> zero_covariance{};
            const auto nav_sat_fix_cdr = serializeRos2NavSatFix(
                cdr_stamp, "f", -1, 1, 1.0, 2.0, 3.0, zero_covariance, 0);
            // Header (18 bytes), int8 status, one byte CDR padding, uint16
            // service, six bytes of padding, three float64 coordinates, a
            // fixed nine-element covariance, and its uint8 type.
            testAssert(nav_sat_fix_cdr.size() == 125, "ROS 2 NavSatFix CDR size");
            testAssert(nav_sat_fix_cdr[18] == 0xFF && nav_sat_fix_cdr[19] == 0x00 &&
                           nav_sat_fix_cdr[20] == 0x01 && nav_sat_fix_cdr[21] == 0x00,
                       "ROS 2 NavSatFix status/service alignment");
            testAssert(nav_sat_fix_cdr[34] == 0xF0 && nav_sat_fix_cdr[35] == 0x3F &&
                           nav_sat_fix_cdr[42] == 0x00 && nav_sat_fix_cdr[43] == 0x40 &&
                           nav_sat_fix_cdr[50] == 0x08 && nav_sat_fix_cdr[51] == 0x40 &&
                           nav_sat_fix_cdr.back() == 0x00,
                       "ROS 2 NavSatFix coordinates/covariance type");

            const auto magnetic_field_cdr = serializeRos2MagneticField(
                cdr_stamp, "f", 1.0, 2.0, 3.0, zero_covariance);
            // Header is followed by two bytes of padding before Vector3's
            // first float64, then the fixed covariance array.
            testAssert(magnetic_field_cdr.size() == 116, "ROS 2 MagneticField CDR size");
            testAssert(magnetic_field_cdr[18] == 0x00 && magnetic_field_cdr[19] == 0x00 &&
                           magnetic_field_cdr[26] == 0xF0 && magnetic_field_cdr[27] == 0x3F &&
                           magnetic_field_cdr[34] == 0x00 && magnetic_field_cdr[35] == 0x40 &&
                           magnetic_field_cdr[42] == 0x08 && magnetic_field_cdr[43] == 0x40,
                       "ROS 2 MagneticField CDR alignment");

            const auto altimeter_cdr = serializeRos2Altimeter(cdr_stamp, "f", 1.0f, 2.0f, 3.0f);
            testAssert(altimeter_cdr.size() == 32, "ROS 2 Altimeter CDR size");
            testAssert(altimeter_cdr[18] == 0x00 && altimeter_cdr[19] == 0x00 &&
                           altimeter_cdr[22] == 0x80 && altimeter_cdr[23] == 0x3F &&
                           altimeter_cdr[26] == 0x00 && altimeter_cdr[27] == 0x40 &&
                           altimeter_cdr[30] == 0x40 && altimeter_cdr[31] == 0x40,
                       "ROS 2 Altimeter CDR alignment");

            const auto camera_info_cdr = serializeRos2CameraInfo(
                cdr_stamp, "f", 480, 640, 1.0, 2.0, 3.0, 4.0);
            // CDR payload: Header, dimensions, empty distortion model and D,
            // K/R/P matrices, binning, then RegionOfInterest. The final bool
            // is deliberately not padded at the end of a ROS 2 message.
            testAssert(camera_info_cdr.size() == 309, "ROS 2 CameraInfo CDR size");
            testAssert(camera_info_cdr[44] == 0x00 && camera_info_cdr[50] == 0xF0 &&
                           camera_info_cdr[51] == 0x3F,
                       "ROS 2 CameraInfo K[0]");
            testAssert(camera_info_cdr[60] == 0x00 && camera_info_cdr[66] == 0x08 &&
                           camera_info_cdr[67] == 0x40,
                       "ROS 2 CameraInfo K[2]");
            testAssert(camera_info_cdr[188] == 0x00 && camera_info_cdr[194] == 0xF0 &&
                           camera_info_cdr[195] == 0x3F,
                       "ROS 2 CameraInfo P[0]");
            testAssert(camera_info_cdr.back() == 0, "ROS 2 CameraInfo ROI do_rectify");

            std::cout << "RecordingCaptureTest passed fields=" << hf << std::endl;
        }
    };

}
} // namespace
#endif
