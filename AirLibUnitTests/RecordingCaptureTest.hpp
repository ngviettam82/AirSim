#ifndef msr_AirLibUnitTests_RecordingCaptureTest_hpp
#define msr_AirLibUnitTests_RecordingCaptureTest_hpp

#include "TestBase.hpp"
#include "common/RecordingCapture.hpp"
#include <iostream>
#include <string>

namespace msr
{
namespace airlib
{

    class RecordingCaptureTest : public TestBase
    {
    public:
        virtual void run() override
        {
            testAssert(recordingSensorAgeNs(1000, 800) == 200, "age+");
            testAssert(recordingSensorAgeNs(800, 1000) == -200, "age-");
            testAssert(RecordingCapture::namesEqualIgnoreCase("Imu", "imu"), "case");
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
            const std::string row = c.toRecordLine() + "img.png";
            const size_t hf = RecordingCapture::countFields(header);
            const size_t rf = RecordingCapture::countFields(row);
            testAssert(hf == rf, "header/row field count");
            testAssert(hf == RecordingCapture::expectedFieldCount(2, 0, true), "expected count");

            RecordingCapture missing = c;
            missing.sensors.clear();
            testAssert(RecordingCapture::countFields(missing.toRecordLine() + "x") == hf, "missing cols");

            testAssert(row.find("\t900\t") != std::string::npos, "native ts");
            testAssert(row.find("\t100\t") != std::string::npos, "age");

            c.vehicle_extra_header = "Throttle\tSteering\tBrake\tGear\tHandbrake\tRPM\tSpeed\t";
            c.vehicle_extra_values = "0.1\t0\t0\t1\t0\t1000\t5\t";
            testAssert(RecordingCapture::countFields(c.fullHeaderLine()) ==
                           RecordingCapture::countFields(c.toRecordLine() + "img.png"),
                       "car fields");

            // restart sequence independent
            RecordingCapture r2;
            r2.sequence_id = 1;
            testAssert(r2.sequence_id == 1, "restart seq");

            std::cout << "RecordingCaptureTest passed fields=" << hf << std::endl;
        }
    };

}
} // namespace
#endif
