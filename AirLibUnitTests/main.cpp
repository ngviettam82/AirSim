#include <iostream>
#include <memory>
#include "SettingsTest.hpp"
#include "PixhawkTest.hpp"
#include "SimpleFlightTest.hpp"
#include "WorkerThreadTest.hpp"
#include "QuaternionTest.hpp"
#include "CelestialTests.hpp"
#include "RecordingCaptureTest.hpp"

int main()
{
    using namespace msr::airlib;

    std::unique_ptr<TestBase> tests[] = {
        std::unique_ptr<TestBase>(new QuaternionTest()),
        std::unique_ptr<TestBase>(new CelestialTest()),
        std::unique_ptr<TestBase>(new SettingsTest()),
        std::unique_ptr<TestBase>(new RecordingCaptureTest()),
        std::unique_ptr<TestBase>(new SimpleFlightTest())
        //,
        //std::unique_ptr<TestBase>(new PixhawkTest()),
        //std::unique_ptr<TestBase>(new WorkerThreadTest())
    };

    try {
        for (auto& test : tests)
            test->run();
    }
    catch (const std::exception& ex) {
        std::cerr << "TEST FAILED: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
