#include <iostream>
#include <memory>
#include <typeinfo>
#include <string>
#include <vector>
#include "SettingsTest.hpp"
#include "PixhawkTest.hpp"
#include "SimpleFlightTest.hpp"
#include "WorkerThreadTest.hpp"
#include "QuaternionTest.hpp"
#include "CelestialTests.hpp"
#include "RecordingCaptureTest.hpp"
#include "MultirotorPhysicsTest.hpp"

int main(int argc, char** argv)
{
    using namespace msr::airlib;

    // Optional filter: AirLibUnitTests.exe MultirotorPhysics SimpleFlight
    std::vector<std::string> filter;
    for (int i = 1; i < argc; ++i)
        filter.emplace_back(argv[i]);

    auto want = [&](const std::string& key) {
        if (filter.empty())
            return true;
        for (const auto& f : filter) {
            if (key.find(f) != std::string::npos || f.find(key) != std::string::npos)
                return true;
        }
        return false;
    };

    struct Entry {
        const char* key;
        std::unique_ptr<TestBase> test;
        bool enabled_by_default;
    };

    // CelestialTest is environment/time dependent and not part of multirotor realism;
    // keep it opt-in via CLI filter only.
    std::vector<Entry> tests;
    tests.push_back({ "Quaternion", std::unique_ptr<TestBase>(new QuaternionTest()), true });
    tests.push_back({ "Settings", std::unique_ptr<TestBase>(new SettingsTest()), true });
    tests.push_back({ "RecordingCapture", std::unique_ptr<TestBase>(new RecordingCaptureTest()), true });
    tests.push_back({ "MultirotorPhysics", std::unique_ptr<TestBase>(new MultirotorPhysicsTest()), true });
    tests.push_back({ "SimpleFlight", std::unique_ptr<TestBase>(new SimpleFlightTest()), true });
    tests.push_back({ "Celestial", std::unique_ptr<TestBase>(new CelestialTest()), false });

    int ran = 0;
    try {
        for (auto& entry : tests) {
            const bool run_it = filter.empty() ? entry.enabled_by_default : want(entry.key);
            if (!run_it)
                continue;
            std::cout << "Running " << entry.key << " ..." << std::endl;
            entry.test->run();
            std::cout << "  passed" << std::endl;
            ++ran;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "TEST FAILED: " << ex.what() << std::endl;
        return 1;
    }

    if (ran == 0) {
        std::cerr << "No tests selected." << std::endl;
        return 2;
    }

    std::cout << "All " << ran << " selected unit tests passed." << std::endl;
    return 0;
}
