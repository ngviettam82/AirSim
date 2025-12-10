// Minimal Gazebo (gz / Ignition) transport bridge to AirSim
// This file is intended for use with Ignition/Gazebo (gz) releases such as
// Fortress/Garden/Harmonic. It subscribes to a pose topic and forwards the
// first pose to AirSim via RPC (simSetVehiclePose).

#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <thread>
#include <functional>
#include <mutex>

#include <iostream>
#include <string>
#include <atomic>
#include <chrono>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

using namespace msr::airlib;

static constexpr int NWIDTH = 7;
static constexpr int MESSAGE_THROTTLE = 100;

int main(int argc, char** argv)
{
    // Simple command-line parsing for common options
    std::string rpc_host = "127.0.0.1";
    int rpc_port = 41451;
    std::string topic = "/pose/local/info"; // default; many gz setups use /model/<name>/pose
    std::string vehicle_name = "";
    std::string gz_model = "";  // Gazebo model name to filter from Pose_V
    std::string airsim_vehicle = "";  // AirSim vehicle name to update
    bool verbose = false;
    int message_throttle = MESSAGE_THROTTLE;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--rpc_host" && i + 1 < argc) rpc_host = argv[++i];
        else if (a == "--rpc_port" && i + 1 < argc) rpc_port = std::stoi(argv[++i]);
    else if (a == "--topic" && i + 1 < argc) topic = argv[++i];
    else if (a == "--vehicle" && i + 1 < argc) vehicle_name = argv[++i];
    else if (a == "--gz_model" && i + 1 < argc) gz_model = argv[++i];
    else if (a == "--airsim_vehicle" && i + 1 < argc) airsim_vehicle = argv[++i];
    else if (a == "--verbose") verbose = true;
    else if (a == "--throttle" && i + 1 < argc) message_throttle = std::stoi(argv[++i]);
        else if (a == "--help") {
            std::cout << "Usage: GazeboDrone_gz [--rpc_host HOST] [--rpc_port PORT] [--topic TOPIC]\n";
            std::cout << "  --vehicle NAME          : (legacy) both Gazebo model and AirSim vehicle name\n";
            std::cout << "  --gz_model NAME         : Gazebo model name to filter from Pose_V\n";
            std::cout << "  --airsim_vehicle NAME   : AirSim vehicle name to update\n";
            std::cout << "  --throttle MS           : Update throttle in milliseconds\n";
            std::cout << "  --verbose               : Enable verbose output\n";
            return 0;
        }
    }
    
    // If new parameters are used, they override --vehicle
    if (!gz_model.empty() || !airsim_vehicle.empty()) {
        if (gz_model.empty()) gz_model = airsim_vehicle;
        if (airsim_vehicle.empty()) airsim_vehicle = gz_model;
    } else if (!vehicle_name.empty()) {
        // Legacy mode: --vehicle sets both
        gz_model = vehicle_name;
        airsim_vehicle = vehicle_name;
    }

    // Create RPC client. MultirotorRpcLibClient doesn't currently accept host/port in this header
    // in some builds; try to use constructor that accepts host/port if available.
    msr::airlib::MultirotorRpcLibClient client(rpc_host, rpc_port);
    client.confirmConnection();
    std::cout << "Connected to AirSim at " << rpc_host << ":" << rpc_port << std::endl;

    gz::transport::Node node;

    std::atomic<int> msg_count{0};
    // Selected model name extracted from Pose_V messages (used when --vehicle not provided)
    std::string selected_name;
    std::mutex selected_name_m;


    // Callback for single Pose messages
    std::function<void(const gz::msgs::Pose&)> cb_pose =
        [&](const gz::msgs::Pose &_pmsg) {
        int count = ++msg_count;
        double x = _pmsg.position().x();
        double y = _pmsg.position().y();
        double z = _pmsg.position().z();
        double ow = _pmsg.orientation().w();
        double ox = _pmsg.orientation().x();
        double oy = _pmsg.orientation().y();
        double oz = _pmsg.orientation().z();

        Vector3r pos((real_T)x, (real_T)-y, (real_T)-z);
        Quaternionr ori((real_T)ow, (real_T)ox, (real_T)-oy, (real_T)-oz);
        try {
            // Use airsim_vehicle if set, otherwise fall back to selected_name
            std::string target;
            if (!airsim_vehicle.empty()) {
                target = airsim_vehicle;
            } else {
                std::lock_guard<std::mutex> lk(selected_name_m);
                target = selected_name;
            }
            if (target.empty()) {
                client.simSetVehiclePose(Pose(pos, ori), true);
            } else {
                if (verbose) std::cout << "Calling simSetVehiclePose for vehicle '" << target << "'" << std::endl;
                client.simSetVehiclePose(Pose(pos, ori), true, target);
            }
            if (verbose) std::cout << "simSetVehiclePose succeeded" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "simSetVehiclePose exception: " << e.what() << std::endl;
        }

        // Intentionally no per-message debug print to avoid flooding logs
    };

    // Callback for Odometry messages (common for px4/gz models)
    std::function<void(const gz::msgs::Odometry&)> cb_odom =
        [&](const gz::msgs::Odometry &_omsg) {
        int count = ++msg_count;
    // Odometry contains a pose field (some gz versions expose the nested Pose directly)
        const auto &p = _omsg.pose();
        double x = p.position().x();
        double y = p.position().y();
        double z = p.position().z();
        double ow = p.orientation().w();
        double ox = p.orientation().x();
        double oy = p.orientation().y();
        double oz = p.orientation().z();

        Vector3r pos((real_T)x, (real_T)-y, (real_T)-z);
        Quaternionr ori((real_T)ow, (real_T)ox, (real_T)-oy, (real_T)-oz);
        try {
            if (airsim_vehicle.empty())
                client.simSetVehiclePose(Pose(pos, ori), true);
            else
                client.simSetVehiclePose(Pose(pos, ori), true, airsim_vehicle);
            if (verbose) std::cout << "simSetVehiclePose succeeded" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "simSetVehiclePose exception: " << e.what() << std::endl;
        }

        // Intentionally no per-message debug print to avoid flooding logs
    };

    // Callback for Pose_V (vector of poses), used by world-level pose topics
    std::function<void(const gz::msgs::Pose_V&)> cb_pose_v =
        [&](const gz::msgs::Pose_V &_pv) {
        int count = ++msg_count;
        if (_pv.pose_size() <= 0) return;

        // Choose which pose in the Pose_V to use:
        // 1) If gz_model provided, match by substring
        // 2) Otherwise pick the first pose with a non-zero position (likely the model)
        int selected = -1;
        if (!gz_model.empty()) {
            for (int i = 0; i < _pv.pose_size(); ++i) {
                const auto &pp = _pv.pose(i);
                try {
                    if (pp.name().find(gz_model) != std::string::npos) { selected = i; break; }
                } catch (...) {}
            }
        }
        if (selected == -1) {
            for (int i = 0; i < _pv.pose_size(); ++i) {
                const auto &pp = _pv.pose(i);
                if (pp.position().x() != 0.0 || pp.position().y() != 0.0 || pp.position().z() != 0.0) {
                    selected = i; break;
                }
            }
        }
        if (selected == -1) selected = 0; // fallback

        const auto &pmsg = _pv.pose(selected);
        double x = pmsg.position().x();
        double y = pmsg.position().y();
        double z = pmsg.position().z();
        double ow = pmsg.orientation().w();
        double ox = pmsg.orientation().x();
        double oy = pmsg.orientation().y();
        double oz = pmsg.orientation().z();

        // Removed verbose per-message listing to avoid flooding logs

        // Save the selected model name for other callbacks to use when --vehicle is not provided
        if (!pmsg.name().empty()) {
            std::lock_guard<std::mutex> lk(selected_name_m);
            selected_name = pmsg.name();
        }

        Vector3r pos((real_T)x, (real_T)-y, (real_T)-z);
        Quaternionr ori((real_T)ow, (real_T)ox, (real_T)-oy, (real_T)-oz);
        try {
            if (airsim_vehicle.empty())
                client.simSetVehiclePose(Pose(pos, ori), true);
            else
                client.simSetVehiclePose(Pose(pos, ori), true, airsim_vehicle);
            if (verbose) std::cout << "simSetVehiclePose succeeded" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "simSetVehiclePose exception: " << e.what() << std::endl;
        }

        // Intentionally no per-message debug print to avoid flooding logs
    };

    // Subscribe to Odometry if topic name suggests it, otherwise subscribe to Pose
    try {
        std::string tlower = topic;
        std::transform(tlower.begin(), tlower.end(), tlower.begin(), ::tolower);
        bool subscribed = false;
        // Heuristic: if topic looks like a world-level pose topic, try Pose_V first
        if (tlower.find("/world/") != std::string::npos || tlower.find("pose/info") != std::string::npos) {
            subscribed = node.Subscribe<gz::msgs::Pose_V>(topic, cb_pose_v);
            std::cout << "Attempted subscribe Pose_V -> " << (subscribed ? "OK" : "FAIL") << std::endl;
        }

        if (!subscribed && (tlower.find("odometry") != std::string::npos || tlower.find("odom") != std::string::npos)) {
            // try Odometry next
            subscribed = node.Subscribe<gz::msgs::Odometry>(topic, cb_odom);
            std::cout << "Attempted subscribe Odometry -> " << (subscribed ? "OK" : "FAIL") << std::endl;
        }

        if (!subscribed) {
            // try Pose (single model pose)
            subscribed = node.Subscribe<gz::msgs::Pose>(topic, cb_pose);
            std::cout << "Attempted subscribe Pose -> " << (subscribed ? "OK" : "FAIL") << std::endl;
        }
        if (!subscribed) {
            std::cerr << "Warning: subscribe failed for '" << topic << "' (no matching message type)" << std::endl;
            return 1;
        }
    } catch (const std::exception &e) {
        std::cerr << "Warning: subscribe failed for '" << topic << "': " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Subscribed to topic: " << topic << ", forwarding pose[0] to AirSim at " << rpc_host << ":" << rpc_port << std::endl;

    // No heartbeat thread to avoid periodic flooding of logs

    // Spin loop
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
