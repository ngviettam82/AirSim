#include <rclcpp/rclcpp.hpp>
#include "airsim_ros_wrapper.h"

#include <cmath>
#include <exception>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("airsim_node", node_options);
    std::shared_ptr<rclcpp::Node> nh_img = nh->create_sub_node("img");
    std::shared_ptr<rclcpp::Node> nh_lidar = nh->create_sub_node("lidar");
    std::shared_ptr<rclcpp::Node> nh_gpulidar = nh->create_sub_node("gpulidar");
    std::shared_ptr<rclcpp::Node> nh_echo = nh->create_sub_node("echo");
    std::string host_ip = "localhost";
    uint16_t host_port = 41451;
    double rpc_timeout_sec = 5.0;
    bool enable_api_control = false;
    bool enable_object_transforms_list = true;
    nh->get_parameter_or("host_ip", host_ip, host_ip);
    nh->get_parameter_or("host_port", host_port, host_port);
    nh->get_parameter_or("rpc_timeout_sec", rpc_timeout_sec, rpc_timeout_sec);
    nh->get_parameter_or("enable_api_control", enable_api_control, enable_api_control);
    nh->get_parameter_or("enable_object_transforms_list", enable_object_transforms_list,
                         enable_object_transforms_list);
    if (!std::isfinite(rpc_timeout_sec) || rpc_timeout_sec < 0.1 || rpc_timeout_sec > 300.0) {
        RCLCPP_FATAL(nh->get_logger(), "rpc_timeout_sec must be between 0.1 and 300 seconds");
        rclcpp::shutdown();
        return 1;
    }
    try {
        AirsimROSWrapper airsim_ros_wrapper(nh, nh_img, nh_lidar, nh_gpulidar, nh_echo, host_ip, enable_api_control, enable_object_transforms_list, host_port, static_cast<float>(rpc_timeout_sec));

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(nh);
        executor.spin();
    }
    catch (const std::exception& error) {
        RCLCPP_FATAL(nh->get_logger(), "AirSim ROS 2 startup failed: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    catch (...) {
        RCLCPP_FATAL(nh->get_logger(), "AirSim ROS 2 startup failed with an unknown exception");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
