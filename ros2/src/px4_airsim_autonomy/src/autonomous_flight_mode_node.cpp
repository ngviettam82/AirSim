#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "px4_airsim_autonomy/types.hpp"
#include "px4_airsim_autonomy/algorithm_factory.hpp"
#include "px4_airsim_autonomy/enterprise/enterprise_flight_supervisor.hpp"

#include <memory>
#include <string>
#include <algorithm>
#include <mutex>
#include <cmath>

namespace px4_airsim_autonomy {

static const std::string kModeName = "AirSimAutonomy";

class AutonomousFlightMode : public px4_ros2::ModeBase {
public:
    explicit AutonomousFlightMode(rclcpp::Node& node, bool standalone = false)
        : ModeBase(node, Settings{kModeName}), is_standalone_(standalone)
    {
        // 1. Initialize Setpoint Generators (Goto and Trajectory)
        goto_setpoint_ = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
        trajectory_setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

        // 2. Local Odometry state from PX4
        local_position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this, true);

        // 3. Parameters
        std::string vehicle_name = node.declare_parameter<std::string>("vehicle_name", "drone1");
        std::string camera_name = node.declare_parameter<std::string>("camera_name", "cam1");
        std::string selected_algorithm = node.declare_parameter<std::string>("algorithm", "");
        std::string depth_topic = node.declare_parameter<std::string>(
            "depth_topic", "/airsim_node/" + vehicle_name + "/" + camera_name + "_DepthPlanar/image");

        // 4. Initialize Enterprise Flight Safety Supervisor
        supervisor_.init(node);

        // 5. Algorithm Selection & Initialization
        setAlgorithm(selected_algorithm);

        // 6. Sensor Subscriptions
        depth_sub_ = node.create_subscription<sensor_msgs::msg::Image>(
            depth_topic, rclcpp::SensorDataQoS(),
            std::bind(&AutonomousFlightMode::onDepthImage, this, std::placeholders::_1));

        target_sub_ = node.create_subscription<geometry_msgs::msg::PointStamped>(
            "/autonomy/target", 10,
            std::bind(&AutonomousFlightMode::onTargetPoint, this, std::placeholders::_1));

        // 7. Diagnostics & Status Publisher and 1Hz Heartbeat
        status_pub_ = node.create_publisher<std_msgs::msg::String>("/autonomy/status", 10);
        cmd_vel_pub_ = node.create_publisher<geometry_msgs::msg::TwistStamped>("/autonomy/cmd_vel", 10);

        heartbeat_timer_ = node.create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                std_msgs::msg::String msg;
                msg.data = "[Heartbeat] Mode: " + std::string(is_active_ ? "ACTIVE" : (is_standalone_ ? "STANDALONE" : "STANDBY")) +
                           " | Mission: " + (current_algorithm_ ? current_algorithm_->getName() : "None") +
                           " | NavTier: " + production::DegradedNavigationFsm::tierToString(supervisor_.getNavFsm().getCurrentTier()) +
                           " | MinDepth: " + std::to_string(latest_min_depth_) + "m" +
                           " | TargetDetected: " + (latest_target_detected_ ? "YES" : "NO");
                status_pub_->publish(msg);
            });

        // 7. Standalone Simulation Loop (active when FMU registration is bypassed)
        if (is_standalone_) {
            sim_timer_ = node.create_wall_timer(
                std::chrono::milliseconds(50),
                [this]() {
                    updateSetpoint(0.05f);
                });
        }

        // 8. Dynamic Parameter Change Callback
        param_cb_handle_ = node.add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto& param : parameters) {
                    if (param.get_name() == "algorithm" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                        setAlgorithm(param.as_string());
                    }
                }
                return result;
            });

        RCLCPP_INFO(node.get_logger(),
                    "[AutonomousFlightMode] Initialized (Algorithm: '%s', Depth topic: '%s', Standalone: %s)",
                    current_algorithm_ ? current_algorithm_->getName().c_str() : "none (Standby)",
                    depth_topic.c_str(), is_standalone_ ? "YES" : "NO");
    }

    void onActivate() override
    {
        is_active_ = true;
        RCLCPP_INFO(node().get_logger(), ">>> Autonomous Flight Mode ACTIVATED in PX4 <<<");
        if (current_algorithm_) {
            current_algorithm_->onActivate();
        } else {
            RCLCPP_INFO(node().get_logger(), "No mission algorithm active - vehicle holding in standby hover");
        }
    }

    void onDeactivate() override
    {
        is_active_ = false;
        RCLCPP_INFO(node().get_logger(), ">>> Autonomous Flight Mode DEACTIVATED <<<");
        if (current_algorithm_) {
            current_algorithm_->onDeactivate();
        }
    }

    void updateSetpoint(float dt) override
    {
        // 1. Populate current sensor snapshot
        SensorSnapshot snapshot;
        snapshot.dt = dt;
        
        // Position & Velocity from PX4 Odometry (local NED converted to ENU for user algorithms)
        Eigen::Vector3f pos_ned = Eigen::Vector3f::Zero();
        Eigen::Vector3f vel_ned = Eigen::Vector3f::Zero();
        float heading_ned = 0.0f;
        float yaw_enu = 0.0f;

        if (local_position_ && local_position_->lastValid()) {
            try {
                pos_ned = local_position_->positionNed();
                vel_ned = local_position_->velocityNed();
                heading_ned = local_position_->heading();
                yaw_enu = px4_ros2::yawNedToEnu(heading_ned);
            } catch (const std::exception& e) {
                // Odometry telemetry not yet received or transiently dropped
            }
        }

        snapshot.position = Eigen::Vector3f(pos_ned.y(), pos_ned.x(), -pos_ned.z());
        snapshot.velocity = Eigen::Vector3f(vel_ned.y(), vel_ned.x(), -vel_ned.z());
        snapshot.yaw = yaw_enu;
        snapshot.yaw_ned = heading_ned;

        // Populate Depth and Target metrics with freshness watchdogs
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            snapshot.min_depth_distance = latest_min_depth_;
            snapshot.center_sector_distance = latest_center_depth_;
            snapshot.left_sector_distance = latest_left_depth_;
            snapshot.right_sector_distance = latest_right_depth_;
            snapshot.upper_sector_distance = latest_upper_depth_;
            snapshot.lower_sector_distance = latest_lower_depth_;

            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    snapshot.sectors_3d[r][c] = latest_sectors_3d_[r][c];
                }
            }

            // Target freshness watchdog: invalidate if no update received in >1.0s
            bool target_fresh = (latest_target_time_.nanoseconds() > 0) &&
                                ((node().now() - latest_target_time_).seconds() < 1.0);
            snapshot.target_detected = target_fresh && latest_target_detected_;

            if (latest_target_is_world_) {
                // Transform target from World ENU to Body FLU
                Eigen::Vector3f delta_enu = latest_target_pos_ - snapshot.position;
                float cos_yaw = std::cos(yaw_enu);
                float sin_yaw = std::sin(yaw_enu);

                float flu_x =  delta_enu.x() * cos_yaw + delta_enu.y() * sin_yaw;
                float flu_y = -delta_enu.x() * sin_yaw + delta_enu.y() * cos_yaw;
                float flu_z =  delta_enu.z();
                snapshot.target_position = Eigen::Vector3f(flu_x, flu_y, flu_z);
            } else {
                snapshot.target_position = latest_target_pos_;
            }
        }

        // 2. Run the active modular mission algorithm (or safe standby hover hold)
        AutonomyCommand mission_cmd;
        if (current_algorithm_) {
            mission_cmd = current_algorithm_->update(snapshot, dt);
        } else {
            mission_cmd.type = ControlType::Velocity;
            mission_cmd.vector = Eigen::Vector3f::Zero();
            mission_cmd.yaw_or_yaw_rate = 0.0f;
            mission_cmd.status_message = "Standby (Hover Hold)";
        }

        // 3. Enterprise Safety Supervisor Real-Time Preemption (Geofence, Smart RTH, ADS-B, Degraded Nav FSM)
        enterprise::SupervisorSafetyReport safety = supervisor_.evaluate(snapshot, dt, mission_cmd);
        AutonomyCommand command = safety.override_command;

        if (safety.emergency_override_active) {
            RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 2000,
                                 "[SAFETY OVERRIDE by %s]: %s",
                                 safety.override_source.c_str(), safety.override_reason.c_str());
        }

        // 4. Dispatch setpoint to PX4 through px4_ros2_cpp (when running in flight mode)
        if (!is_standalone_) {
            if (command.type == ControlType::Velocity) {
                float cos_yaw = std::cos(heading_ned);
                float sin_yaw = std::sin(heading_ned);

                float v_fwd  = command.vector.x();
                float v_left = command.vector.y();
                float v_up   = command.vector.z();

                // Correct Body FLU to Earth NED rotation:
                // V_N = V_fwd * cos(yaw) + V_left * sin(yaw)
                // V_E = V_fwd * sin(yaw) - V_left * cos(yaw)
                // V_D = -V_up
                float vx_ned =  v_fwd * cos_yaw + v_left * sin_yaw;
                float vy_ned =  v_fwd * sin_yaw - v_left * cos_yaw;
                float vz_ned = -v_up;

                // Invert yaw rate: ROS FLU CCW (+) -> PX4 NED CW (-)
                float yaw_rate_ned = -command.yaw_or_yaw_rate;

                px4_ros2::TrajectorySetpoint setpoint;
                setpoint.withVelocity(Eigen::Vector3f(vx_ned, vy_ned, vz_ned));
                setpoint.withYawRate(yaw_rate_ned);
                trajectory_setpoint_->update(setpoint);
            } else if (command.type == ControlType::Position) {
                // Position setpoint in NED frame (x=North, y=East, z=Down from ENU x=East, y=North, z=Up)
                Eigen::Vector3f target_ned(command.vector.y(), command.vector.x(), -command.vector.z());
                
                // Convert ENU yaw (angle from East CCW) to NED yaw (angle from North CW):
                float target_yaw_ned = px4_ros2::yawEnuToNed(command.yaw_or_yaw_rate);
                goto_setpoint_->update(target_ned, target_yaw_ned);
            }
        }

        // 4. Publish diagnostics status & commanded velocity
        if (command.type == ControlType::Velocity) {
            geometry_msgs::msg::TwistStamped twist_msg;
            twist_msg.header.stamp = node().now();
            twist_msg.header.frame_id = "base_link";
            twist_msg.twist.linear.x = command.vector.x();
            twist_msg.twist.linear.y = command.vector.y();
            twist_msg.twist.linear.z = command.vector.z();
            twist_msg.twist.angular.z = command.yaw_or_yaw_rate;
            cmd_vel_pub_->publish(twist_msg);
        }

        if (!command.status_message.empty()) {
            std_msgs::msg::String status_msg;
            status_msg.data = "[" + (current_algorithm_ ? current_algorithm_->getName() : "Standby") + "] " + command.status_message;
            status_pub_->publish(status_msg);
        }
    }

    void setAlgorithm(const std::string& name)
    {
        if (name.empty() || name == "none" || name == "blank") {
            current_algorithm_ = nullptr;
            RCLCPP_INFO(node().get_logger(), "Active algorithm set to: 'none' (Standby / Hover Hold)");
            return;
        }

        auto alg = AlgorithmFactory::createAlgorithm(name);
        if (alg) {
            current_algorithm_ = alg;
            current_algorithm_->init(node());
            current_algorithm_->reset();
            RCLCPP_INFO(node().get_logger(), "Active algorithm set to: '%s'", current_algorithm_->getName().c_str());
        } else {
            RCLCPP_WARN(node().get_logger(), "Unknown algorithm '%s'. Keeping previous or standby.", name.c_str());
        }
    }

private:
    void onDepthImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding != "32FC1") {
            RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000,
                                 "Unsupported depth encoding: '%s' (expected '32FC1')", msg->encoding.c_str());
            return;
        }

        if (msg->width == 0 || msg->height == 0 || msg->data.size() < static_cast<size_t>(msg->height * msg->step)) {
            return;
        }

        int width = static_cast<int>(msg->width);
        int height = static_cast<int>(msg->height);
        const int row_stride = static_cast<int>(msg->step / sizeof(float));

        // Direct zero-copy floating point depth buffer (AirSim DepthPlanar / 32FC1)
        const float* depth_ptr = reinterpret_cast<const float*>(msg->data.data());

        // 3D Spatial Grid: 3 rows (Upper, Mid, Lower) x 3 cols (Left, Center, Right)
        float grid[3][3];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                grid[r][c] = 100.0f;
            }
        }

        int band_height = height / 3;
        int sector_width = width / 3;

        // Subsample step for 4x CPU efficiency on high-resolution depth streams
        const int r_step = 2;
        const int c_step = 2;

        float overall_min = 100.0f;

        for (int r_band = 0; r_band < 3; ++r_band) {
            int r_start = r_band * band_height;
            int r_end = (r_band == 2) ? height : (r_band + 1) * band_height;

            for (int c_sector = 0; c_sector < 3; ++c_sector) {
                int c_start = c_sector * sector_width;
                int c_end = (c_sector == 2) ? width : (c_sector + 1) * sector_width;

                float cell_min = 100.0f;

                for (int r = r_start; r < r_end; r += r_step) {
                    int row_offset = r * row_stride;
                    for (int c = c_start; c < c_end; c += c_step) {
                        float d = depth_ptr[row_offset + c];
                        if (d > 0.2f && !std::isnan(d)) {
                            if (d < cell_min) cell_min = d;
                            if (d < overall_min) overall_min = d;
                        }
                    }
                }
                grid[r_band][c_sector] = cell_min;
            }
        }

        float left_mid = grid[1][0];
        float center_mid = grid[1][1];
        float right_mid = grid[1][2];

        float upper_min = std::min({grid[0][0], grid[0][1], grid[0][2]});
        float lower_min = std::min({grid[2][0], grid[2][1], grid[2][2]});

        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            latest_left_depth_ = left_mid;
            latest_center_depth_ = center_mid;
            latest_right_depth_ = right_mid;
            latest_upper_depth_ = upper_min;
            latest_lower_depth_ = lower_min;
            latest_min_depth_ = overall_min;

            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    latest_sectors_3d_[r][c] = grid[r][c];
                }
            }

            latest_depth_time_ = node().now();
        }
    }

    void onTargetPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        latest_target_pos_ = Eigen::Vector3f(msg->point.x, msg->point.y, msg->point.z);
        
        // Check frame ID: world-fixed frames vs body-relative frames
        const std::string& frame = msg->header.frame_id;
        latest_target_is_world_ = (frame == "map" || frame == "odom" || frame == "world" || frame == "earth");
        
        latest_target_detected_ = true;
        latest_target_time_ = node().now();
    }

    bool is_standalone_{false};

    // Setpoint and odometry interfaces from px4_ros2_cpp
    std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> goto_setpoint_;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> local_position_;

    // Active modular algorithm
    std::shared_ptr<IAutonomyAlgorithm> current_algorithm_;

    // Enterprise Flight Safety Supervisor
    enterprise::EnterpriseFlightSupervisor supervisor_;

    // ROS 2 Subscriptions & Publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // Mode state
    std::atomic<bool> is_active_{false};

    // Thread-safe cached sensor metrics
    std::mutex sensor_mutex_;
    float latest_min_depth_{100.0f};
    float latest_center_depth_{100.0f};
    float latest_left_depth_{100.0f};
    float latest_right_depth_{100.0f};
    float latest_upper_depth_{100.0f};
    float latest_lower_depth_{100.0f};
    float latest_sectors_3d_[3][3] = {
        {100.0f, 100.0f, 100.0f},
        {100.0f, 100.0f, 100.0f},
        {100.0f, 100.0f, 100.0f}
    };

    bool latest_target_detected_{false};
    bool latest_target_is_world_{false};
    Eigen::Vector3f latest_target_pos_{Eigen::Vector3f::Zero()};
    rclcpp::Time latest_depth_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_target_time_{0, 0, RCL_ROS_TIME};
};

} // namespace px4_airsim_autonomy

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("autonomous_flight_mode_node");

    bool enable_fmu_registration = node->declare_parameter<bool>("enable_fmu_registration", true);

    auto autonomy_mode = std::make_shared<px4_airsim_autonomy::AutonomousFlightMode>(
        *node, !enable_fmu_registration);

    if (enable_fmu_registration) {
        RCLCPP_INFO(node->get_logger(), "Registering Autonomous Flight Mode with PX4 FMU...");
        if (!autonomy_mode->doRegister()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to register Autonomous Flight Mode with PX4 FMU");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Autonomous Flight Mode registered successfully with PX4 FMU.");
    } else {
        RCLCPP_WARN(node->get_logger(), "Running in standalone simulation mode (PX4 FMU registration bypassed).");
    }

    RCLCPP_INFO(node->get_logger(), "AirSim Autonomous Flight Mode Node spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
