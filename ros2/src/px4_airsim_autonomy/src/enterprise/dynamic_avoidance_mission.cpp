#include "px4_airsim_autonomy/enterprise/dynamic_avoidance_mission.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>

namespace px4_airsim_autonomy {
namespace enterprise {

DynamicAvoidanceMission::DynamicAvoidanceMission()
{
    production::DynamicStoppingBubbleConfig cfg;
    cfg.a_max = 3.0f;
    cfg.t_latency = 0.15f;
    cfg.d_margin = 1.5f;
    cfg.r_lateral = 1.0f;
    cfg.r_vertical = 0.8f;
    bubble_ = production::DynamicStoppingBubble(cfg);
}

void DynamicAvoidanceMission::init(rclcpp::Node& node)
{
    cruise_speed_ = node.has_parameter("avoidance.cruise_speed") ? static_cast<float>(node.get_parameter("avoidance.cruise_speed").as_double()) : node.declare_parameter<float>("avoidance.cruise_speed", 2.0f);
    max_accel_ = node.has_parameter("avoidance.max_accel") ? static_cast<float>(node.get_parameter("avoidance.max_accel").as_double()) : node.declare_parameter<float>("avoidance.max_accel", 3.0f);
    safety_margin_ = node.has_parameter("avoidance.safety_margin") ? static_cast<float>(node.get_parameter("avoidance.safety_margin").as_double()) : node.declare_parameter<float>("avoidance.safety_margin", 1.5f);
    cruise_altitude_ = node.has_parameter("avoidance.cruise_altitude") ? static_cast<float>(node.get_parameter("avoidance.cruise_altitude").as_double()) : node.declare_parameter<float>("avoidance.cruise_altitude", 10.0f);

    production::DynamicStoppingBubbleConfig cfg;
    cfg.a_max = max_accel_;
    cfg.t_latency = 0.15f;
    cfg.d_margin = safety_margin_;
    bubble_.setConfig(cfg);

    RCLCPP_INFO(node.get_logger(),
                "[DynamicAvoidanceMission] Initialized (CruiseSpeed=%.1f m/s, MaxAccel=%.1f m/s^2, SafetyMargin=%.1f m)",
                cruise_speed_, max_accel_, safety_margin_);
}

void DynamicAvoidanceMission::onActivate()
{
    current_velocity_cmd_ = Eigen::Vector3f::Zero();
}

void DynamicAvoidanceMission::onDeactivate()
{
    current_velocity_cmd_ = Eigen::Vector3f::Zero();
}

void DynamicAvoidanceMission::reset()
{
    current_velocity_cmd_ = Eigen::Vector3f::Zero();
}

AutonomyCommand DynamicAvoidanceMission::update(const SensorSnapshot& sensors, float dt)
{
    AutonomyCommand cmd;
    cmd.type = ControlType::Velocity;

    // 1. Update stopping bubble state based on current velocity
    bubble_.update(sensors.position, sensors.velocity);

    // 2. Check for obstacle penetration using 3D sectors and minimum depth
    production::BubbleViolation violation = bubble_.checkSensorSnapshot(sensors);

    // Also verify minimum depth directly
    float d_stop = bubble_.computeStoppingDistance(sensors.velocity.norm());
    if (sensors.min_depth_distance < d_stop) {
        violation.is_violated = true;
        violation.obstacle_distance = sensors.min_depth_distance;
    }

    // 3. Compute desired guidance velocity
    Eigen::Vector3f desired_vel = Eigen::Vector3f::Zero();
    float commanded_yaw_rate = 0.0f;
    std::string status;

    if (violation.is_violated) {
        // Emergency deceleration braking vector: a_brake = -a_max * (v / ||v||)
        Eigen::Vector3f a_brake = bubble_.computeBrakingAcceleration(sensors.velocity);
        
        // Decelerate forward component
        desired_vel.x() = std::max(0.0f, current_velocity_cmd_.x() + a_brake.x() * dt);

        // Evaluate lateral clearance (left vs right sectors)
        float left_clearance = sensors.left_sector_distance;
        float right_clearance = sensors.right_sector_distance;

        if (left_clearance > right_clearance) {
            desired_vel.y() = 1.0f;  // Evade left (+Y in Body FLU)
            commanded_yaw_rate = 0.4f;
            status = "STOPPING BUBBLE BRAKE: Evading Left (clearance " + std::to_string(left_clearance) + "m)";
        } else {
            desired_vel.y() = -1.0f; // Evade right (-Y in Body FLU)
            commanded_yaw_rate = -0.4f;
            status = "STOPPING BUBBLE BRAKE: Evading Right (clearance " + std::to_string(right_clearance) + "m)";
        }

        // Evaluate vertical clearance if lateral is constrained
        if (left_clearance < 2.5f && right_clearance < 2.5f) {
            if (sensors.upper_sector_distance > 3.0f) {
                desired_vel.z() = 0.8f; // Climb over
                status += " + Vertical Climb";
            } else if (sensors.lower_sector_distance > 3.0f) {
                desired_vel.z() = -0.6f; // Descend under
                status += " + Vertical Dive";
            }
        }
    } else {
        // Clear flight corridor: cruise forward at target speed
        float target_vx = cruise_speed_;
        float dv = target_vx - current_velocity_cmd_.x();
        float max_dv = max_accel_ * dt;
        desired_vel.x() = current_velocity_cmd_.x() + std::clamp(dv, -max_dv, max_dv);

        // Regulate altitude back to cruise altitude
        float alt_err = cruise_altitude_ - sensors.position.z();
        desired_vel.z() = std::clamp(alt_err * 0.8f, -1.5f, 1.5f);

        desired_vel.y() = 0.0f;
        commanded_yaw_rate = 0.0f;
        status = "CRUISING: Safe Flight Corridor clear (d_stop=" + std::to_string(d_stop) + "m)";
    }

    // Rate-limit acceleration commands to prevent rate-gyro clipping
    current_velocity_cmd_ = desired_vel;

    cmd.vector = current_velocity_cmd_;
    cmd.yaw_or_yaw_rate = commanded_yaw_rate;
    cmd.status_message = status;

    return cmd;
}

} // namespace enterprise
} // namespace px4_airsim_autonomy
