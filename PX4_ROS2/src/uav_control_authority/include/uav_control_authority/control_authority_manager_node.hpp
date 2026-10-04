#ifndef UAV_CONTROL_AUTHORITY__CONTROL_AUTHORITY_MANAGER_NODE_HPP_
#define UAV_CONTROL_AUTHORITY__CONTROL_AUTHORITY_MANAGER_NODE_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/control_authority.hpp>
#include <uav_interfaces/msg/control_command.hpp>
#include <uav_interfaces/srv/clear_fault.hpp>
#include <uav_interfaces/srv/set_control_authority.hpp>

#include "uav_control_authority/authority_arbiter.hpp"

namespace uav_control_authority
{

/// Wires the four /control/cmd_* topics through AuthorityArbiter onto the
/// single /control/command_selected writer (R6). A pure relay: see S:1 of
/// the plan before touching anything here.
///
/// Three things to know before changing this file:
///   1. This node NEVER synthesizes a setpoint. If every source is silent it
///      publishes nothing at all -- the gateway's own 0.5 s timeout is what
///      then lets PX4 failsafe, and that silence is deliberate (S:1).
///   2. All safety-critical arithmetic (priority, hysteresis, latch) lives in
///      the ROS-free AuthorityArbiter so it is pinned by test_authority_arbiter
///      without a domain. This file only converts messages and wires topics.
///   3. Content is never edited except the `source` field on a restamp. No
///      clamping, no smoothing -- doing that to a SAFETY command would be
///      delivering a life-saving setpoint late.
///   4. AuthorityArbiter is NOT synchronised (R24): every subscription, the
///      monitor timer, AND both services (SetControlAuthority,
///      clear_safety_latch) share ONE MutuallyExclusive callback group
///      (io_group_) so none of them can run concurrently. Never give a
///      service (or anything else touching arbiter_) its own callback group
///      -- that reintroduces the B1 data race a review caught 2026-08-20
///      (SAFETY could lose authority to a service call computed against a
///      stale snapshot).
///   5. Parameters are refused at runtime (Y17): the same reasoning that
///      makes the node refuse to START on unusable parameters (S:3d) applies
///      to changing them while it is already flying. Restart to reconfigure.
class ControlAuthorityManagerNode : public rclcpp::Node
{
public:
  explicit ControlAuthorityManagerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  ArbiterParams declareParams();
  void onSourceMessage(uint8_t channel, const uav_interfaces::msg::ControlCommand::SharedPtr msg);
  void onMonitorTick();
  void handleSetControlAuthority(
    const std::shared_ptr<uav_interfaces::srv::SetControlAuthority::Request> request,
    std::shared_ptr<uav_interfaces::srv::SetControlAuthority::Response> response);
  /// P8 R2: the ONLY way to clear a SAFETY latch (arbiter's requestLatch
  /// still unconditionally refuses RELEASE at SAFETY, Y6). The legitimate
  /// caller is uav_safety, AFTER its own ClearFault has accepted -- this
  /// node never judges the fault, only executes the release.
  void handleClearSafetyLatch(
    const std::shared_ptr<uav_interfaces::srv::ClearFault::Request> request,
    std::shared_ptr<uav_interfaces::srv::ClearFault::Response> response);

  void logDrop(uint8_t channel, const ArrivalResult & result);
  void publishAuthority(uint8_t previous_source, uint8_t active_source, const std::string & reason);
  void checkDuplicatePublishers();
  void publishDiagnostics();
  /// Y17: arbitration parameters are validated once, at startup (S:3d) --
  /// runtime changes are refused outright, never silently applied unvalidated.
  rcl_interfaces::msg::SetParametersResult rejectParameterChange(
    const std::vector<rclcpp::Parameter> & parameters) const;

  double nowSeconds() const {return now().seconds();}

  std::string uav_id_;
  std::string prefix_;
  ArbiterParams params_;
  AuthorityArbiter arbiter_;

  unsigned tick_count_ = 0;
  // Monitor runs at 1/kMonitorPeriodSec = 20 Hz (contract, N-f).
  unsigned heartbeat_every_ticks_ = 10;      // 20 Hz monitor / 10 = 2 Hz heartbeat (contract)
  unsigned diagnostics_every_ticks_ = 20;    // 20 Hz monitor / 20 = 1 Hz diagnostics (contract)

  // Y2: publisher-graph queries only run at the diagnostics cadence, not
  // every monitor tick -- get_publishers_info_by_topic() walks graph state.
  bool duplicate_publisher_seen_ = false;
  std::array<std::pair<std::string, size_t>, 5> publisher_counts_{};

  // R24: the ONLY callback group for this node -- subs, timer AND the
  // service all share it so arbiter_ is never touched by two threads at once.
  rclcpp::CallbackGroup::SharedPtr io_group_;

  rclcpp::Subscription<uav_interfaces::msg::ControlCommand>::SharedPtr safety_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::ControlCommand>::SharedPtr operator_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::ControlCommand>::SharedPtr mission_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::ControlCommand>::SharedPtr test_subscription_;

  rclcpp::Publisher<uav_interfaces::msg::ControlCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<uav_interfaces::msg::ControlAuthority>::SharedPtr authority_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  rclcpp::Service<uav_interfaces::srv::SetControlAuthority>::SharedPtr set_authority_service_;
  // P8 R2: reuses ClearFault (no new interface) -- see handleClearSafetyLatch.
  rclcpp::Service<uav_interfaces::srv::ClearFault>::SharedPtr clear_safety_latch_service_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  // Y17: keeps rejectParameterChange() registered for the node's lifetime.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace uav_control_authority

#endif  // UAV_CONTROL_AUTHORITY__CONTROL_AUTHORITY_MANAGER_NODE_HPP_
