#include "uav_observability/diagnostics_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/key_value.hpp>

namespace uav_observability
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using uav_interfaces::msg::VehicleState;

namespace
{

// P10.9b: single source of truth for the phase predicate's cadence item --
// referenced both by declareAndValidateParams() (require it is present in
// always_names) and by the ctor (a dedicated typed sub on this same topic).
constexpr const char * kVehicleStateItemName = "state/vehicle";

void require(bool condition, const std::string & detail)
{
  if (!condition) {
    throw std::invalid_argument("diagnostics_node params: " + detail);
  }
}

Level levelWhenMissingFromString(const std::string & value, const std::string & item_name)
{
  if (value == "warn") {return Level::kWarn;}
  if (value == "error") {return Level::kError;}
  require(false, "level_when_missing must be warn|error: " + item_name);
  return Level::kWarn;   // unreachable, require() above always throws
}

rclcpp::QoS qosFromProfile(const std::string & profile, int64_t depth)
{
  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(depth)));
  if (profile == "reliable") {
    qos.reliable();
  } else if (profile == "reliable_tl") {
    qos.reliable().transient_local();
  } else {
    // "be" and anything else -- validate() already forbids anything else
    // reaching here (same convention as rosbag_manager_node.cpp).
    qos.best_effort();
  }
  return qos;
}

KeyValue keyValue(const std::string & key, double value)
{
  KeyValue pair;
  pair.key = key;
  pair.value = std::to_string(value);
  return pair;
}

KeyValue keyValueStr(const std::string & key, const std::string & value)
{
  KeyValue pair;
  pair.key = key;
  pair.value = value;
  return pair;
}

// StalenessBoard's kUnknown has no diagnostic_msgs equivalent -- STALE
// ("cannot measure") is the nearest fit, same fallback rosbag_manager uses.
uint8_t boardLevelToDiagnosticLevel(Level level)
{
  switch (level) {
    case Level::kOk: return DiagnosticStatus::OK;
    case Level::kWarn: return DiagnosticStatus::WARN;
    case Level::kError: return DiagnosticStatus::ERROR;
    default: return DiagnosticStatus::STALE;
  }
}

// ERROR > STALE(unmeasured) > WARN > OK -- R30: not-measured outranks WARN.
int severityRank(uint8_t diagnostic_level)
{
  switch (diagnostic_level) {
    case DiagnosticStatus::ERROR: return 3;
    case DiagnosticStatus::STALE: return 2;
    case DiagnosticStatus::WARN: return 1;
    default: return 0;
  }
}

const char * goNoGoName(GoNoGo state)
{
  switch (state) {
    case GoNoGo::kGo: return "GO";
    case GoNoGo::kNoGo: return "NO_GO";
    default: return "UNKNOWN";
  }
}

uint8_t goNoGoToDiagnosticLevel(GoNoGo state)
{
  switch (state) {
    case GoNoGo::kGo: return DiagnosticStatus::OK;
    case GoNoGo::kNoGo: return DiagnosticStatus::ERROR;
    default: return DiagnosticStatus::STALE;
  }
}

const char * phaseName(Phase phase)
{
  return phase == Phase::kFlight ? "flight" : "preflight";
}

const char * phaseSourceName(PhaseSource source)
{
  return source == PhaseSource::kMeasured ? "measured" : "unmeasured_strict";
}

const char * gateModeOverrideName(GateModeOverride value)
{
  switch (value) {
    case GateModeOverride::kPreflight: return "preflight";
    case GateModeOverride::kFlight: return "flight";
    default: return "auto";
  }
}

GateModeOverride gateModeOverrideFromString(const std::string & value)
{
  if (value == "auto") {return GateModeOverride::kAuto;}
  if (value == "preflight") {return GateModeOverride::kPreflight;}
  if (value == "flight") {return GateModeOverride::kFlight;}
  require(false, "gate_mode_override must be auto|preflight|flight");
  return GateModeOverride::kAuto;   // unreachable, require() above always throws
}

bool startsWithAny(const std::string & value, std::initializer_list<const char *> prefixes)
{
  for (const char * prefix_c : prefixes) {
    const std::string prefix(prefix_c);
    if (value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}

// G1: PREFIX match, never full-string -- enforcement_enabled=false dry-run
// values carry a "(dry-run, ...)" suffix (safety_supervisor_node.cpp's
// describeAction(), contract S:2.18's own stated rule).
ActionClass actionClassOf(const std::string & raw)
{
  if (startsWithAny(raw, {"hold", "would_hold"})) {return ActionClass::kHold;}
  if (startsWithAny(raw, {"inhibit", "would_inhibit"})) {return ActionClass::kInhibit;}
  if (startsWithAny(raw, {"report"})) {return ActionClass::kReport;}
  return ActionClass::kUnrecognized;   // missing key, "none", or anything else
}

std::string findActionKeyValue(const DiagnosticStatus & status)
{
  for (const KeyValue & kv : status.values) {
    if (kv.key == "action") {
      return kv.value;
    }
  }
  return "";
}

// P10.9b NW1/NW2 (design panel G1/G2): a child this "never-waivable" MUST be
// counted regardless of owner/phase/waiver-table -- the ONE exception is
// owner-gating itself stays outside this function's authority (O1's
// blackbox exclusion is an absolute structural rule, stronger than NW's
// "don't let a waiver hide a live latch" argument; the caller applies
// owner_counted separately, see onEvalTick()). action-class is a ONE-WAY
// veto: kReport/kUnrecognized-but-not-action_aware never GRANT immunity,
// they simply fail to trigger this forcing, falling through to ordinary
// waiver matching.
bool isNeverWaivable(bool source_action_aware, uint8_t level, const std::string & action)
{
  if (level != DiagnosticStatus::ERROR) {
    return false;
  }
  const ActionClass action_class = actionClassOf(action);
  if (action_class == ActionClass::kHold || action_class == ActionClass::kInhibit) {
    return true;   // NW1
  }
  if (source_action_aware && action_class == ActionClass::kUnrecognized) {
    return true;   // NW2: fail-closed when an action_aware source can't resolve "action"
  }
  return false;
}

// VÀNG#9: cached_system_health_'s C++ default-member-init (a plain
// DiagnosticStatus()) has level=0=OK -- a subscriber racing the FIRST
// onEvalTick() (e.g. publish_period_sec_ misconfigured shorter than
// report_period_sec_, guarded separately below, but belt-and-suspenders
// against any other future ordering change) must never be able to observe
// a synthetic "system is OK" before any real evaluation happened (R30).
DiagnosticStatus makeUnknownSystemHealth(const std::string & uav_id)
{
  DiagnosticStatus status;
  status.name = "system";
  status.hardware_id = uav_id;
  status.level = goNoGoToDiagnosticLevel(GoNoGo::kUnknown);
  status.message = std::string("go_no_go=") + goNoGoName(GoNoGo::kUnknown) + " (not yet evaluated)";
  status.values.push_back(keyValueStr("go_no_go", goNoGoName(GoNoGo::kUnknown)));
  status.values.push_back(keyValue("unknown_count", 0.0));
  status.values.push_back(keyValue("error_count", 0.0));
  status.values.push_back(keyValue("warn_count", 0.0));
  status.values.push_back(keyValue("stale_count", 0.0));
  status.values.push_back(keyValueStr("worst_item", ""));
  status.values.push_back(keyValue("clock_regressions", 0.0));
  status.values.push_back(keyValueStr("gate_mode", ""));
  status.values.push_back(keyValueStr("gate_mode_source", ""));
  status.values.push_back(keyValue("waived_count", 0.0));
  status.values.push_back(keyValue("waiver_unmatched", 0.0));
  status.values.push_back(keyValueStr("blocking", ""));
  status.values.push_back(keyValue("watch_failed", 0.0));
  // C6: NaN, not 0.0/epoch -- this value was never actually evaluated, so
  // it must never look "freshly stamped at t=0" to a consumer applying
  // R32's age check.
  status.values.push_back(keyValue("stamp_sec", std::numeric_limits<double>::quiet_NaN()));
  status.values.push_back(keyValue("wall_stamp_sec", std::numeric_limits<double>::quiet_NaN()));
  return status;
}

// C2: "" (unowned -- unconditionally relevant), "perception", or "blackbox".
void requireValidOwner(
  const std::string & group_label, const std::string & owner, const std::string & item_name)
{
  require(
    owner.empty() || owner == "perception" || owner == "blackbox",
    group_label + ": owner must be \"\"|perception|blackbox: " + item_name);
}

// V-O2: the 9 parallel arrays that describe one watch group must agree on
// length -- a mismatch would index out of bounds below.
void requireSameLength(
  const std::string & group_label, const std::vector<std::string> & names,
  const std::vector<std::string> & topics, const std::vector<std::string> & types,
  const std::vector<std::string> & qos_profiles, const std::vector<int64_t> & depths,
  const std::vector<double> & expected_hz, const std::vector<double> & timeout_sec,
  const std::vector<std::string> & level_when_missing, const std::vector<std::string> & owner)
{
  const size_t length = names.size();
  require(
    topics.size() == length && types.size() == length && qos_profiles.size() == length &&
    depths.size() == length && expected_hz.size() == length && timeout_sec.size() == length &&
    level_when_missing.size() == length && owner.size() == length,
    group_label +
    ": names/topics/types/qos/depths/expected_hz/timeout_sec/level_when_missing/owner "
    "must be the same length (V-O2)");
}

std::vector<WatchItem> parseWatchGroup(
  const std::string & group_label, const std::vector<std::string> & names,
  const std::vector<std::string> & topics, const std::vector<std::string> & types,
  const std::vector<std::string> & qos_profiles, const std::vector<int64_t> & depths,
  const std::vector<double> & expected_hz, const std::vector<double> & timeout_sec,
  const std::vector<std::string> & level_when_missing, const std::vector<std::string> & owner,
  StalenessBoard & board, std::unordered_set<std::string> & cadence_item_names_out)
{
  requireSameLength(
    group_label, names, topics, types, qos_profiles, depths, expected_hz, timeout_sec,
    level_when_missing, owner);

  std::set<std::string> unique_names(names.begin(), names.end());
  require(unique_names.size() == names.size(), group_label + ": item names must not be duplicated");

  std::vector<WatchItem> items;
  items.reserve(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    require(!names[i].empty(), group_label + ": item name must not be empty at index " +
      std::to_string(i));
    // Sentinel pairing: an item fed by a Sub-B typed subscription instead of
    // a GenericSubscription leaves BOTH topic and type empty, never just one.
    require(
      topics[i].empty() == types[i].empty(),
      group_label + ": topic/type must be both-empty or both-set: " + names[i]);
    // V1 (review lượt 3): validated UNCONDITIONALLY, same as depth below --
    // a sentinel row's (topic=="") qos_profile is STILL used to build its
    // real typed subscription (the ctor's dedicated state/vehicle sub reuses
    // this exact field via qosFromProfile()), so gating this check on
    // !topics[i].empty() let an invalid value on a sentinel row silently
    // fall through qosFromProfile()'s "anything else -> best_effort" branch
    // instead of refusing to start.
    require(
      qos_profiles[i] == "be" || qos_profiles[i] == "reliable" ||
      qos_profiles[i] == "reliable_tl",
      group_label + ": qos must be be|reliable|reliable_tl: " + names[i]);
    require(
      std::isfinite(static_cast<double>(depths[i])) && depths[i] > 0,
      group_label + ": depth must be finite and > 0: " + names[i]);
    requireValidOwner(group_label, owner[i], names[i]);

    // V-O1/V-O4 (finite hz > 0, timeout >= 2/hz) are enforced by addItem()
    // itself -- it already reports the item name in its exception message.
    board.addItem(
      names[i], expected_hz[i], timeout_sec[i], levelWhenMissingFromString(
        level_when_missing[i], names[i]));
    cadence_item_names_out.insert(names[i]);

    WatchItem item;
    item.name = names[i];
    item.topic = topics[i];
    item.type = types[i];
    item.qos_profile = qos_profiles[i];
    item.depth = depths[i];
    item.owner = owner[i];
    items.push_back(item);
  }
  return items;
}

}  // namespace

DiagnosticsNode::DiagnosticsNode(const rclcpp::NodeOptions & options)
try
: Node("diagnostics_node", options)
{
  declareAndValidateParams();   // throws std::invalid_argument -- refuse to start (yaml sai)
  prefix_ = "/uav/" + uav_id_;
  // VÀNG#9: explicit UNKNOWN, not the default-constructed DiagnosticStatus's
  // level=0=OK -- set before ANYTHING (publishers, timers) could possibly
  // cause a publish of this value.
  cached_system_health_ = makeUnknownSystemHealth(uav_id_);

  // R24: the ONLY group -- every sub AND both timers.
  io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  aggregated_publisher_ = create_publisher<DiagnosticArray>(
    prefix_ + "/diagnostics/aggregated", rclcpp::QoS(10));
  system_health_publisher_ = create_publisher<DiagnosticStatus>(
    prefix_ + "/state/system_health", rclcpp::QoS(1).reliable().transient_local());

  diag_source_content_.resize(diag_sources_.size());

  subscribeWatchGroup(always_items_, io_group_);
  subscribeWatchGroup(flight_only_items_, io_group_);
  subscribeDiagSources(io_group_);

  // Review lượt 2 (N2): dedicated typed sub on /state/vehicle -- the ONLY
  // subscriber on that topic now ("state/vehicle" is a sentinel always-item,
  // topic/type both "", so subscribeWatchGroup() above created no
  // GenericSubscription for it). Reuses that item's own configured
  // qos/depth so there is exactly one source of truth for how this topic
  // is subscribed; the topic NAME itself comes from kVehicleStateItemName
  // (the item's topic field is empty by construction for a sentinel row).
  {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = io_group_;
    const WatchItem & vehicle_item = always_items_[vehicle_state_item_index_];
    vehicle_state_subscription_ = create_subscription<VehicleState>(
      prefix_ + "/" + kVehicleStateItemName,
      qosFromProfile(vehicle_item.qos_profile, vehicle_item.depth),
      [this](const VehicleState::SharedPtr message) {onVehicleState(message);}, sub_options);
  }

  param_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return onSetParameters(parameters);
    });

  eval_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(report_period_sec_),
    [this]() {onEvalTick();}, io_group_);
  publish_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(publish_period_sec_),
    [this]() {onPublishTick();}, io_group_);

  RCLCPP_INFO(
    get_logger(), "diagnostics_node ready for %s: gate_mode_override=%s always=%zu "
    "flight_only=%zu diag_sources=%zu waivers=%zu (watch_failed=%u)",
    uav_id_.c_str(), gateModeOverrideName(gate_mode_override_.load()), always_items_.size(),
    flight_only_items_.size(), diag_sources_.size(), waivers_.size(), watch_failed_);
}
catch (const std::invalid_argument & error)
{
  RCLCPP_FATAL(
    rclcpp::get_logger("diagnostics_node"), "refusing to start: %s", error.what());
  throw;
}

void DiagnosticsNode::declareAndValidateParams()
{
  uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
  // P10.9b: replaces the old "gate_mode" param (which WROTE the phase
  // directly -- a blind spot in flight, contract S:2.20). This override can
  // only TIGHTEN the measured phase, never loosen it -- see effectivePhase().
  const std::string gate_mode_override_param =
    declare_parameter<std::string>("gate_mode_override", "auto");
  gate_mode_override_ = gateModeOverrideFromString(gate_mode_override_param);

  report_period_sec_ = declare_parameter<double>("report_period_sec", 0.05);
  publish_period_sec_ = declare_parameter<double>("publish_period_sec", 1.0);
  sub_b_stale_after_sec_ = declare_parameter<double>("sub_b_stale_after_sec", 3.0);
  // C2: sim.launch.py wires these to its own `perception`/`blackbox` flags
  // (launch file, not this yaml -- the SAME 2 flags that decide whether
  // camera_health_node/rosbag_manager_node even exist in this bringup).
  perception_enabled_ = declare_parameter<bool>("perception_enabled", true);
  blackbox_enabled_ = declare_parameter<bool>("blackbox_enabled", true);

  const auto finitePositive = [](double value) {return std::isfinite(value) && value > 0.0;};
  require(finitePositive(report_period_sec_), "report_period_sec must be finite and > 0");
  require(finitePositive(publish_period_sec_), "publish_period_sec must be finite and > 0");
  require(finitePositive(sub_b_stale_after_sec_), "sub_b_stale_after_sec must be finite and > 0");
  // VÀNG#9: a publish tick racing AHEAD of the first evaluation tick could
  // publish a still-initial cached_system_health_ (defended against
  // separately, see makeUnknownSystemHealth()) or -- once real evaluation
  // starts -- interleave with onEvalTick() in a way that makes the on-change
  // detection unreliable. publish_period_sec_ must never be tighter than
  // report_period_sec_.
  require(
    publish_period_sec_ >= report_period_sec_,
    "publish_period_sec must be >= report_period_sec");

  // Explicit vector{} (not {}) -- ambiguous with the ParameterDescriptor
  // overload otherwise, same note as rosbag_manager_node.cpp.
  const auto declareStrings = [this](const std::string & name) {
      return declare_parameter<std::vector<std::string>>(name, std::vector<std::string>{});
    };
  const auto declareDoubles = [this](const std::string & name) {
      return declare_parameter<std::vector<double>>(name, std::vector<double>{});
    };
  const auto declareInts = [this](const std::string & name) {
      return declare_parameter<std::vector<int64_t>>(name, std::vector<int64_t>{});
    };
  const auto declareBools = [this](const std::string & name) {
      return declare_parameter<std::vector<bool>>(name, std::vector<bool>{});
    };

  const auto always_names = declareStrings("always_names");
  const auto always_topics = declareStrings("always_topics");
  const auto always_types = declareStrings("always_types");
  const auto always_qos = declareStrings("always_qos");
  const auto always_depths = declareInts("always_depths");
  const auto always_hz = declareDoubles("always_expected_hz");
  const auto always_timeout = declareDoubles("always_timeout_sec");
  const auto always_level = declareStrings("always_level_when_missing");
  const auto always_owner = declareStrings("always_owner");

  const auto flight_names = declareStrings("flight_only_names");
  const auto flight_topics = declareStrings("flight_only_topics");
  const auto flight_types = declareStrings("flight_only_types");
  const auto flight_qos = declareStrings("flight_only_qos");
  const auto flight_depths = declareInts("flight_only_depths");
  const auto flight_hz = declareDoubles("flight_only_expected_hz");
  const auto flight_timeout = declareDoubles("flight_only_timeout_sec");
  const auto flight_level = declareStrings("flight_only_level_when_missing");
  const auto flight_owner = declareStrings("flight_only_owner");

  require(!always_names.empty(), "always_names must not be empty");

  always_items_ = parseWatchGroup(
    "always", always_names, always_topics, always_types, always_qos, always_depths, always_hz,
    always_timeout, always_level, always_owner, cadence_board_, cadence_item_names_);
  flight_only_items_ = parseWatchGroup(
    "flight_only", flight_names, flight_topics, flight_types, flight_qos, flight_depths,
    flight_hz, flight_timeout, flight_level, flight_owner, cadence_board_, cadence_item_names_);

  // P10.9b: the phase predicate reads THIS item's own Sub-A verdict
  // (age_sec vs its own always_timeout_sec) as "tươi" -- resolved ONCE here
  // by name, never a per-tick string search.
  //
  // Review lượt 2 (N2): the required shape flipped -- this MUST now be a
  // sentinel row (topic/type both ""), fed exclusively by the dedicated
  // typed subscription's own onVehicleState() callback (single reader, see
  // the ctor and the class-level comment on vehicle_state_subscription_).
  // A real GenericSubscription row here would resurrect the two-independent
  // -readers bug this fix removes.
  {
    const auto it = std::find_if(
      always_items_.begin(), always_items_.end(),
      [](const WatchItem & item) {return item.name == kVehicleStateItemName;});
    require(
      it != always_items_.end(),
      std::string("always_names must contain \"") + kVehicleStateItemName +
      "\" (phase predicate, P10.9b)");
    require(
      it->topic.empty(),
      std::string(kVehicleStateItemName) +
      " entry must be a SENTINEL row (topic/type both \"\", N2 review lượt 2) -- fed "
      "exclusively by the dedicated typed subscription in onVehicleState(), never a second "
      "GenericSubscription on the same topic");
    vehicle_state_item_index_ = static_cast<size_t>(std::distance(always_items_.begin(), it));

    // V4 (review lượt 3): the dwell window must outlast state/vehicle's OWN
    // GRACE period (always_timeout_sec), not merely clear 2 of its
    // sampling periods -- StalenessBoard reads a sample as fresh for its
    // WHOLE timeout_sec regardless of whether a newer one ever arrives
    // (staleness_board.cpp: age<=timeout_sec), so a dwell window shorter
    // than that timeout can confirm a downgrade off a single sample that
    // never even went stale by its own rule, let alone got reconfirmed.
    // Read off the SAME always_timeout_sec entry already declared above (no
    // new magic number) -- R29+R33.
    require(
      static_cast<double>(kPreflightDwellTicks) * report_period_sec_ >
      always_timeout[vehicle_state_item_index_],
      std::string("kPreflightDwellTicks * report_period_sec must be > ") +
      kVehicleStateItemName + "'s own always_timeout_sec, not just 2/expected_hz (V4, R29+R33)");
  }

  const auto diag_names = declareStrings("diag_source_names");
  const auto diag_topics = declareStrings("diag_source_topics");
  const auto diag_types = declareStrings("diag_source_types");
  const auto diag_qos = declareStrings("diag_source_qos");
  const auto diag_depths = declareInts("diag_source_depths");
  const auto diag_owner = declareStrings("diag_source_owner");
  // P10.9b: action_aware gates NW2 (fail-closed on an unresolved "action");
  // summary_child names this source's verified rollup child, if any (G8 --
  // "" for every source except diagnostics/safety, the only one this
  // property has been proven for on real code, see isNeverWaivable()'s
  // header comment and docs/interface-contract-v0.1.md S:2.20).
  const auto diag_action_aware = declareBools("diag_source_action_aware");
  const auto diag_summary_child = declareStrings("diag_source_summary_child");

  require(!diag_names.empty(), "diag_source_names must not be empty");
  require(
    diag_topics.size() == diag_names.size() && diag_types.size() == diag_names.size() &&
    diag_qos.size() == diag_names.size() && diag_depths.size() == diag_names.size() &&
    diag_owner.size() == diag_names.size() && diag_action_aware.size() == diag_names.size() &&
    diag_summary_child.size() == diag_names.size(),
    "diag_source_names/topics/types/qos/depths/owner/action_aware/summary_child "
    "must be the same length (V-O2)");

  std::set<std::string> unique_diag_names(diag_names.begin(), diag_names.end());
  require(
    unique_diag_names.size() == diag_names.size(), "diag_source_names must not be duplicated");

  double min_timeout_sec = sub_b_stale_after_sec_;
  for (double timeout : always_timeout) {min_timeout_sec = std::min(min_timeout_sec, timeout);}
  for (double timeout : flight_timeout) {min_timeout_sec = std::min(min_timeout_sec, timeout);}
  require(
    report_period_sec_ <= min_timeout_sec / 2.0,
    "report_period_sec must be <= min(timeout_sec)/2 (V-O3)");

  // VÀNG#7: the LOOSEST expected_hz that still satisfies V-O1
  // (timeout_sec >= 2/expected_hz) at exact equality with sub_b_stale_
  // after_sec_ itself -- self-consistent by construction for ANY value of
  // sub_b_stale_after_sec_, so it can never accidentally violate V-O1. This
  // replaces a hand-typed literal (1.0) that silently disagreed with real
  // measured Sub-B cadences (~0.8-1.0 Hz across sources) -- it was ALWAYS a
  // formality (R32's fixed sub_b_stale_after_sec_ timeout is what actually
  // governs staleness here, not this value), so it is now visibly one.
  const double sub_b_formal_expected_hz = 2.0 / sub_b_stale_after_sec_;

  diag_sources_.reserve(diag_names.size());
  for (size_t i = 0; i < diag_names.size(); ++i) {
    require(!diag_names[i].empty(), "diag_source_names entry must not be empty");
    require(!diag_topics[i].empty(), "diag_source_topics entry must not be empty: " + diag_names[i]);
    require(!diag_types[i].empty(), "diag_source_types entry must not be empty: " + diag_names[i]);
    require(
      diag_qos[i] == "be" || diag_qos[i] == "reliable" || diag_qos[i] == "reliable_tl",
      "diag_source_qos must be be|reliable|reliable_tl: " + diag_names[i]);
    require(
      std::isfinite(static_cast<double>(diag_depths[i])) && diag_depths[i] > 0,
      "diag_source_depths entry must be finite and > 0: " + diag_names[i]);
    requireValidOwner("diag_source", diag_owner[i], diag_names[i]);

    diag_source_board_.addItem(
      diag_names[i], sub_b_formal_expected_hz, sub_b_stale_after_sec_, Level::kError);

    DiagSource source;
    source.name = diag_names[i];
    source.topic = diag_topics[i];
    source.type = diag_types[i];
    source.qos_profile = diag_qos[i];
    source.depth = diag_depths[i];
    source.owner = diag_owner[i];
    source.action_aware = diag_action_aware[i];
    source.summary_child = diag_summary_child[i];
    diag_sources_.push_back(source);
  }

  // P10.9b: config/preflight_waivers.yaml's 4 parallel arrays. Parsed AFTER
  // diag_sources_ so "cấm trỏ waiver vào summary" can look the real
  // summary_child up per source instead of duplicating that mapping.
  const auto waiver_source = declareStrings("waiver_source");
  const auto waiver_child = declareStrings("waiver_child");
  const auto waiver_when = declareStrings("waiver_when");
  const auto waiver_reason = declareStrings("waiver_reason");
  require(
    waiver_child.size() == waiver_source.size() && waiver_when.size() == waiver_source.size() &&
    waiver_reason.size() == waiver_source.size(),
    "waiver_source/waiver_child/waiver_when/waiver_reason must be the same length");

  waivers_.reserve(waiver_source.size());
  for (size_t i = 0; i < waiver_source.size(); ++i) {
    const std::string row_label = waiver_source[i] + ":" + waiver_child[i];
    require(!waiver_reason[i].empty(), "waiver_reason must not be empty: " + row_label);
    const auto source_it = std::find_if(
      diag_sources_.begin(), diag_sources_.end(),
      [&](const DiagSource & source) {return source.name == waiver_source[i];});
    require(
      source_it != diag_sources_.end(),
      "waiver_source must name a configured diag_source_names entry: " + waiver_source[i]);
    require(
      waiver_when[i] == "preflight" || waiver_when[i] == "perception_off",
      "waiver_when must be preflight|perception_off: " + row_label);
    require(
      source_it->summary_child.empty() || waiver_child[i] != source_it->summary_child,
      "waiver_child must not target " + waiver_source[i] +
      "'s summary/rollup child (G8): " + waiver_child[i]);

    WaiverRow row;
    row.source = waiver_source[i];
    row.child = waiver_child[i];
    row.when = waiver_when[i];
    row.reason = waiver_reason[i];
    waivers_.push_back(row);
  }
  waiver_row_seen_.assign(waivers_.size(), false);
}

bool DiagnosticsNode::ownerIsCounted(const std::string & owner) const
{
  if (owner == "perception") {
    return perception_enabled_;
  }
  if (owner == "blackbox") {
    // O1: the evidence layer's own health (rosbag_manager_node,
    // event_logger_node) must NEVER gate a flight -- a full disk or a
    // disabled recorder is not a reason to refuse takeoff, regardless of
    // blackbox_enabled_ (unlike "perception", there is no configuration in
    // which this owner is allowed to count).
    return false;
  }
  return true;   // unowned -- unconditionally relevant, unchanged behaviour
}

Phase DiagnosticsNode::effectivePhase(
  const ItemVerdict & vehicle_state_verdict, PhaseSource & phase_source_out)
{
  // Review lượt 2 (N1): px4_state_adapter_node.cpp only assigns `armed`
  // INSIDE `if (connected)` -- losing the PX4 link still publishes at full
  // cadence (verdict.level stays kOk) with armed defaulted back to false.
  // Without the connected_ term this read as a genuine disarm and (after
  // dwell) tụt to PREFLIGHT while gate_mode_source claimed "measured" --
  // the worst possible label for a sample that cannot actually be trusted.
  const bool fresh = vehicle_state_verdict.level == Level::kOk && connected_;
  Phase raw;
  if (!fresh) {
    // "!tươi ⇒ FLIGHT strict" -- never measurable enough to trust a
    // downgrade to preflight (which would open the preflight waiver rows).
    raw = Phase::kFlight;
    phase_source_out = PhaseSource::kUnmeasuredStrict;
    preflight_dwell_ticks_seen_ = 0;
  } else {
    phase_source_out = PhaseSource::kMeasured;
    if (armed_) {
      raw = Phase::kFlight;
      preflight_dwell_ticks_seen_ = 0;   // upgrade (->flight) is immediate, G4
    } else {
      // G4: downgrading (->preflight) requires kPreflightDwellTicks
      // CONSECUTIVE ticks of "fresh && !armed_" -- a single glitchy
      // armed_==false tick must never re-open preflight waiver eligibility
      // mid-flight.
      if (preflight_dwell_ticks_seen_ < kPreflightDwellTicks) {
        ++preflight_dwell_ticks_seen_;
      }
      raw = preflight_dwell_ticks_seen_ >= kPreflightDwellTicks ?
        Phase::kPreflight : Phase::kFlight;
    }
  }
  // gate_mode_override_ may only TIGHTEN (force kFlight), never loosen a
  // measured/strict kFlight toward kPreflight -- contract S:2.20 "chỉ được
  // SIẾT, không bao giờ NỚI", implemented as max(measured, override).
  if (gate_mode_override_.load() == GateModeOverride::kFlight) {
    return Phase::kFlight;
  }
  return raw;
}

rcl_interfaces::msg::SetParametersResult DiagnosticsNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter & parameter : parameters) {
    if (parameter.get_name() != "gate_mode_override") {
      // V3 (review lượt 3): every OTHER declared parameter (perception_
      // enabled, the whole watch/diag_source/waiver config, etc.) is read
      // EXACTLY ONCE in declareAndValidateParams() and never looked at
      // again -- accepting a runtime set here would report "ros2 param
      // set" as successful while silently changing nothing. Refuse instead
      // of lying (same discipline as control_authority_manager_node.cpp's
      // rejectParameterChange()); do NOT make them live -- disabling
      // perception mid-flight this way would be worse than the lie.
      result.successful = false;
      result.reason = "read-only after startup";
      RCLCPP_ERROR(
        get_logger(), "refusing runtime change to '%s': read-only after startup",
        parameter.get_name().c_str());
      continue;
    }
    const std::string value = parameter.as_string();
    if (value != "auto" && value != "preflight" && value != "flight") {
      result.successful = false;
      result.reason = "gate_mode_override must be auto|preflight|flight";
      return result;
    }
    gate_mode_override_ = value == "flight" ? GateModeOverride::kFlight :
      value == "preflight" ? GateModeOverride::kPreflight : GateModeOverride::kAuto;
    RCLCPP_INFO(get_logger(), "gate_mode_override -> %s", value.c_str());
  }
  return result;
}

void DiagnosticsNode::subscribeWatchGroup(
  const std::vector<WatchItem> & items, const rclcpp::CallbackGroup::SharedPtr & group)
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = group;

  for (const WatchItem & item : items) {
    if (item.topic.empty()) {
      continue;   // fed by a Sub-B typed subscription instead (sentinel row)
    }
    // VÀNG#7: item.depth directly -- already required finite>0 by
    // parseWatchGroup(); the old default_subscription_depth_ fallback was
    // unreachable dead code (removed along with the param).
    try {
      auto subscription = create_generic_subscription(
        prefix_ + "/" + item.topic, item.type, qosFromProfile(item.qos_profile, item.depth),
        [this, name = item.name](std::shared_ptr<rclcpp::SerializedMessage>) {
          cadence_board_.onArrival(name, nowSeconds());
        }, sub_options);
      generic_subscriptions_.push_back(subscription);
    } catch (const std::exception & error) {
      ++watch_failed_;
      RCLCPP_WARN(
        get_logger(), "watch item %s (%s, %s) unavailable, permanently UNKNOWN: %s",
        item.name.c_str(), item.topic.c_str(), item.type.c_str(), error.what());
    }
  }
}

void DiagnosticsNode::subscribeDiagSources(const rclcpp::CallbackGroup::SharedPtr & group)
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = group;

  for (size_t i = 0; i < diag_sources_.size(); ++i) {
    const DiagSource & source = diag_sources_[i];
    // VÀNG#7: source.depth directly -- already required finite>0 above.
    try {
      auto subscription = create_subscription<DiagnosticArray>(
        prefix_ + "/" + source.topic, qosFromProfile(source.qos_profile, source.depth),
        [this, i](const DiagnosticArray::SharedPtr message) {onDiagSourceMessage(i, message);},
        sub_options);
      diag_source_subscriptions_.push_back(subscription);
    } catch (const std::exception & error) {
      ++watch_failed_;
      RCLCPP_WARN(
        get_logger(), "diag source %s (%s) unavailable, permanently UNKNOWN: %s",
        source.name.c_str(), source.topic.c_str(), error.what());
    }
  }
}

void DiagnosticsNode::onDiagSourceMessage(
  size_t index, const DiagnosticArray::SharedPtr & message)
{
  const double now_sec = nowSeconds();
  const std::string & name = diag_sources_[index].name;
  diag_source_board_.onArrival(name, now_sec);

  DiagSourceContent content;
  content.received = true;
  if (message->status.size() > kMaxChildrenPerSource) {
    // P10.9b: fail-closed -- do not even try to classify this many children
    // individually, onEvalTick() folds the whole source into one synthetic
    // ERROR "unclassified" row.
    content.over_cap = true;
  } else {
    content.children.reserve(message->status.size());
    for (const DiagnosticStatus & status : message->status) {
      ChildContent child;
      child.name = status.name;
      child.level = status.level;
      child.action = findActionKeyValue(status);
      content.children.push_back(std::move(child));
    }
  }
  diag_source_content_[index] = std::move(content);

  // The 4 diagnostics/* sources double as Sub-A cadence items (localization/
  // control_authority/safety in always, perception in flight_only) -- their
  // arrival here also counts as this cadence item's freshness.
  if (cadence_item_names_.count(name) > 0) {
    cadence_board_.onArrival(name, now_sec);
  }
}

void DiagnosticsNode::onVehicleState(const VehicleState::SharedPtr & message)
{
  // Review lượt 2 (N1): connected_ gates freshness in effectivePhase() --
  // a lost PX4 link must never be read as a measured downgrade to
  // preflight (armed defaults false while !connected, px4_state_adapter_
  // node.cpp:121's own documented behaviour).
  connected_ = message->connected;
  armed_ = message->armed;
  // Review lượt 2 (N2): "state/vehicle" is now a SENTINEL always-item (see
  // declareAndValidateParams()) -- this is the ONLY subscriber on that
  // topic, so it feeds cadence_board_ itself, same convention as the 3
  // diagnostics/* sentinel entries in onDiagSourceMessage(). Previously
  // this topic ALSO had a separate GenericSubscription cadence entry --
  // two independent DDS readers whose arrivals could land out of step with
  // each other (worst case: a diagnostics_node restart mid-flight, one
  // reader matching the still-running publisher before the other). One
  // reader now drives both armed_/connected_ AND this freshness signal, so
  // that divergence is impossible by construction.
  cadence_board_.onArrival(kVehicleStateItemName, nowSeconds());
}

double DiagnosticsNode::nowSeconds() const
{
  return now().seconds();
}

double DiagnosticsNode::nowWallSeconds()
{
  // C6: deliberately system_clock, NEVER this node's own (possibly
  // use_sim_time) clock -- same pattern as event_logger_node.cpp's own
  // nowWallSeconds(), the whole point being a reading that keeps moving
  // even if /clock itself is frozen.
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

void DiagnosticsNode::onEvalTick()
{
  const double now_sec = nowSeconds();
  const auto subA = cadence_board_.evaluate(now_sec);
  const auto subB = diag_source_board_.evaluate(now_sec);

  PhaseSource phase_source;
  const Phase effective_phase = effectivePhase(subA[vehicle_state_item_index_], phase_source);

  DiagnosticArray aggregated;
  aggregated.header.stamp = now();

  unsigned unknown_count = 0;
  unsigned warn_count = 0;
  unsigned error_count = 0;
  unsigned waived_count = 0;
  int worst_rank = -1;
  std::string worst_item;
  std::vector<std::pair<int, std::string>> blocking_candidates;

  const auto considerForWorst = [&](const std::string & name, uint8_t diagnostic_level) {
      const int rank = severityRank(diagnostic_level);
      if (rank > worst_rank) {
        worst_rank = rank;
        worst_item = name;
      }
      if (diagnostic_level != DiagnosticStatus::OK) {
        blocking_candidates.emplace_back(rank, name);
      }
    };

  // V6 (review lượt 3): phase_source was WIRE-ONLY -- gate_mode_source=
  // unmeasured_strict never fed unknown_count/error_count/warn_count below,
  // so contract S:2.20's "unmeasured_strict never coincides with GO" held
  // only by ACCIDENT, whenever state/vehicle's OWN Sub-A liveness also
  // happened to go stale at the same time. N1's fix (connected_ gating
  // freshness) opened a branch where that accident no longer holds: the
  // topic keeps arriving AT CADENCE (Sub-A liveness reads kOk, verdict
  // above) while only `connected` says the PX4 link is gone -- that case
  // contributed ZERO to every counter below. Force it in directly,
  // structurally, instead of leaning on a coincidence.
  if (phase_source == PhaseSource::kUnmeasuredStrict) {
    const std::string display_name =
      std::string(kVehicleStateItemName) + ":phase_unmeasurable";
    DiagnosticStatus status;
    status.name = display_name;
    status.hardware_id = uav_id_;
    status.level = DiagnosticStatus::STALE;
    status.message = "go/no-go phase cannot be measured (state/vehicle stale or PX4 link lost)";
    status.values.push_back(keyValueStr("counted", "true"));
    aggregated.status.push_back(status);
    ++unknown_count;
    considerForWorst(display_name, DiagnosticStatus::STALE);
  }

  // ---- Sub A: always_items_ then flight_only_items_ (addItem() order). ----
  // LIVENESS -- always gates, no waiver table involved at all (P10.9b core
  // rule: "LIVENESS luôn gate"). flight_only_counted is the ONLY thing that
  // changed here vs pre-P10.9b: it now follows the MEASURED effective_phase
  // instead of a hand-set gate_mode_ param.
  const auto reportSubA = [&](
    const std::vector<WatchItem> & items, size_t verdict_offset, bool phase_counted) {
      for (size_t i = 0; i < items.size(); ++i) {
        const ItemVerdict & verdict = subA[verdict_offset + i];
        const uint8_t diagnostic_level = boardLevelToDiagnosticLevel(verdict.level);
        // C2: phase (preflight/flight) AND owner (perception/blackbox
        // possibly disabled) are TWO INDEPENDENT reasons an item can be
        // watched-but-not-counted -- both must hold for it to count.
        const bool owner_counted = ownerIsCounted(items[i].owner);
        const bool counted = phase_counted && owner_counted;

        DiagnosticStatus status;
        status.name = items[i].name;
        status.hardware_id = uav_id_;
        status.level = diagnostic_level;
        if (counted) {
          status.message = "counted";
        } else if (!phase_counted) {
          status.message = "watched (not counted, preflight)";
        } else {
          status.message = "watched (not counted, " + items[i].owner + " disabled)";
        }
        status.values.push_back(keyValue("age_sec", verdict.age_sec));
        status.values.push_back(keyValue("measured_hz", verdict.measured_hz));
        status.values.push_back(keyValueStr("counted", counted ? "true" : "false"));
        status.values.push_back(keyValueStr("owner", items[i].owner));
        aggregated.status.push_back(status);

        if (!counted) {
          continue;
        }
        if (verdict.level == Level::kUnknown) {
          ++unknown_count;
        } else if (verdict.level == Level::kWarn) {
          ++warn_count;
        } else if (verdict.level == Level::kError) {
          ++error_count;
        }
        considerForWorst(items[i].name, diagnostic_level);
      }
    };
  reportSubA(always_items_, 0, true);
  reportSubA(flight_only_items_, always_items_.size(), effective_phase == Phase::kFlight);

  // ---- Sub B: per-child (contract S:2.20 "chấm theo TỪNG CHILD"). ----
  // CONTENT -- gated by owner, then (in order) summary-exclusion (G8),
  // never-waive (NW1/NW2), and the waiver table. See diagnostics_node.hpp's
  // class-level comment for the authoritative 7-rule order.
  for (size_t i = 0; i < diag_sources_.size(); ++i) {
    const DiagSource & source = diag_sources_[i];
    const ItemVerdict & liveness = subB[i];
    const bool owner_counted = ownerIsCounted(source.owner);
    const bool self_stale = liveness.level != Level::kOk;

    if (self_stale) {
      // Rule 0 (R32): never arrived, or > sub_b_stale_after_sec_ old --
      // "cannot measure" this source's content at all, not the same as it
      // reporting OK. No children to iterate this tick.
      DiagnosticStatus status;
      status.name = source.name;
      status.hardware_id = uav_id_;
      status.level = DiagnosticStatus::STALE;
      status.message = "self-stale (no message within sub_b_stale_after_sec_)";
      status.values.push_back(keyValueStr("self_stale", "true"));
      status.values.push_back(keyValue("age_sec", liveness.age_sec));
      status.values.push_back(keyValueStr("counted", owner_counted ? "true" : "false"));
      status.values.push_back(keyValueStr("owner", source.owner));
      aggregated.status.push_back(status);
      if (owner_counted) {
        ++unknown_count;
        considerForWorst(source.name, DiagnosticStatus::STALE);
      }
      continue;
    }

    const DiagSourceContent & content = diag_source_content_[i];
    if (content.over_cap) {
      // > kMaxChildrenPerSource: fail-closed, ONE synthetic ERROR row, never
      // waivable (the waiver table cannot possibly have been checked
      // against real per-child data here -- counting is the safe default).
      const std::string display_name = source.name + ":unclassified";
      DiagnosticStatus status;
      status.name = display_name;
      status.hardware_id = uav_id_;
      status.level = DiagnosticStatus::ERROR;
      status.message = "fail-closed: source exceeded " +
        std::to_string(kMaxChildrenPerSource) + " children, cannot classify individually";
      status.values.push_back(keyValueStr("counted", owner_counted ? "true" : "false"));
      status.values.push_back(keyValueStr("owner", source.owner));
      aggregated.status.push_back(status);
      if (owner_counted) {
        ++error_count;
        considerForWorst(display_name, DiagnosticStatus::ERROR);
      }
      continue;
    }

    for (const ChildContent & child : content.children) {
      const std::string display_name = source.name + ":" + child.name;
      const bool is_summary =
        !source.summary_child.empty() && child.name == source.summary_child;
      const bool never_waivable =
        isNeverWaivable(source.action_aware, child.level, child.action);

      bool waived = false;
      std::string waived_reason;
      for (size_t w = 0; w < waivers_.size(); ++w) {
        const WaiverRow & row = waivers_[w];
        if (row.source != source.name || row.child != child.name) {
          continue;
        }
        // Bookkeeping (waiver_unmatched, F2) runs REGARDLESS of whether
        // this row currently grants an exemption -- drift detection cares
        // that the (source,child) pair exists on the wire at all.
        waiver_row_seen_[w] = true;
        if (waived || !owner_counted || is_summary || never_waivable) {
          continue;   // already granted, or this child is not eligible at all
        }
        const bool when_matches =
          (row.when == "preflight" && effective_phase == Phase::kPreflight) ||
          (row.when == "perception_off" && !perception_enabled_);
        if (when_matches) {
          waived = true;
          waived_reason = row.reason;
        }
      }
      if (waived) {
        ++waived_count;
      }

      bool counted;
      std::string message;
      if (!owner_counted) {
        counted = false;
        message = "watched (not counted, " + source.owner + " disabled)";
      } else if (is_summary) {
        counted = false;
        message = "summary (rollup of already-counted children, never counted -- G8)";
      } else if (never_waivable) {
        counted = true;
        message = "never-waivable (action-class veto, cannot be waived)";
      } else if (waived) {
        counted = false;
        message = "waived: " + waived_reason;
      } else {
        counted = true;
        message = "counted";
      }

      DiagnosticStatus status;
      status.name = display_name;
      status.hardware_id = uav_id_;
      status.level = child.level;
      status.message = message;
      status.values.push_back(keyValueStr("source", source.name));
      status.values.push_back(keyValueStr("child", child.name));
      status.values.push_back(keyValueStr("action", child.action));
      status.values.push_back(keyValueStr("counted", counted ? "true" : "false"));
      status.values.push_back(keyValueStr("owner", source.owner));
      status.values.push_back(keyValueStr("never_waivable", never_waivable ? "true" : "false"));
      status.values.push_back(keyValueStr("waived", waived ? "true" : "false"));
      status.values.push_back(keyValueStr("waived_reason", waived_reason));
      aggregated.status.push_back(status);

      if (!counted) {
        continue;
      }
      switch (child.level) {
        case DiagnosticStatus::WARN:
          ++warn_count;
          break;
        case DiagnosticStatus::ERROR:
          ++error_count;
          break;
        case DiagnosticStatus::OK:
          break;
        default:
          // A child publishing STALE (or anything else) at this granularity
          // is not produced by any current publisher's own encoding
          // (self-staleness is handled at the whole-source level, rule 0) --
          // fold conservatively into unknown_count rather than dropping it
          // silently (R30).
          ++unknown_count;
          break;
      }
      considerForWorst(display_name, child.level);
    }
  }

  GoNoGo go_no_go;
  if (unknown_count > 0) {
    go_no_go = GoNoGo::kUnknown;
  } else if (error_count > 0 || warn_count > 0) {
    go_no_go = GoNoGo::kNoGo;
  } else {
    go_no_go = GoNoGo::kGo;
  }
  const unsigned stale_count = warn_count + error_count;

  unsigned waiver_unmatched = 0;
  for (bool seen : waiver_row_seen_) {
    if (!seen) {
      ++waiver_unmatched;
    }
  }

  // "nặng-trước" (worst severity first), stable so ties keep encounter
  // order -- same tie-break spirit as worst_item's strict `>` comparison.
  std::stable_sort(
    blocking_candidates.begin(), blocking_candidates.end(),
    [](const std::pair<int, std::string> & a, const std::pair<int, std::string> & b) {
      return a.first > b.first;
    });
  std::string blocking;
  const size_t shown = std::min<size_t>(blocking_candidates.size(), 5);
  for (size_t i = 0; i < shown; ++i) {
    if (i > 0) {
      blocking += ", ";
    }
    blocking += blocking_candidates[i].second;
  }
  if (blocking_candidates.size() > shown) {
    blocking += " (+" + std::to_string(blocking_candidates.size() - shown) + " more)";
  }

  const unsigned clock_regressions =
    cadence_board_.clockRegressions() + diag_source_board_.clockRegressions();

  DiagnosticStatus system_status;
  system_status.name = "system";
  system_status.hardware_id = uav_id_;
  system_status.level = goNoGoToDiagnosticLevel(go_no_go);
  system_status.message = std::string("go_no_go=") + goNoGoName(go_no_go);
  system_status.values.push_back(keyValueStr("go_no_go", goNoGoName(go_no_go)));
  system_status.values.push_back(keyValue("unknown_count", static_cast<double>(unknown_count)));
  system_status.values.push_back(keyValue("error_count", static_cast<double>(error_count)));
  system_status.values.push_back(keyValue("warn_count", static_cast<double>(warn_count)));
  system_status.values.push_back(keyValue("stale_count", static_cast<double>(stale_count)));
  system_status.values.push_back(keyValueStr("worst_item", worst_item));
  system_status.values.push_back(
    keyValue("clock_regressions", static_cast<double>(clock_regressions)));
  system_status.values.push_back(keyValueStr("gate_mode", phaseName(effective_phase)));
  system_status.values.push_back(keyValueStr("gate_mode_source", phaseSourceName(phase_source)));
  system_status.values.push_back(keyValue("waived_count", static_cast<double>(waived_count)));
  system_status.values.push_back(
    keyValue("waiver_unmatched", static_cast<double>(waiver_unmatched)));
  system_status.values.push_back(keyValueStr("blocking", blocking));
  system_status.values.push_back(keyValue("watch_failed", static_cast<double>(watch_failed_)));
  // C6: DiagnosticStatus has no header/stamp of its own -- /state/system_
  // health is TransientLocal/KeepLast(1), so a frozen /clock (Gazebo
  // paused/dead) used to freeze onEvalTick()/onPublishTick() TOGETHER,
  // leaving the last-known-GO sample latched forever with NOTHING in the
  // message itself for a consumer to measure its own age against (R32).
  // stamp_sec is this node's own (possibly sim) clock; wall_stamp_sec is
  // real wall-clock UTC epoch seconds -- carried SEPARATELY so a frozen
  // /clock is itself detectable (stamp_sec stops advancing while
  // wall_stamp_sec keeps moving on every REAL evaluation).
  system_status.values.push_back(keyValue("stamp_sec", now_sec));
  system_status.values.push_back(keyValue("wall_stamp_sec", nowWallSeconds()));

  aggregated.status.push_back(system_status);

  cached_aggregated_ = aggregated;
  cached_system_health_ = system_status;
  cached_go_no_go_ = go_no_go;

  if (!has_evaluated_once_ || cached_go_no_go_ != last_published_go_no_go_) {
    publishSystemHealthNow();
  }
  has_evaluated_once_ = true;
}

void DiagnosticsNode::onPublishTick()
{
  publishAggregatedNow();
  publishSystemHealthNow();
}

void DiagnosticsNode::publishAggregatedNow()
{
  cached_aggregated_.header.stamp = now();
  aggregated_publisher_->publish(cached_aggregated_);
}

void DiagnosticsNode::publishSystemHealthNow()
{
  cached_system_health_.hardware_id = uav_id_;
  system_health_publisher_->publish(cached_system_health_);
  last_published_go_no_go_ = cached_go_no_go_;
}

}  // namespace uav_observability
