#include "uav_mission/mission_registry.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

namespace uav_mission
{

namespace
{

void require(bool condition, const std::string & detail)
{
  if (!condition) {
    throw std::invalid_argument("mission registry parameters: " + detail);
  }
}

// Whitelist of BT node tag names the shipped (and any future) mission XML
// may use (plan P9 S:1: "guard in CODE, body in XML"). MissionGuard is
// deliberately absent: it is a C++ decorator wrapped AROUND the SubTree
// loaded from this file by mission_executor_node (P9.4), never a tag a
// mission body could name, omit, or edit away.
const std::set<std::string> & allowedXmlTags()
{
  static const std::set<std::string> kAllowed = {
    "root", "BehaviorTree",
    "Sequence", "Fallback", "ReactiveFallback",
    "RetryUntilSuccessful", "Repeat", "Timeout", "Inverter",
    "NavAction", "WaypointCursor", "Dwell", "MarkerSeen", "TargetSeen",
    "SearchBudgetAvailable",
  };
  return kAllowed;
}

void walkAndCheckTags(const tinyxml2::XMLElement * elem, const std::string & xml_path)
{
  if (elem == nullptr) {
    return;
  }
  const std::string tag = elem->Name() != nullptr ? elem->Name() : "";
  if (allowedXmlTags().find(tag) == allowedXmlTags().end()) {
    throw std::invalid_argument(
      "mission XML " + xml_path + " uses non-whitelisted node <" + tag + ">");
  }
  for (const tinyxml2::XMLElement * child = elem->FirstChildElement(); child != nullptr;
    child = child->NextSiblingElement())
  {
    walkAndCheckTags(child, xml_path);
  }
}

void requireKnownKeys(
  const YAML::Node & node, const std::set<std::string> & allowed, const std::string & context)
{
  if (!node.IsMap()) {
    throw std::invalid_argument(context + " must be a YAML mapping");
  }
  for (const auto & kv : node) {
    const std::string key = kv.first.as<std::string>();
    if (allowed.find(key) == allowed.end()) {
      throw std::invalid_argument(context + ": unknown key '" + key + "'");
    }
  }
}

double requireFiniteDouble(const YAML::Node & node, const std::string & context)
{
  double value = 0.0;
  try {
    value = node.as<double>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument(context + ": not a number");
  }
  if (!std::isfinite(value)) {
    throw std::invalid_argument(context + ": must be finite (got NaN/inf)");
  }
  return value;
}

Waypoint parseWaypoint(const YAML::Node & node, const std::string & context)
{
  static const std::set<std::string> kAllowed = {"x", "y", "z"};
  requireKnownKeys(node, kAllowed, context);
  if (!node["x"] || !node["y"] || !node["z"]) {
    throw std::invalid_argument(context + ": requires x, y, z");
  }
  Waypoint w;
  w.x = requireFiniteDouble(node["x"], context + ".x");
  w.y = requireFiniteDouble(node["y"], context + ".y");
  w.z = requireFiniteDouble(node["z"], context + ".z");
  return w;
}

double legDistance(const Waypoint & a, const Waypoint & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

IndoorPatrolParams defaultIndoorPatrol()
{
  IndoorPatrolParams p;
  p.waypoints = {
    Waypoint{1.0, 0.0, 1.5}, Waypoint{1.0, 1.0, 1.5},
    Waypoint{0.0, 1.0, 1.5}, Waypoint{0.0, 0.0, 1.5},
  };
  p.loops = 4;
  p.dwell_sec = 2.0;
  return p;
}

FollowTargetParams defaultFollowTarget()
{
  FollowTargetParams p;
  p.target_speed_mps = 0.2;
  p.standoff_m = 2.0;
  p.timeout_sec = 60.0;
  return p;
}

InspectPointParams defaultInspectPoint()
{
  InspectPointParams p;
  p.approach = Waypoint{2.0, 0.0, 2.0};
  p.marker_id = -1;
  p.dwell_sec = 3.0;
  return p;
}

IndoorPatrolParams parseIndoorPatrol(const YAML::Node & root, const MissionRegistryParams & params)
{
  static const std::set<std::string> kAllowed = {"loops", "waypoints", "dwell_sec"};
  IndoorPatrolParams p = defaultIndoorPatrol();

  if (root && !root.IsNull()) {
    requireKnownKeys(root, kAllowed, "indoor_patrol params");
    if (root["loops"]) {
      p.loops = static_cast<int32_t>(requireFiniteDouble(root["loops"], "indoor_patrol.loops"));
    }
    if (root["dwell_sec"]) {
      p.dwell_sec = requireFiniteDouble(root["dwell_sec"], "indoor_patrol.dwell_sec");
    }
    if (root["waypoints"]) {
      const YAML::Node wps = root["waypoints"];
      if (!wps.IsSequence() || wps.size() == 0) {
        throw std::invalid_argument("indoor_patrol.waypoints must be a non-empty sequence");
      }
      p.waypoints.clear();
      for (std::size_t i = 0; i < wps.size(); ++i) {
        p.waypoints.push_back(
          parseWaypoint(wps[i], "indoor_patrol.waypoints[" + std::to_string(i) + "]"));
      }
    }
  }

  require(p.loops >= 1 && p.loops <= params.max_loops, "indoor_patrol.loops must be in [1, max_loops]");
  require(
    std::isfinite(p.dwell_sec) && p.dwell_sec > 0.0,
    "indoor_patrol.dwell_sec must be finite and positive");
  require(!p.waypoints.empty(), "indoor_patrol.waypoints must not be empty");
  for (const Waypoint & w : p.waypoints) {
    require(
      w.z >= params.min_alt_copy_m && w.z <= params.max_alt_copy_m,
      "indoor_patrol waypoint z outside [min_alt_copy_m, max_alt_copy_m]");
  }
  const std::size_t n = p.waypoints.size();
  for (std::size_t i = 0; i < n; ++i) {
    const double leg = legDistance(p.waypoints[i], p.waypoints[(i + 1) % n]);
    require(leg <= params.goto_reach_copy_m, "indoor_patrol leg distance exceeds goto_reach_copy_m");
  }
  return p;
}

FollowTargetParams parseFollowTarget(const YAML::Node & root, const MissionRegistryParams & params)
{
  static const std::set<std::string> kAllowed = {"target_speed_mps", "standoff_m", "timeout_sec"};
  FollowTargetParams p = defaultFollowTarget();

  if (root && !root.IsNull()) {
    requireKnownKeys(root, kAllowed, "follow_target params");
    if (root["target_speed_mps"]) {
      p.target_speed_mps = requireFiniteDouble(root["target_speed_mps"], "follow_target.target_speed_mps");
    }
    if (root["standoff_m"]) {
      p.standoff_m = requireFiniteDouble(root["standoff_m"], "follow_target.standoff_m");
    }
    if (root["timeout_sec"]) {
      p.timeout_sec = requireFiniteDouble(root["timeout_sec"], "follow_target.timeout_sec");
    }
  }

  require(
    std::isfinite(p.target_speed_mps) && p.target_speed_mps > 0.0 &&
      p.target_speed_mps <= params.target_speed_max_mps,
    "follow_target.target_speed_mps must be in (0, target_speed_max_mps]");
  require(
    std::isfinite(p.standoff_m) && p.standoff_m > 0.0, "follow_target.standoff_m must be finite and positive");
  require(
    std::isfinite(p.timeout_sec) && p.timeout_sec > 0.0,
    "follow_target.timeout_sec must be finite and positive");
  // 🟡-3 (2026-08-23): a caller-supplied timeout_sec above step_timeout_
  // copy_sec would let Y1's own step-timeout cancel a healthy mid-track
  // TrackTarget leg (reported ABORTED_TIMEOUT) well before this outer
  // Timeout ever gets a say -- reject the goal instead of a confusing
  // wrong-cause abort at runtime.
  require(
    p.timeout_sec <= params.step_timeout_copy_sec,
    "follow_target.timeout_sec must be <= step_timeout_copy_sec (else Y1's own step_timeout_sec "
    "would cut off a healthy mid-track leg first)");
  return p;
}

InspectPointParams parseInspectPoint(const YAML::Node & root, const MissionRegistryParams & params)
{
  static const std::set<std::string> kAllowed = {"approach", "marker_id", "dwell_sec"};
  InspectPointParams p = defaultInspectPoint();

  if (root && !root.IsNull()) {
    requireKnownKeys(root, kAllowed, "inspect_point params");
    if (root["approach"]) {
      p.approach = parseWaypoint(root["approach"], "inspect_point.approach");
    }
    if (root["marker_id"]) {
      p.marker_id = static_cast<int32_t>(requireFiniteDouble(root["marker_id"], "inspect_point.marker_id"));
    }
    if (root["dwell_sec"]) {
      p.dwell_sec = requireFiniteDouble(root["dwell_sec"], "inspect_point.dwell_sec");
    }
  }

  require(
    p.approach.z >= params.min_alt_copy_m && p.approach.z <= params.max_alt_copy_m,
    "inspect_point.approach.z outside [min_alt_copy_m, max_alt_copy_m]");
  require(p.marker_id >= -1, "inspect_point.marker_id must be -1 (any) or >= 0");
  require(
    std::isfinite(p.dwell_sec) && p.dwell_sec > 0.0, "inspect_point.dwell_sec must be finite and positive");
  return p;
}

}  // namespace

MissionRegistry::MissionRegistry(const MissionRegistryParams & params, std::string missions_dir)
: params_(params), missions_dir_(std::move(missions_dir))
{
  validate();
  // Fail fast, same discipline as FailsafePolicy/MissionPolicy: a broken
  // shipped mission file must refuse construction, not surface as a runtime
  // surprise the first time someone loads it.
  for (const MissionDescriptor & descriptor : list()) {
    validateXmlWhitelist(descriptor.xml_path);
  }
}

void MissionRegistry::validate() const
{
  auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};

  require(positive(params_.min_alt_copy_m), "min_alt_copy_m must be finite and positive");
  require(positive(params_.max_alt_copy_m), "max_alt_copy_m must be finite and positive");
  require(params_.max_alt_copy_m > params_.min_alt_copy_m, "max_alt_copy_m must exceed min_alt_copy_m");
  require(positive(params_.goto_reach_copy_m), "goto_reach_copy_m must be finite and positive");
  require(
    positive(params_.navigator_max_speed_copy_mps),
    "navigator_max_speed_copy_mps must be finite and positive");
  require(
    positive(params_.navigator_goto_timeout_copy_sec),
    "navigator_goto_timeout_copy_sec must be finite and positive");
  require(positive(params_.target_speed_max_mps), "target_speed_max_mps must be finite and positive");
  require(params_.max_loops >= 1, "max_loops must be >= 1");

  require(
    params_.goto_reach_copy_m <=
      params_.navigator_max_speed_copy_mps * params_.navigator_goto_timeout_copy_sec,
    "goto_reach_copy_m must be <= navigator_max_speed_copy_mps x navigator_goto_timeout_copy_sec "
    "(a leg the navigator's own timeout could never legitimately complete)");
  require(
    params_.target_speed_max_mps <= 0.5 * params_.navigator_max_speed_copy_mps,
    "target_speed_max_mps must be <= 0.5 x navigator_max_speed_copy_mps (plan P9 S:3)");
}

void MissionRegistry::validateXmlWhitelist(const std::string & xml_path) const
{
  tinyxml2::XMLDocument doc;
  const tinyxml2::XMLError err = doc.LoadFile(xml_path.c_str());
  if (err != tinyxml2::XML_SUCCESS) {
    throw std::invalid_argument(
      "mission XML failed to load: " + xml_path + " (" +
      (doc.ErrorStr() != nullptr ? doc.ErrorStr() : "unknown error") + ")");
  }
  const tinyxml2::XMLElement * root = doc.RootElement();
  if (root == nullptr) {
    throw std::invalid_argument("mission XML has no root element: " + xml_path);
  }
  walkAndCheckTags(root, xml_path);
}

MissionPlan MissionRegistry::load(const std::string & mission_id, const std::string & params_str) const
{
  MissionPlan plan;
  plan.mission_id = mission_id;
  plan.xml_path = missions_dir_ + "/" + mission_id + ".xml";

  YAML::Node root;
  if (!params_str.empty()) {
    try {
      root = YAML::Load(params_str);
    } catch (const YAML::Exception & e) {
      throw std::invalid_argument(std::string("mission_params YAML parse error: ") + e.what());
    }
  }

  if (mission_id == "indoor_patrol") {
    plan.kind = MissionKind::kIndoorPatrol;
    plan.indoor_patrol = parseIndoorPatrol(root, params_);
    // Repeat(loops) IS the total number of waypoint-visits (see header):
    // total_steps tracks it directly, not waypoints.size().
    plan.total_steps = static_cast<uint32_t>(plan.indoor_patrol.loops);
  } else if (mission_id == "follow_target") {
    plan.kind = MissionKind::kFollowTarget;
    plan.follow_target = parseFollowTarget(root, params_);
    plan.total_steps = 1;   // single continuous reactive behaviour, see header
  } else if (mission_id == "inspect_point") {
    plan.kind = MissionKind::kInspectPoint;
    plan.inspect_point = parseInspectPoint(root, params_);
    plan.total_steps = 4;   // approach, find_marker, goto_over_marker, dwell -- see header
  } else {
    throw std::invalid_argument("unknown mission_id: " + mission_id);
  }

  return plan;
}

std::vector<MissionDescriptor> MissionRegistry::list() const
{
  return {
    MissionDescriptor{
      "indoor_patrol", missions_dir_ + "/indoor_patrol.xml",
      "Loop through indoor waypoints, dwelling at each."},
    MissionDescriptor{
      "follow_target", missions_dir_ + "/follow_target.xml",
      "Reactively track a moving target; search a fixed pattern when lost."},
    MissionDescriptor{
      "inspect_point", missions_dir_ + "/inspect_point.xml",
      "Approach a point, locate an ArUco marker, hover over it, and record its pose."},
  };
}

MissionDescriptor MissionRegistry::describe(const std::string & mission_id) const
{
  for (const MissionDescriptor & descriptor : list()) {
    if (descriptor.mission_id == mission_id) {
      return descriptor;
    }
  }
  throw std::invalid_argument("unknown mission_id: " + mission_id);
}

}  // namespace uav_mission
