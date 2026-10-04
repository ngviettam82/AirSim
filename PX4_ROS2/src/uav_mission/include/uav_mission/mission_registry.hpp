#ifndef UAV_MISSION__MISSION_REGISTRY_HPP_
#define UAV_MISSION__MISSION_REGISTRY_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace uav_mission
{

// A single indoor_patrol waypoint. Plain struct, not geometry_msgs -- this
// library is ROS-free (same rule as mission_policy.hpp).
struct Waypoint
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// mission_params schema for "indoor_patrol". total_steps = loops: the XML
// body is Repeat(loops)[WaypointCursor -> GotoPose -> Dwell] (plan P9 S:3),
// a SINGLE Repeat decorator whose own count IS the total number of
// waypoint-visits the mission performs -- WaypointCursor cycles through
// `waypoints` internally, so `loops` need not equal waypoints.size().
struct IndoorPatrolParams
{
  std::vector<Waypoint> waypoints;
  int32_t loops = 4;
  double dwell_sec = 2.0;
};

// mission_params schema for "follow_target". The body (ReactiveFallback
// [TargetSeen->TrackTarget, Search] wrapped in Timeout) is a single
// continuous reactive behaviour with no discrete waypoint progression --
// total_steps is fixed at 1 (see MissionRegistry::load()).
struct FollowTargetParams
{
  double target_speed_mps = 0.2;
  double standoff_m = 2.0;
  double timeout_sec = 60.0;
};

// mission_params schema for "inspect_point". marker_id == -1 means "any
// marker accepted" (MarkerSeen's own port, not validated as a real ID here).
struct InspectPointParams
{
  Waypoint approach;
  int32_t marker_id = -1;
  double dwell_sec = 3.0;
};

// Which typed params member of MissionPlan is populated -- avoids a
// std::variant dependency for exactly 3 known, closed-set mission kinds.
enum class MissionKind : uint8_t
{
  kIndoorPatrol,
  kFollowTarget,
  kInspectPoint,
};

struct MissionPlan
{
  std::string mission_id;
  MissionKind kind = MissionKind::kIndoorPatrol;
  std::string xml_path;
  // total_steps counts the mission's named, sequential top-level steps --
  // NOT a raw BT leaf-node count. Defined per mission kind:
  //   indoor_patrol: loops (each Repeat iteration = one waypoint visit)
  //   follow_target: fixed 1 (single continuous tracking behaviour)
  //   inspect_point: fixed 4 (approach, find_marker, goto_over_marker, dwell
  //     -- "ghi pose"/record is a bookkeeping side effect of the dwell step
  //     completing, not a separate BT leaf; flagged for P9.4 to confirm when
  //     it wires MissionStatus.current_step_index for real)
  uint32_t total_steps = 0;

  IndoorPatrolParams indoor_patrol;
  FollowTargetParams follow_target;
  InspectPointParams inspect_point;
};

struct MissionDescriptor
{
  std::string mission_id;
  std::string xml_path;
  std::string description;
};

// Copies of thresholds this package cannot see directly (same convention as
// uav_safety's FailsafePolicyParams / this package's own MissionPolicyParams)
// plus the whitelist of BT node tag names the 3 shipped XML files (and any
// future one) are allowed to use -- MissionGuard is NOT in this list: it is
// a C++ decorator wrapped AROUND the SubTree loaded from XML, never a tag a
// mission body file could name or omit (plan P9 S:1, "guard in CODE, body
// in XML" -- a text file must never be able to remove the safety wrapper).
struct MissionRegistryParams
{
  double min_alt_copy_m = 0.5;          // COPY: navigator_action_server_node min_altitude_m
  // Mission's OWN more conservative indoor ceiling -- NOT a literal copy of
  // navigator's max_altitude_m (20.0): uav_arena_indoor is a smaller world,
  // this is the mission layer's own tighter envelope for it.
  double max_alt_copy_m = 8.0;
  // Mission's own conservative per-leg cap, engineering-derived (not a
  // literal navigator field): validate() requires it stay within
  // navigator_max_speed_copy_mps x navigator_goto_timeout_copy_sec.
  double goto_reach_copy_m = 40.0;
  double navigator_max_speed_copy_mps = 0.55;      // COPY: navigator_action_server_node max_speed
  double navigator_goto_timeout_copy_sec = 240.0;  // COPY: navigator_action_server_node goto_timeout_sec
  // Plan P9 S:3: "0.5 x max_speed (0.55)" -- the moving-target box speed cap.
  double target_speed_max_mps = 0.275;

  int32_t max_loops = 1000;   // administrative sanity ceiling, not a safety bound
  // 🟡-3 (2026-08-23): COPY of MissionPolicyParams::step_timeout_sec --
  // same shared-key precedent as navigator_goto_timeout_copy_sec (declared
  // once in mission_executor_node.cpp, read back here, NOT re-declared).
  // Caller-supplied follow_target.timeout_sec must never exceed it, else a
  // healthy mid-track TrackTarget leg gets cut off by Y1's own step_
  // timeout_sec cancel and reports a spurious ABORTED_TIMEOUT.
  double step_timeout_copy_sec = 300.0;
};

class MissionRegistry
{
public:
  /// Throws std::invalid_argument if params are unusable -- refuse to
  /// start, same rule as MissionPolicy/FailsafePolicy.
  /// missions_dir: directory containing the shipped XML files
  /// (config/missions in the source tree, share/uav_mission/missions once
  /// installed).
  MissionRegistry(const MissionRegistryParams & params, std::string missions_dir);

  /// Parses mission_params (YAML text; YAML is a superset of JSON, so a
  /// JSON caller works unchanged) exactly once, validates it against
  /// mission_id's known schema, and returns a fully-typed MissionPlan.
  /// Throws std::invalid_argument on: unknown mission_id, a YAML parse
  /// error, an unrecognized key, a non-finite number, z outside
  /// [min_alt_copy_m, max_alt_copy_m], a leg longer than goto_reach_copy_m,
  /// or target_speed_mps above target_speed_max_mps. An empty params_str
  /// loads this mission's shipped defaults.
  MissionPlan load(const std::string & mission_id, const std::string & params_str) const;

  /// The 3 shipped mission_ids and their XML paths/descriptions.
  std::vector<MissionDescriptor> list() const;

  /// Single-entry form of list(); throws std::invalid_argument if unknown.
  MissionDescriptor describe(const std::string & mission_id) const;

private:
  void validate() const;
  /// Loads config/missions/<mission_id>.xml via tinyxml2, checks every tag
  /// name against the whitelist. Throws std::invalid_argument on any
  /// non-whitelisted tag (a plain grep/text edit cannot remove the safety
  /// wrapper since MissionGuard is never a tag in the file to begin with,
  /// but THIS check is what would catch e.g. a typo'd or hand-added
  /// unknown leaf silently doing nothing at runtime).
  void validateXmlWhitelist(const std::string & xml_path) const;

  MissionRegistryParams params_;
  std::string missions_dir_;
};

}  // namespace uav_mission

#endif  // UAV_MISSION__MISSION_REGISTRY_HPP_
