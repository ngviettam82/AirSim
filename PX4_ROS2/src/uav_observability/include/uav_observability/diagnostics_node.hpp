#ifndef UAV_OBSERVABILITY__DIAGNOSTICS_NODE_HPP_
#define UAV_OBSERVABILITY__DIAGNOSTICS_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>

#include "uav_observability/staleness_board.hpp"

namespace uav_observability
{

/// P10.9b: the effective go/no-go PHASE -- MEASURED from /state/vehicle
/// (armed AND connected, review lượt 2's N1 -- a link-lost sample must never
/// read as a genuine disarm), not a hand-set param (plan panel S:4.a).
/// Preflight: flight_only items are watched but excluded from go/no-go, AND
/// Sub-B waiver rows with when=preflight may apply. Flight: flight_only
/// items count like every other item, and preflight waivers stop applying.
enum class Phase : uint8_t
{
  kPreflight,
  kFlight,
};

/// Whether Phase above was actually MEASURED off a fresh /state/vehicle
/// sample, or forced strict because that sample is stale/never-arrived.
/// Carried on the wire as "gate_mode_source" so a consumer never mistakes
/// a fail-closed guess for a real reading (R30).
enum class PhaseSource : uint8_t
{
  kMeasured,
  kUnmeasuredStrict,
};

/// Runtime-settable knob (param "gate_mode_override"), P10.9b: the OLD
/// "gate_mode" param used to WRITE the phase directly -- a blind spot in
/// flight (nothing re-derives it off real vehicle state). This override can
/// only TIGHTEN the measured phase (force kFlight), never loosen it toward
/// kPreflight -- see effectivePhase()'s max() combination.
enum class GateModeOverride : uint8_t
{
  kAuto,
  kPreflight,
  kFlight,
};

enum class GoNoGo : uint8_t
{
  kGo,
  kNoGo,
  kUnknown,
};

/// P10.9b G1/G2: classification of a Sub-B child's "action" KeyValue
/// (already on the wire from safety_supervisor_node -- describeAction()).
/// Prefix-matched, NEVER full-string-matched: enforcement_enabled=false
/// dry-run strings are "would_hold (dry-run, ...)"/"would_inhibit (...)".
/// ONE-WAY veto only (G1's own rule): kHold/kInhibit/kUnrecognized can
/// FORCE a child to be counted (never-waivable); kReport NEVER grants
/// immunity by itself -- it only fails to trigger that forcing, falling
/// through to ordinary waiver matching (still waivable, same as before).
enum class ActionClass : uint8_t
{
  kReport,
  kHold,
  kInhibit,
  kUnrecognized,   // missing key, or a value not matching any known prefix
};

// C2: which optional subsystem, if any, this item belongs to. "" = always
// relevant (core items, unconditionally counted toward go/no-go). Gating
// rule (see ownerIsCounted() in the .cpp): "perception" is counted iff
// perception_enabled_ (that subsystem CAN legitimately be off, e.g. sim
// default `perception:=false`); "blackbox" is NEVER counted, regardless of
// blackbox_enabled_ (O1: the evidence layer's own health must never gate a
// flight -- a full disk or a disabled recorder is not a reason to refuse
// takeoff). Both still APPEAR in /diagnostics/aggregated either way.
struct WatchItem
{
  std::string name;
  std::string topic;   // suffix, "" when fed by a Sub-B typed subscription instead
  std::string type;
  std::string qos_profile;
  int64_t depth = 0;
  std::string owner;
};

/// P10.9b: a Sub-B source now also carries WHETHER its "action" KeyValue is
/// meaningful (action_aware -- NW2 fail-closed gate) and, if it publishes a
/// rollup child whose level is PROVEN to be a subset of its own children's
/// (summary_child -- G8, verified on failsafe_policy.cpp's overall_level:
/// std::max() over VIOLATING codes only), that child's bare name so it is
/// never double-counted. "" summary_child means this source has NO verified
/// rollup child -- its content is counted per-child like anything else,
/// which is the conservative (never LESS safe) default for any source this
/// property has not been proven for.
struct DiagSource
{
  std::string name;
  std::string topic;
  std::string type;
  std::string qos_profile;
  int64_t depth = 0;
  std::string owner;
  bool action_aware = false;
  std::string summary_child;
};

/// One child DiagnosticStatus as last received on a Sub-B source's
/// DiagnosticArray -- P10.9b replaces the old single worst-of-source
/// (worst_level, worst_child_name) with the FULL list (contract S:2.20
/// "chấm theo TỪNG CHILD, không lấy max toàn source").
struct ChildContent
{
  std::string name;
  uint8_t level = 0;   // diagnostic_msgs::msg::DiagnosticStatus level
  std::string action;  // raw "action" KeyValue value, "" if absent
};

struct DiagSourceContent
{
  bool received = false;
  std::vector<ChildContent> children;
  // P10.9b: > kMaxChildrenPerSource -- the whole source reads as a single
  // synthetic "unclassified" ERROR child, fail-closed, never waivable. A
  // vector this long is itself a signal something upstream is broken.
  bool over_cap = false;
};

/// P10.9b: one row of config/preflight_waivers.yaml. `when` is one of
/// "preflight" | "perception_off" -- contract S:2.20's own two conditions,
/// nothing else (adding a third value is a contract change, not a config
/// edit). A row that never matches ANY live (source,child) pair seen this
/// process lifetime is drift (P8 renamed a child, or a typo) -- tracked via
/// waiver_row_seen_, surfaced as the wire KeyValue "waiver_unmatched".
struct WaiverRow
{
  std::string source;
  std::string child;
  std::string when;
  std::string reason;
};

/// Evidence-layer rollup node (P10.4, plan S:2.2; go/no-go semantics
/// redesigned P10.9b, plan panel `.claude/plan/P10-gonogo-design-panel.md`
/// S:4 "operator-checklist" + G1/G2/G4/G5/G6/G7/G8 grafts). Sub A:
/// GenericSubscription cadence board over ~21 topics (arrival time only, no
/// deserialize) -- LIVENESS, always gates, never waivable. Sub B: typed
/// DiagnosticArray sources, now scored PER CHILD (contract S:2.20) -- this
/// is CONTENT, gated by phase/waiver/never-waive as described below.
/// Publishes /diagnostics/aggregated + /state/system_health.
/// R24: every sub and both timers share ONE MutuallyExclusive group -- the
/// two StalenessBoard instances and the Sub-B cache are not internally
/// synchronised, same discipline as RosbagManagerNode.
///
/// P10.9b core rule, verbatim from the design panel (kept here because it
/// is easy to lose in the middle of the Sub-B loop): LIVENESS ALWAYS GATES.
/// CONTENT only gates when its PRECONDITION exists. A seizing predicate
/// (HOLD/INHIBIT) actually firing is NEVER exempted by any precondition.
///
/// Sub-B per-child scoring order (exactly this order, first match wins):
///   0. source itself self-stale (> sub_b_stale_after_sec_) -> whole source
///      UNKNOWN (R32), no children to iterate that tick.
///   1. child is this source's verified summary/rollup (DiagSource::
///      summary_child) -> never counted by level (G8: it is PROVEN a subset
///      of children already counted individually, for `safety` only).
///   2. NW1: action prefix in {hold, inhibit, would_hold, would_inhibit}
///      AND level==ERROR -> ALWAYS counted, no phase/owner/waiver exempts
///      it (this is the ONE case that also survives owner=off, unlike
///      everything else -- see isNeverWaivable()'s caller).
///   3. NW2: source.action_aware, level==ERROR, action missing/unresolved
///      -> counted (fail-closed: an action_aware source that stops
///      publishing an "action" KeyValue must never read as silently safe).
///   4. owner gate: item's source.owner not currently enabled -> not
///      counted (perception off, or blackbox -- O1, always excluded).
///   5. waiver row matches (source,child) AND its `when` condition holds
///      (effective_phase==PREFLIGHT, or perception_enabled_==false) -> not
///      counted, waived_reason carried for visibility.
///   6. otherwise -> counted by level (OK/WARN/ERROR).
class DiagnosticsNode : public rclcpp::Node
{
public:
  explicit DiagnosticsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareAndValidateParams();
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  void subscribeWatchGroup(
    const std::vector<WatchItem> & items, const rclcpp::CallbackGroup::SharedPtr & group);
  void subscribeDiagSources(const rclcpp::CallbackGroup::SharedPtr & group);
  void onDiagSourceMessage(
    size_t index, const diagnostic_msgs::msg::DiagnosticArray::SharedPtr & message);
  void onVehicleState(const uav_interfaces::msg::VehicleState::SharedPtr & message);

  void onEvalTick();
  void onPublishTick();
  void publishAggregatedNow();
  void publishSystemHealthNow();

  double nowSeconds() const;
  static double nowWallSeconds();   // C6: wall clock, NOT the node's (possibly sim) clock
  // C2: true iff an item/source with this owner counts toward go/no-go
  // RIGHT NOW (still always reported in aggregated either way).
  bool ownerIsCounted(const std::string & owner) const;
  // P10.9b: measured phase off armed_/connected_ and the "state/vehicle"
  // cadence verdict (N1: fresh also requires connected_), asymmetric dwell
  // (G4) applied, then combined with gate_mode_override_ via max() --
  // override can only TIGHTEN. Also fills phase_source_out.
  Phase effectivePhase(const ItemVerdict & vehicle_state_verdict, PhaseSource & phase_source_out);

  // -------------------------------------------------------------- parameters
  std::string uav_id_;
  std::string prefix_;
  double report_period_sec_ = 0.0;    // internal StalenessBoard re-evaluation tick
  double publish_period_sec_ = 0.0;   // /diagnostics/aggregated + /state/system_health cadence
  double sub_b_stale_after_sec_ = 0.0;
  // C2: launch-supplied (sim.launch.py's `perception`/`blackbox` flags) --
  // NOT auto-derived from traffic, so an owned item's absence is legible as
  // "this subsystem is off" rather than silently starving go/no-go forever.
  bool perception_enabled_ = true;
  bool blackbox_enabled_ = true;

  std::vector<WatchItem> always_items_;
  std::vector<WatchItem> flight_only_items_;
  std::vector<DiagSource> diag_sources_;
  std::vector<WaiverRow> waivers_;

  // atomic: onSetParameters() (parameter-service thread) and onEvalTick()
  // (executor thread) touch this without a shared callback group.
  std::atomic<GateModeOverride> gate_mode_override_{GateModeOverride::kAuto};

  // R24: the ONLY callback group -- every sub, both timers.
  rclcpp::CallbackGroup::SharedPtr io_group_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> generic_subscriptions_;
  std::vector<rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr>
  diag_source_subscriptions_;
  // Review lượt 2 (N2): "state/vehicle" is now a SENTINEL always-item
  // (topic/type both "", same convention already used for the 3
  // diagnostics/* Sub-B-fed entries) -- this typed sub is the ONLY
  // subscriber on that topic, and its callback (onVehicleState()) feeds
  // cadence_board_ directly. Previously there were TWO independent
  // subscriptions here (this one, plus a GenericSubscription cadence entry)
  // that could see arrivals out of step with each other -- collapsing to
  // one removes that class of bug by construction instead of racing it.
  rclcpp::Subscription<uav_interfaces::msg::VehicleState>::SharedPtr vehicle_state_subscription_;
  rclcpp::TimerBase::SharedPtr eval_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr aggregated_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr system_health_publisher_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Sub A: cadence-only board over always_items_ then flight_only_items_
  // (addItem order -- evaluate() indices line up with those two vectors).
  StalenessBoard cadence_board_;
  // Names actually registered in cadence_board_ -- lets the Sub-B callback
  // decide whether to also feed cadence_board_.onArrival() for a source.
  std::unordered_set<std::string> cadence_item_names_;
  // Index of "state/vehicle" inside always_items_/the Sub-A verdict vector
  // -- resolved once at startup (declareAndValidateParams() requires the
  // name to be present), reused every tick instead of a string search.
  size_t vehicle_state_item_index_ = 0;

  // Sub B: fixed sub_b_stale_after_sec_ self-staleness board, one item per
  // diag_sources_ entry (addItem order matches diag_sources_ index).
  StalenessBoard diag_source_board_;
  std::vector<DiagSourceContent> diag_source_content_;   // index-aligned with diag_sources_

  // P10.9b: armed_ is written only from vehicle_state_subscription_'s
  // callback and read only from onEvalTick() -- both run on io_group_
  // (R24), so this needs no synchronisation of its own.
  bool armed_ = false;
  // Review lượt 2 (N1): mirrors VehicleState.connected -- effectivePhase()
  // ANDs this into "fresh" so a topic that keeps publishing AT CADENCE
  // while PX4 is unreachable (px4_state_adapter_node.cpp only assigns
  // `armed` inside `if (connected)`, defaulting it back to false otherwise)
  // reads as unmeasurable, not as a genuine disarm.
  bool connected_ = false;
  // G4: consecutive eval ticks the RAW (pre-override) predicate has read
  // "candidate preflight" (fresh && !armed_) -- downgrading FLIGHT->
  // PREFLIGHT requires this to reach kPreflightDwellTicks; upgrading is
  // immediate (reset to 0 the instant armed_ or !fresh is seen).
  unsigned preflight_dwell_ticks_seen_ = 0;
  // V4 (review lượt 3): was 5 (0,25s @ default report_period_sec_=0.05s),
  // itself a fix over an original 3 -- BOTH prior values were checked
  // against 2/expected_hz (a formal SAMPLING-period bound), never against
  // the REAL grace period a stale sample survives under: StalenessBoard
  // reads a sample as fresh for its WHOLE timeout_sec regardless of
  // whether a newer one ever arrives (staleness_board.cpp: age<=
  // timeout_sec). At the shipped defaults, always_timeout_sec["state/
  // vehicle"]=0,3s > 2/expected_hz=0,2s, so a 0,25s dwell window could
  // still confirm a downgrade off ONE sample that had not even gone stale
  // by its own rule, let alone been reconfirmed by a fresh arrival.
  // declareAndValidateParams() now enforces kPreflightDwellTicks*
  // report_period_sec_ > always_timeout_sec["state/vehicle"] (R29+R33) --
  // 7 gives comfortable margin (0,35s) above the shipped 0,3s timeout at
  // the 0.05s default report_period_sec_.
  static constexpr unsigned kPreflightDwellTicks = 7;
  static constexpr size_t kMaxChildrenPerSource = 64;   // over this -> fail-closed "unclassified"

  // P10.9b: per-waiver-row bookkeeping for the wire KeyValue
  // "waiver_unmatched" -- true once a live (source,child) pair matching
  // that row has been observed at least once THIS process lifetime,
  // regardless of whether `when` currently applies. Index-aligned with
  // waivers_, never reset (drift is itself evidence, R30).
  std::vector<bool> waiver_row_seen_;

  unsigned watch_failed_ = 0;

  // Built once per eval tick (onEvalTick); onPublishTick only re-stamps and
  // republishes -- keeps evaluation and wire cadence decoupled (V-O3).
  diagnostic_msgs::msg::DiagnosticArray cached_aggregated_;
  diagnostic_msgs::msg::DiagnosticStatus cached_system_health_;
  GoNoGo cached_go_no_go_ = GoNoGo::kUnknown;
  GoNoGo last_published_go_no_go_ = GoNoGo::kUnknown;
  bool has_evaluated_once_ = false;
};

}  // namespace uav_observability

#endif  // UAV_OBSERVABILITY__DIAGNOSTICS_NODE_HPP_
