#ifndef UAV_OBSERVABILITY__ROSBAG_MANAGER_NODE_HPP_
#define UAV_OBSERVABILITY__ROSBAG_MANAGER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rosbag2_cpp/writer.hpp>

namespace uav_observability
{

/// State published on /uav/<id>/diagnostics/observability (P10 plan S:2.1).
/// No STARTING value: the node is one of these three by the time the
/// constructor returns (open succeeds -> RECORDING, else DISABLED_NO_SPACE).
enum class ObservabilityState : uint8_t
{
  kRecording,
  kDegraded,
  kDisabledNoSpace,
};

/// Black-box recorder node -- P10.2 (plan S:2.1). GenericSubscription on N
/// yaml-declared topics -> rosbag2_cpp::Writer. Deliberately dumb about
/// message CONTENT: every topic is a byte string in, a byte string out,
/// including /fmu/* (Q-P10-1: a string in yaml is data, not a px4_msgs
/// symbol -- see docs/interface-contract-v0.1.md S:2.20 once P10.9 lands).
/// enforcement-free by design: a bad topic degrades that ONE topic
/// (topics_failed++), a full disk degrades RECORDING, nothing here ever
/// throws after construction (O1: the black box must never cost a flight).
/// Numbered points referenced from the .cpp as "header point N":
///   1. R24: ONE MutuallyExclusive io_group_ for every sub + both timers --
///      rosbag2_cpp::Writer is not thread-safe (P10 plan S:2.1).
///   2. Bag directory NAME is wall-clock UTC (system_clock), never sim time
///      -- sim time can start at/near 0 or run backwards across restarts,
///      which would collide bag directory names and silently overwrite
///      evidence. Message timestamps inside the bag are still node clock
///      (use_sim_time), only the directory name is wall clock.
///   3. Per-topic failure (bad type, missing typesupport, create_topic
///      throwing) never takes the node down -- caught, counted, WARN once,
///      the other topics keep recording (O1).
///   4. Compression is real (rosbag2_compression::SequentialCompressionWriter)
///      when compression_mode != "none", not a stub -- G-O1 (P10.3) picks
///      the actual mode/format empirically.
///   5. Each topic's TopicMetadata.offered_qos_profiles is populated (P10.8c
///      fix) -- without it `ros2 bag play` cannot durability-match
///      TransientLocal topics on replay. C3 (review, this pass): the VALUE
///      populated is the real PUBLISHER's offered QoS when discoverable, not
///      just "whatever we subscribed with" -- see offeredQosOrFallback().
///   6. Retention (onRetentionTick): disk_free_bytes/mtime that could not be
///      measured are NaN, never 0.0/epoch (C1(a)/(d)) -- "cannot measure"
///      must never read as "definitely full" / "definitely oldest".
///   7. Retention never runs with no active bag to protect (bag_dir_ empty,
///      e.g. DISABLED_NO_SPACE at boot) -- C1(c), an empty active_path used
///      to match nothing, making every existing bag a delete candidate.
///   8. Retention only ever considers directories that both (a) start with
///      uav_id_ + "_" and (b) contain a recognisable bag file -- C1(e): a
///      full disk must never make this node start deleting unrelated
///      directories under bag_root.
///   9. Every "/uav/*" topic must belong to THIS node's uav_id_ (C5) --
///      caught at construction, not silently recording nothing forever.
class RosbagManagerNode : public rclcpp::Node
{
public:
  explicit RosbagManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RosbagManagerNode() override;

private:
  void declareAndValidateParams();
  void tryOpenWriter();
  void closeWriterLocked();
  std::unique_ptr<rosbag2_cpp::Writer> makeWriterImpl() const;
  rosbag2_storage::StorageOptions buildStorageOptions(const std::string & uri) const;
  std::string makeBagDirName() const;
  bool topicIsFmu(const std::string & topic) const;
  // C3: the QoS the RECORDER subscribed with (subscribed_qos, e.g. "be" per
  // Q-P10-2) is not necessarily what the real PUBLISHER offers -- returns the
  // publisher's actual offered QoS when exactly one is discovered at topic-
  // creation time, else falls back to subscribed_qos with a WARN (no
  // publisher up yet, or more than one with disagreeing QoS -- ambiguous).
  rclcpp::QoS offeredQosOrFallback(const std::string & topic, const rclcpp::QoS & subscribed_qos);
  // C5: every "/uav/*" entry in topics_ must belong to THIS node's uav_id_
  // (whitelist "/tf", "/tf_static", "/fmu/*") -- refuse to start otherwise
  // (a yaml wired to the wrong drone would silently record an empty bag
  // while still reporting RECORDING).
  void requireTopicsMatchOwnNamespace() const;
  // C1(e): bag_root itself must never be $HOME or the filesystem root --
  // retention deleting a "foreign" directory under either would be
  // catastrophic, not just an evidence-layer bug.
  void requireBagRootIsNotHomeOrFilesystemRoot() const;

  void onMessage(size_t index, std::shared_ptr<rclcpp::SerializedMessage> message);
  void onRetentionTick();
  void onDiagnosticsTick();

  // ------------------------------------------------------------- parameters
  std::string uav_id_;
  std::string prefix_;
  std::string bag_root_;
  std::string storage_id_;
  std::string compression_mode_;     // "none" | "file" | "message"
  std::string compression_format_;   // "none" | "zstd"
  int64_t max_bagfile_size_bytes_ = 0;
  int64_t max_bagfile_duration_sec_ = 0;
  int64_t max_cache_size_bytes_ = 0;
  double max_total_bytes_ = 0.0;
  double min_free_bytes_ = 0.0;
  double retention_period_sec_ = 0.0;
  int64_t write_error_limit_ = 0;
  bool record_px4_topics_ = true;

  // 4 parallel arrays (V-O2 style, P10 plan S:2.2 precedent): topics_[i]
  // subscribes as types_[i] with QoS qos_profiles_[i] in {be, reliable,
  // reliable_tl} and KeepLast(depths_[i]).
  std::vector<std::string> topics_;
  std::vector<std::string> types_;
  std::vector<std::string> qos_profiles_;
  std::vector<int64_t> depths_;

  // R24: the ONLY callback group -- every sub, both timers (header point 1).
  rclcpp::CallbackGroup::SharedPtr io_group_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  // C3: index-aligned with topics_. subscribed_qos_[i] is the QoS this node
  // itself subscribed with (offeredQosOrFallback()'s fallback value);
  // topic_metadata_created_[i] tracks whether writer_->create_topic() has
  // run yet for that topic (deferred to the FIRST message, see onMessage()).
  std::vector<rclcpp::QoS> subscribed_qos_;
  std::vector<bool> topic_metadata_created_;
  rclcpp::TimerBase::SharedPtr retention_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  // Null whenever nothing should be written (DISABLED_NO_SPACE at boot,
  // or DEGRADED after write_error_limit_ consecutive failures) -- onMessage()
  // checks this and no-ops rather than branching on state_ directly.
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string bag_dir_;   // "" when writer_ is null
  ObservabilityState state_ = ObservabilityState::kDisabledNoSpace;

  // ------------------------------------------------------------- counters
  size_t topics_total_ = 0;
  size_t topics_subscribed_ = 0;
  size_t topics_failed_ = 0;
  uint64_t messages_written_ = 0;
  uint64_t bytes_written_ = 0;
  uint64_t write_errors_ = 0;
  // Resets to 0 on every successful write -- write_error_limit_ is a
  // CONSECUTIVE ceiling (P10 plan S:2.1), not a lifetime one.
  uint64_t consecutive_write_errors_ = 0;
  uint64_t bags_deleted_ = 0;
  double disk_free_bytes_ = 0.0;
};

}  // namespace uav_observability

#endif  // UAV_OBSERVABILITY__ROSBAG_MANAGER_NODE_HPP_
