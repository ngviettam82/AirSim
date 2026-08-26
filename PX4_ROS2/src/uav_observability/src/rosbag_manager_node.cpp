#include "uav_observability/rosbag_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>
#include <system_error>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rosbag2_compression/compression_options.hpp>
#include <rosbag2_compression/sequential_compression_writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_transport/qos.hpp>

#include "uav_observability/bag_retention.hpp"

namespace uav_observability
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace
{

namespace fs = std::filesystem;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

void require(bool condition, const std::string & detail)
{
  if (!condition) {
    throw std::invalid_argument("rosbag_manager params: " + detail);
  }
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

std::string expandHome(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

rclcpp::QoS qosFromProfile(const std::string & profile, int64_t depth)
{
  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(depth)));
  if (profile == "reliable") {
    qos.reliable();
  } else if (profile == "reliable_tl") {
    qos.reliable().transient_local();
  } else {
    // "be" and anything else (validate() already forbids anything else
    // reaching here) -- O1's stated default, zero backpressure on the bus.
    qos.best_effort();
  }
  return qos;
}

// P10.8c fix: TopicMetadata::offered_qos_profiles was left "" (constructed
// but never assigned) -- metadata.yaml then carries no durability info for
// any topic, so `ros2 bag play` falls back to Volatile for everything and a
// TransientLocal subscriber joining after playback starts gets nothing.
// Rosbag2QoS's OWN YAML::convert is reused (not a hand-rolled string) so the
// format is guaranteed to round-trip through the SAME decoder
// rosbag2_transport::Player calls when it reads metadata.yaml back.
std::string offeredQosProfilesYaml(const rclcpp::QoS & qos)
{
  const std::vector<rosbag2_transport::Rosbag2QoS> offers{rosbag2_transport::Rosbag2QoS(qos)};
  return YAML::Dump(YAML::Node(offers));
}

const char * stateName(ObservabilityState state)
{
  switch (state) {
    case ObservabilityState::kRecording: return "RECORDING";
    case ObservabilityState::kDegraded: return "DEGRADED";
    case ObservabilityState::kDisabledNoSpace: return "DISABLED_NO_SPACE";
    default: return "UNKNOWN";
  }
}

uint8_t stateToDiagnosticLevel(ObservabilityState state)
{
  switch (state) {
    case ObservabilityState::kRecording: return DiagnosticStatus::OK;
    case ObservabilityState::kDegraded: return DiagnosticStatus::WARN;
    case ObservabilityState::kDisabledNoSpace: return DiagnosticStatus::ERROR;
    default: return DiagnosticStatus::STALE;
  }
}

// Recursive directory byte total ("du -s" equivalent) -- retention's own
// BagDirInfo.bytes (bag_retention.hpp). Missing/unreadable entries are
// skipped rather than failing the whole scan: a half-deleted bag directory
// must not wedge the retention tick.
std::uintmax_t directorySizeBytes(const fs::path & dir)
{
  std::uintmax_t total = 0;
  std::error_code walk_ec;
  fs::recursive_directory_iterator it(
    dir, fs::directory_options::skip_permission_denied, walk_ec);
  const fs::recursive_directory_iterator end;
  for (; !walk_ec && it != end; it.increment(walk_ec)) {
    std::error_code type_ec;
    if (!it->is_regular_file(type_ec) || type_ec) {
      continue;
    }
    std::error_code size_ec;
    const auto size = it->file_size(size_ec);
    if (!size_ec) {
      total += size;
    }
  }
  return total;
}

// GCC libstdc++ pre-C++20: file_time_type's clock is not guaranteed
// system_clock -- this now-vs-now offset trick converts without assuming
// the two clocks share an epoch (portable across GCC versions in R7's
// Ubuntu 22.04 target).
double fileTimeToEpochSeconds(fs::file_time_type file_time)
{
  const auto system_time = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
    file_time - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
  return std::chrono::duration<double>(system_time.time_since_epoch()).count();
}

// C1(e): a retention candidate must look like an actual bag, not just any
// directory that happens to live under bag_root -- a stray non-bag
// directory (px4_logs/, an operator's own scratch folder) must never be
// treated as evidence to reclaim. rosbag2 always writes metadata.yaml on a
// clean close; *.db3 (sqlite3) or *.mcap covers an unclean shutdown where
// metadata.yaml was never written (G-O1).
bool looksLikeBagDir(const fs::path & dir)
{
  std::error_code iter_ec;
  fs::directory_iterator it(dir, fs::directory_options::skip_permission_denied, iter_ec);
  const fs::directory_iterator end;
  for (; !iter_ec && it != end; it.increment(iter_ec)) {
    std::error_code type_ec;
    if (!it->is_regular_file(type_ec) || type_ec) {
      continue;
    }
    if (it->path().filename() == "metadata.yaml") {
      return true;
    }
    const std::string extension = it->path().extension().string();
    if (extension == ".db3" || extension == ".mcap") {
      return true;
    }
  }
  return false;
}

}  // namespace

RosbagManagerNode::RosbagManagerNode(const rclcpp::NodeOptions & options)
try
: Node("rosbag_manager_node", options)
{
  // C5: prefix_ is now assigned INSIDE declareAndValidateParams(), right
  // after uav_id_ itself is declared -- the topic-namespace check
  // (requireTopicsMatchOwnNamespace()) needs it, and this is the single
  // place that builds it (no second, driftable "/uav/" + uav_id_ elsewhere).
  declareAndValidateParams();   // throws std::invalid_argument -- refuse to start (yaml sai)

  // R24 (header point 1): ONE group for every sub AND both timers --
  // rosbag2_cpp::Writer is not thread-safe.
  io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  tryOpenWriter();   // disk-space-at-boot gate; sets state_ either way, never throws

  diagnostics_publisher_ = create_publisher<DiagnosticArray>(
    prefix_ + "/diagnostics/observability", rclcpp::QoS(10));

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = io_group_;

  topics_total_ = topics_.size();
  // Sized up front, index-aligned with topics_ -- entries for a topic that
  // never subscribed successfully are simply never read (onMessage(i, ...)
  // can only fire for an i that DID subscribe).
  subscribed_qos_.assign(topics_.size(), rclcpp::QoS(1));
  topic_metadata_created_.assign(topics_.size(), false);
  for (size_t i = 0; i < topics_.size(); ++i) {
    if (!record_px4_topics_ && topicIsFmu(topics_[i])) {
      RCLCPP_INFO(
        get_logger(), "skipping %s (record_px4_topics=false)", topics_[i].c_str());
      continue;
    }
    // V-O2/depths_[i]>0 already enforced by declareAndValidateParams() --
    // depths_[i] is used directly, no default_subscription_depth_ fallback
    // (that fallback was dead code: unreachable once every entry is
    // required positive, VÀNG#7).
    const int64_t depth = depths_[i];
    // header point 3: EITHER call throwing (bad type/typesupport missing,
    // or the writer refusing the topic) is caught HERE -- one topic never
    // takes the node down (O1).
    try {
      const rclcpp::QoS subscribed_qos = qosFromProfile(qos_profiles_[i], depth);
      auto subscription = create_generic_subscription(
        topics_[i], types_[i], subscribed_qos,
        [this, i](std::shared_ptr<rclcpp::SerializedMessage> message) {
          onMessage(i, message);
        }, sub_options);
      // C3: the writer's create_topic() (and the offered-QoS DECISION) is
      // now made lazily, on the FIRST message actually received for this
      // topic (see onMessage()) -- not here. At construction time almost no
      // real publisher has connected yet (P10.6: this node is deliberately
      // launched FIRST, precisely to win the TransientLocal race #967),
      // so querying offered QoS here would nearly always fall back to
      // "whatever we subscribed with" -- silently reproducing the exact bug
      // this fix targets for every topic. subscribed_qos_[i] is kept as the
      // FALLBACK offeredQosOrFallback() uses if a real publisher's QoS still
      // cannot be determined by then.
      subscribed_qos_[i] = subscribed_qos;
      subscriptions_.push_back(subscription);
      ++topics_subscribed_;
    } catch (const std::exception & error) {
      ++topics_failed_;
      RCLCPP_WARN(
        get_logger(), "topic %s (%s) unavailable: %s",
        topics_[i].c_str(), types_[i].c_str(), error.what());
    }
  }

  retention_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(retention_period_sec_),
    [this]() {onRetentionTick();}, io_group_);
  diagnostics_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0),
    [this]() {onDiagnosticsTick();}, io_group_);

  RCLCPP_INFO(
    get_logger(), "rosbag manager ready for %s: state=%s topics=%zu/%zu (failed=%zu)",
    uav_id_.c_str(), stateName(state_), topics_subscribed_, topics_total_, topics_failed_);
}
catch (const std::invalid_argument & error)
{
  RCLCPP_FATAL(
    rclcpp::get_logger("rosbag_manager_node"), "refusing to start: %s", error.what());
  throw;
}

RosbagManagerNode::~RosbagManagerNode()
{
  closeWriterLocked();
}

void RosbagManagerNode::declareAndValidateParams()
{
  uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
  // C5: assigned here, right after uav_id_ itself is known -- the topic-
  // namespace check below (requireTopicsMatchOwnNamespace()) needs it, and
  // every OTHER use of prefix_ (diagnostics topic, etc.) now reads this
  // exact value too, so there is only ever one "/uav/" + uav_id_ built.
  prefix_ = "/uav/" + uav_id_;
  bag_root_ = expandHome(declare_parameter<std::string>("bag_root", "~/uav_bags"));
  // VÀNG#8: sqlite3, not mcap -- G-O1 (P10.3) measured mcap losing 100% of
  // a bag (0/10 readable) on an unclean shutdown, sqlite3 10/10 via reindex
  // (docs/interface-contract-v0.1.md S:2.20). The yaml already overrides
  // this correctly; this is the fallback a future config that FORGETS to
  // set storage_id would silently get -- it must not be the format known to
  // lose everything.
  storage_id_ = declare_parameter<std::string>("storage_id", "sqlite3");
  compression_mode_ = declare_parameter<std::string>("compression_mode", "none");
  compression_format_ = declare_parameter<std::string>("compression_format", "none");
  max_bagfile_size_bytes_ = declare_parameter<int64_t>(
    "max_bagfile_size_bytes", 512 * 1024 * 1024);
  max_bagfile_duration_sec_ = declare_parameter<int64_t>("max_bagfile_duration_sec", 300);
  max_cache_size_bytes_ = declare_parameter<int64_t>("max_cache_size_bytes", 32 * 1024 * 1024);
  max_total_bytes_ = declare_parameter<double>(
    "max_total_bytes", 8.0 * 1024.0 * 1024.0 * 1024.0);
  min_free_bytes_ = declare_parameter<double>("min_free_bytes", 2.0 * 1024.0 * 1024.0 * 1024.0);
  retention_period_sec_ = declare_parameter<double>("retention_period_sec", 10.0);
  write_error_limit_ = declare_parameter<int64_t>("write_error_limit", 50);
  record_px4_topics_ = declare_parameter<bool>("record_px4_topics", true);

  if (storage_id_ == "mcap") {
    RCLCPP_WARN(
      get_logger(),
      "storage_id=mcap: G-O1 measured 0/10 bags recoverable after an unclean shutdown "
      "(kill -9) -- sqlite3 is the reviewed default (P10.3/Q-P10-4), pick it unless you have "
      "a specific reason not to");
  }

  // Explicit vector{} (not {}) -- {} is ambiguous with the ParameterDescriptor
  // overload and warns under -Wall.
  topics_ = declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
  types_ = declare_parameter<std::vector<std::string>>("types", std::vector<std::string>{});
  qos_profiles_ =
    declare_parameter<std::vector<std::string>>("qos_profiles", std::vector<std::string>{});
  depths_ = declare_parameter<std::vector<int64_t>>("depths", std::vector<int64_t>{});

  require(!bag_root_.empty(), "bag_root must not be empty");
  requireBagRootIsNotHomeOrFilesystemRoot();   // C1(e)
  require(storage_id_ == "mcap" || storage_id_ == "sqlite3", "storage_id must be mcap|sqlite3");
  require(
    compression_mode_ == "none" || compression_mode_ == "file" || compression_mode_ == "message",
    "compression_mode must be none|file|message");
  require(
    compression_format_ == "none" || compression_format_ == "zstd",
    "compression_format must be none|zstd");
  require(
    compression_mode_ == "none" || compression_format_ == "zstd",
    "compression_mode != none requires compression_format=zstd");

  const auto finitePositive = [](double value) {return std::isfinite(value) && value > 0.0;};
  require(
    finitePositive(static_cast<double>(max_bagfile_size_bytes_)),
    "max_bagfile_size_bytes must be finite and > 0");
  require(
    finitePositive(static_cast<double>(max_bagfile_duration_sec_)),
    "max_bagfile_duration_sec must be finite and > 0");
  require(
    finitePositive(static_cast<double>(max_cache_size_bytes_)),
    "max_cache_size_bytes must be finite and > 0");
  // VÀNG#7: delegate to bag_retention's OWN validateBudget() instead of
  // duplicating its exact rule ad hoc -- its header docstring already
  // claims this is how a bad budget fails "on the ground", but nothing
  // called it; this is that missing caller. Widens max_total_bytes_/
  // min_free_bytes_==0.0 from rejected to accepted (validateBudget's own
  // documented contract: "0 is a legitimate, if degenerate, budget") --
  // no existing test pins zero as an error here.
  validateBudget(RetentionBudget{max_total_bytes_, min_free_bytes_});
  require(finitePositive(retention_period_sec_), "retention_period_sec must be finite and > 0");
  require(
    finitePositive(static_cast<double>(write_error_limit_)),
    "write_error_limit must be finite and > 0");

  require(!topics_.empty(), "topics must not be empty");
  require(
    topics_.size() == types_.size() && topics_.size() == qos_profiles_.size() &&
    topics_.size() == depths_.size(),
    "topics/types/qos_profiles/depths must be the same length");

  std::set<std::string> unique_topics(topics_.begin(), topics_.end());
  require(unique_topics.size() == topics_.size(), "topics must not contain duplicates");

  for (const std::string & profile : qos_profiles_) {
    require(
      profile == "be" || profile == "reliable" || profile == "reliable_tl",
      "qos_profiles entries must be be|reliable|reliable_tl (got '" + profile + "')");
  }
  for (int64_t depth : depths_) {
    require(finitePositive(static_cast<double>(depth)), "every depths entry must be finite and > 0");
  }

  requireTopicsMatchOwnNamespace();   // C5
}

bool RosbagManagerNode::topicIsFmu(const std::string & topic) const
{
  return topic.rfind("/fmu/", 0) == 0;
}

void RosbagManagerNode::requireTopicsMatchOwnNamespace() const
{
  // C5: a yaml wired to the wrong drone (uav_id:=uav1 but topics: still say
  // /uav/uav0/...) previously subscribed FINE (create_generic_subscription
  // does not require a publisher to exist) -- topics_failed_ stayed 0,
  // state stayed RECORDING, and the bag silently held nothing from the
  // ACTUAL running stack. Global/whitelisted exemptions (never namespaced
  // by design, same list as R2's own exceptions): /tf, /tf_static, /fmu/*.
  for (const std::string & topic : topics_) {
    if (topic == "/tf" || topic == "/tf_static" || topicIsFmu(topic)) {
      continue;
    }
    if (topic.rfind("/uav/", 0) != 0) {
      continue;   // not namespaced under /uav/ at all -- not this check's business
    }
    require(
      topic.rfind(prefix_ + "/", 0) == 0,
      "topic " + topic + " does not belong to this node's uav_id (" + uav_id_ +
      ") -- recording would silently sit on an empty bag while reporting RECORDING");
  }
}

void RosbagManagerNode::requireBagRootIsNotHomeOrFilesystemRoot() const
{
  // C1(e): retention deletes whole directories under bag_root -- bag_root
  // resolving to $HOME or "/" would eventually let a full disk delete
  // unrelated files outside the evidence layer entirely.
  require(
    fs::path(bag_root_).lexically_normal() != fs::path("/").lexically_normal(),
    "bag_root must not be the filesystem root");
  const char * home = std::getenv("HOME");
  if (home != nullptr && *home != '\0') {
    require(
      fs::path(bag_root_).lexically_normal() != fs::path(home).lexically_normal(),
      "bag_root must not equal $HOME (retention would delete unrelated files)");
  }
}

std::unique_ptr<rosbag2_cpp::Writer> RosbagManagerNode::makeWriterImpl() const
{
  if (compression_mode_ == "none") {
    return std::make_unique<rosbag2_cpp::Writer>();
  }
  // header point 4: real compression, wired for G-O1 (P10.3) to pick from
  // empirically -- not a stub.
  rosbag2_compression::CompressionOptions options;
  options.compression_format = compression_format_;
  options.compression_mode = compression_mode_ == "file" ?
    rosbag2_compression::CompressionMode::FILE : rosbag2_compression::CompressionMode::MESSAGE;
  options.compression_queue_size = 1;
  options.compression_threads = 1;
  return std::make_unique<rosbag2_cpp::Writer>(
    std::make_unique<rosbag2_compression::SequentialCompressionWriter>(options));
}

rosbag2_storage::StorageOptions RosbagManagerNode::buildStorageOptions(
  const std::string & uri) const
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri;
  storage_options.storage_id = storage_id_;
  storage_options.max_bagfile_size = static_cast<uint64_t>(max_bagfile_size_bytes_);
  storage_options.max_bagfile_duration = static_cast<uint64_t>(max_bagfile_duration_sec_);
  storage_options.max_cache_size = static_cast<uint64_t>(max_cache_size_bytes_);
  return storage_options;
}

std::string RosbagManagerNode::makeBagDirName() const
{
  // header point 2: wall clock UTC, never sim time.
  const auto now_wall = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now_wall);
  std::tm utc_tm{};
  gmtime_r(&now_time, &utc_tm);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%SZ", &utc_tm);
  return uav_id_ + "_" + std::string(buffer);
}

void RosbagManagerNode::tryOpenWriter()
{
  std::error_code create_ec;
  fs::create_directories(bag_root_, create_ec);   // best-effort; space check below is the real gate

  std::error_code space_ec;
  const fs::space_info space = fs::space(bag_root_, space_ec);
  // C1(a): NaN, never 0.0 -- a measurement FAILURE is not the same fact as
  // "definitely zero bytes free" (R30). Boot still refuses to start either
  // way (an unmeasurable disk is not a safe one to start writing on), but
  // the DIAGNOSTIC field itself must say what actually happened.
  const double free_bytes = space_ec ? kNan : static_cast<double>(space.available);
  disk_free_bytes_ = free_bytes;

  if (!std::isfinite(free_bytes) || free_bytes < min_free_bytes_) {
    RCLCPP_ERROR(
      get_logger(),
      "insufficient or unmeasurable disk at boot (free=%.0f, min_free_bytes=%.0f) -- "
      "DISABLED_NO_SPACE",
      free_bytes, min_free_bytes_);
    state_ = ObservabilityState::kDisabledNoSpace;
    return;
  }

  // C1(b): fs::path's operator/ never produces a doubled separator even if
  // bag_root_ itself has a trailing '/' -- unlike the old bag_root_ + "/" +
  // name string concatenation, this always matches what a real
  // fs::directory_iterator later reports for the SAME directory.
  bag_dir_ = (fs::path(bag_root_) / makeBagDirName()).lexically_normal().string();
  try {
    writer_ = makeWriterImpl();
    writer_->open(buildStorageOptions(bag_dir_));
    state_ = ObservabilityState::kRecording;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "failed to open bag writer at %s: %s", bag_dir_.c_str(), error.what());
    writer_.reset();
    bag_dir_.clear();
    state_ = ObservabilityState::kDisabledNoSpace;
  }
}

void RosbagManagerNode::closeWriterLocked()
{
  if (!writer_) {
    return;
  }
  try {
    writer_->close();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "error closing bag writer: %s", error.what());
  }
  writer_.reset();
}

rclcpp::QoS RosbagManagerNode::offeredQosOrFallback(
  const std::string & topic, const rclcpp::QoS & subscribed_qos)
{
  const auto publishers = get_publishers_info_by_topic(topic);
  if (publishers.size() != 1) {
    // 0 discovered, or >1 with potentially disagreeing QoS -- either way
    // "what we subscribed with" is the only value that stays unambiguous.
    // C3: explicitly a GUESS, not a measurement -- logged as such so a
    // reviewer of this WARN knows offered_qos_profiles for this topic may
    // not match the real publisher.
    RCLCPP_WARN(
      get_logger(),
      "%s: %zu publisher(s) discovered -- offered_qos_profiles falls back to our OWN "
      "subscribed QoS (a guess, not a measured value)", topic.c_str(), publishers.size());
    return subscribed_qos;
  }
  return publishers.front().qos_profile();
}

void RosbagManagerNode::onMessage(size_t index, std::shared_ptr<rclcpp::SerializedMessage> message)
{
  if (!writer_) {
    // DISABLED_NO_SPACE or DEGRADED -- nothing to write, no counter moves.
    return;
  }
  try {
    if (!topic_metadata_created_[index]) {
      // C3: deferred to the FIRST real message on this topic (see the
      // constructor's own comment on subscribed_qos_) -- a real publisher
      // is now guaranteed discoverable, so offeredQosOrFallback() can
      // report its ACTUAL offered QoS instead of a guess. Still happens
      // inside the SAME callback that received the message, so this does
      // not reopen the TransientLocal race (#967): the SUBSCRIPTION itself
      // was already created eagerly at construction.
      rosbag2_storage::TopicMetadata metadata;
      metadata.name = topics_[index];
      metadata.type = types_[index];
      metadata.serialization_format = "cdr";
      metadata.offered_qos_profiles =
        offeredQosProfilesYaml(offeredQosOrFallback(topics_[index], subscribed_qos_[index]));
      writer_->create_topic(metadata);
      topic_metadata_created_[index] = true;
    }
    // Writer::write() TRANSFERS ownership of message's buffer (its own
    // doc: "the serialized data will no longer be managed by message") --
    // must read size() BEFORE the call, never after (was a use-after-
    // transfer: read 0 and crashed the writer's cache thread later).
    const size_t serialized_bytes = message->size();
    writer_->write(message, topics_[index], types_[index], now());
    ++messages_written_;
    bytes_written_ += serialized_bytes;
    consecutive_write_errors_ = 0;
  } catch (const std::exception & error) {
    // Covers BOTH a create_topic() failure (first message on this topic)
    // and a write() failure -- same counter/DEGRADED path either way, and
    // topic_metadata_created_[index] is left false on a create_topic()
    // failure so the NEXT message retries it.
    ++write_errors_;
    ++consecutive_write_errors_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "write/create_topic failed on %s: %s",
      topics_[index].c_str(), error.what());
    if (consecutive_write_errors_ > static_cast<uint64_t>(write_error_limit_)) {
      RCLCPP_ERROR(
        get_logger(), "write_error_limit (%ld) exceeded consecutively -- closing writer, DEGRADED",
        static_cast<long>(write_error_limit_));
      closeWriterLocked();
      state_ = ObservabilityState::kDegraded;
    }
  }
}

void RosbagManagerNode::onRetentionTick()
{
  // C1(c): no active bag to protect (DISABLED_NO_SPACE at boot, or DEGRADED
  // after write_error_limit_) -- bag_dir_="" used to match no real path, so
  // EVERY existing bag directory (including evidence from a PREVIOUS
  // flight) became a normal deletion candidate. Nothing to reclaim space
  // FOR here anyway.
  if (!writer_ || bag_dir_.empty()) {
    return;
  }

  std::error_code exists_ec;
  if (!fs::exists(bag_root_, exists_ec) || exists_ec) {
    return;
  }

  std::vector<BagDirInfo> bags;
  std::error_code iter_ec;
  fs::directory_iterator it(bag_root_, iter_ec);
  const fs::directory_iterator end;
  for (; !iter_ec && it != end; it.increment(iter_ec)) {
    std::error_code type_ec;
    if (!it->is_directory(type_ec) || type_ec) {
      continue;
    }
    // C1(e): only ever consider a directory that (a) belongs to THIS uav_id
    // and (b) actually looks like a bag -- a full disk must never make this
    // node start deleting unrelated directories under bag_root (px4_logs/,
    // an operator's own scratch folder, another uav_id's evidence).
    const std::string dirname = it->path().filename().string();
    if (dirname.rfind(uav_id_ + "_", 0) != 0) {
      continue;
    }
    if (!looksLikeBagDir(it->path())) {
      continue;
    }
    std::error_code mtime_ec;
    const auto mtime = it->last_write_time(mtime_ec);
    BagDirInfo info;
    info.path = it->path().string();
    info.bytes = static_cast<double>(directorySizeBytes(it->path()));
    // C1(d): NaN, never 0.0/epoch -- an unmeasurable mtime must sort LAST
    // (bag_retention::plan()'s olderFirst comparator), never be treated as
    // "oldest, delete first".
    info.mtime_wall_sec = mtime_ec ? kNan : fileTimeToEpochSeconds(mtime);
    bags.push_back(info);
  }

  std::error_code space_ec;
  const fs::space_info space = fs::space(bag_root_, space_ec);
  // C1(a): NaN, never 0.0 -- plan()'s free-space criterion (a NaN
  // comparison is always false) then correctly contributes NOTHING rather
  // than reading "cannot measure" as "the disk is completely full", while
  // max_total_bytes (independently measured from each bag's own byte size)
  // still gates normally.
  const double free_bytes = space_ec ? kNan : static_cast<double>(space.available);
  disk_free_bytes_ = free_bytes;

  RetentionBudget budget;
  budget.max_total_bytes = max_total_bytes_;
  budget.min_free_bytes = min_free_bytes_;

  const RetentionPlan retention_plan = plan(bags, bag_dir_, budget, free_bytes);

  for (const std::string & path : retention_plan.delete_paths) {
    std::error_code remove_ec;
    const auto removed = fs::remove_all(path, remove_ec);
    if (remove_ec) {
      RCLCPP_ERROR(
        get_logger(), "failed to delete old bag %s: %s", path.c_str(), remove_ec.message().c_str());
    } else if (removed > 0) {
      ++bags_deleted_;
      RCLCPP_WARN(get_logger(), "retention deleted old bag %s", path.c_str());
    }
  }

  if (retention_plan.must_stop_recording && writer_) {
    RCLCPP_ERROR(
      get_logger(),
      "disk budget still violated after deleting every non-active bag -- stopping recording");
    closeWriterLocked();
    state_ = ObservabilityState::kDisabledNoSpace;
  }
}

void RosbagManagerNode::onDiagnosticsTick()
{
  DiagnosticArray array;
  array.header.stamp = now();

  DiagnosticStatus status;
  status.name = "rosbag_manager";
  status.hardware_id = uav_id_;
  status.level = stateToDiagnosticLevel(state_);
  status.message = std::string("state=") + stateName(state_);
  status.values.push_back(keyValueStr("state", stateName(state_)));
  status.values.push_back(keyValueStr("bag_dir", bag_dir_));
  status.values.push_back(keyValue("bytes_written", static_cast<double>(bytes_written_)));
  status.values.push_back(keyValue("topics_subscribed", static_cast<double>(topics_subscribed_)));
  status.values.push_back(keyValue("topics_failed", static_cast<double>(topics_failed_)));
  status.values.push_back(keyValue("topics_total", static_cast<double>(topics_total_)));
  status.values.push_back(keyValue("messages_written", static_cast<double>(messages_written_)));
  status.values.push_back(keyValue("write_errors", static_cast<double>(write_errors_)));
  status.values.push_back(keyValue("bags_deleted", static_cast<double>(bags_deleted_)));
  status.values.push_back(keyValue("disk_free_bytes", disk_free_bytes_));
  array.status.push_back(status);

  diagnostics_publisher_->publish(array);
}

}  // namespace uav_observability
