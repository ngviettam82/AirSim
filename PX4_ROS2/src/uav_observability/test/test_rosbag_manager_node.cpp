// P10.2: the real node, driven through real topics by a probe (R20, hence
// the isolated domain ENV ROS_DOMAIN_ID=99, see CMakeLists.txt). Every wait
// is event-anchored (waitFor + a hang ceiling), never a bare sleep (R21).
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_transport/qos.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "uav_observability/rosbag_manager_node.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using namespace std::chrono_literals;

namespace uav_observability
{
namespace
{

namespace fs = std::filesystem;

bool waitFor(const std::function<bool()> & happened, double ceiling_seconds)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(ceiling_seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (happened()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return happened();
}

std::string diagnosticStr(const DiagnosticStatus & status, const std::string & key)
{
  for (const KeyValue & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return "";
}

double diagnosticNum(const DiagnosticStatus & status, const std::string & key)
{
  const std::string raw = diagnosticStr(status, key);
  if (raw.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  try {
    return std::stod(raw);
  } catch (const std::exception &) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

/// Unique, empty, auto-removed temp dir per Harness instance -- never shared
/// across tests (parallel ctest runs, and RosbagManagerNode's own boot-time
/// disk scan must see exactly what THIS test put there).
class TempDir
{
public:
  explicit TempDir(const std::string & tag)
  {
    static std::atomic<uint64_t> counter{0};
    const auto salt = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = fs::temp_directory_path() /
      ("uav_observability_" + tag + "_" + std::to_string(counter.fetch_add(1)) + "_" +
      std::to_string(salt));
    fs::create_directories(path_);
  }
  ~TempDir() {std::error_code ec; fs::remove_all(path_, ec);}
  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;

  const fs::path & path() const {return path_;}

private:
  fs::path path_;
};

/// Directory + one file inside it (retention.bytes source) + a metadata.yaml
/// marker (so it satisfies looksLikeBagDir(), C1(e) -- a REAL rosbag2 writer
/// always leaves one on a clean close), with an explicit mtime set via the
/// SAME now-vs-now offset trick the node itself uses (rosbag_manager_node.
/// cpp's fileTimeToEpochSeconds) -- portable across the pre-/post-C++20
/// file_clock ambiguity. Callers that want a candidate retention can
/// actually delete must pass a NAME already prefixed uav_id + "_" (C1(e)'s
/// OTHER half of the filter).
void makeFakeBagDir(
  const fs::path & root, const std::string & name,
  std::chrono::system_clock::time_point mtime, size_t payload_bytes)
{
  const fs::path dir = root / name;
  fs::create_directories(dir);
  std::ofstream file((dir / "data.bin").string(), std::ios::binary);
  const std::vector<char> buffer(payload_bytes, 'x');
  file.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));
  file.close();
  std::ofstream metadata((dir / "metadata.yaml").string());
  metadata << "rosbag2_bagfile_information: {}\n";
  metadata.close();

  const auto age = std::chrono::system_clock::now() - mtime;
  const auto file_time = fs::file_time_type::clock::now() -
    std::chrono::duration_cast<fs::file_time_type::duration>(age);
  fs::last_write_time(dir, file_time);
}

/// Node + probe + spinning executor + a /diagnostics/observability tap.
/// Each test builds its OWN parameter set (topics differ per case), so this
/// is a plain class instantiated inside each TEST(), not a shared Fixture.
class Harness
{
public:
  Harness(const std::string & uav_id, const std::vector<rclcpp::Parameter> & params)
  : uav_id_(uav_id), prefix_("/uav/" + uav_id)
  {
    // MUST run before the executor is constructed (below): its ctor creates
    // a guard condition against the global context, which is null until
    // init() -- a default member initializer would run this too early
    // (member-init phase precedes the constructor body).
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    // The node's OWN uav_id_ (bag dir naming + diagnostics topic prefix)
    // defaults to "uav0" -- without this override every Harness after the
    // first publishes diagnostics on a topic the probe below never
    // subscribes to (real bug hit here: silent topic mismatch, 5 s of
    // "no diagnostics tick" that was actually just the wrong topic name).
    // Inlined rather than calling the overrideParam() free helper: that
    // helper is declared LATER in this file (namespace-scope lookup from an
    // inline member function body does not see it).
    std::vector<rclcpp::Parameter> node_params = params;
    bool uav_id_already_present = false;
    for (rclcpp::Parameter & param : node_params) {
      if (param.get_name() == "uav_id") {
        param = rclcpp::Parameter("uav_id", uav_id);
        uav_id_already_present = true;
      }
    }
    if (!uav_id_already_present) {
      node_params.push_back(rclcpp::Parameter("uav_id", uav_id));
    }

    rclcpp::NodeOptions options;
    options.parameter_overrides(node_params);
    node_ = std::make_shared<RosbagManagerNode>(options);

    probe_ = std::make_shared<rclcpp::Node>("rosbag_probe_" + uav_id);
    diagnostics_sub_ = probe_->create_subscription<DiagnosticArray>(
      prefix_ + "/diagnostics/observability", rclcpp::QoS(10),
      [this](const DiagnosticArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!msg->status.empty()) {
          last_diagnostics_ = msg->status.front();
          has_diagnostics_ = true;
        }
      });

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor_->add_node(node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
  }

  ~Harness()
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    probe_.reset();
  }

  Harness(const Harness &) = delete;
  Harness & operator=(const Harness &) = delete;

  /// Stops the executor COMPLETELY before destroying the node -- a bare
  /// remove_node() does not wait for an in-flight callback on another
  /// executor thread to finish, so destroying the node right after it is a
  /// use-after-free race (segfaulted under MultiThreadedExecutor here: the
  /// same reason test_safety_supervisor_node.cpp's TearDown cancels+joins
  /// before resetting anything). probe_ stops spinning too; fine, no test
  /// needs it live after this call.
  void shutdownNode()
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
  }

  rclcpp::Node::SharedPtr probe() {return probe_;}
  const std::string & prefix() const {return prefix_;}

  bool waitForDiagnostics(double ceiling_seconds)
  {
    return waitFor([this]() {std::lock_guard<std::mutex> lock(mutex_); return has_diagnostics_;},
      ceiling_seconds);
  }

  DiagnosticStatus lastDiagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_diagnostics_;
  }

private:
  std::string uav_id_;
  std::string prefix_;
  std::shared_ptr<RosbagManagerNode> node_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::mutex mutex_;
  DiagnosticStatus last_diagnostics_;
  bool has_diagnostics_ = false;
};

std::string nextUavId()
{
  static std::atomic<int> counter{0};
  return "uav" + std::to_string(counter.fetch_add(1));
}

/// Minimal, fast, single-topic base config -- every test starts here and
/// overrides only what it needs (same shape as uav_safety's test doubles).
std::vector<rclcpp::Parameter> baseParams(const fs::path & bag_root)
{
  return {
    rclcpp::Parameter("bag_root", bag_root.string()),
    rclcpp::Parameter("storage_id", std::string("mcap")),
    rclcpp::Parameter("compression_mode", std::string("none")),
    rclcpp::Parameter("compression_format", std::string("none")),
    rclcpp::Parameter("max_bagfile_size_bytes", static_cast<int64_t>(64 * 1024 * 1024)),
    rclcpp::Parameter("max_bagfile_duration_sec", static_cast<int64_t>(300)),
    rclcpp::Parameter("max_cache_size_bytes", static_cast<int64_t>(1024 * 1024)),
    rclcpp::Parameter("max_total_bytes", 1.0e12),
    rclcpp::Parameter("min_free_bytes", 1024.0),       // small: real disk always clears this
    rclcpp::Parameter("retention_period_sec", 30.0),   // long: most tests don't want a tick firing
    rclcpp::Parameter("write_error_limit", static_cast<int64_t>(50)),
    rclcpp::Parameter("record_px4_topics", true),
    // C5: must match the default uav_id ("uav0") -- KnownGoodConfigDoesNotThrow
    // exercises the full validation path unmodified, including the
    // topic-namespace check.
    rclcpp::Parameter(
      "topics", std::vector<std::string>{"/uav/uav0/test/topic_a"}),
    rclcpp::Parameter("types", std::vector<std::string>{"std_msgs/msg/String"}),
    rclcpp::Parameter("qos_profiles", std::vector<std::string>{"be"}),
    rclcpp::Parameter("depths", std::vector<int64_t>{10}),
  };
}

/// Replaces the single-topic array trio in baseParams() with a caller-given
/// set -- every array override is applied together so length stays in sync.
void setTopics(
  std::vector<rclcpp::Parameter> & params, const std::vector<std::string> & topics,
  const std::vector<std::string> & types, const std::vector<std::string> & qos_profiles,
  const std::vector<int64_t> & depths)
{
  for (rclcpp::Parameter & param : params) {
    if (param.get_name() == "topics") {param = rclcpp::Parameter("topics", topics);}
    if (param.get_name() == "types") {param = rclcpp::Parameter("types", types);}
    if (param.get_name() == "qos_profiles") {param = rclcpp::Parameter("qos_profiles", qos_profiles);}
    if (param.get_name() == "depths") {param = rclcpp::Parameter("depths", depths);}
  }
}

void overrideParam(std::vector<rclcpp::Parameter> & params, const rclcpp::Parameter & replacement)
{
  for (rclcpp::Parameter & param : params) {
    if (param.get_name() == replacement.get_name()) {
      param = replacement;
      return;
    }
  }
  params.push_back(replacement);
}

}  // namespace

// ===========================================================================
// (a) + (f): boots, records two real topics, round-trips through Reader,
// and the diagnostics counters agree with what actually happened.
// ===========================================================================

TEST(RosbagManagerNode, RecordsTwoTopicsAndDiagnosticsAndBagAgreeAfterRoundTrip)
{
  TempDir bag_root("roundtrip");
  const std::string uav_id = nextUavId();
  const std::string topic_a = "/uav/" + uav_id + "/test/topic_a";
  const std::string topic_b = "/uav/" + uav_id + "/test/topic_b";

  auto params = baseParams(bag_root.path());
  setTopics(
    params, {topic_a, topic_b}, {"std_msgs/msg/String", "std_msgs/msg/Int32"}, {"be", "be"},
    {10, 10});

  Harness harness(uav_id, params);

  auto pub_a = harness.probe()->create_publisher<std_msgs::msg::String>(topic_a, rclcpp::QoS(10));
  auto pub_b = harness.probe()->create_publisher<std_msgs::msg::Int32>(topic_b, rclcpp::QoS(10));

  ASSERT_TRUE(
    waitFor(
      [&]() {return pub_a->get_subscription_count() > 0 && pub_b->get_subscription_count() > 0;},
      10.0))
    << "FAILED TO MEASURE: recorder never subscribed to either test topic";

  constexpr int kRounds = 8;
  for (int i = 0; i < kRounds; ++i) {
    std_msgs::msg::String msg_a;
    msg_a.data = "round " + std::to_string(i);
    pub_a->publish(msg_a);
    std_msgs::msg::Int32 msg_b;
    msg_b.data = i;
    pub_b->publish(msg_b);
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(harness.waitForDiagnostics(5.0)) << "FAILED TO MEASURE: no diagnostics tick observed";
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticNum(harness.lastDiagnostics(), "messages_written") >= 2 * kRounds;},
      5.0))
    << "FAILED TO MEASURE: messages_written never reached " << 2 * kRounds;

  const DiagnosticStatus diag = harness.lastDiagnostics();
  EXPECT_EQ(diagnosticStr(diag, "state"), "RECORDING");
  EXPECT_EQ(diagnosticNum(diag, "topics_subscribed"), 2.0);
  EXPECT_EQ(diagnosticNum(diag, "topics_failed"), 0.0);
  EXPECT_EQ(diagnosticNum(diag, "topics_total"), 2.0);
  EXPECT_GT(diagnosticNum(diag, "bytes_written"), 0.0);

  harness.shutdownNode();   // flush + close the writer before reading it back

  std::vector<fs::path> bag_dirs;
  for (const auto & entry : fs::directory_iterator(bag_root.path())) {
    if (entry.is_directory()) {
      bag_dirs.push_back(entry.path());
    }
  }
  ASSERT_EQ(bag_dirs.size(), 1u) << "expected exactly one bag directory";

  rosbag2_cpp::Reader reader;
  ASSERT_NO_THROW(reader.open(bag_dirs.front().string()));
  const auto metadata = reader.get_metadata();
  EXPECT_GE(metadata.message_count, static_cast<uint64_t>(2 * kRounds));

  bool saw_a = false;
  bool saw_b = false;
  for (const auto & topic_info : metadata.topics_with_message_count) {
    if (topic_info.topic_metadata.name == topic_a) {
      saw_a = topic_info.message_count > 0;
    }
    if (topic_info.topic_metadata.name == topic_b) {
      saw_b = topic_info.message_count > 0;
    }
  }
  EXPECT_TRUE(saw_a) << "topic_a never made it into the bag";
  EXPECT_TRUE(saw_b) << "topic_b never made it into the bag";
}

// ===========================================================================
// (g) P10.8c: offered_qos_profiles round-trips through metadata.yaml with
// the RIGHT durability PER TOPIC -- a reliable_tl topic decodes back to
// TRANSIENT_LOCAL, and a be topic (negative control, same test) decodes to
// VOLATILE, not just "non-empty". Before the P10.8c fix this field was "" for
// every topic; C3 (this pass) additionally requires a REAL publisher to be
// seen (offered_qos_profiles now reflects the ACTUAL publisher, not just our
// own subscribed_qos -- see OfferedQosReflectsRealPublisherNotOurOwnBestEffort
// Subscription for the case where they DISAGREE), so this test now publishes
// one message per topic with a publisher QoS matching the recorder's own
// config -- the common case where subscriber and publisher agree.
// ===========================================================================

TEST(RosbagManagerNode, OfferedQosProfilesEncodesDurabilityPerTopic)
{
  TempDir bag_root("qosmeta");
  const std::string uav_id = nextUavId();
  const std::string topic_tl = "/uav/" + uav_id + "/test/topic_tl";
  const std::string topic_be = "/uav/" + uav_id + "/test/topic_be";

  auto params = baseParams(bag_root.path());
  setTopics(
    params, {topic_tl, topic_be}, {"std_msgs/msg/String", "std_msgs/msg/String"},
    {"reliable_tl", "be"}, {10, 10});

  Harness harness(uav_id, params);
  ASSERT_TRUE(harness.waitForDiagnostics(5.0)) << "FAILED TO MEASURE: no diagnostics tick observed";
  ASSERT_EQ(diagnosticNum(harness.lastDiagnostics(), "topics_subscribed"), 2.0);

  // C3: create_topic() (and the QoS DECISION) is now deferred to the first
  // real message per topic -- a publisher is required for either topic to
  // even appear in the bag's metadata.
  auto pub_tl = harness.probe()->create_publisher<std_msgs::msg::String>(
    topic_tl, rclcpp::QoS(10).reliable().transient_local());
  auto pub_be = harness.probe()->create_publisher<std_msgs::msg::String>(
    topic_be, rclcpp::QoS(10).best_effort());
  ASSERT_TRUE(
    waitFor(
      [&]() {
        return pub_tl->get_subscription_count() > 0 && pub_be->get_subscription_count() > 0;
      }, 10.0))
    << "FAILED TO MEASURE: recorder never subscribed to either topic";
  for (int i = 0; i < 5; ++i) {
    std_msgs::msg::String msg;
    msg.data = "round " + std::to_string(i);
    pub_tl->publish(msg);
    pub_be->publish(msg);
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticNum(harness.lastDiagnostics(), "messages_written") >= 2.0;}, 5.0))
    << "FAILED TO MEASURE: messages never counted as written";

  harness.shutdownNode();   // flush + close the writer before reading it back

  std::vector<fs::path> bag_dirs;
  for (const auto & entry : fs::directory_iterator(bag_root.path())) {
    if (entry.is_directory()) {
      bag_dirs.push_back(entry.path());
    }
  }
  ASSERT_EQ(bag_dirs.size(), 1u) << "expected exactly one bag directory";

  rosbag2_cpp::Reader reader;
  ASSERT_NO_THROW(reader.open(bag_dirs.front().string()));
  const auto metadata = reader.get_metadata();

  std::string yaml_tl;
  std::string yaml_be;
  for (const auto & topic_info : metadata.topics_with_message_count) {
    if (topic_info.topic_metadata.name == topic_tl) {
      yaml_tl = topic_info.topic_metadata.offered_qos_profiles;
    }
    if (topic_info.topic_metadata.name == topic_be) {
      yaml_be = topic_info.topic_metadata.offered_qos_profiles;
    }
  }

  ASSERT_FALSE(yaml_tl.empty()) << "P10.8c bug: offered_qos_profiles empty for the reliable_tl topic";
  ASSERT_FALSE(yaml_be.empty()) << "P10.8c bug: offered_qos_profiles empty for the be topic";

  std::vector<rosbag2_transport::Rosbag2QoS> decoded_tl;
  std::vector<rosbag2_transport::Rosbag2QoS> decoded_be;
  ASSERT_NO_THROW(decoded_tl = YAML::Load(yaml_tl).as<std::vector<rosbag2_transport::Rosbag2QoS>>());
  ASSERT_NO_THROW(decoded_be = YAML::Load(yaml_be).as<std::vector<rosbag2_transport::Rosbag2QoS>>());
  ASSERT_EQ(decoded_tl.size(), 1u);
  ASSERT_EQ(decoded_be.size(), 1u);

  EXPECT_EQ(decoded_tl.front().durability(), rclcpp::DurabilityPolicy::TransientLocal)
    << "reliable_tl topic must decode back to TRANSIENT_LOCAL, or a late `ros2 bag play` "
       "subscriber never receives the latched sample it exists to preserve";
  EXPECT_EQ(decoded_tl.front().reliability(), rclcpp::ReliabilityPolicy::Reliable);

  // Negative control: the be topic must NOT come back TRANSIENT_LOCAL --
  // proves the encode is genuinely per-topic, not a constant string.
  EXPECT_EQ(decoded_be.front().durability(), rclcpp::DurabilityPolicy::Volatile)
    << "be topic decoded as TRANSIENT_LOCAL -- offered_qos_profiles is not per-topic";
  EXPECT_EQ(decoded_be.front().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
}

// ===========================================================================
// (b) a bad type degrades ONLY that topic; the node stays up and keeps
// recording the healthy topic. Positive control: same config minus the
// bad topic -> topics_failed == 0.
// ===========================================================================

TEST(RosbagManagerNode, OneBadTypeCountsAsTopicsFailedWithoutStoppingTheOtherTopic)
{
  TempDir bag_root("badtype");
  const std::string uav_id = nextUavId();
  const std::string topic_good = "/uav/" + uav_id + "/test/topic_a";
  const std::string topic_bad = "/uav/" + uav_id + "/test/topic_bad";

  auto params = baseParams(bag_root.path());
  setTopics(
    params, {topic_good, topic_bad},
    {"std_msgs/msg/String", "totally_bogus_package/msg/DoesNotExist"}, {"be", "be"}, {10, 10});

  Harness harness(uav_id, params);
  ASSERT_TRUE(harness.waitForDiagnostics(5.0)) << "FAILED TO MEASURE: no diagnostics tick observed";

  const DiagnosticStatus diag = harness.lastDiagnostics();
  EXPECT_EQ(diagnosticNum(diag, "topics_total"), 2.0);
  EXPECT_EQ(diagnosticNum(diag, "topics_subscribed"), 1.0);
  EXPECT_EQ(diagnosticNum(diag, "topics_failed"), 1.0);
  // The node must stay alive and keep RECORDING on the surviving topic --
  // one bad topic is never fatal (O1).
  EXPECT_EQ(diagnosticStr(diag, "state"), "RECORDING");
}

TEST(RosbagManagerNode, SameConfigWithoutTheBadTopicHasZeroTopicsFailed)
{
  TempDir bag_root("goodtype");
  const std::string uav_id = nextUavId();
  const std::string topic_good = "/uav/" + uav_id + "/test/topic_a";

  auto params = baseParams(bag_root.path());
  setTopics(params, {topic_good}, {"std_msgs/msg/String"}, {"be"}, {10});

  Harness harness(uav_id, params);
  ASSERT_TRUE(harness.waitForDiagnostics(5.0)) << "FAILED TO MEASURE: no diagnostics tick observed";

  const DiagnosticStatus diag = harness.lastDiagnostics();
  EXPECT_EQ(diagnosticNum(diag, "topics_failed"), 0.0);
  EXPECT_EQ(diagnosticNum(diag, "topics_subscribed"), 1.0);
}

// ===========================================================================
// (c) disk starved AT BOOT -> DISABLED_NO_SPACE, no bag directory ever
// created, node stays alive.
// ===========================================================================

TEST(RosbagManagerNode, AbsurdMinFreeBytesDisablesRecordingWithoutCreatingABagDir)
{
  TempDir bag_root("nospace");
  const std::string uav_id = nextUavId();
  const std::string topic = "/uav/" + uav_id + "/test/topic_a";

  auto params = baseParams(bag_root.path());
  setTopics(params, {topic}, {"std_msgs/msg/String"}, {"be"}, {10});
  overrideParam(params, rclcpp::Parameter("min_free_bytes", 1.0e18));   // 1 EiB: unreachable

  Harness harness(uav_id, params);
  ASSERT_TRUE(harness.waitForDiagnostics(5.0)) << "FAILED TO MEASURE: no diagnostics tick observed";

  const DiagnosticStatus diag = harness.lastDiagnostics();
  EXPECT_EQ(diagnosticStr(diag, "state"), "DISABLED_NO_SPACE");
  // Subscriptions are an INDEPENDENT concern from disk space (S:2.1) --
  // topics_subscribed still counts, only the writer never opened.
  EXPECT_EQ(diagnosticNum(diag, "topics_subscribed"), 1.0);

  size_t bag_dir_count = 0;
  for (const auto & entry : fs::directory_iterator(bag_root.path())) {
    if (entry.is_directory()) {
      ++bag_dir_count;
    }
  }
  EXPECT_EQ(bag_dir_count, 0u) << "no bag directory should exist when boot-time disk is insufficient";
}

// ===========================================================================
// (d) retention deletes the OLDEST non-active bag directory once the budget
// is exceeded; the actively-recording bag is never touched.
// ===========================================================================

TEST(RosbagManagerNode, RetentionDeletesOnlyTheOldestNonActiveBagOnceOverBudget)
{
  TempDir bag_root("retention");
  const std::string uav_id = nextUavId();
  const std::string topic = "/uav/" + uav_id + "/test/topic_a";

  const auto now = std::chrono::system_clock::now();
  // C1(e): must be prefixed uav_id + "_" to be a real deletion candidate --
  // makeFakeBagDir() also now leaves a metadata.yaml marker so these pass
  // looksLikeBagDir() too.
  makeFakeBagDir(bag_root.path(), uav_id + "_old_fake", now - 300s, 2000);
  makeFakeBagDir(bag_root.path(), uav_id + "_mid_fake", now - 200s, 2000);
  makeFakeBagDir(bag_root.path(), uav_id + "_new_fake", now - 100s, 2000);

  auto params = baseParams(bag_root.path());
  setTopics(params, {topic}, {"std_msgs/msg/String"}, {"be"}, {10});
  overrideParam(params, rclcpp::Parameter("retention_period_sec", 0.5));
  // 3 fake dirs = 6000 bytes; the freshly-opened active bag starts near 0.
  // Deleting exactly the oldest (2000) brings the total under 5000.
  overrideParam(params, rclcpp::Parameter("max_total_bytes", 5000.0));

  Harness harness(uav_id, params);

  ASSERT_TRUE(
    waitFor(
      [&]() {return !fs::exists(bag_root.path() / (uav_id + "_old_fake"));}, 10.0))
    << "FAILED TO MEASURE: retention never deleted the oldest bag";

  EXPECT_TRUE(fs::exists(bag_root.path() / (uav_id + "_mid_fake")))
    << "retention over-deleted past the budget";
  EXPECT_TRUE(fs::exists(bag_root.path() / (uav_id + "_new_fake")))
    << "retention over-deleted past the budget";

  // old_fake, mid_fake, new_fake, AND the real active bag ALL share the
  // uav_id + "_" prefix now (C1(e)) -- the surviving count must be exactly
  // 3 (mid_fake + new_fake + the actively-recording bag), proving the real
  // active bag specifically was never touched, not just "something" survived.
  size_t remaining_dirs = 0;
  for (const auto & entry : fs::directory_iterator(bag_root.path())) {
    if (entry.is_directory() && entry.path().filename().string().rfind(uav_id + "_", 0) == 0) {
      ++remaining_dirs;
    }
  }
  EXPECT_EQ(remaining_dirs, 3u)
    << "expected mid_fake + new_fake + the actively-recording bag to all survive";

  ASSERT_TRUE(harness.waitForDiagnostics(5.0));
  EXPECT_GE(diagnosticNum(harness.lastDiagnostics(), "bags_deleted"), 1.0);
}

// ===========================================================================
// (e) validate(): one test per rule (R33) -- each mutates ONE field off a
// known-good base and expects the constructor to refuse to start. A
// positive control proves the base config itself is NOT the thing throwing.
// ===========================================================================

class RosbagManagerValidateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    bag_root_ = std::make_unique<TempDir>("validate");
    params_ = baseParams(bag_root_->path());
  }

  std::unique_ptr<TempDir> bag_root_;
  std::vector<rclcpp::Parameter> params_;
};

TEST_F(RosbagManagerValidateTest, KnownGoodConfigDoesNotThrow)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_NO_THROW({RosbagManagerNode node(options);});
}

TEST_F(RosbagManagerValidateTest, MismatchedArrayLengthsThrows)
{
  // types has one FEWER entry than topics -- V-O2 style rule.
  setTopics(
    params_, {"/uav/t/a", "/uav/t/b"}, {"std_msgs/msg/String"}, {"be", "be"}, {10, 10});
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, DuplicateTopicNamesThrows)
{
  setTopics(
    params_, {"/uav/t/a", "/uav/t/a"}, {"std_msgs/msg/String", "std_msgs/msg/String"},
    {"be", "be"}, {10, 10});
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, UnknownStorageIdThrows)
{
  overrideParam(params_, rclcpp::Parameter("storage_id", std::string("not_a_real_storage_id")));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, NegativeMinFreeBytesThrows)
{
  overrideParam(params_, rclcpp::Parameter("min_free_bytes", -1.0));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, NanMaxTotalBytesThrows)
{
  overrideParam(
    params_, rclcpp::Parameter("max_total_bytes", std::numeric_limits<double>::quiet_NaN()));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, InvalidQosProfileValueThrows)
{
  setTopics(params_, {"/uav/t/a"}, {"std_msgs/msg/String"}, {"sometimes"}, {10});
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, ZeroDepthThrows)
{
  setTopics(params_, {"/uav/t/a"}, {"std_msgs/msg/String"}, {"be"}, {0});
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, EmptyTopicsArrayThrows)
{
  setTopics(params_, {}, {}, {}, {});
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, CompressionModeWithoutMatchingFormatThrows)
{
  overrideParam(params_, rclcpp::Parameter("compression_mode", std::string("file")));
  overrideParam(params_, rclcpp::Parameter("compression_format", std::string("none")));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

// ===========================================================================
// C5: every "/uav/*" topic must belong to THIS node's uav_id_. A yaml wired
// to the wrong drone used to subscribe fine (no publisher required) and
// report RECORDING while the bag silently held nothing real.
// ===========================================================================

TEST_F(RosbagManagerValidateTest, TopicNotMatchingUavIdNamespaceThrows)
{
  overrideParam(params_, rclcpp::Parameter("uav_id", std::string("uav0")));
  setTopics(params_, {"/uav/uav1/test/topic_a"}, {"std_msgs/msg/String"}, {"be"}, {10});
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, TfAndFmuTopicsAreExemptFromUavIdNamespaceCheck)
{
  overrideParam(params_, rclcpp::Parameter("uav_id", std::string("uav0")));
  setTopics(
    params_, {"/tf", "/tf_static", "/fmu/out/vehicle_status"},
    {"tf2_msgs/msg/TFMessage", "tf2_msgs/msg/TFMessage", "std_msgs/msg/String"},
    {"be", "be", "be"}, {10, 10, 10});
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_NO_THROW({RosbagManagerNode node(options);});
}

// ===========================================================================
// C1(e): bag_root resolving to $HOME or "/" must refuse to start -- a full
// disk deleting whole directories there would be catastrophic.
// ===========================================================================

TEST_F(RosbagManagerValidateTest, BagRootEqualToHomeThrows)
{
  const char * home = std::getenv("HOME");
  ASSERT_NE(home, nullptr);
  overrideParam(params_, rclcpp::Parameter("bag_root", std::string(home)));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

TEST_F(RosbagManagerValidateTest, BagRootEqualToFilesystemRootThrows)
{
  overrideParam(params_, rclcpp::Parameter("bag_root", std::string("/")));
  rclcpp::NodeOptions options;
  options.parameter_overrides(params_);
  EXPECT_THROW({RosbagManagerNode node(options);}, std::invalid_argument);
}

// ===========================================================================
// C1(a): fs::space() failing at boot must report the diagnostic free-space
// reading as "cannot measure" (NaN), never a misleading 0.0 ("disk
// completely full"). Deterministic, portable trigger: bag_root's parent is
// a REGULAR FILE, not a directory (ENOTDIR), same failure family as a
// flaky SD card returning EIO from fs::space().
// ===========================================================================

TEST(RosbagManagerNode, DiskSpaceQueryFailureAtBootReportsNanNotZeroFreeBytes)
{
  TempDir root_container("spacefail");
  const fs::path blocker_file = root_container.path() / "blocker";
  {
    std::ofstream f(blocker_file.string());
    f << "x";
  }
  const fs::path bag_root = blocker_file / "bags";   // a FILE, not a directory, as ancestor

  const std::string uav_id = nextUavId();
  auto params = baseParams(bag_root);
  setTopics(params, {"/uav/" + uav_id + "/test/topic_a"}, {"std_msgs/msg/String"}, {"be"}, {10});
  overrideParam(params, rclcpp::Parameter("min_free_bytes", 1024.0));

  Harness harness(uav_id, params);
  ASSERT_TRUE(harness.waitForDiagnostics(5.0)) << "FAILED TO MEASURE: no diagnostics tick observed";

  const DiagnosticStatus diag = harness.lastDiagnostics();
  EXPECT_EQ(diagnosticStr(diag, "state"), "DISABLED_NO_SPACE")
    << "an unmeasurable disk must still refuse to start writing (fail-safe at boot)";
  EXPECT_TRUE(std::isnan(diagnosticNum(diag, "disk_free_bytes")))
    << "fs::space() FAILING must report disk_free_bytes=NaN ('cannot measure'), never 0.0 "
       "('the disk is definitely full') -- R30";
}

// ===========================================================================
// C1(c): no active bag to protect (DISABLED_NO_SPACE at boot, bag_dir_
// empty) -- retention must never run at all, or an empty active_path used
// to match nothing, turning EVERY pre-existing bag (evidence from a
// PREVIOUS flight) into a normal deletion candidate.
// ===========================================================================

TEST(RosbagManagerNode, DisabledNoSpaceAtBootNeverRunsRetentionAgainstPreExistingEvidence)
{
  TempDir bag_root("noretention");
  const std::string uav_id = nextUavId();
  const auto now = std::chrono::system_clock::now();
  makeFakeBagDir(bag_root.path(), uav_id + "_old_evidence", now - 300s, 2000);

  auto params = baseParams(bag_root.path());
  setTopics(params, {"/uav/" + uav_id + "/test/topic_a"}, {"std_msgs/msg/String"}, {"be"}, {10});
  overrideParam(params, rclcpp::Parameter("min_free_bytes", 1.0e18));   // boot fails -- DISABLED_NO_SPACE
  overrideParam(params, rclcpp::Parameter("retention_period_sec", 0.2));   // several ticks quickly

  Harness harness(uav_id, params);
  ASSERT_TRUE(harness.waitForDiagnostics(5.0));
  ASSERT_EQ(diagnosticStr(harness.lastDiagnostics(), "state"), "DISABLED_NO_SPACE");

  std::this_thread::sleep_for(1500ms);   // let several retention ticks elapse
  EXPECT_TRUE(fs::exists(bag_root.path() / (uav_id + "_old_evidence")))
    << "retention must never run when there is no active bag to protect (bag_dir_ empty)";
}

// ===========================================================================
// C1(e): retention must only ever touch a directory that both (a) belongs
// to THIS uav_id and (b) actually looks like a bag -- a full disk must
// never delete unrelated directories under bag_root.
// ===========================================================================

TEST(RosbagManagerNode, RetentionNeverDeletesForeignOrNonBagDirectories)
{
  TempDir bag_root("foreign");
  const std::string uav_id = nextUavId();
  const auto now = std::chrono::system_clock::now();
  makeFakeBagDir(bag_root.path(), "px4_logs", now - 300s, 5000);   // foreign prefix, old, large
  fs::create_directories(bag_root.path() / (uav_id + "_incomplete"));   // ours, but no bag content

  auto params = baseParams(bag_root.path());
  setTopics(params, {"/uav/" + uav_id + "/test/topic_a"}, {"std_msgs/msg/String"}, {"be"}, {10});
  overrideParam(params, rclcpp::Parameter("retention_period_sec", 0.3));
  // Absurdly tight -- forces a real deletion attempt on every tick.
  overrideParam(params, rclcpp::Parameter("max_total_bytes", 1.0));

  Harness harness(uav_id, params);
  std::this_thread::sleep_for(1500ms);

  EXPECT_TRUE(fs::exists(bag_root.path() / "px4_logs"))
    << "a directory with a foreign prefix must never be treated as a deletable bag";
  EXPECT_TRUE(fs::exists(bag_root.path() / (uav_id + "_incomplete")))
    << "a same-prefix directory with no bag content markers must never be deleted";
}

// ===========================================================================
// C3: offered_qos_profiles must reflect the REAL publisher's QoS, not just
// what this recorder itself subscribed with -- several control topics
// deliberately subscribe "be" (Q-P10-2, zero backpressure) while their real
// production publishers offer Reliable; recording "be" broke replay for a
// Reliable-only consumer (observed live: arbiter got 0 cmd_mission messages
// from a replayed bag, ~/gate_logs/o3_play_loop.log, RELIABILITY_QOS_POLICY).
// ===========================================================================

TEST(RosbagManagerNode, OfferedQosReflectsRealPublisherNotOurOwnBestEffortSubscription)
{
  TempDir bag_root("qospublisher");
  const std::string uav_id = nextUavId();
  const std::string topic = "/uav/" + uav_id + "/test/cmd_like_topic";

  auto params = baseParams(bag_root.path());
  // Same shape as cmd_mission/cmd_operator/cmd_safety/cmd_test: recorder
  // subscribes "be", but the real publisher below offers Reliable.
  setTopics(params, {topic}, {"std_msgs/msg/String"}, {"be"}, {10});

  Harness harness(uav_id, params);

  auto real_publisher =
    harness.probe()->create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(10).reliable());
  ASSERT_TRUE(
    waitFor([&]() {return real_publisher->get_subscription_count() > 0;}, 10.0))
    << "FAILED TO MEASURE: recorder never subscribed to the test topic";

  constexpr int kRounds = 5;
  for (int i = 0; i < kRounds; ++i) {
    std_msgs::msg::String msg;
    msg.data = "round " + std::to_string(i);
    real_publisher->publish(msg);
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_TRUE(harness.waitForDiagnostics(5.0)) << "FAILED TO MEASURE: no diagnostics tick observed";
  ASSERT_TRUE(
    waitFor(
      [&]() {return diagnosticNum(harness.lastDiagnostics(), "messages_written") >= 1.0;}, 5.0))
    << "FAILED TO MEASURE: message never counted as written";

  harness.shutdownNode();

  std::vector<fs::path> bag_dirs;
  for (const auto & entry : fs::directory_iterator(bag_root.path())) {
    if (entry.is_directory()) {
      bag_dirs.push_back(entry.path());
    }
  }
  ASSERT_EQ(bag_dirs.size(), 1u);

  rosbag2_cpp::Reader reader;
  ASSERT_NO_THROW(reader.open(bag_dirs.front().string()));
  const auto metadata = reader.get_metadata();

  std::string yaml_qos;
  for (const auto & topic_info : metadata.topics_with_message_count) {
    if (topic_info.topic_metadata.name == topic) {
      yaml_qos = topic_info.topic_metadata.offered_qos_profiles;
    }
  }
  ASSERT_FALSE(yaml_qos.empty());

  std::vector<rosbag2_transport::Rosbag2QoS> decoded;
  ASSERT_NO_THROW(decoded = YAML::Load(yaml_qos).as<std::vector<rosbag2_transport::Rosbag2QoS>>());
  ASSERT_EQ(decoded.size(), 1u);
  EXPECT_EQ(decoded.front().reliability(), rclcpp::ReliabilityPolicy::Reliable)
    << "offered_qos_profiles must reflect the REAL publisher (Reliable), not this node's own "
       "BestEffort subscribed_qos -- recording the wrong one is exactly why a Reliable-only "
       "replay consumer received nothing (observed live in the o3-loop gate logs)";
}

}  // namespace uav_observability
