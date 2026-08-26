#include "uav_observability/event_ledger.hpp"

#include <cstdio>
#include <sstream>

namespace uav_observability
{

namespace
{

std::string formatDouble(double value)
{
  std::ostringstream out;
  out.precision(17);
  out << value;
  return out.str();
}

const char * levelName(EventLevel level)
{
  switch (level) {
    case EventLevel::kInfo: return "INFO";
    case EventLevel::kWarn: return "WARN";
    case EventLevel::kError: return "ERROR";
    default: return "UNKNOWN";
  }
}

}  // namespace

std::string escapeJson(const std::string & value)
{
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

bool EventLedger::FieldKey::operator<(const FieldKey & other) const
{
  if (src != other.src) {
    return src < other.src;
  }
  return field < other.field;
}

EventLedger::EventLedger(double fsync_period_sec)
: fsync_period_sec_(fsync_period_sec)
{
}

std::optional<std::string> EventLedger::observe(
  const std::string & src, const std::string & field, const std::string & new_value,
  EventLevel level, double t_sim, double t_wall,
  const std::map<std::string, std::string> & detail)
{
  const FieldKey key{src, field};
  const auto it = last_value_.find(key);
  const bool first_sighting = (it == last_value_.end());
  const std::string from = first_sighting ? "" : it->second;

  if (!first_sighting && it->second == new_value) {
    return std::nullopt;   // no edge -- no line, no seq spent
  }
  last_value_[key] = new_value;

  const uint64_t seq = next_seq_++;

  std::ostringstream line;
  line << "{";
  line << "\"t_sim\":" << formatDouble(t_sim) << ",";
  line << "\"t_wall\":" << formatDouble(t_wall) << ",";
  line << "\"seq\":" << seq << ",";
  line << "\"src\":\"" << escapeJson(src) << "\",";
  line << "\"field\":\"" << escapeJson(field) << "\",";
  line << "\"from\":\"" << escapeJson(from) << "\",";
  line << "\"to\":\"" << escapeJson(new_value) << "\",";
  line << "\"level\":\"" << levelName(level) << "\",";
  line << "\"detail\":{";
  bool first_detail = true;
  for (const auto & [detail_key, detail_value] : detail) {
    if (!first_detail) {
      line << ",";
    }
    first_detail = false;
    line << "\"" << escapeJson(detail_key) << "\":\"" << escapeJson(detail_value) << "\"";
  }
  line << "}";
  line << "}";
  return line.str();
}

bool EventLedger::needsFsync(EventLevel level, double t_wall) const
{
  if (level >= EventLevel::kError) {
    return true;
  }
  return (t_wall - last_fsync_t_wall_) >= fsync_period_sec_;
}

void EventLedger::markFsynced(double t_wall)
{
  last_fsync_t_wall_ = t_wall;
}

}  // namespace uav_observability
