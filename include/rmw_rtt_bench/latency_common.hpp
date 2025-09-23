#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <unistd.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

namespace rmw_rtt_bench {

inline std::optional<bool> parse_bool(const std::string & s) {
  if (s == "1" || s == "true" || s == "True" || s == "TRUE" || s == "yes" || s == "on") return true;
  if (s == "0" || s == "false" || s == "False" || s == "FALSE" || s == "no" || s == "off") return false;
  return std::nullopt;
}

inline std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return s;
}

struct QoSOptions {
  std::string reliability{"best_effort"};
  std::string history{"keep_last"};
  int depth{10};
};

inline bool build_qos(const QoSOptions & opts, rclcpp::QoS & qos_out, std::string & error) {
  using ReliabilityPolicy = rclcpp::ReliabilityPolicy;
  using HistoryPolicy = rclcpp::HistoryPolicy;

  std::string rel = to_lower(opts.reliability);
  std::string hist = to_lower(opts.history);

  HistoryPolicy history_policy;
  if (hist == "keep_last") {
    history_policy = HistoryPolicy::KeepLast;
  } else if (hist == "keep_all") {
    history_policy = HistoryPolicy::KeepAll;
  } else {
    error = "Unknown qos-history: " + opts.history + " (expected keep_last|keep_all)";
    return false;
  }

  ReliabilityPolicy rel_policy;
  if (rel == "best_effort") {
    rel_policy = ReliabilityPolicy::BestEffort;
  } else if (rel == "reliable") {
    rel_policy = ReliabilityPolicy::Reliable;
  } else {
    error = "Unknown qos-reliability: " + opts.reliability + " (expected best_effort|reliable)";
    return false;
  }

  // Depth only matters for KeepLast
  const int depth = opts.depth > 0 ? opts.depth : 1;
  rclcpp::QoS qos(depth);
  if (history_policy == HistoryPolicy::KeepAll) {
    qos.keep_all();
  } else {
    qos.keep_last(depth);
  }
  qos.reliability(rel_policy);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);

  qos_out = qos;
  return true;
}

inline std::string get_hostname() {
  char buf[256];
  buf[0] = '\0';
  if (gethostname(buf, sizeof(buf)) == 0) {
    buf[sizeof(buf) - 1] = '\0';
    return std::string(buf);
  }
  return std::string("unknown");
}

inline std::string get_rmw_implementation() {
  const char * id = rmw_get_implementation_identifier();
  if (id && id[0] != '\0') return std::string(id);
  const char * env = std::getenv("RMW_IMPLEMENTATION");
  if (env && env[0] != '\0') return std::string(env);
  return std::string("unknown");
}

// ParsedArgs と一方向引数パーサはRTT専用化に伴い削除しました。

} // namespace rmw_rtt_bench
