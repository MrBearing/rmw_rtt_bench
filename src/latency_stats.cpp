#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace rmw_rtt_bench {

struct LatencyStatsResult {
  size_t count{0};
  std::optional<size_t> expected_count;
  double mean_ms{0.0};
  double median_ms{0.0};
  double p95_ms{0.0};
  double p99_ms{0.0};
  double max_ms{0.0};
  double stddev_ms{0.0};
  double drop_rate{-1.0};
};

static double ns_to_ms(double ns) { return ns / 1e6; }

static LatencyStatsResult compute_latency_stats_internal(const std::vector<int64_t> & lat_ns, std::optional<size_t> expected) {
  LatencyStatsResult res;
  res.count = lat_ns.size();
  res.expected_count = expected;
  if (expected && *expected > 0) {
    res.drop_rate = 1.0 - (static_cast<double>(res.count) / static_cast<double>(*expected));
  }
  if (lat_ns.empty()) {
    return res;
  }

  std::vector<double> v;
  v.reserve(lat_ns.size());
  for (auto ns : lat_ns) v.push_back(static_cast<double>(ns));

  double sum = 0.0;
  for (double x : v) sum += x;
  double mean_ns = sum / static_cast<double>(v.size());

  std::vector<double> sorted = v;
  std::sort(sorted.begin(), sorted.end());
  auto percentile = [&](double p)->double{
    if (sorted.empty()) return NAN;
    double idx = (p / 100.0) * (sorted.size() - 1);
    size_t i = static_cast<size_t>(std::floor(idx));
    size_t j = static_cast<size_t>(std::ceil(idx));
    if (i == j) return sorted[i];
    double frac = idx - static_cast<double>(i);
    return sorted[i] * (1.0 - frac) + sorted[j] * frac;
  };

  double median_ns = percentile(50.0);
  double p95_ns = percentile(95.0);
  double p99_ns = percentile(99.0);
  double max_ns = sorted.back();

  double var = 0.0;
  for (double x : v) {
    double d = x - mean_ns;
    var += d * d;
  }
  var /= static_cast<double>(v.size());
  double stddev_ns = std::sqrt(var);

  res.mean_ms = ns_to_ms(mean_ns);
  res.median_ms = ns_to_ms(median_ns);
  res.p95_ms = ns_to_ms(p95_ns);
  res.p99_ms = ns_to_ms(p99_ns);
  res.max_ms = ns_to_ms(max_ns);
  res.stddev_ms = ns_to_ms(stddev_ns);
  return res;
}

static void print_latency_stats(const LatencyStatsResult & s) {
  auto logger = rclcpp::get_logger("latency_stats");
  if (s.expected_count) {
    RCLCPP_INFO(logger, "count=%zu expected=%zu drop_rate=%.4f", s.count, *s.expected_count, s.drop_rate);
  } else {
    RCLCPP_INFO(logger, "count=%zu", s.count);
  }
  RCLCPP_INFO(logger, "mean=%.6f ms median=%.6f ms p95=%.6f ms p99=%.6f ms max=%.6f ms stddev=%.6f ms",
              s.mean_ms, s.median_ms, s.p95_ms, s.p99_ms, s.max_ms, s.stddev_ms);
}

} // namespace rmw_rtt_bench

// Expose C-style wrappers in global namespace for straightforward linkage
extern "C" {
  struct LatencyStatsC {
    size_t count;
    size_t expected_count;
    int has_expected;
    double mean_ms, median_ms, p95_ms, p99_ms, max_ms, stddev_ms;
    double drop_rate;
  };

  LatencyStatsC compute_latency_stats(const int64_t * data, size_t n, int has_expected, size_t expected) {
    std::vector<int64_t> v;
    v.reserve(n);
    for (size_t i = 0; i < n; ++i) v.push_back(data[i]);
    auto res = rmw_rtt_bench::compute_latency_stats_internal(v, has_expected ? std::optional<size_t>(expected) : std::nullopt);
    LatencyStatsC out{};
    out.count = res.count;
    out.has_expected = res.expected_count.has_value() ? 1 : 0;
    out.expected_count = res.expected_count.value_or(0);
    out.mean_ms = res.mean_ms;
    out.median_ms = res.median_ms;
    out.p95_ms = res.p95_ms;
    out.p99_ms = res.p99_ms;
    out.max_ms = res.max_ms;
    out.stddev_ms = res.stddev_ms;
    out.drop_rate = res.drop_rate;
    return out;
  }

  void print_latency_stats_c(LatencyStatsC s) {
    rmw_rtt_bench::LatencyStatsResult r{};
    r.count = s.count;
    if (s.has_expected) r.expected_count = s.expected_count;
    r.mean_ms = s.mean_ms;
    r.median_ms = s.median_ms;
    r.p95_ms = s.p95_ms;
    r.p99_ms = s.p99_ms;
    r.max_ms = s.max_ms;
    r.stddev_ms = s.stddev_ms;
    r.drop_rate = s.drop_rate;
    rmw_rtt_bench::print_latency_stats(r);
  }
}
