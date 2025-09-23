#include <chrono>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "rmw_rtt_bench/latency_common.hpp"
#include "rmw_rtt_bench/msg/rtt.hpp"

namespace rlb = rmw_rtt_bench;
using rmw_rtt_bench::msg::Rtt;
using namespace std::chrono_literals;

struct RttArgs {
  std::string role{"pinger"}; // pinger or ponger
  std::string req_topic{"/latency_rtt_req"};
  std::string rep_topic{"/latency_rtt_rep"};
  rlb::QoSOptions qos;
  // pinger
  double hz{100.0};
  int payload_size{0};
  int duration_sec{30};
  long long trials{-1};
  int timeout_sec{-1};
  bool intra_process{false};
  std::string csv_path{"latency_rtt.csv"};
  std::string transport_tag;
  std::string notes;
};

static bool parse_rtt_args(std::vector<std::string> args, RttArgs & out, std::string & error) {
  auto get_value = [&](size_t & i) -> std::optional<std::string> {
    if (i + 1 >= args.size()) return std::nullopt; return args[++i];
  };
  for (size_t i = 0; i < args.size(); ++i) {
    const std::string & a = args[i];
    if (a == "--role") { auto v = get_value(i); if (!v) { error = "--role requires value"; return false; } out.role = rlb::to_lower(*v);
    } else if (a == "--req-topic") { auto v = get_value(i); if (!v) { error = "--req-topic requires value"; return false; } out.req_topic = *v;
    } else if (a == "--rep-topic") { auto v = get_value(i); if (!v) { error = "--rep-topic requires value"; return false; } out.rep_topic = *v;
    } else if (a == "--qos-reliability") { auto v = get_value(i); if (!v) { error = "--qos-reliability requires value"; return false; } out.qos.reliability = *v;
    } else if (a == "--qos-history") { auto v = get_value(i); if (!v) { error = "--qos-history requires value"; return false; } out.qos.history = *v;
    } else if (a == "--qos-depth") { auto v = get_value(i); if (!v) { error = "--qos-depth requires value"; return false; } try { out.qos.depth = std::stoi(*v); } catch (...) { error = "--qos-depth expects integer"; return false; }
    } else if (a == "--hz") { auto v = get_value(i); if (!v) { error = "--hz requires value"; return false; } try { out.hz = std::stod(*v); } catch (...) { error = "--hz expects number"; return false; }
    } else if (a == "--payload-size") { auto v = get_value(i); if (!v) { error = "--payload-size requires value"; return false; } try { out.payload_size = std::stoi(*v); } catch (...) { error = "--payload-size expects integer"; return false; }
    } else if (a == "--duration") { auto v = get_value(i); if (!v) { error = "--duration requires value"; return false; } try { out.duration_sec = std::stoi(*v); } catch (...) { error = "--duration expects integer seconds"; return false; }
    } else if (a == "--trials") { auto v = get_value(i); if (!v) { error = "--trials requires value"; return false; } try { out.trials = std::stoll(*v); } catch (...) { error = "--trials expects integer"; return false; }
    } else if (a == "--timeout") { auto v = get_value(i); if (!v) { error = "--timeout requires value"; return false; } try { out.timeout_sec = std::stoi(*v); } catch (...) { error = "--timeout expects integer seconds"; return false; }
    } else if (a == "--intra-process") { auto v = get_value(i); if (!v) { error = "--intra-process requires value"; return false; } auto b = rlb::parse_bool(*v); if (!b) { error = "--intra-process expects true|false"; return false; } out.intra_process = *b;
    } else if (a == "--csv") { auto v = get_value(i); if (!v) { error = "--csv requires value"; return false; } out.csv_path = *v;
    } else if (a == "--transport-tag") { auto v = get_value(i); if (!v) { error = "--transport-tag requires value"; return false; } out.transport_tag = *v;
    } else if (a == "--notes") { auto v = get_value(i); if (!v) { error = "--notes requires value"; return false; } out.notes = *v;
    }
  }
  if (out.role != "pinger" && out.role != "ponger") { error = "--role must be pinger|ponger"; return false; }
  if (out.hz <= 0.0 && out.role == "pinger") { error = "--hz must be > 0"; return false; }
  if (out.trials == 0) { error = "--trials must be > 0 when provided"; return false; }
  if (out.duration_sec <= 0) { error = "--duration must be > 0"; return false; }
  if (out.timeout_sec == 0) { error = "--timeout must be > 0 when provided"; return false; }
  rclcpp::QoS dummy(10); std::string err; if (!rlb::build_qos(out.qos, dummy, err)) { error = err; return false; }
  return true;
}

class RttNode : public rclcpp::Node {
public:
  explicit RttNode(const RttArgs & cfg, const rclcpp::NodeOptions & options)
  : Node("latency_rtt", options), cfg_(cfg) {
    if (!rlb::build_qos(cfg_.qos, qos_, err_)) { throw std::runtime_error(err_); }
    if (cfg_.role == "pinger") init_pinger(); else init_ponger();
  }

private:
  // ---------- Pinger ----------
  void init_pinger() {
    pub_req_ = this->create_publisher<Rtt>(cfg_.req_topic, qos_);
    sub_rep_ = this->create_subscription<Rtt>(cfg_.rep_topic, qos_,
      std::bind(&RttNode::on_reply, this, std::placeholders::_1));

    // CSV
    namespace fs = std::filesystem;
    try {
      fs::path out_path(cfg_.csv_path);
      if (!out_path.parent_path().empty()) {
        fs::create_directories(out_path.parent_path());
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to ensure CSV directory: %s", e.what());
    }
    csv_.open(cfg_.csv_path, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) throw std::runtime_error("Failed to open CSV: " + cfg_.csv_path);
    csv_ << "seq,t0_ns,t1_ns,t2_ns,t3_ns,rtt_ns,proc_ns,oneway_est_ns,payload_size,rmw,qos_rel,qos_hist,qos_depth,transport_tag,host,notes\n";
    csv_.flush();

    RCLCPP_INFO(get_logger(), "RTT pinger: req=%s rep=%s hz=%.3f payload=%d depth=%d rel=%s hist=%s duration=%ds trials=%lld timeout=%ds intra=%s",
      cfg_.req_topic.c_str(), cfg_.rep_topic.c_str(), cfg_.hz, cfg_.payload_size, cfg_.qos.depth,
      cfg_.qos.reliability.c_str(), cfg_.qos.history.c_str(), cfg_.duration_sec, cfg_.trials, cfg_.timeout_sec,
      cfg_.intra_process?"true":"false");

    start_time_ = std::chrono::steady_clock::now();
    auto d1 = start_time_ + std::chrono::seconds(cfg_.duration_sec);
    if (cfg_.timeout_sec > 0) deadline_ = std::min(d1, start_time_ + std::chrono::seconds(cfg_.timeout_sec));
    else deadline_ = d1;

    const auto period = std::chrono::duration<double>(1.0 / cfg_.hz);
    timer_pub_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RttNode::on_tick, this));

    // Safety end timer
    auto secs_left = std::chrono::duration_cast<std::chrono::seconds>(deadline_ - start_time_);
    end_timer_ = this->create_wall_timer(secs_left, [this]() {
      RCLCPP_INFO(this->get_logger(), "Time cap reached. Stopping pinger.");
      timer_pub_->cancel();
      rclcpp::shutdown();
    });
  }

  void on_tick() {
    if (cfg_.trials > 0 && static_cast<long long>(seq_) >= cfg_.trials) {
      RCLCPP_INFO(this->get_logger(), "Trial count reached (%lld). Stopping pinger.", cfg_.trials);
      timer_pub_->cancel();
      rclcpp::shutdown();
      return;
    }
    if (std::chrono::steady_clock::now() >= deadline_) {
      timer_pub_->cancel();
      rclcpp::shutdown();
      return;
    }
    Rtt msg;
    msg.seq = static_cast<uint32_t>(seq_++);
    msg.t0_pub_send = this->get_clock()->now();
    msg.t1_sub_recv = builtin_interfaces::msg::Time(); // zero
    msg.t2_sub_send = builtin_interfaces::msg::Time(); // zero
    msg.payload_size_bytes = static_cast<uint32_t>(cfg_.payload_size);
    msg.payload.resize(cfg_.payload_size, 0);
    try { pub_req_->publish(std::move(msg)); }
    catch (const std::exception & e) { RCLCPP_ERROR(get_logger(), "Publish request failed: %s", e.what()); }
  }

  void on_reply(const Rtt::SharedPtr rep) {
    auto t3 = this->get_clock()->now();
    int64_t t0 = rclcpp::Time(rep->t0_pub_send).nanoseconds();
    int64_t t1 = rclcpp::Time(rep->t1_sub_recv).nanoseconds();
    int64_t t2 = rclcpp::Time(rep->t2_sub_send).nanoseconds();
    int64_t t3ns = t3.nanoseconds();
    int64_t rtt = t3ns - t0;
    int64_t proc = (t2 - t1);
    int64_t oneway_est = (rtt >= proc) ? (rtt - proc) / 2 : -1;

    // record
    csv_ << rep->seq << ','
         << t0 << ',' << t1 << ',' << t2 << ',' << t3ns << ','
         << rtt << ',' << proc << ',' << oneway_est << ','
         << rep->payload_size_bytes << ','
         << rlb::get_rmw_implementation() << ','
         << cfg_.qos.reliability << ','
         << cfg_.qos.history << ','
         << cfg_.qos.depth << ','
         << cfg_.transport_tag << ','
         << rlb::get_hostname() << ','
         << cfg_.notes << '\n';

    latencies_ns_.push_back(rtt);
    if (++lines_since_flush_ % 1000 == 0) csv_.flush();
  }

  // ---------- Ponger ----------
  void init_ponger() {
    sub_req_ = this->create_subscription<Rtt>(cfg_.req_topic, qos_,
      std::bind(&RttNode::on_request, this, std::placeholders::_1));
    pub_rep_ = this->create_publisher<Rtt>(cfg_.rep_topic, qos_);

    RCLCPP_INFO(get_logger(), "RTT ponger: req=%s rep=%s depth=%d rel=%s hist=%s duration=%ds intra=%s",
      cfg_.req_topic.c_str(), cfg_.rep_topic.c_str(), cfg_.qos.depth,
      cfg_.qos.reliability.c_str(), cfg_.qos.history.c_str(), cfg_.duration_sec,
      cfg_.intra_process?"true":"false");

    end_timer_ = this->create_wall_timer(std::chrono::seconds(cfg_.duration_sec), [this]() {
      RCLCPP_INFO(this->get_logger(), "Duration reached. Stopping ponger.");
      rclcpp::shutdown();
    });
  }

  void on_request(const Rtt::SharedPtr req) {
    auto t1 = this->get_clock()->now();
    Rtt rep;
    rep.seq = req->seq;
    rep.t0_pub_send = req->t0_pub_send;
    rep.t1_sub_recv = t1;
    rep.t2_sub_send = this->get_clock()->now(); // as close to publish as practical
    // Mirror payload size to keep wire size similar
    rep.payload_size_bytes = req->payload_size_bytes;
    rep.payload.resize(rep.payload_size_bytes, 0);
    try { pub_rep_->publish(std::move(rep)); }
    catch (const std::exception & e) { RCLCPP_ERROR(get_logger(), "Publish reply failed: %s", e.what()); }
  }

private:
  RttArgs cfg_;
  rclcpp::QoS qos_{10};
  std::string err_;

  // pinger
  rclcpp::Publisher<Rtt>::SharedPtr pub_req_;
  rclcpp::Subscription<Rtt>::SharedPtr sub_rep_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  rclcpp::TimerBase::SharedPtr end_timer_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point deadline_;
  uint64_t seq_{0};
  std::ofstream csv_;
  size_t lines_since_flush_{0};
  std::vector<int64_t> latencies_ns_;

  // ponger
  rclcpp::Subscription<Rtt>::SharedPtr sub_req_;
  rclcpp::Publisher<Rtt>::SharedPtr pub_rep_;
};

int main(int argc, char ** argv) {
  rclcpp::InitOptions init_options{};
  rclcpp::init(argc, argv, init_options);
  auto remaining = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<std::string> args(remaining.begin() + 1, remaining.end());

  RttArgs cfg;
  std::string error;
  if (!parse_rtt_args(args, cfg, error)) {
    RCLCPP_ERROR(rclcpp::get_logger("ros2_latency_rtt"), "%s", error.c_str());
    RCLCPP_INFO(rclcpp::get_logger("ros2_latency_rtt"),
      "Usage: ros2 run ros2_latency_bench ros2_latency_rtt -- --role pinger|ponger [--req-topic <name>] [--rep-topic <name>] [--qos-reliability best_effort|reliable] [--qos-history keep_last|keep_all] [--qos-depth <n>] [--hz <rate>] [--payload-size <bytes>] [--duration <sec>] [--trials <count>] [--timeout <sec>] [--intra-process true|false] [--csv <path>] [--transport-tag <str>] [--notes <str>]");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(cfg.intra_process);

  try {
    auto node = std::make_shared<RttNode>(cfg, options);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ros2_latency_rtt"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
