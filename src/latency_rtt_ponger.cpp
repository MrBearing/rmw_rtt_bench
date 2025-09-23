#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "rmw_rtt_bench/latency_common.hpp"
#include "rmw_rtt_bench/msg/rtt.hpp"

namespace rlb = rmw_rtt_bench;
using rmw_rtt_bench::msg::Rtt;

struct PongerArgs {
  std::string req_topic{"/latency_rtt_req"};
  std::string rep_topic{"/latency_rtt_rep"};
  rlb::QoSOptions qos;
  int duration_sec{35};
  bool intra_process{false};
  std::string transport_tag;
  std::string notes;
};

static bool parse_ponger_args(std::vector<std::string> args, PongerArgs & out, std::string & error) {
  auto get_value = [&](size_t & i) -> std::optional<std::string> { if (i + 1 >= args.size()) return std::nullopt; return args[++i]; };
  for (size_t i = 0; i < args.size(); ++i) {
    const std::string & a = args[i];
    if (a == "--req-topic") { auto v = get_value(i); if (!v) { error = "--req-topic requires value"; return false; } out.req_topic = *v; }
    else if (a == "--rep-topic") { auto v = get_value(i); if (!v) { error = "--rep-topic requires value"; return false; } out.rep_topic = *v; }
    else if (a == "--qos-reliability") { auto v = get_value(i); if (!v) { error = "--qos-reliability requires value"; return false; } out.qos.reliability = *v; }
    else if (a == "--qos-history") { auto v = get_value(i); if (!v) { error = "--qos-history requires value"; return false; } out.qos.history = *v; }
    else if (a == "--qos-depth") { auto v = get_value(i); if (!v) { error = "--qos-depth requires value"; return false; } try { out.qos.depth = std::stoi(*v); } catch (...) { error = "--qos-depth expects integer"; return false; } }
    else if (a == "--duration") { auto v = get_value(i); if (!v) { error = "--duration requires value"; return false; } try { out.duration_sec = std::stoi(*v); } catch (...) { error = "--duration expects integer seconds"; return false; } }
    else if (a == "--intra-process") { auto v = get_value(i); if (!v) { error = "--intra-process requires value"; return false; } auto b = rlb::parse_bool(*v); if (!b) { error = "--intra-process expects true|false"; return false; } out.intra_process = *b; }
    else if (a == "--transport-tag") { auto v = get_value(i); if (!v) { error = "--transport-tag requires value"; return false; } out.transport_tag = *v; }
    else if (a == "--notes") { auto v = get_value(i); if (!v) { error = "--notes requires value"; return false; } out.notes = *v; }
  }
  if (out.duration_sec <= 0) { error = "--duration must be > 0"; return false; }
  rclcpp::QoS dummy(10); std::string err; if (!rlb::build_qos(out.qos, dummy, err)) { error = err; return false; }
  return true;
}

class PongerNode : public rclcpp::Node {
public:
  explicit PongerNode(const PongerArgs & cfg, const rclcpp::NodeOptions & options)
  : Node("latency_rtt_ponger", options), cfg_(cfg) {
    std::string err; if (!rlb::build_qos(cfg_.qos, qos_, err)) { throw std::runtime_error(err); }
    sub_req_ = this->create_subscription<Rtt>(cfg_.req_topic, qos_, std::bind(&PongerNode::on_request, this, std::placeholders::_1));
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

private:
  void on_request(const Rtt::SharedPtr req) {
    auto t1 = this->get_clock()->now();
    Rtt rep;
    rep.seq = req->seq;
    rep.t0_pub_send = req->t0_pub_send;
    rep.t1_sub_recv = t1;
    rep.t2_sub_send = this->get_clock()->now();
    rep.payload_size_bytes = req->payload_size_bytes;
    rep.payload.resize(rep.payload_size_bytes, 0);
    try { pub_rep_->publish(std::move(rep)); } catch (const std::exception & e) { RCLCPP_ERROR(get_logger(), "Publish reply failed: %s", e.what()); }
  }

  PongerArgs cfg_;
  rclcpp::QoS qos_{10};
  rclcpp::Subscription<Rtt>::SharedPtr sub_req_;
  rclcpp::Publisher<Rtt>::SharedPtr pub_rep_;
  rclcpp::TimerBase::SharedPtr end_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::InitOptions init_options{};
  rclcpp::init(argc, argv, init_options);
  auto remaining = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<std::string> args(remaining.begin() + 1, remaining.end());

  PongerArgs cfg; std::string error;
  if (!parse_ponger_args(args, cfg, error)) {
    RCLCPP_ERROR(rclcpp::get_logger("rtt_ponger"), "%s", error.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rtt_ponger"),
      "Usage: ros2 run rmw_rtt_bench rtt_ponger -- [--req-topic <name>] [--rep-topic <name>] [--qos-reliability best_effort|reliable] [--qos-history keep_last|keep_all] [--qos-depth <n>] [--duration <sec>] [--intra-process true|false] [--transport-tag <str>] [--notes <str>]");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::NodeOptions options; options.use_intra_process_comms(cfg.intra_process);
  try {
    auto node = std::make_shared<PongerNode>(cfg, options);
    rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node); exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rtt_ponger"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
