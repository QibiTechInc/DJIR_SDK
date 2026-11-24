// ROS 2 driver for DJI RS 2 (DJI R SDK) gimbals over Linux SocketCAN.
//
// Publishes
//   joint_states            sensor_msgs/JointState     yaw/roll/pitch joint (encoder) angles [rad]
//   ~/attitude              geometry_msgs/Vector3Stamped  attitude angles [rad] (x=roll, y=pitch, z=yaw)
//   /diagnostics            CAN link + gimbal status
// Subscribes
//   ~/joint_command         sensor_msgs/JointState     position command [rad]; optional
//                                                      per-joint velocity [rad/s] sets move speed.
//                                                      absolute mode: target joint angles, axes whose
//                                                        value did not change are not sent.
//                                                      incremental mode: relative moves, axes with 0
//                                                        (or missing) are not sent.
//   ~/velocity_command      sensor_msgs/JointState     joint speeds [rad/s] (velocity field)
// Services
//   ~/recenter              std_srvs/Trigger           gimbal Recenter
//   ~/calibrate             std_srvs/SetBool           start (true) / stop (false) auto calibration
//   ~/reset                 std_srvs/Trigger           reopen CAN, clear counters and command cache
//   ~/set_absolute_mode     std_srvs/SetBool           true: absolute (default), false: incremental
// Parameter "command_mode" (absolute|incremental) can also be changed at runtime.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "DJIR_SDK.h"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;
using sensor_msgs::msg::JointState;
using SteadyClock = std::chrono::steady_clock;

namespace {

enum Axis { YAW = 0, ROLL = 1, PITCH = 2 };
constexpr double kTenthDegToRad = M_PI / 1800.0;

int16_t rad_to_tenth_deg(double rad, int16_t limit)
{
  double v = std::round(rad / kTenthDegToRad);
  v = std::max<double>(-limit, std::min<double>(limit, v));
  return static_cast<int16_t>(v);
}

double tenth_deg_to_rad(int16_t v) { return v * kTenthDegToRad; }

double seconds_since(SteadyClock::time_point t)
{
  return std::chrono::duration<double>(SteadyClock::now() - t).count();
}

const char * calib_state_str(DJIR_SDK::CalibrationState s)
{
  switch (s) {
    case DJIR_SDK::CalibrationState::IDLE: return "idle";
    case DJIR_SDK::CalibrationState::RUNNING: return "running";
    case DJIR_SDK::CalibrationState::COMPLETED: return "completed";
    case DJIR_SDK::CalibrationState::FAILED: return "failed";
  }
  return "unknown";
}

}  // namespace

class DjirDriverNode : public rclcpp::Node
{
public:
  DjirDriverNode()
  : Node("djir_driver"), diag_(this)
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"gimbal_yaw_joint", "gimbal_roll_joint", "gimbal_pitch_joint"});
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    publish_attitude_ = declare_parameter<bool>("publish_attitude", true);
    use_push_ = declare_parameter<bool>("use_push", true);
    command_mode_ = declare_parameter<std::string>("command_mode", "absolute");
    default_move_time_ = declare_parameter<double>("default_move_time", 1.0);
    min_move_time_ = declare_parameter<double>("min_move_time", 0.1);
    max_velocity_ = declare_parameter<double>("max_velocity", 0.0);
    angle_limit_ = declare_parameter<double>("command_angle_limit", M_PI);
    velocity_timeout_ = declare_parameter<double>("velocity_timeout", 0.5);
    max_command_velocity_ = declare_parameter<double>("max_command_velocity", 2.0 * M_PI);
    reply_timeout_ms_ = declare_parameter<int>("reply_timeout_ms", 100);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 1.0);
    reconnect_period_ = declare_parameter<double>("reconnect_period", 2.0);

    if (joint_names_.size() != 3) {
      throw std::runtime_error("joint_names must have 3 entries: yaw, roll, pitch");
    }
    if (command_mode_ != "absolute" && command_mode_ != "incremental") {
      throw std::runtime_error("command_mode must be 'absolute' or 'incremental'");
    }
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return on_set_parameters(params);});
    publish_rate_ = std::max(1.0, publish_rate_);
    min_move_time_ = std::max(0.1, min_move_time_);
    angle_limit_tenth_ = static_cast<int16_t>(
      std::min(1800.0, std::abs(angle_limit_) / kTenthDegToRad));

    joint_state_pub_ = create_publisher<JointState>("joint_states", 10);
    if (publish_attitude_) {
      attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("~/attitude", 10);
    }

    joint_cmd_sub_ = create_subscription<JointState>(
      "~/joint_command", 10, [this](JointState::ConstSharedPtr m) {on_joint_command(*m);});
    velocity_cmd_sub_ = create_subscription<JointState>(
      "~/velocity_command", 10, [this](JointState::ConstSharedPtr m) {on_velocity_command(*m);});

    recenter_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/recenter",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {on_recenter(*res);});
    calibrate_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/calibrate",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {on_calibrate(*req, *res);});
    mode_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_absolute_mode",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        const std::string mode = req->data ? "absolute" : "incremental";
        auto r = set_parameter(rclcpp::Parameter("command_mode", mode));
        res->success = r.successful;
        res->message = r.successful ? "command_mode = " + mode : r.reason;
      });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {on_reset(*res);});

    diag_.setHardwareID("DJI R SDK gimbal @ " + can_interface_);
    diag_.add("CAN link", this, &DjirDriverNode::diagnose_link);
    diag_.add("Gimbal", this, &DjirDriverNode::diagnose_gimbal);

    open_connection();

    feedback_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_), [this]() {on_feedback_timer();});
    velocity_timer_ = create_wall_timer(100ms, [this]() {on_velocity_timer();});

    RCLCPP_INFO(
      get_logger(), "DJI gimbal driver on %s (command_mode=%s, %.0f Hz)",
      can_interface_.c_str(), command_mode_.c_str(), publish_rate_);
  }

  ~DjirDriverNode() override
  {
    if (velocity_active_) {
      release_velocity_control();
    }
    gimbal_.disconnect();
  }

private:
  // ---------------------------------------------------------------- connection
  bool open_connection()
  {
    last_connect_attempt_ = SteadyClock::now();
    if (!gimbal_.connect(can_interface_)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Cannot open SocketCAN interface '%s'. Bring it up with: "
        "sudo ip link set %s up type can bitrate 1000000",
        can_interface_.c_str(), can_interface_.c_str());
      return false;
    }
    gimbal_.set_reply_timeout(reply_timeout_ms_);
    push_requested_ = false;
    version_.clear();
    return true;
  }

  // ------------------------------------------------------------------ feedback
  void on_feedback_timer()
  {
    if (!gimbal_.is_connected()) {
      if (seconds_since(last_connect_attempt_) >= reconnect_period_) {
        open_connection();
      }
      return;
    }

    // Ask the gimbal to push its parameters (retry until it answers).
    if (use_push_ && !push_requested_ && seconds_since(last_push_request_) > 1.0) {
      last_push_request_ = SteadyClock::now();
      push_requested_ = gimbal_.set_push_enabled(true);
    }
    if (version_.empty() && seconds_since(last_version_request_) > 5.0) {
      last_version_request_ = SteadyClock::now();
      uint32_t v = 0;
      if (gimbal_.get_version(v)) {
        char buf[32];
        std::snprintf(
          buf, sizeof(buf), "%u.%u.%u.%u", (v >> 24) & 0xFF, (v >> 16) & 0xFF,
          (v >> 8) & 0xFF, v & 0xFF);
        version_ = buf;
      }
    }

    DJIR_SDK::Angles joint, att;
    bool have_joint = false, have_att = false;

    DJIR_SDK::GimbalPushData push;
    const double max_push_age = std::max(0.1, 3.0 / publish_rate_);
    if (use_push_ && gimbal_.get_push_data(push) && push.angles_valid &&
      seconds_since(push.stamp) < max_push_age && push.count != last_push_count_)
    {
      last_push_count_ = push.count;
      joint = push.joint;
      att = push.attitude;
      have_joint = have_att = true;
      feedback_source_ = "push";
    } else if (!(use_push_ && push.count > 0 && seconds_since(push.stamp) < max_push_age)) {
      // No fresh push: poll.
      have_joint = gimbal_.get_joint_angles(joint.yaw, joint.roll, joint.pitch);
      if (publish_attitude_) {
        have_att = gimbal_.get_attitude_angles(att.yaw, att.roll, att.pitch);
      }
      feedback_source_ = "poll";
    }

    const auto stamp = now();
    if (have_joint) {
      current_joint_ = {joint.yaw, joint.roll, joint.pitch};
      have_feedback_ = true;
      last_feedback_ = SteadyClock::now();

      JointState js;
      js.header.stamp = stamp;
      js.name = joint_names_;
      js.position = {
        tenth_deg_to_rad(joint.yaw), tenth_deg_to_rad(joint.roll), tenth_deg_to_rad(joint.pitch)};
      joint_state_pub_->publish(js);
    }
    if (have_att && attitude_pub_) {
      geometry_msgs::msg::Vector3Stamped v;
      v.header.stamp = stamp;
      v.vector.x = tenth_deg_to_rad(att.roll);
      v.vector.y = tenth_deg_to_rad(att.pitch);
      v.vector.z = tenth_deg_to_rad(att.yaw);
      attitude_pub_->publish(v);
    }
  }

  // ---------------------------------------------------------- position command
  int joint_index(const std::string & name) const
  {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (joint_names_[i] == name) {return static_cast<int>(i);}
    }
    return -1;
  }

  // Extract per-axis values from a JointState field. Empty names => yaw, roll, pitch order.
  bool extract(
    const JointState & msg, const std::vector<double> & values,
    std::array<bool, 3> & has, std::array<double, 3> & out)
  {
    has = {false, false, false};
    if (msg.name.empty()) {
      for (size_t i = 0; i < 3 && i < values.size(); ++i) {
        if (std::isfinite(values[i])) {has[i] = true; out[i] = values[i];}
      }
      return true;
    }
    for (size_t i = 0; i < msg.name.size() && i < values.size(); ++i) {
      int idx = joint_index(msg.name[i]);
      if (idx < 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Unknown joint '%s' in command",
          msg.name[i].c_str());
        continue;
      }
      if (std::isfinite(values[i])) {has[idx] = true; out[idx] = values[i];}
    }
    return true;
  }

  void on_joint_command(const JointState & msg)
  {
    std::array<bool, 3> has_pos, has_vel;
    std::array<double, 3> pos{}, vel{};
    extract(msg, msg.position, has_pos, pos);
    extract(msg, msg.velocity, has_vel, vel);

    if (velocity_active_) {
      release_velocity_control();
    }

    const bool absolute = command_mode_ == "absolute";

    // value[a]: absolute target or relative delta (0.1 deg); changed[a]: send this axis
    std::array<int16_t, 3> value{0, 0, 0};
    std::array<bool, 3> changed{false, false, false};
    bool any = false;
    for (int a = 0; a < 3; ++a) {
      if (!has_pos[a]) {continue;}
      value[a] = rad_to_tenth_deg(pos[a], angle_limit_tenth_);
      changed[a] = absolute ?
        (!last_cmd_valid_[a] || value[a] != last_cmd_[a]) :
        value[a] != 0;
      any |= changed[a];
    }
    if (!any) {
      return;  // nothing to move: do not send anything
    }

    // Move time: from per-joint velocity or max_velocity, else default_move_time.
    double move_time = default_move_time_;
    double t = -1.0;
    for (int a = 0; a < 3; ++a) {
      if (!changed[a]) {continue;}
      double v = (has_vel[a] && vel[a] > 0.0) ? vel[a] : max_velocity_;
      if (v <= 0.0) {continue;}
      double dist;
      if (!absolute) {
        dist = std::abs(tenth_deg_to_rad(value[a]));
      } else if (have_feedback_) {
        dist = std::abs(tenth_deg_to_rad(value[a] - current_joint_[a]));
      } else {
        continue;  // unknown distance
      }
      t = std::max(t, dist / v);
    }
    if (t >= 0.0) {move_time = t;}
    move_time = std::max(min_move_time_, std::min(25.5, move_time));
    uint16_t time_ms = static_cast<uint16_t>(std::lround(move_time * 1000.0));

    bool ok = gimbal_.move_axes(
      value[YAW], value[ROLL], value[PITCH], time_ms,
      absolute ? DJIR_SDK::MoveMode::ABSOLUTE_CONTROL : DJIR_SDK::MoveMode::INCREMENTAL_CONTROL,
      changed[YAW], changed[ROLL], changed[PITCH]);

    last_cmd_result_ = DJIR_SDK::to_string(gimbal_.last_return_code());
    last_cmd_ok_ = ok;
    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Position command failed: %s",
        last_cmd_result_.c_str());
      return;  // keep the cache so the next message retries
    }
    if (absolute) {
      for (int a = 0; a < 3; ++a) {
        if (changed[a]) {last_cmd_[a] = value[a]; last_cmd_valid_[a] = true;}
      }
    } else {
      invalidate_command_cache();  // the position moved; absolute targets must be re-sent
    }
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      if (p.get_name() == "command_mode") {
        const std::string mode = p.as_string();
        if (mode != "absolute" && mode != "incremental") {
          result.successful = false;
          result.reason = "command_mode must be 'absolute' or 'incremental'";
          return result;
        }
        if (mode != command_mode_) {
          command_mode_ = mode;
          invalidate_command_cache();
          RCLCPP_INFO(get_logger(), "Position command mode: %s", mode.c_str());
        }
      }
    }
    return result;
  }

  void invalidate_command_cache() {last_cmd_valid_ = {false, false, false};}

  // ---------------------------------------------------------- velocity command
  void on_velocity_command(const JointState & msg)
  {
    std::array<bool, 3> has;
    std::array<double, 3> vel{};
    extract(msg, msg.velocity, has, vel);

    const int16_t lim = static_cast<int16_t>(
      std::min(3600.0, std::abs(max_command_velocity_) / kTenthDegToRad));
    for (int a = 0; a < 3; ++a) {
      velocity_cmd_[a] = has[a] ? rad_to_tenth_deg(vel[a], lim) : 0;
    }
    last_velocity_cmd_ = SteadyClock::now();

    if (!velocity_active_) {
      gimbal_.set_speed_mode(DJIR_SDK::SpeedControl::ENABLED, DJIR_SDK::FocalControl::DISABLED);
      velocity_active_ = true;
    }
    send_velocity();
    invalidate_command_cache();  // the gimbal is moving away from the last position target
  }

  void send_velocity()
  {
    bool ok = gimbal_.set_speed(velocity_cmd_[YAW], velocity_cmd_[ROLL], velocity_cmd_[PITCH]);
    last_cmd_result_ = DJIR_SDK::to_string(gimbal_.last_return_code());
    last_cmd_ok_ = ok;
  }

  void release_velocity_control()
  {
    velocity_cmd_ = {0, 0, 0};
    send_velocity();  // stop
    gimbal_.set_speed_mode(DJIR_SDK::SpeedControl::DISABLED, DJIR_SDK::FocalControl::DISABLED);
    gimbal_.set_speed(0, 0, 0);  // release speed control
    velocity_active_ = false;
  }

  // The gimbal stops a speed command after 0.5 s, so re-send while commands keep coming.
  void on_velocity_timer()
  {
    if (!velocity_active_) {return;}
    if (seconds_since(last_velocity_cmd_) > velocity_timeout_) {
      release_velocity_control();
    } else {
      send_velocity();
    }
  }

  // ------------------------------------------------------------------ services
  void on_recenter(std_srvs::srv::Trigger::Response & res)
  {
    if (velocity_active_) {release_velocity_control();}
    res.success = gimbal_.recenter();
    res.message = res.success ? "Recenter started" :
      std::string("Recenter failed: ") + DJIR_SDK::to_string(gimbal_.last_return_code());
    invalidate_command_cache();
    RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  }

  void on_calibrate(
    const std_srvs::srv::SetBool::Request & req, std_srvs::srv::SetBool::Response & res)
  {
    if (velocity_active_) {release_velocity_control();}
    res.success = gimbal_.set_auto_calibration(req.data);
    if (res.success) {
      res.message = req.data ?
        "Auto calibration started; the gimbal will move. Progress is in /diagnostics." :
        "Auto calibration stopped";
    } else {
      res.message = std::string("Auto calibration command failed: ") +
        DJIR_SDK::to_string(gimbal_.last_return_code());
    }
    invalidate_command_cache();
    RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  }

  void on_reset(std_srvs::srv::Trigger::Response & res)
  {
    if (velocity_active_) {release_velocity_control();}
    gimbal_.disconnect();
    invalidate_command_cache();
    have_feedback_ = false;
    last_push_count_ = 0;
    last_cmd_ok_ = true;
    last_cmd_result_ = "none";
    link_snapshot_ = DJIR_SDK::LinkStats();
    res.success = open_connection();
    if (!res.success) {
      res.message = "Could not reopen " + can_interface_;
      return;
    }
    // Probe the gimbal so the caller knows whether it answers.
    int16_t y, r, p;
    if (gimbal_.get_joint_angles(y, r, p)) {
      res.message = "Reconnected; gimbal is responding";
    } else {
      res.success = false;
      auto s = gimbal_.get_link_stats();
      res.message = "CAN reopened but gimbal does not respond (bus " + s.bus_state + ")";
      if (s.bus_state == "BUS-OFF" || s.bus_state == "ERROR-PASSIVE") {
        res.message += ". Restart the CAN interface: sudo ip link set " + can_interface_ +
          " down && sudo ip link set " + can_interface_ + " up type can bitrate 1000000";
      }
    }
    RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  }

  // --------------------------------------------------------------- diagnostics
  void diagnose_link(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    auto s = gimbal_.get_link_stats();
    const uint64_t new_timeouts = s.timeouts - std::min(s.timeouts, link_snapshot_.timeouts);
    const uint64_t new_acks = s.can_ack_errors -
      std::min(s.can_ack_errors, link_snapshot_.can_ack_errors);
    const uint64_t new_tx_fail = s.tx_failures -
      std::min(s.tx_failures, link_snapshot_.tx_failures);
    const uint64_t new_crc = s.crc_errors - std::min(s.crc_errors, link_snapshot_.crc_errors);
    link_snapshot_ = s;

    if (!s.connected) {
      stat.summary(DiagnosticStatus::ERROR, "CAN socket not open");
    } else if (!s.interface_up) {
      stat.summary(DiagnosticStatus::ERROR, can_interface_ + " is down");
    } else if (s.bus_state == "BUS-OFF") {
      stat.summary(DiagnosticStatus::ERROR, "CAN bus-off (restart the interface)");
    } else if (s.bus_state == "ERROR-PASSIVE" || new_acks > 0) {
      stat.summary(
        DiagnosticStatus::WARN,
        "CAN frames not acknowledged (gimbal off or wiring/termination problem)");
    } else if (new_tx_fail > 0) {
      stat.summary(DiagnosticStatus::WARN, "CAN transmit failures");
    } else if (new_timeouts > 0) {
      stat.summary(DiagnosticStatus::WARN, "Gimbal did not reply to some requests");
    } else if (new_crc > 0) {
      stat.summary(DiagnosticStatus::WARN, "Packets with CRC errors received");
    } else {
      stat.summary(DiagnosticStatus::OK, "OK");
    }

    stat.add("interface", can_interface_);
    stat.add("interface up", s.interface_up);
    stat.add("bus state", s.bus_state);
    stat.add("tx packets", s.tx_packets);
    stat.add("tx failures", s.tx_failures);
    stat.add("rx packets", s.rx_packets);
    stat.add("crc errors", s.crc_errors);
    stat.add("reply timeouts", s.timeouts);
    stat.add("can error frames", s.can_error_frames);
    stat.add("can ack errors", s.can_ack_errors);
    stat.add("last rx age [s]", s.any_rx ? seconds_since(s.last_rx) : -1.0);
  }

  void diagnose_gimbal(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const double age = have_feedback_ ? seconds_since(last_feedback_) : -1.0;
    const auto calib = gimbal_.get_calibration_status();

    if (!have_feedback_) {
      stat.summary(DiagnosticStatus::ERROR, "No feedback from gimbal");
    } else if (age > feedback_timeout_) {
      stat.summary(DiagnosticStatus::ERROR, "Gimbal feedback stale");
    } else {
      stat.summary(DiagnosticStatus::OK, "OK");
    }
    if (calib.received && calib.state == DJIR_SDK::CalibrationState::FAILED) {
      stat.mergeSummary(DiagnosticStatus::ERROR, "Auto calibration failed");
    } else if (calib.received && calib.state == DJIR_SDK::CalibrationState::RUNNING) {
      stat.mergeSummary(
        DiagnosticStatus::WARN, "Auto calibration running (" +
        std::to_string(calib.progress) + "%)");
    }
    if (!last_cmd_ok_) {
      stat.mergeSummary(DiagnosticStatus::WARN, "Last command rejected: " + last_cmd_result_);
    }

    stat.add("firmware version", version_.empty() ? "unknown" : version_);
    stat.add("feedback source", feedback_source_);
    stat.add("feedback age [s]", age);
    if (have_feedback_) {
      stat.add("yaw joint [deg]", current_joint_[YAW] / 10.0);
      stat.add("roll joint [deg]", current_joint_[ROLL] / 10.0);
      stat.add("pitch joint [deg]", current_joint_[PITCH] / 10.0);
    }
    stat.add("command mode", command_mode_);
    stat.add("velocity control active", velocity_active_);
    stat.add("last command result", last_cmd_result_);
    stat.add(
      "calibration", calib.received ?
      std::string(calib_state_str(calib.state)) + " " + std::to_string(calib.progress) + "%" :
      std::string("no status"));

    DJIR_SDK::GimbalPushData push;
    if (gimbal_.get_push_data(push)) {
      if (push.limits_valid) {
        char buf[96];
        std::snprintf(
          buf, sizeof(buf), "yaw %u/%u  roll %u/%u  pitch %u/%u", push.yaw_min, push.yaw_max,
          push.roll_min, push.roll_max, push.pitch_min, push.pitch_max);
        stat.add("angle limits min/max [deg]", buf);
      }
      if (push.stiffness_valid) {
        char buf[64];
        std::snprintf(
          buf, sizeof(buf), "yaw %u  roll %u  pitch %u", push.yaw_stiffness,
          push.roll_stiffness, push.pitch_stiffness);
        stat.add("motor stiffness", buf);
      }
    }
  }

  // ------------------------------------------------------------------- members
  DJIR_SDK::DJIRonin gimbal_;
  diagnostic_updater::Updater diag_;

  std::string can_interface_;
  std::vector<std::string> joint_names_;
  double publish_rate_;
  bool publish_attitude_;
  bool use_push_;
  std::string command_mode_;
  double default_move_time_, min_move_time_, max_velocity_, angle_limit_;
  int16_t angle_limit_tenth_;
  double velocity_timeout_, max_command_velocity_;
  int reply_timeout_ms_;
  double feedback_timeout_, reconnect_period_;

  rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_cmd_sub_, velocity_cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recenter_srv_, reset_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr calibrate_srv_, mode_srv_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr feedback_timer_, velocity_timer_;

  // state
  SteadyClock::time_point last_connect_attempt_{}, last_push_request_{}, last_version_request_{};
  bool push_requested_ = false;
  uint64_t last_push_count_ = 0;
  std::string version_;
  std::string feedback_source_ = "none";

  bool have_feedback_ = false;
  SteadyClock::time_point last_feedback_{};
  std::array<int16_t, 3> current_joint_{0, 0, 0};

  std::array<int16_t, 3> last_cmd_{0, 0, 0};
  std::array<bool, 3> last_cmd_valid_{false, false, false};
  bool last_cmd_ok_ = true;
  std::string last_cmd_result_ = "none";

  bool velocity_active_ = false;
  std::array<int16_t, 3> velocity_cmd_{0, 0, 0};
  SteadyClock::time_point last_velocity_cmd_{};

  DJIR_SDK::LinkStats link_snapshot_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DjirDriverNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("djir_driver"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
