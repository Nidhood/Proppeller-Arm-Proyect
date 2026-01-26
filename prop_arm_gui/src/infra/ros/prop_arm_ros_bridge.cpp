#include "prop_arm_gui/infra/ros/prop_arm_ros_bridge.hpp"
#include "prop_arm_gui/core/models/telemetry_signal_type.hpp"
#include "prop_arm_gui/core/models/command_type.hpp"

#include <QTimer>
#include <chrono>
#include <mutex>

namespace prop_arm::infra::ros {

using prop_arm::models::CommandType;
using prop_arm::models::Domain;
using prop_arm::models::TelemetrySignalType;

PropArmRosBridge::PropArmRosBridge(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<const TopicRegistry> topics,
    QObject* parent)
    : QObject(parent),
      node_(std::move(node)),
      topics_(std::move(topics))
{
    real_.domain = Domain::Real;
    sim_.domain = Domain::Sim;

    setupPublishers_();
    setupSubscribers_();
    setupConnectionTimer_();
    setupUiPublishTimer_();
}

prop_arm::models::TelemetrySample PropArmRosBridge::toSample_(const DomainTelemetry& t)
{
    prop_arm::models::TelemetrySample s;
    s.domain = t.domain;
    s.t = t.t;
    s.arm_angle_rad = t.arm_angle_rad;
    s.motor_speed_rad_s = t.motor_speed_rad_s;
    s.ref_angle_rad = t.ref_angle_rad;
    s.pwm_us = t.pwm_us;
    s.valid = t.valid;
    return s;
}

void PropArmRosBridge::setupPublishers_()
{
    auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(3))
                   .reliable()
                   .durability_volatile();

    // Publishers for Real domain
    pub_ref_angle_real_ = node_->create_publisher<std_msgs::msg::Float64>(
                              topics_->topic(Domain::Real, CommandType::RefAngle), qos_cmd);

    pub_pwm_real_ = node_->create_publisher<std_msgs::msg::UInt16>(
                        topics_->topic(Domain::Real, CommandType::PwmCmd), qos_cmd);

    pub_auto_real_ = node_->create_publisher<std_msgs::msg::Bool>(
                         topics_->topic(Domain::Real, CommandType::AutoMode), qos_cmd);

    // Publishers for Sim domain
    pub_ref_angle_sim_ = node_->create_publisher<std_msgs::msg::Float64>(
                             topics_->topic(Domain::Sim, CommandType::RefAngle), qos_cmd);

    pub_pwm_sim_ = node_->create_publisher<std_msgs::msg::UInt16>(
                       topics_->topic(Domain::Sim, CommandType::PwmCmd), qos_cmd);

    pub_auto_sim_ = node_->create_publisher<std_msgs::msg::Bool>(
                        topics_->topic(Domain::Sim, CommandType::AutoMode), qos_cmd);
}

void PropArmRosBridge::setupSubscribers_()
{
    auto qos_real_sensor = rclcpp::QoS(rclcpp::KeepLast(500)).best_effort().durability_volatile();
    auto qos_real_pwm    = rclcpp::QoS(rclcpp::KeepLast(500)).best_effort().durability_volatile();
    auto qos_sim_angle   = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort().durability_volatile();
    auto qos_sim_motor   = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort().durability_volatile();
    auto qos_sim_pwm     = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort().durability_volatile();

    // ========== REAL DOMAIN ==========
    subscribe_<std_msgs::msg::Float64>(
        topics_->topic(Domain::Real, TelemetrySignalType::Angle),
        qos_real_sensor,
    [this](const std_msgs::msg::Float64& m) {
        {
            std::scoped_lock<std::mutex> lk(telemetry_mtx_);
            real_.t = std::chrono::steady_clock::now();
            real_.arm_angle_rad = m.data;
            real_.valid = true;
            real_dirty_ = true;
        }
        touchRx_(Domain::Real, node_->now());
    });

    subscribe_<std_msgs::msg::Float64>(
        topics_->topic(Domain::Real, TelemetrySignalType::MotorSpeed),
        qos_real_sensor,
    [this](const std_msgs::msg::Float64& m) {
        {
            std::scoped_lock<std::mutex> lk(telemetry_mtx_);
            real_.motor_speed_rad_s = m.data;
            real_.valid = true;
            real_dirty_ = true;
        }
        touchRx_(Domain::Real, node_->now());
    });

    subscribe_<std_msgs::msg::UInt16>(
        topics_->topic(Domain::Real, TelemetrySignalType::PwmFeedback),
        qos_real_pwm,
    [this](const std_msgs::msg::UInt16& m) {
        {
            std::scoped_lock<std::mutex> lk(telemetry_mtx_);
            real_.pwm_us = static_cast<std::uint16_t>(m.data);
            real_.valid = true;
            real_dirty_ = true;
        }
        touchRx_(Domain::Real, node_->now());
    });

    subscribe_<std_msgs::msg::Float64>(
        topics_->topic(Domain::Real, TelemetrySignalType::AngleRef),
        qos_real_sensor,
    [this](const std_msgs::msg::Float64& m) {
        {
            std::scoped_lock<std::mutex> lk(telemetry_mtx_);
            real_.ref_angle_rad = m.data;
            real_.valid = true;
            real_dirty_ = true;
        }
        touchRx_(Domain::Real, node_->now());
    });

    // ========== SIM DOMAIN ==========
    subscribe_<std_msgs::msg::Float64>(
        topics_->topic(Domain::Sim, TelemetrySignalType::Angle),
        qos_sim_angle,
    [this](const std_msgs::msg::Float64& m) {
        {
            std::scoped_lock<std::mutex> lk(telemetry_mtx_);
            sim_.t = std::chrono::steady_clock::now();
            sim_.arm_angle_rad = m.data;
            sim_.valid = true;
            sim_dirty_ = true;
        }
        touchRx_(Domain::Sim, node_->now());
    });

    subscribe_<std_msgs::msg::Float64>(
        topics_->topic(Domain::Sim, TelemetrySignalType::MotorSpeed),
        qos_sim_motor,
    [this](const std_msgs::msg::Float64& m) {
        {
            std::scoped_lock<std::mutex> lk(telemetry_mtx_);
            sim_.motor_speed_rad_s = m.data;
            sim_.valid = true;
            sim_dirty_ = true;
        }
        touchRx_(Domain::Sim, node_->now());
    });

    subscribe_<std_msgs::msg::UInt16>(
        topics_->topic(Domain::Sim, TelemetrySignalType::PwmFeedback),
        qos_sim_pwm,
    [this](const std_msgs::msg::UInt16& m) {
        {
            std::scoped_lock<std::mutex> lk(telemetry_mtx_);
            sim_.pwm_us = static_cast<std::uint16_t>(m.data);
            sim_.valid = true;
            sim_dirty_ = true;
        }
        touchRx_(Domain::Sim, node_->now());
    });

    subscribe_<std_msgs::msg::Float64>(
        topics_->topic(Domain::Sim, TelemetrySignalType::AngleRef),
        qos_sim_angle,
    [this](const std_msgs::msg::Float64& m) {
        {
            std::scoped_lock<std::mutex> lk(telemetry_mtx_);
            sim_.ref_angle_rad = m.data;
            sim_.valid = true;
            sim_dirty_ = true;
        }
        touchRx_(Domain::Sim, node_->now());
    });
}

void PropArmRosBridge::setupConnectionTimer_()
{
    conn_timer_ = node_->create_wall_timer(
                      std::chrono::milliseconds(250),
    [this]() {
        updateConnection_(node_->now());
    });
}

void PropArmRosBridge::setupUiPublishTimer_()
{
    ui_timer_ = new QTimer(this);
    ui_timer_->setInterval(50); // 20 Hz UI update

    connect(ui_timer_, &QTimer::timeout, this, [this]() {
        DomainTelemetry real_copy;
        DomainTelemetry sim_copy;
        bool emit_real = false;
        bool emit_sim = false;

        {
            std::scoped_lock<std::mutex> lk(telemetry_mtx_);
            if (real_dirty_) {
                real_copy = real_;
                real_dirty_ = false;
                emit_real = true;
            }
            if (sim_dirty_) {
                sim_copy = sim_;
                sim_dirty_ = false;
                emit_sim = true;
            }
        }

        // ✅ FIX: emit TelemetrySample (matches AppContext)
        if (emit_real) emit telemetryUpdated(toSample_(real_copy));
        if (emit_sim)  emit telemetryUpdated(toSample_(sim_copy));
    });

    ui_timer_->start();
}

void PropArmRosBridge::touchRx_(Domain d, const rclcpp::Time& now)
{
    last_rx_any_ = now;
    if (d == Domain::Real) last_rx_real_ = now;
    if (d == Domain::Sim)  last_rx_sim_  = now;
}

void PropArmRosBridge::updateConnection_(const rclcpp::Time& now)
{
    if (last_rx_any_.nanoseconds() == 0) {
        if (connected_) {
            connected_ = false;
            emit connectionChanged(false);
        }
    } else {
        const double dt = (now - last_rx_any_).seconds();
        const bool new_connected = dt <= CONNECTION_TIMEOUT_S;

        if (new_connected != connected_) {
            connected_ = new_connected;
            emit connectionChanged(connected_);
        }
    }

    auto calc_live = [&](const rclcpp::Time& last) -> bool {
        if (last.nanoseconds() == 0) return false;
        return (now - last).seconds() <= CONNECTION_TIMEOUT_S;
    };

    const bool new_real_live = calc_live(last_rx_real_);
    const bool new_sim_live  = calc_live(last_rx_sim_);

    if (new_real_live != real_live_) {
        real_live_ = new_real_live;
        emit streamLiveChanged(Domain::Real, real_live_);
    }

    if (new_sim_live != sim_live_) {
        sim_live_ = new_sim_live;
        emit streamLiveChanged(Domain::Sim, sim_live_);
    }
}

// ============================================================================
// Command Publishing - Broadcast to both domains
// ============================================================================

void PropArmRosBridge::sendRefAngle(double rad)
{
    std_msgs::msg::Float64 msg;
    msg.data = rad;
    publish_(pub_ref_angle_real_, msg);
    publish_(pub_ref_angle_sim_, msg);
}

void PropArmRosBridge::sendPwm(std::uint16_t us)
{
    std_msgs::msg::UInt16 msg;
    msg.data = us;
    publish_(pub_pwm_real_, msg);
    publish_(pub_pwm_sim_, msg);
}

void PropArmRosBridge::sendAutoMode(bool enabled)
{
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    publish_(pub_auto_real_, msg);
    publish_(pub_auto_sim_, msg);
}

void PropArmRosBridge::sendStop()
{
    sendAutoMode(false);
    sendPwm(0);
}

// ============================================================================
// Command Publishing - Domain-specific (for calibration)
// ============================================================================

void PropArmRosBridge::sendAngleReference(Domain domain, double rad)
{
    std_msgs::msg::Float64 msg;
    msg.data = rad;

    if (domain == Domain::Real) {
        publish_(pub_ref_angle_real_, msg);
    } else if (domain == Domain::Sim) {
        publish_(pub_ref_angle_sim_, msg);
    }
}

void PropArmRosBridge::sendPwmCommand(Domain domain, std::uint16_t us)
{
    std_msgs::msg::UInt16 msg;
    msg.data = us;

    if (domain == Domain::Real) {
        publish_(pub_pwm_real_, msg);
    } else if (domain == Domain::Sim) {
        publish_(pub_pwm_sim_, msg);
    }
}

void PropArmRosBridge::sendAutoModeCommand(Domain domain, bool enabled)
{
    std_msgs::msg::Bool msg;
    msg.data = enabled;

    if (domain == Domain::Real) {
        publish_(pub_auto_real_, msg);
    } else if (domain == Domain::Sim) {
        publish_(pub_auto_sim_, msg);
    }
}

bool PropArmRosBridge::isLive(Domain domain) const noexcept
{
    if (domain == Domain::Real) return real_live_;
    if (domain == Domain::Sim)  return sim_live_;
    return false;
}

} // namespace prop_arm::infra::ros
