#pragma once

#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/infra/ros/topic_registry.hpp"

#include <QObject>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>

QT_BEGIN_NAMESPACE
class QTimer;
QT_END_NAMESPACE

namespace prop_arm::infra::ros {


struct DomainTelemetry {
    prop_arm::models::Domain domain{prop_arm::models::Domain::Real};
    std::chrono::steady_clock::time_point t{};

    double arm_angle_rad{0.0};
    double motor_speed_rad_s{0.0};
    double ref_angle_rad{0.0};
    std::uint16_t pwm_us{0};

    bool valid{false};
};

class PropArmRosBridge final : public QObject {
    Q_OBJECT

public:
    explicit PropArmRosBridge(
        std::shared_ptr<rclcpp::Node> node,
        std::shared_ptr<const TopicRegistry> topics,
        QObject* parent = nullptr);

    ~PropArmRosBridge() override = default;

    // Command publishing - broadcasts to both Real and Sim
    void sendRefAngle(double rad);
    void sendPwm(std::uint16_t us);
    void sendAutoMode(bool enabled);
    void sendStop();

    // Command publishing - domain-specific (for calibration)
    void sendAngleReference(prop_arm::models::Domain domain, double rad);
    void sendPwmCommand(prop_arm::models::Domain domain, std::uint16_t us);
    void sendAutoModeCommand(prop_arm::models::Domain domain, bool enabled);

    // Connection status
    [[nodiscard]] bool isConnected() const noexcept {
        return connected_;
    }
    [[nodiscard]] bool isLive(prop_arm::models::Domain domain) const noexcept;

signals:
    void telemetryUpdated(prop_arm::models::TelemetrySample sample);
    void connectionChanged(bool connected);
    void streamLiveChanged(prop_arm::models::Domain domain, bool live);

private:
    void setupPublishers_();
    void setupSubscribers_();
    void setupConnectionTimer_();
    void setupUiPublishTimer_();

    void touchRx_(prop_arm::models::Domain d, const rclcpp::Time& now);
    void updateConnection_(const rclcpp::Time& now);

    [[nodiscard]] static prop_arm::models::TelemetrySample toSample_(const DomainTelemetry& t);

    template<typename MsgT>
    void subscribe_(const std::string& topic,
                    const rclcpp::QoS& qos,
                    std::function<void(const MsgT&)> callback);

    template<typename MsgT, typename PubT>
    void publish_(const PubT& pub, const MsgT& msg);

private:
    static constexpr double CONNECTION_TIMEOUT_S = 2.0;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<const TopicRegistry> topics_;

    // Publishers - Real domain
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ref_angle_real_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_pwm_real_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_auto_real_;

    // Publishers - Sim domain
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ref_angle_sim_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_pwm_sim_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_auto_sim_;

    // Subscribers (stored to prevent deletion)
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

    // Timers
    rclcpp::TimerBase::SharedPtr conn_timer_;
    QTimer* ui_timer_{nullptr};

    // Telemetry data
    mutable std::mutex telemetry_mtx_;
    DomainTelemetry real_;
    DomainTelemetry sim_;
    bool real_dirty_{false};
    bool sim_dirty_{false};

    // Connection tracking
    rclcpp::Time last_rx_any_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_rx_real_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_rx_sim_{0, 0, RCL_ROS_TIME};

    bool connected_{false};
    bool real_live_{false};
    bool sim_live_{false};
};

template<typename MsgT>
void PropArmRosBridge::subscribe_(
    const std::string& topic,
    const rclcpp::QoS& qos,
    std::function<void(const MsgT&)> callback)
{
    auto sub = node_->create_subscription<MsgT>(
                   topic, qos,
    [cb = std::move(callback)](const typename MsgT::SharedPtr msg) {
        cb(*msg);
    });

    subscriptions_.push_back(sub);
}

template<typename MsgT, typename PubT>
void PropArmRosBridge::publish_(const PubT& pub, const MsgT& msg)
{
    if (pub) {
        pub->publish(msg);
    }
}

} // namespace prop_arm::infra::ros
