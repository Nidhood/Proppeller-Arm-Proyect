#include "prop_arm_gui/infra/ros/topic_registry.hpp"

#include <stdexcept>

namespace prop_arm::infra::ros {

using prop_arm::models::CommandType;
using prop_arm::models::Domain;
using prop_arm::models::TelemetrySignalType;

namespace {

std::string getStringEither(rclcpp::Node& node,
                            const std::string& a,
                            const std::string& b,
                            const std::string& fallback) {
    if (!node.has_parameter(a)) node.declare_parameter<std::string>(a, fallback);
    if (!node.has_parameter(b)) node.declare_parameter<std::string>(b, fallback);
    const auto pa = node.get_parameter(a).as_string();
    const auto pb = node.get_parameter(b).as_string();
    if (!pa.empty() && pa != fallback) return pa;
    if (!pb.empty() && pb != fallback) return pb;
    return fallback;
}

}  // namespace

TopicRegistry::TopicRegistry(rclcpp::Node& node) {
    // Namespaces
    if (!node.has_parameter("topic_ns.real")) node.declare_parameter<std::string>("topic_ns.real", "/arm");
    if (!node.has_parameter("topic_ns.sim"))  node.declare_parameter<std::string>("topic_ns.sim",  "/arm_sim");

    real_ns_ = normalizeNs_(node.get_parameter("topic_ns.real").as_string());
    sim_ns_  = normalizeNs_(node.get_parameter("topic_ns.sim").as_string());

    // Signals (support both YAML styles)
    angle_        = getStringEither(node, "topics.signals.angle",        "topic.signals.angle",        "angle_rad");
    motor_speed_  = getStringEither(node, "topics.signals.motor_speed",  "topic.signals.motor_speed",  "motor_vel_rad");
    pwm_feedback_ = getStringEither(node, "topics.signals.pwm_feedback", "topic.signals.pwm_feedback", "esc/pwm_us_fb");
    angle_ref_    = getStringEither(node, "topics.signals.angle_ref",    "topic.signals.angle_ref",    "ref_angle_rad");
    ref_angle_ = getStringEither(node, "topics.commands.ref_angle", "topic.commands.ref_angle", "ref_angle_rad");
    pwm_cmd_   = getStringEither(node, "topics.commands.pwm_cmd",   "topic.commands.pwm_cmd",   "esc/pwm_us");
    auto_mode_ = getStringEither(node, "topics.commands.auto_mode", "topic.commands.auto_mode", "esc/auto_mode");
}

std::string TopicRegistry::normalizeNs_(std::string ns) {
    if (ns.empty()) return ns;
    if (ns.front() != '/') ns.insert(ns.begin(), '/');
    while (ns.size() > 1 && ns.back() == '/') ns.pop_back();
    return ns;
}

std::string TopicRegistry::join_(const std::string& ns, const std::string& name) {
    if (name.empty()) return ns;  // defensive
    if (ns.empty()) return (name.front() == '/') ? name : ("/" + name);

    if (name.front() == '/') return ns + name;
    return ns + "/" + name;
}

std::string TopicRegistry::topic(Domain d, TelemetrySignalType sig) const {
    const std::string& ns = (d == Domain::Real) ? real_ns_ : sim_ns_;

    switch (sig) {
    case TelemetrySignalType::Angle:
        return join_(ns, angle_);
    case TelemetrySignalType::MotorSpeed:
        return join_(ns, motor_speed_);
    case TelemetrySignalType::PwmFeedback:
        return join_(ns, pwm_feedback_);
    case TelemetrySignalType::AngleRef:
        return join_(ns, angle_ref_);
    default:
        break;
    }
    throw std::runtime_error("TopicRegistry::topic(): Unknown TelemetrySignalType");
}

std::string TopicRegistry::topic(Domain d, CommandType cmd) const {
    const std::string& ns = (d == Domain::Real) ? real_ns_ : sim_ns_;

    switch (cmd) {
    case CommandType::RefAngle:
        return join_(ns, ref_angle_);
    case CommandType::PwmCmd:
        return join_(ns, pwm_cmd_);
    case CommandType::AutoMode:
        return join_(ns, auto_mode_);
    default:
        break;
    }
    throw std::runtime_error("TopicRegistry::topic(): Unknown CommandType");
}

}  // namespace prop_arm::infra::ros
