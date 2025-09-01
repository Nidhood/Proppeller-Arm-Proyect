#include "prop_arm_gui/prop_arm_gui_node.hpp"
#include <cmath>
#include <algorithm>

PropArmGuiNode::PropArmGuiNode(const rclcpp::NodeOptions &options)
    : QObject(), rclcpp::Node("prop_arm_gui_node", options)
{
    // [Previous parameter declarations remain the same...]

    setupSubscribers();
    setupPublishers();

    // Initialize timestamps
    last_data_time_ = this->get_clock()->now();
    current_data_.timestamp = last_data_time_;

    RCLCPP_INFO(this->get_logger(), "PropArm GUI Node initialized with Vref and VPWM monitoring");
    RCLCPP_INFO(this->get_logger(), "Monitoring topics: /prop_arm/arm_angle_deg, /prop_arm/motor_speed_est, /prop_arm/vemf, /prop_arm/vpwm, /prop_arm/cmd/vref");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /velocity_controller/commands");
}

void PropArmGuiNode::setupSubscribers()
{
    // Use sensor data QoS for real-time performance
    auto qos_profile = rclcpp::QoS(100)
                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                           .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                           .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    // Existing subscribers
    arm_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/prop_arm/arm_angle_deg", qos_profile,
        std::bind(&PropArmGuiNode::armAngleCallback, this, std::placeholders::_1));

    motor_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/prop_arm/motor_speed_est", qos_profile,
        std::bind(&PropArmGuiNode::motorSpeedCallback, this, std::placeholders::_1));

    v_emf_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/prop_arm/vemf", qos_profile,
        std::bind(&PropArmGuiNode::vEmfCallback, this, std::placeholders::_1));

    vpwm_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/prop_arm/vpwm", qos_profile,
        std::bind(&PropArmGuiNode::vpwmCallback, this, std::placeholders::_1));

    // Vref input subscription
    vref_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/prop_arm/cmd/vref", qos_profile,
        std::bind(&PropArmGuiNode::vrefCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /prop_arm/vemf for V_EMF monitoring");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /prop_arm/vpwm for VPWM voltage monitoring");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /prop_arm/cmd/vref for Vref input monitoring");
}

void PropArmGuiNode::vrefCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.vref_input = msg->data;
    last_data_time_ = this->get_clock()->now();
    updateConnectionStatus();

    // Log Vref data for debugging
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Received Vref input: %.3f V", msg->data);
}

void PropArmGuiNode::setupPublishers()
{
    auto qos_profile = rclcpp::QoS(10)
                           .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                           .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controller/commands", qos_profile);

    force_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/motor_force_controller/commands", qos_profile);
}

void PropArmGuiNode::updateConnectionStatus()
{
    auto now = this->get_clock()->now();
    double time_since_last = (now - last_data_time_).seconds();

    bool was_connected = connected_;
    connected_ = (time_since_last < CONNECTION_TIMEOUT);

    if (was_connected != connected_)
    {
        emit connectionChanged(connected_);
    }
}

void PropArmGuiNode::armAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_data_.arm_angle_deg = msg->data;
    current_data_.timestamp = this->get_clock()->now();
    current_data_.valid = true;

    // Calculate error
    current_data_.error = target_angle_deg_ - msg->data;
    current_data_.target_angle = target_angle_deg_;

    // Add to history
    history_data_.push_back(current_data_);
    if (history_data_.size() > MAX_HISTORY_SIZE)
    {
        history_data_.pop_front();
    }

    last_data_time_ = current_data_.timestamp;
    updateConnectionStatus();

    // Emit signal for GUI update (thread-safe)
    QMetaObject::invokeMethod(this, [this]()
                              { emit dataUpdated(); }, Qt::QueuedConnection);
}

void PropArmGuiNode::motorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.motor_speed_est = msg->data;
    last_data_time_ = this->get_clock()->now();
    updateConnectionStatus();
}

void PropArmGuiNode::vEmfCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.v_emf = msg->data;
    last_data_time_ = this->get_clock()->now();
    updateConnectionStatus();
}

// VPWM callback implementation
void PropArmGuiNode::vpwmCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.vpwm = msg->data;
    last_data_time_ = this->get_clock()->now();
    updateConnectionStatus();

    // Log VPWM data for debugging
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Received VPWM: %.3f V", msg->data);
}

PropArmData PropArmGuiNode::getCurrentData() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_data_;
}

std::vector<PropArmData> PropArmGuiNode::getHistoryData(size_t max_points) const
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::vector<PropArmData> result;
    if (history_data_.empty())
        return result;

    size_t start_idx = 0;
    if (history_data_.size() > max_points)
    {
        start_idx = history_data_.size() - max_points;
    }

    result.reserve(history_data_.size() - start_idx);
    for (size_t i = start_idx; i < history_data_.size(); ++i)
    {
        result.push_back(history_data_[i]);
    }

    return result;
}

void PropArmGuiNode::sendAngleCommand(double angle_degrees)
{
    target_angle_deg_ = std::clamp(angle_degrees, min_angle_deg_, max_angle_deg_);
    control_mode_ = "ANGLE CONTROL";

    // Convert angle to velocity command (simple proportional control)
    double error = target_angle_deg_ - current_data_.arm_angle_deg;
    double velocity_cmd = error * 5.0;
    velocity_cmd = std::clamp(velocity_cmd, -max_velocity_rad_s_, max_velocity_rad_s_);

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {velocity_cmd};
    velocity_cmd_pub_->publish(msg);

    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.motor_command = velocity_cmd;
}

void PropArmGuiNode::sendForceCommand(double force_newtons)
{
    force_newtons = std::clamp(force_newtons, 0.0, max_force_n_);
    control_mode_ = "Force Control";
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {force_newtons};
    force_cmd_pub_->publish(msg);
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.motor_command = force_newtons;
}

void PropArmGuiNode::sendVelocityCommand(double velocity_rad_s)
{
    velocity_rad_s = std::clamp(velocity_rad_s, -max_velocity_rad_s_, max_velocity_rad_s_);
    control_mode_ = "VELOCITY CONTROL";
    const rclcpp::Time now_time = now();
    const bool time_ok = (!last_velocity_pub_time_.nanoseconds()) || ((now_time - last_velocity_pub_time_).seconds() >= VELOCITY_CMD_DEBOUNCE_S);
    const bool value_ok = (std::isnan(last_velocity_cmd_)) || (std::fabs(velocity_rad_s - last_velocity_cmd_) >= VELOCITY_CMD_MIN_DELTA);
    if (!(time_ok && value_ok))
    {
        return;
    }
    last_velocity_cmd_ = velocity_rad_s;
    last_velocity_pub_time_ = now_time;
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {velocity_rad_s};
    velocity_cmd_pub_->publish(msg);
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.motor_command = velocity_rad_s;
}

void PropArmGuiNode::sendStopCommand()
{
    control_mode_ = "STOPPED";

    auto velocity_msg = std_msgs::msg::Float64MultiArray();
    velocity_msg.data = {0.0};
    velocity_cmd_pub_->publish(velocity_msg);

    auto force_msg = std_msgs::msg::Float64MultiArray();
    force_msg.data = {0.0};
    force_cmd_pub_->publish(force_msg);

    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.motor_command = 0.0;
    target_angle_deg_ = current_data_.arm_angle_deg;
}

bool PropArmGuiNode::isConnected() const
{
    return connected_;
}

std::string PropArmGuiNode::getConnectionStatus() const
{
    return connected_ ? "Connected" : "Disconnected";
}

std::string PropArmGuiNode::getControlMode() const
{
    return control_mode_;
}