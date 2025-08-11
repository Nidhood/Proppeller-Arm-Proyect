#include "prop_arm_gui/prop_arm_gui_node.hpp"
#include <QApplication>

PropArmGuiNode::PropArmGuiNode(const rclcpp::NodeOptions &options)
    : QObject(), rclcpp::Node("prop_arm_gui_node", options)
{
    // Parameters
    this->declare_parameter<double>("target_angle_deg", 0.0);
    this->declare_parameter<double>("max_angle_deg", 90.0);
    this->declare_parameter<double>("min_angle_deg", -90.0);
    this->declare_parameter<double>("max_force_n", 45.0);
    this->declare_parameter<double>("max_velocity_rad_s", 785.0);

    this->get_parameter("target_angle_deg", target_angle_deg_);
    this->get_parameter("max_angle_deg", max_angle_deg_);
    this->get_parameter("min_angle_deg", min_angle_deg_);
    this->get_parameter("max_force_n", max_force_n_);
    this->get_parameter("max_velocity_rad_s", max_velocity_rad_s_);

    setupSubscribers();
    setupPublishers();

    // Setup Qt timer for ROS spinning
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, &QTimer::timeout, this, &PropArmGuiNode::processRosEvents);
    ros_timer_->start(10); // 100 Hz

    // Initialize timestamps
    last_data_time_ = this->get_clock()->now();
    current_data_.timestamp = last_data_time_;

    RCLCPP_INFO(this->get_logger(), "PropArm GUI Node initialized");
}

void PropArmGuiNode::setupSubscribers()
{
    rclcpp::SensorDataQoS sensor_qos;

    arm_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/prop_arm/arm_angle_deg", sensor_qos,
        std::bind(&PropArmGuiNode::armAngleCallback, this, std::placeholders::_1));

    motor_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/prop_arm/motor_speed_est", sensor_qos,
        std::bind(&PropArmGuiNode::motorSpeedCallback, this, std::placeholders::_1));

    v_emf_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/prop_arm/v_emf", sensor_qos,
        std::bind(&PropArmGuiNode::vEmfCallback, this, std::placeholders::_1));

    delta_v_emf_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/prop_arm/delta_v_emf", sensor_qos,
        std::bind(&PropArmGuiNode::deltaVEmfCallback, this, std::placeholders::_1));
}

void PropArmGuiNode::setupPublishers()
{
    velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controller/commands", rclcpp::QoS(10).reliable());

    force_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/motor_force_controller/commands", rclcpp::QoS(10).reliable());
}

void PropArmGuiNode::processRosEvents()
{
    rclcpp::spin_some(this->shared_from_this());
    updateConnectionStatus();
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
    current_data_.error = target_angle_deg_ - msg->data;
    current_data_.timestamp = this->get_clock()->now();
    current_data_.valid = true;

    // Add to history
    history_data_.push_back(current_data_);
    if (history_data_.size() > MAX_HISTORY_SIZE)
    {
        history_data_.pop_front();
    }

    last_data_time_ = current_data_.timestamp;
    emit dataUpdated();
}

void PropArmGuiNode::motorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.motor_speed_est = msg->data;
    current_data_.motor_command = msg->data; // Assume command matches actual for now
    last_data_time_ = this->get_clock()->now();
}

void PropArmGuiNode::vEmfCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.v_emf = msg->data;
    last_data_time_ = this->get_clock()->now();
}

void PropArmGuiNode::deltaVEmfCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.delta_v_emf = msg->data;
    last_data_time_ = this->get_clock()->now();
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
    // Clamp angle to valid range
    angle_degrees = std::clamp(angle_degrees, min_angle_deg_, max_angle_deg_);

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_angle_deg_ = angle_degrees;
        current_data_.target_angle = angle_degrees;
    }

    // For angle control, we would typically use a position controller
    // For now, we'll use the velocity controller with a simple P controller
    double error = angle_degrees - current_data_.arm_angle_deg;
    double velocity_cmd = std::clamp(error * 10.0, 0.0, max_velocity_rad_s_); // Simple P gain

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.push_back(velocity_cmd);
    velocity_cmd_pub_->publish(msg);

    control_mode_ = "Angle Control";

    RCLCPP_INFO(this->get_logger(), "Angle command: %.2f°, velocity: %.1f rad/s",
                angle_degrees, velocity_cmd);
}

void PropArmGuiNode::sendForceCommand(double force_newtons)
{
    // Clamp force to valid range
    force_newtons = std::clamp(force_newtons, 0.0, max_force_n_);

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.push_back(force_newtons);
    force_cmd_pub_->publish(msg);

    control_mode_ = "Force Control";

    RCLCPP_INFO(this->get_logger(), "Force command: %.2f N", force_newtons);
}

void PropArmGuiNode::sendVelocityCommand(double velocity_rad_s)
{
    // Clamp velocity to valid range
    velocity_rad_s = std::clamp(velocity_rad_s, 0.0, max_velocity_rad_s_);

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.push_back(velocity_rad_s);
    velocity_cmd_pub_->publish(msg);

    control_mode_ = "Velocity Control";

    RCLCPP_INFO(this->get_logger(), "Velocity command: %.1f rad/s", velocity_rad_s);
}

void PropArmGuiNode::sendStopCommand()
{
    // Send zero to all controllers
    auto zero_msg = std_msgs::msg::Float64MultiArray();
    zero_msg.data.push_back(0.0);

    velocity_cmd_pub_->publish(zero_msg);
    force_cmd_pub_->publish(zero_msg);

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_angle_deg_ = current_data_.arm_angle_deg; // Hold current position
    }

    control_mode_ = "STOPPED";

    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP - All commands set to zero");
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