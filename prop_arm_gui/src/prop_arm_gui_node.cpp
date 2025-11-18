#include "prop_arm_gui/prop_arm_gui_node.hpp"
#include "prop_arm_gui/data_recorder.hpp"
#include <cmath>
#include <algorithm>

PropArmGuiNode::PropArmGuiNode(const rclcpp::NodeOptions &options)
    : QObject(), rclcpp::Node("prop_arm_gui_node", options)
{
    // Initialize system timestamps
    system_start_timepoint_ = std::chrono::steady_clock::now();
    system_start_time_ = 0.0;

    // Initialize data recorder
    data_recorder_ = std::make_unique<DataRecorder>();

    setupSubscribers();
    setupPublishers();

    // Initialize timestamps and data
    last_data_time_ = this->get_clock()->now();
    last_sim_data_time_ = this->get_clock()->now();
    
    current_data_.system_timestamp = getSystemTimestamp();
    current_data_.datetime = QDateTime::currentDateTime();
    current_data_.timestamp = last_data_time_;
    current_data_.valid = false;
    current_data_.sim_valid = false;

    // Initialize simulation data to zeros
    current_data_.sim_arm_angle_deg = 0.0;
    current_data_.sim_motor_speed_rad_s = 0.0;
    current_data_.sim_pwm_input_us = 0.0;
    current_data_.sim_duty_cycle_percent = 0.0;
}

PropArmGuiNode::~PropArmGuiNode()
{
    RCLCPP_INFO(this->get_logger(), "PropArm GUI Node shutting down.");
    data_recorder_.reset();
}

double PropArmGuiNode::getSystemTimestamp() const
{
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        now - system_start_timepoint_);
    return duration.count() / 1000000.0;
}

void PropArmGuiNode::setupSubscribers()
{
    // Use BEST_EFFORT for real-time data
    auto qos_profile = rclcpp::QoS(100)
                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                           .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                           .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    // REAL data subscribers
    arm_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/arm/angle_rad", qos_profile,
        std::bind(&PropArmGuiNode::armAngleCallback, this, std::placeholders::_1));

    motor_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/arm/motor_vel_rad", qos_profile,
        std::bind(&PropArmGuiNode::motorSpeedCallback, this, std::placeholders::_1));

    pwm_input_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "/esc/pwm_us_fb", qos_profile,
        std::bind(&PropArmGuiNode::pwmInputCallback, this, std::placeholders::_1));

    // SIMULATION data subscribers
    sim_arm_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/arm_sim/angle_rad", qos_profile,
        std::bind(&PropArmGuiNode::simArmAngleCallback, this, std::placeholders::_1));

    sim_motor_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/arm_sim/motor_vel_rad", qos_profile,
        std::bind(&PropArmGuiNode::simMotorSpeedCallback, this, std::placeholders::_1));

    sim_pwm_input_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "/arm_sim/esc/pwm_us_fb", qos_profile,
        std::bind(&PropArmGuiNode::simPwmInputCallback, this, std::placeholders::_1));
}

void PropArmGuiNode::setupPublishers()
{
    auto qos_profile = rclcpp::QoS(10)
                           .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                           .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/arm/motor_vel_rad", qos_profile);
}

double PropArmGuiNode::calculateDutyCycle(double pwm_us) const
{
    return (pwm_us / PWM_PERIOD_US) * 100.0;
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

void PropArmGuiNode::updateSimConnectionStatus()
{
    auto now = this->get_clock()->now();
    double time_since_last_sim = (now - last_sim_data_time_).seconds();

    bool was_sim_connected = sim_connected_;
    sim_connected_ = (time_since_last_sim < CONNECTION_TIMEOUT);

    if (was_sim_connected != sim_connected_)
    {
        if (sim_connected_)
        {
            RCLCPP_INFO(this->get_logger(), "Simulation data connected");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Simulation data disconnected");
        }
    }
}

// REAL DATA CALLBACKS
void PropArmGuiNode::armAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_data_.system_timestamp = getSystemTimestamp();
    current_data_.datetime = QDateTime::currentDateTime();
    current_data_.timestamp = this->get_clock()->now();

    current_data_.arm_angle_deg = msg->data * 180.0 / M_PI;
    current_data_.valid = true;

    current_data_.error = target_angle_deg_ - current_data_.arm_angle_deg;
    current_data_.target_angle = target_angle_deg_;

    history_data_.push_back(current_data_);
    if (history_data_.size() > MAX_HISTORY_SIZE)
    {
        history_data_.pop_front();
    }

    if (data_recorder_ && data_recorder_->isRecording())
    {
        data_recorder_->recordDataPoint(current_data_);

        double remaining = data_recorder_->getRemainingTime();
        size_t count = data_recorder_->getRecordedPointCount();

        QMetaObject::invokeMethod(this, [this, remaining, count]()
                                  { emit recordingProgress(remaining, count); }, Qt::QueuedConnection);

        if (!data_recorder_->isRecording() && count > 0)
        {
            double duration = data_recorder_->getRecordingDuration();
            QMetaObject::invokeMethod(this, [this, count, duration]()
                                      { emit recordingCompleted(count, duration); }, Qt::QueuedConnection);
        }
    }

    last_data_time_ = current_data_.timestamp;
    updateConnectionStatus();

    QMetaObject::invokeMethod(this, [this]()
                              { emit dataUpdated(); }, Qt::QueuedConnection);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "REAL Arm angle: %.2f° (%.4f rad)",
                          current_data_.arm_angle_deg, msg->data);
}

void PropArmGuiNode::motorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_data_.motor_speed_rad_s = msg->data;
    current_data_.system_timestamp = getSystemTimestamp();
    current_data_.datetime = QDateTime::currentDateTime();
    last_data_time_ = this->get_clock()->now();
    updateConnectionStatus();

    QMetaObject::invokeMethod(this, [this]()
                              { emit dataUpdated(); }, Qt::QueuedConnection);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "REAL Motor velocity: %.2f rad/s", msg->data);
}

void PropArmGuiNode::pwmInputCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_data_.pwm_input_us = static_cast<double>(msg->data);
    current_data_.duty_cycle_percent = calculateDutyCycle(current_data_.pwm_input_us);
    current_data_.system_timestamp = getSystemTimestamp();
    current_data_.datetime = QDateTime::currentDateTime();

    last_data_time_ = this->get_clock()->now();
    updateConnectionStatus();

    QMetaObject::invokeMethod(this, [this]()
                              { emit dataUpdated(); }, Qt::QueuedConnection);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "REAL PWM: %u µs, Duty: %.2f%%",
                         msg->data, current_data_.duty_cycle_percent);
}

// SIMULATION DATA CALLBACKS - NOW EMIT SEPARATE SIGNALS
void PropArmGuiNode::simArmAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_data_.sim_arm_angle_deg = msg->data * 180.0 / M_PI;
    current_data_.sim_valid = true;
    
    last_sim_data_time_ = this->get_clock()->now();
    updateSimConnectionStatus();

    // CRITICAL: Emit BOTH signals to ensure updates
    QMetaObject::invokeMethod(this, [this]()
                              { 
                                  emit simDataUpdated();
                                  emit dataUpdated();  // Also emit general signal
                              }, Qt::QueuedConnection);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "SIM Arm angle: %.2f° (%.4f rad)",
                         current_data_.sim_arm_angle_deg, msg->data);
}

void PropArmGuiNode::simMotorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_data_.sim_motor_speed_rad_s = msg->data;
    current_data_.sim_valid = true;
    
    last_sim_data_time_ = this->get_clock()->now();
    updateSimConnectionStatus();

    QMetaObject::invokeMethod(this, [this]()
                              { 
                                  emit simDataUpdated();
                                  emit dataUpdated();
                              }, Qt::QueuedConnection);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "SIM Motor velocity: %.2f rad/s", msg->data);
}

void PropArmGuiNode::simPwmInputCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_data_.sim_pwm_input_us = static_cast<double>(msg->data);
    current_data_.sim_duty_cycle_percent = calculateDutyCycle(current_data_.sim_pwm_input_us);
    current_data_.sim_valid = true;
    
    last_sim_data_time_ = this->get_clock()->now();
    updateSimConnectionStatus();

    QMetaObject::invokeMethod(this, [this]()
                              { 
                                  emit simDataUpdated();
                                  emit dataUpdated();
                              }, Qt::QueuedConnection);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "SIM PWM: %u µs, Duty: %.2f%%",
                         msg->data, current_data_.sim_duty_cycle_percent);
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

    double error = target_angle_deg_ - current_data_.arm_angle_deg;
    double velocity_cmd = error * 5.0;
    velocity_cmd = std::clamp(velocity_cmd, -max_velocity_rad_s_, max_velocity_rad_s_);

    auto msg = std_msgs::msg::Float64();
    msg.data = velocity_cmd;
    velocity_cmd_pub_->publish(msg);

    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.motor_command = velocity_cmd;

    RCLCPP_DEBUG(this->get_logger(),
                 "Angle command: target=%.2f°, current=%.2f°, error=%.2f°, vel_cmd=%.2f rad/s",
                 target_angle_deg_, current_data_.arm_angle_deg, error, velocity_cmd);
}

void PropArmGuiNode::sendVelocityCommand(double velocity_rad_s)
{
    velocity_rad_s = std::clamp(velocity_rad_s, -max_velocity_rad_s_, max_velocity_rad_s_);
    control_mode_ = "VELOCITY CONTROL";

    const rclcpp::Time now_time = now();
    const bool time_ok = (!last_velocity_pub_time_.nanoseconds()) ||
                         ((now_time - last_velocity_pub_time_).seconds() >= VELOCITY_CMD_DEBOUNCE_S);
    const bool value_ok = (std::isnan(last_velocity_cmd_)) ||
                          (std::fabs(velocity_rad_s - last_velocity_cmd_) >= VELOCITY_CMD_MIN_DELTA);

    if (!(time_ok && value_ok))
    {
        return;
    }

    last_velocity_cmd_ = velocity_rad_s;
    last_velocity_pub_time_ = now_time;

    auto msg = std_msgs::msg::Float64();
    msg.data = velocity_rad_s;
    velocity_cmd_pub_->publish(msg);

    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.motor_command = velocity_rad_s;

    RCLCPP_DEBUG(this->get_logger(), "Velocity command: %.2f rad/s", velocity_rad_s);
}

void PropArmGuiNode::sendStopCommand()
{
    control_mode_ = "STOPPED";

    auto velocity_msg = std_msgs::msg::Float64();
    velocity_msg.data = 0.0;
    velocity_cmd_pub_->publish(velocity_msg);

    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.motor_command = 0.0;
    target_angle_deg_ = current_data_.arm_angle_deg;

    RCLCPP_INFO(this->get_logger(), "STOP command sent");
}

bool PropArmGuiNode::isConnected() const
{
    return connected_;
}

bool PropArmGuiNode::isSimConnected() const
{
    return sim_connected_;
}

std::string PropArmGuiNode::getConnectionStatus() const
{
    if (connected_ && sim_connected_)
    {
        return "Real + Sim Connected";
    }
    else if (connected_)
    {
        return "Real Connected";
    }
    else if (sim_connected_)
    {
        return "Sim Connected";
    }
    else
    {
        return "Disconnected";
    }
}

std::string PropArmGuiNode::getControlMode() const
{
    return control_mode_;
}

void PropArmGuiNode::startRecording(double duration_seconds)
{
    if (data_recorder_)
    {
        data_recorder_->startRecording(duration_seconds);

        RCLCPP_INFO(this->get_logger(),
                    "Started recording for %.1f seconds - capturing NEW data only",
                    duration_seconds);

        QMetaObject::invokeMethod(this, [this, duration_seconds]()
                                  { emit recordingStarted(duration_seconds); }, Qt::QueuedConnection);
    }
}

void PropArmGuiNode::stopRecording()
{
    if (data_recorder_)
    {
        size_t count = data_recorder_->getRecordedPointCount();
        double duration = data_recorder_->getRecordingDuration();

        data_recorder_->stopRecording();

        RCLCPP_INFO(this->get_logger(),
                    "Stopped recording - captured %zu points over %.2f seconds",
                    count, duration);

        QMetaObject::invokeMethod(this, [this, count, duration]()
                                  { emit recordingCompleted(count, duration); }, Qt::QueuedConnection);
    }
}

bool PropArmGuiNode::isRecording() const
{
    return data_recorder_ && data_recorder_->isRecording();
}

std::vector<PropArmData> PropArmGuiNode::getRecordedData() const
{
    if (data_recorder_)
    {
        return data_recorder_->getRecordedData();
    }
    return std::vector<PropArmData>();
}

size_t PropArmGuiNode::getRecordedPointCount() const
{
    if (data_recorder_)
    {
        return data_recorder_->getRecordedPointCount();
    }
    return 0;
}

double PropArmGuiNode::getRecordingDuration() const
{
    if (data_recorder_)
    {
        return data_recorder_->getRecordingDuration();
    }
    return 0.0;
}

double PropArmGuiNode::getRecordingRemainingTime() const
{
    if (data_recorder_)
    {
        return data_recorder_->getRemainingTime();
    }
    return 0.0;
}