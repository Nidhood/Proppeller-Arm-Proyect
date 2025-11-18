#include "prop_arm_gui/prop_arm_gui_node.hpp"
#include "prop_arm_gui/data_recorder.hpp"
#include <cmath>
#include <algorithm>

PropArmGuiNode::PropArmGuiNode(const rclcpp::NodeOptions &options)
    : QObject(), rclcpp::Node("prop_arm_gui_node", options)
{
    system_start_timepoint_ = std::chrono::steady_clock::now();
    system_start_time_ = 0.0;

    data_recorder_ = std::make_unique<DataRecorder>();

    setupSubscribers();
    setupPublishers();

    last_data_time_ = this->get_clock()->now();
    current_data_.system_timestamp = getSystemTimestamp();
    current_data_.datetime = QDateTime::currentDateTime();
    current_data_.timestamp = last_data_time_;

    current_data_.sim_arm_angle_deg = 0.0;
    current_data_.sim_motor_speed_rad_s = 0.0;
    current_data_.sim_pwm_input_us = 0.0;
    current_data_.sim_duty_cycle_percent = 0.0;
    current_data_.sim_ref_angle_deg = 0.0;
}

PropArmGuiNode::~PropArmGuiNode()
{
    if (step_timer_)
    {
        step_timer_->cancel();
    }
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
    auto qos_profile = rclcpp::QoS(100)
                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                           .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                           .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    arm_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/arm/angle_rad", qos_profile,
        std::bind(&PropArmGuiNode::armAngleCallback, this, std::placeholders::_1));

    motor_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/arm/motor_vel_rad", qos_profile,
        std::bind(&PropArmGuiNode::motorSpeedCallback, this, std::placeholders::_1));

    pwm_input_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "/arm/esc/pwm_us_fb", qos_profile,
        std::bind(&PropArmGuiNode::pwmInputCallback, this, std::placeholders::_1));

    sim_arm_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/arm_sim/angle_rad", qos_profile,
        std::bind(&PropArmGuiNode::simArmAngleCallback, this, std::placeholders::_1));

    sim_motor_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/arm_sim/motor_vel_rad", qos_profile,
        std::bind(&PropArmGuiNode::simMotorSpeedCallback, this, std::placeholders::_1));

    sim_pwm_input_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "/arm_sim/esc/pwm_us_fb", qos_profile,
        std::bind(&PropArmGuiNode::simPwmInputCallback, this, std::placeholders::_1));

    sim_ref_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/arm_sim/ref_angle_rad", qos_profile,
        std::bind(&PropArmGuiNode::simRefAngleCallback, this, std::placeholders::_1));
}

void PropArmGuiNode::setupPublishers()
{
    auto qos_profile = rclcpp::QoS(10)
                           .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                           .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    ref_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/arm/ref_angle_rad", qos_profile);

    pwm_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt16>(
        "/arm/esc/pwm_us", qos_profile);

    auto_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/arm/esc/auto_mode", qos_profile);
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

void PropArmGuiNode::armAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_data_.system_timestamp = getSystemTimestamp();
    current_data_.datetime = QDateTime::currentDateTime();
    current_data_.timestamp = this->get_clock()->now();

    current_data_.arm_angle_deg = msg->data * 180.0 / M_PI;
    current_data_.valid = true;

    current_data_.error = current_data_.ref_angle_deg - current_data_.arm_angle_deg;

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
}

void PropArmGuiNode::motorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_data_.motor_speed_rad_s = msg->data;
    current_data_.system_timestamp = getSystemTimestamp();
    current_data_.datetime = QDateTime::currentDateTime();
    last_data_time_ = this->get_clock()->now();
    updateConnectionStatus();
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
}

void PropArmGuiNode::simArmAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.sim_arm_angle_deg = msg->data * 180.0 / M_PI;
    QMetaObject::invokeMethod(this, [this]()
                              { emit dataUpdated(); }, Qt::QueuedConnection);
}

void PropArmGuiNode::simMotorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.sim_motor_speed_rad_s = msg->data;
    QMetaObject::invokeMethod(this, [this]()
                              { emit dataUpdated(); }, Qt::QueuedConnection);
}

void PropArmGuiNode::simPwmInputCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.sim_pwm_input_us = static_cast<double>(msg->data);
    current_data_.sim_duty_cycle_percent = calculateDutyCycle(current_data_.sim_pwm_input_us);
    QMetaObject::invokeMethod(this, [this]()
                              { emit dataUpdated(); }, Qt::QueuedConnection);
}

void PropArmGuiNode::simRefAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.sim_ref_angle_deg = msg->data * 180.0 / M_PI;
    QMetaObject::invokeMethod(this, [this]()
                              { emit dataUpdated(); }, Qt::QueuedConnection);
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

void PropArmGuiNode::sendAngleCommand(double angle_rad)
{
    auto msg = std_msgs::msg::Float64();
    msg.data = angle_rad;
    ref_angle_pub_->publish(msg);

    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.ref_angle_deg = angle_rad * 180.0 / M_PI;
}

void PropArmGuiNode::sendPWMCommand(uint16_t pwm_us)
{
    auto msg = std_msgs::msg::UInt16();
    msg.data = pwm_us;
    pwm_cmd_pub_->publish(msg);
}

void PropArmGuiNode::sendAutoModeCommand(bool auto_mode)
{
    auto msg = std_msgs::msg::Bool();
    msg.data = auto_mode;
    auto_mode_pub_->publish(msg);

    control_mode_ = auto_mode ? "Automatic" : "Manual";
}

void PropArmGuiNode::sendStopCommand()
{
    control_mode_ = "STOPPED";

    auto angle_msg = std_msgs::msg::Float64();
    angle_msg.data = current_data_.arm_angle_deg * M_PI / 180.0;
    ref_angle_pub_->publish(angle_msg);

    auto pwm_msg = std_msgs::msg::UInt16();
    pwm_msg.data = 1000;
    pwm_cmd_pub_->publish(pwm_msg);

    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.ref_angle_deg = current_data_.arm_angle_deg;
}

void PropArmGuiNode::startStepTest(double angle_low_deg, double angle_high_deg, double time_up_s, double time_down_s)
{
    step_angle_low_deg_ = angle_low_deg;
    step_angle_high_deg_ = angle_high_deg;
    step_time_up_s_ = time_up_s;
    step_time_down_s_ = time_down_s;
    step_high_phase_ = false;
    step_phase_start_ = this->get_clock()->now();
    step_test_active_ = true;

    if (!step_timer_)
    {
        step_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PropArmGuiNode::stepTestTimerCallback, this));
    }

    sendAngleCommand(step_angle_low_deg_ * M_PI / 180.0);
}

void PropArmGuiNode::stopStepTest()
{
    step_test_active_ = false;
    if (step_timer_)
    {
        step_timer_->cancel();
        step_timer_.reset();
    }
}

void PropArmGuiNode::stepTestTimerCallback()
{
    if (!step_test_active_)
        return;

    auto now = this->get_clock()->now();
    double elapsed = (now - step_phase_start_).seconds();
    double phase_duration = step_high_phase_ ? step_time_up_s_ : step_time_down_s_;

    if (elapsed >= phase_duration)
    {
        step_high_phase_ = !step_high_phase_;
        step_phase_start_ = now;

        double target_angle = step_high_phase_ ? step_angle_high_deg_ : step_angle_low_deg_;
        sendAngleCommand(target_angle * M_PI / 180.0);
    }
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

void PropArmGuiNode::startRecording(double duration_seconds)
{
    if (data_recorder_)
    {
        data_recorder_->startRecording(duration_seconds);
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