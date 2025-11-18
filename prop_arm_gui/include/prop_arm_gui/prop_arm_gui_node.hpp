#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <QObject>
#include <QTimer>
#include <QDateTime>
#include <memory>
#include <deque>
#include <mutex>
#include <limits>
#include <chrono>

struct PropArmData
{
    double arm_angle_deg = 0.0;
    double motor_speed_rad_s = 0.0;
    double pwm_input_us = 0.0;
    double duty_cycle_percent = 0.0;
    double ref_angle_deg = 0.0;
    double error = 0.0;

    double sim_arm_angle_deg = 0.0;
    double sim_motor_speed_rad_s = 0.0;
    double sim_pwm_input_us = 0.0;
    double sim_duty_cycle_percent = 0.0;
    double sim_ref_angle_deg = 0.0;

    double system_timestamp = 0.0;
    QDateTime datetime;
    rclcpp::Time timestamp;
    bool valid = false;
};

class DataRecorder;

class PropArmGuiNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit PropArmGuiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~PropArmGuiNode() override;

    PropArmData getCurrentData() const;
    std::vector<PropArmData> getHistoryData(size_t max_points = 1000) const;

    void sendAngleCommand(double angle_rad);
    void sendPWMCommand(uint16_t pwm_us);
    void sendAutoModeCommand(bool auto_mode);
    void sendStopCommand();

    bool isConnected() const;
    std::string getConnectionStatus() const;
    std::string getControlMode() const;
    double getSimulationStartTime() const { return system_start_time_; }

    void startRecording(double duration_seconds = 120.0);
    void stopRecording();
    bool isRecording() const;
    std::vector<PropArmData> getRecordedData() const;
    size_t getRecordedPointCount() const;
    double getRecordingDuration() const;
    double getRecordingRemainingTime() const;

    void startStepTest(double angle_low_deg, double angle_high_deg, double time_up_s, double time_down_s);
    void stopStepTest();
    bool isStepTestRunning() const { return step_test_active_; }

signals:
    void dataUpdated();
    void connectionChanged(bool connected);
    void errorOccurred(const QString &error);
    void recordingStarted(double duration);
    void recordingProgress(double remaining_time, size_t point_count);
    void recordingCompleted(size_t point_count, double duration);

private:
    void setupSubscribers();
    void setupPublishers();
    void updateConnectionStatus();
    double getSystemTimestamp() const;
    double calculateDutyCycle(double pwm_us) const;
    void stepTestTimerCallback();

    void armAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void motorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void pwmInputCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void simArmAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void simMotorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void simPwmInputCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void simRefAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ref_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pwm_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_mode_pub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr arm_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motor_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pwm_input_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_arm_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_motor_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sim_pwm_input_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_ref_angle_sub_;

    mutable std::mutex data_mutex_;
    PropArmData current_data_;
    std::deque<PropArmData> history_data_;
    static constexpr size_t MAX_HISTORY_SIZE = 5000;

    std::unique_ptr<DataRecorder> data_recorder_;

    std::chrono::steady_clock::time_point system_start_timepoint_;
    double system_start_time_;

    rclcpp::Time last_data_time_;
    bool connected_ = false;
    std::string control_mode_ = "Manual";
    static constexpr double CONNECTION_TIMEOUT = 2.0;

    bool step_test_active_ = false;
    double step_angle_low_deg_ = 0.0;
    double step_angle_high_deg_ = 45.0;
    double step_time_up_s_ = 5.0;
    double step_time_down_s_ = 5.0;
    bool step_high_phase_ = false;
    rclcpp::Time step_phase_start_;
    rclcpp::TimerBase::SharedPtr step_timer_;

    static constexpr double PWM_PERIOD_US = 20000.0;
};