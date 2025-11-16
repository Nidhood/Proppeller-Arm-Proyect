#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int16.hpp>
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
    double motor_command = 0.0;
    double error = 0.0;
    double target_angle = 0.0;

    // NUEVO: Datos de simulación
    double sim_arm_angle_deg = 0.0;
    double sim_motor_speed_rad_s = 0.0;
    double sim_pwm_input_us = 0.0;
    double sim_duty_cycle_percent = 0.0;

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

    void sendAngleCommand(double angle_degrees);
    void sendVelocityCommand(double velocity_rad_s);
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

    // Callbacks para datos reales
    void armAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void motorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void pwmInputCallback(const std_msgs::msg::UInt16::SharedPtr msg);

    // NUEVO: Callbacks para datos de simulación
    void simArmAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void simMotorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void simPwmInputCallback(const std_msgs::msg::UInt16::SharedPtr msg);

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_cmd_pub_;

    // Subscribers reales
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr arm_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motor_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pwm_input_sub_;

    // NUEVO: Subscribers simulados
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_arm_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_motor_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sim_pwm_input_sub_;

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

    double target_angle_deg_ = 0.0;
    double max_angle_deg_ = 90.0;
    double min_angle_deg_ = -90.0;
    double max_velocity_rad_s_ = 785.0;

    double last_velocity_cmd_ = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Time last_velocity_pub_time_;
    static constexpr double VELOCITY_CMD_MIN_DELTA = 0.5;
    static constexpr double VELOCITY_CMD_DEBOUNCE_S = 0.08;

    static constexpr double PWM_PERIOD_US = 20000.0;
};