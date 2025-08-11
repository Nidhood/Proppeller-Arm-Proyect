#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <QObject>
#include <QTimer>
#include <memory>
#include <deque>
#include <mutex>

struct PropArmData
{
    double arm_angle_deg = 0.0;
    double motor_speed_est = 0.0;
    double v_emf = 0.0;
    double delta_v_emf = 0.0;
    double motor_command = 0.0;
    double error = 0.0;
    double target_angle = 0.0;

    rclcpp::Time timestamp;
    bool valid = false;
};

class PropArmGuiNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit PropArmGuiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~PropArmGuiNode() = default;

    // Data access methods
    PropArmData getCurrentData() const;
    std::vector<PropArmData> getHistoryData(size_t max_points = 1000) const;

    // Command methods
    void sendAngleCommand(double angle_degrees);
    void sendForceCommand(double force_newtons);
    void sendVelocityCommand(double velocity_rad_s);
    void sendStopCommand();

    // Status methods
    bool isConnected() const;
    std::string getConnectionStatus() const;
    std::string getControlMode() const;

signals:
    void dataUpdated();
    void connectionChanged(bool connected);
    void errorOccurred(const QString &error);

private slots:
    void processRosEvents();

private:
    void setupSubscribers();
    void setupPublishers();
    void updateConnectionStatus();

    // Callback methods
    void armAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void motorSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void vEmfCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void deltaVEmfCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_cmd_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr arm_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motor_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr v_emf_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr delta_v_emf_sub_;

    // Data management
    mutable std::mutex data_mutex_;
    PropArmData current_data_;
    std::deque<PropArmData> history_data_;
    static constexpr size_t MAX_HISTORY_SIZE = 5000;

    // Connection management
    QTimer *ros_timer_;
    rclcpp::Time last_data_time_;
    bool connected_ = false;
    std::string control_mode_ = "Manual";
    static constexpr double CONNECTION_TIMEOUT = 2.0; // seconds

    // Parameters
    double target_angle_deg_ = 0.0;
    double max_angle_deg_ = 90.0;
    double min_angle_deg_ = -90.0;
    double max_force_n_ = 45.0;
    double max_velocity_rad_s_ = 785.0;
};