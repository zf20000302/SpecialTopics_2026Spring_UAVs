#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <actuator_msgs/msg/actuators.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

using namespace std::chrono_literals;

class HoverNode : public rclcpp::Node
{
public:
    HoverNode() : Node("hover_node")
    {
        target_height_ = this->declare_parameter<double>("target_height", 3.0);
        hover_time_ = this->declare_parameter<double>("hover_time", 30.0);

        omega_min_ = this->declare_parameter<double>("omega_min", 0.0);
        omega_max_ = this->declare_parameter<double>("omega_max", 900.0);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/X3/odometry", 10,
            std::bind(&HoverNode::odom_callback, this, std::placeholders::_1));

        motor_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(
            "/X3/gazebo/command/motor_speed", 10);

        start_time_ = this->now();

        log_file_.open("hover_log.csv");
        log_file_ << "time,z,vz,motor0,motor1,motor2,motor3\n";

        timer_ = this->create_wall_timer(
            10ms, std::bind(&HoverNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Hover node started.");
    }

    ~HoverNode()
    {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_z_ = msg->pose.pose.position.z;
        current_vz_ = msg->twist.twist.linear.z;
        has_odom_ = true;
    }

    void timer_callback()
    {
        if (!has_odom_) {
            return;
        }

        const double elapsed = (this->now() - start_time_).seconds();

        if (elapsed > hover_time_) {
            publish_motor_speed(0.0, 0.0, 0.0, 0.0);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Finished. Motors stopped.");
            return;
        }

        const double z_ref = target_height_;
        const double vz_ref = 0.0;

        const double z = current_z_;
        const double vz = current_vz_;

        double omega0 = 0.0;
        double omega1 = 0.0;
        double omega2 = 0.0;
        double omega3 = 0.0;

        // ============================================================
        // TODO: Controller Design
        // ============================================================
        //
        // Goal:
        //   Design a controller to make the UAV hover at z_ref = 3.0 m.
        //
        // Available states:
        //   z  : current height [m]
        //   vz : current vertical velocity [m/s]
        //
        // Output:
        //   omega0, omega1, omega2, omega3 : motor speeds
        //
        // Notes:
        //   - You may use PID, PD, LQR, MPC, sliding mode, etc.
        //   - For pure altitude control, you may set all motors equal.
        //   - Recommended motor speed range: [0, 900].
        //
        // Example structure:
        //
        //   double ez = z_ref - z;
        //   double evz = vz_ref - vz;
        //
        //   double u = ...;
        //
        //   omega0 = ...;
        //   omega1 = ...;
        //   omega2 = ...;
        //   omega3 = ...;
        //
        // ============================================================

        // TODO 1: Compute tracking errors
        double ez = 0.0;
        double evz = 0.0;

        // TODO 2: Design your control law
        double u = 0.0;

        // TODO 3: Convert control signal to motor speeds
        omega0 = 0.0;
        omega1 = 0.0;
        omega2 = 0.0;
        omega3 = 0.0;

        // ============================================================
        // End of Controller Design
        // ============================================================

        omega0 = std::clamp(omega0, omega_min_, omega_max_);
        omega1 = std::clamp(omega1, omega_min_, omega_max_);
        omega2 = std::clamp(omega2, omega_min_, omega_max_);
        omega3 = std::clamp(omega3, omega_min_, omega_max_);

        publish_motor_speed(omega0, omega1, omega2, omega3);
        write_log(elapsed, omega0, omega1, omega2, omega3);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "t=%.2f z=%.2f vz=%.2f motor=[%.1f %.1f %.1f %.1f]",
            elapsed, z, vz, omega0, omega1, omega2, omega3);
    }

    void publish_motor_speed(double omega0, double omega1, double omega2, double omega3)
    {
        actuator_msgs::msg::Actuators msg;
        msg.velocity = {omega0, omega1, omega2, omega3};
        motor_pub_->publish(msg);
    }

    void write_log(
        double time_sec,
        double omega0,
        double omega1,
        double omega2,
        double omega3)
    {
        if (!log_file_.is_open()) {
            return;
        }

        log_file_ << std::fixed << std::setprecision(4)
                  << time_sec << ","
                  << current_z_ << ","
                  << current_vz_ << ","
                  << omega0 << ","
                  << omega1 << ","
                  << omega2 << ","
                  << omega3 << "\n";
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::ofstream log_file_;
    rclcpp::Time start_time_;

    bool has_odom_ = false;

    double target_height_ = 3.0;
    double hover_time_ = 30.0;

    double current_z_ = 0.0;
    double current_vz_ = 0.0;

    double omega_min_ = 0.0;
    double omega_max_ = 900.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoverNode>());
    rclcpp::shutdown();
    return 0;
}
