#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <memory>
#include <thread>

#include "imu_tools/serialib.h"

class imu_publisher : public rclcpp::Node
{
public:
    imu_publisher() : Node("imu_publisher") {

        this->declare_parameter("publish_calibrated", true);
        // --- serial ---
        this->declare_parameter("serial_port",       "/dev/ttyUSB0");
        this->declare_parameter("baud_rate",          115200);
        this->declare_parameter("buffer_size",        255);
        this->declare_parameter("timer_interval_ms",  10);

        // --- gyro offsets (rad/s) ---
        this->declare_parameter("gyro.offset_x",  0.0);
        this->declare_parameter("gyro.offset_y",  0.0);
        this->declare_parameter("gyro.offset_z",  0.0);

        // --- accel: offset (m/s²), scale (m/s²) ---
        this->declare_parameter("accel.offset_x", 0.0);
        this->declare_parameter("accel.offset_y", 0.0);
        this->declare_parameter("accel.offset_z", 0.0);
        this->declare_parameter("accel.scale_x",  1.0);
        this->declare_parameter("accel.scale_y",  1.0);
        this->declare_parameter("accel.scale_z",  1.0);

        // --- mag: offset (T), scale (1/T) ---
        this->declare_parameter("mag.offset_x",   0.0);
        this->declare_parameter("mag.offset_y",   0.0);
        this->declare_parameter("mag.offset_z",   0.0);
        this->declare_parameter("mag.scale_x",    1.0);
        this->declare_parameter("mag.scale_y",    1.0);
        this->declare_parameter("mag.scale_z",    1.0);

        // --- read all ---
        publish_calibrated_ = this->get_parameter("publish_calibrated").as_bool();
        serial_port_   = this->get_parameter("serial_port").as_string();
        baud_rate_     = this->get_parameter("baud_rate").as_int();
        buffer_size_   = this->get_parameter("buffer_size").as_int();
        int interval   = this->get_parameter("timer_interval_ms").as_int();

        g_off_[0] = this->get_parameter("gyro.offset_x").as_double();
        g_off_[1] = this->get_parameter("gyro.offset_y").as_double();
        g_off_[2] = this->get_parameter("gyro.offset_z").as_double();

        a_off_[0] = this->get_parameter("accel.offset_x").as_double();
        a_off_[1] = this->get_parameter("accel.offset_y").as_double();
        a_off_[2] = this->get_parameter("accel.offset_z").as_double();
        a_sc_[0]  = this->get_parameter("accel.scale_x").as_double();
        a_sc_[1]  = this->get_parameter("accel.scale_y").as_double();
        a_sc_[2]  = this->get_parameter("accel.scale_z").as_double();

        m_off_[0] = this->get_parameter("mag.offset_x").as_double();
        m_off_[1] = this->get_parameter("mag.offset_y").as_double();
        m_off_[2] = this->get_parameter("mag.offset_z").as_double();
        m_sc_[0]  = this->get_parameter("mag.scale_x").as_double();
        m_sc_[1]  = this->get_parameter("mag.scale_y").as_double();
        m_sc_[2]  = this->get_parameter("mag.scale_z").as_double();

        RCLCPP_INFO(this->get_logger(), "Publishing %s data", publish_calibrated_ ? "calibrated" : "raw");
        RCLCPP_INFO(this->get_logger(), "Calibration loaded:");
        RCLCPP_INFO(this->get_logger(), "  Gyro  offsets: [%.6f, %.6f, %.6f]", g_off_[0], g_off_[1], g_off_[2]);
        RCLCPP_INFO(this->get_logger(), "  Accel offsets: [%.6f, %.6f, %.6f]", a_off_[0], a_off_[1], a_off_[2]);
        RCLCPP_INFO(this->get_logger(), "  Accel scales:  [%.6f, %.6f, %.6f]", a_sc_[0],  a_sc_[1],  a_sc_[2]);
        RCLCPP_INFO(this->get_logger(), "  Mag   offsets: [%.6e, %.6e, %.6e]", m_off_[0], m_off_[1], m_off_[2]);
        RCLCPP_INFO(this->get_logger(), "  Mag   scales:  [%.6e, %.6e, %.6e]", m_sc_[0],  m_sc_[1],  m_sc_[2]);

        if (serial_init() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Cannot open serial port, shutting down.");
            rclcpp::shutdown();
            return;
        }
        serial_.flushReceiver();

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval),
            std::bind(&imu_publisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "imu_publisher started, publishing on /imu/data_raw and /imu/mag");
    }

    ~imu_publisher() {
        serial_.closeDevice();
    }

private:
    // ------------------------------------------------------------------ serial
    int serial_init() {
        char err = serial_.openDevice(serial_port_.c_str(), baud_rate_);
        if (err != 1) {
            RCLCPP_ERROR(this->get_logger(), "Error opening %s: %d", serial_port_.c_str(), (int)err);
            return -1;
        }
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened.", serial_port_.c_str());
        return 0;
    }

    // ------------------------------------------------------------------ read
    // Returns 9 raw doubles: ax ay az gx gy gz mx my mz
    // Returns empty vector on error / incomplete frame
    std::vector<double> read_raw() {
        char raw[256] = {0};
        int result = serial_.readString(raw, '\n', buffer_size_, 20);
        if (result <= 0) return {};

        std::string line(raw);
        while (!line.empty() &&
               (line.back() == '\n' || line.back() == '\r' || line.back() == ' '))
            line.pop_back();

        std::vector<double> vals;
        std::stringstream ss(line);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            if (!tok.empty()) {
                try { vals.push_back(std::stod(tok)); }
                catch (...) { return {}; }
            }
        }
        if (vals.size() != 9) return {};
        return vals;
    }

    // ------------------------------------------------------------------ apply calibration
    // Accel: calibrated = (raw - offset) / scale
    // Gyro:  calibrated = raw - offset
    // Mag:   calibrated = (raw - offset) * scale
    void apply_cal(const std::vector<double>& raw,
                   double& ax, double& ay, double& az,
                   double& gx, double& gy, double& gz,
                   double& mx, double& my, double& mz)
    {
        ax = (raw[0] - a_off_[0]) / a_sc_[0];
        ay = (raw[1] - a_off_[1]) / a_sc_[1];
        az = (raw[2] - a_off_[2]) / a_sc_[2];

        gx = raw[3] - g_off_[0];
        gy = raw[4] - g_off_[1];
        gz = raw[5] - g_off_[2];

        mx = (raw[6] - m_off_[0]) * m_sc_[0];
        my = (raw[7] - m_off_[1]) * m_sc_[1];
        mz = (raw[8] - m_off_[2]) * m_sc_[2];
    }

    // ------------------------------------------------------------------ timer
    void timer_callback() {
        auto raw = read_raw();
        if (raw.empty()) return;

        double ax, ay, az, gx, gy, gz, mx, my, mz;
        if (publish_calibrated_) {
            apply_cal(raw, ax, ay, az, gx, gy, gz, mx, my, mz);
        }
        else {
            ax = raw[0];
            ay = raw[1];
            az = raw[2];
            gx = raw[3];
            gy = raw[4];
            gz = raw[5];
            mx = raw[6];
            my = raw[7];
            mz = raw[8];
        }

        auto stamp = this->now();

        // IMU message
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp    = stamp;
        imu_msg.header.frame_id = "imu_link";

        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;

        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;

        // orientation unknown — fill covariance with -1 to signal that
        imu_msg.orientation_covariance[0] = -1.0;

        imu_pub_->publish(imu_msg);

        // Magnetic field message
        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp    = stamp;
        mag_msg.header.frame_id = "imu_link";

        mag_msg.magnetic_field.x = mx;
        mag_msg.magnetic_field.y = my;
        mag_msg.magnetic_field.z = mz;

        mag_pub_->publish(mag_msg);
    }

    // ------------------------------------------------------------------ members
    serialib serial_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

    std::string serial_port_;
    int baud_rate_;
    int buffer_size_;

    double g_off_[3];
    double a_off_[3], a_sc_[3];
    double m_off_[3], m_sc_[3];
    bool publish_calibrated_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imu_publisher>());
    rclcpp::shutdown();
    return 0;
}