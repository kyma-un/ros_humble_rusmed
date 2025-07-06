#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <pigpiod_if2.h>

#include <chrono>
#include <cstring>
#include <cmath>

using namespace std::chrono_literals;

class IMUNode : public rclcpp::Node
{
public:
    IMUNode() : Node("imu_node")
    {
        // Crea el publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

        // Inicia pigpiod_if2
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to start pigpio");
            throw std::runtime_error("Failed to start pigpio");
        }

        // Configura I2C
        i2c_handle_ = i2c_open(pi_, 1, 0x69, 0);
        if (i2c_handle_ < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to open I2C device");
            throw std::runtime_error("Failed to open I2C device");
        }

        // Setup timer
        timer_ = this->create_wall_timer(
            100ms, std::bind(&IMUNode::read_and_publish, this));
    }

    ~IMUNode()
    {
        i2c_close(pi_, i2c_handle_);
        pigpio_stop(pi_);
    }

private:
    void read_and_publish()
    {
        uint8_t buffer[12] = {0};

        // BMI160 accel start register: 0x12
        int reg = 0x12;
        int count = i2c_read_i2c_block_data(pi_, i2c_handle_, reg, (char *)buffer, 12);
        if (count != 12)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read IMU data");
            return;
        }

        int16_t ax = (buffer[1] << 8) | buffer[0];
        int16_t ay = (buffer[3] << 8) | buffer[2];
        int16_t az = (buffer[5] << 8) | buffer[4];
        int16_t gx = (buffer[7] << 8) | buffer[6];
        int16_t gy = (buffer[9] << 8) | buffer[8];
        int16_t gz = (buffer[11] << 8) | buffer[10];

        double accel_scale = 9.81 / 16384.0;      // Simpe example for +-2g
        double gyro_scale = M_PI / 180.0 / 131.0; // example scale for °/s to rad/s

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link"; // Base reference system for TF2

        imu_msg.linear_acceleration.x = ax * accel_scale;
        imu_msg.linear_acceleration.y = ay * accel_scale;
        imu_msg.linear_acceleration.z = az * accel_scale;

        imu_msg.angular_velocity.x = gx * gyro_scale;
        imu_msg.angular_velocity.y = gy * gyro_scale;
        imu_msg.angular_velocity.z = gz * gyro_scale;

        // Calculo de direccion
        double ax_si = ax * accel_scale;
        double ay_si = ay * accel_scale;
        double az_si = az * accel_scale;

        // Roll y Pitch
        double roll = atan2(ay_si, az_si);
        double pitch = atan2(-ax_si, sqrt(ay_si * ay_si + az_si * az_si));

        // Yaw es desconocido => 0
        double yaw = 0.0;

        // Convertir a quaternion (Roll-Pitch-Yaw → Quaternion)
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

        // Indica covarianza válida
        imu_msg.orientation_covariance[0] = 0.1; // ajusta según precisión

        imu_pub_->publish(imu_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int pi_;
    int i2c_handle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}
