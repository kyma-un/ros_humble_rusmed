#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

class I2CNode : public rclcpp::Node
{
public:
  I2CNode() : Node("i2c_node")
  {
    // Abre el bus I2C
    const char *device = "/dev/i2c-1";
    fd_ = open(device, O_RDWR);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir %s", device);
      return;
    }

    addr_ = 0x42; // Dirección I2C del esclavo

    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo configurar esclavo 0x%02x", addr_);
      return;
    }

    sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "i2c_write",
      10,
      std::bind(&I2CNode::writeI2C, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Nodo I2C listo. Esperando datos en 'i2c_write'...");
  }

  ~I2CNode()
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void writeI2C(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    uint8_t value = msg->data;
    if (write(fd_, &value, 1) != 1) {
      RCLCPP_WARN(this->get_logger(), "Fallo al escribir en I2C");
    } else {
      RCLCPP_INFO(this->get_logger(), "Dato enviado I2C: 0x%02X", value);
    }
  }

  int fd_;
  int addr_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<I2CNode>());
  rclcpp::shutdown();
  return 0;
}
