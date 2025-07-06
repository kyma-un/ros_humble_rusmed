#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class IMUTfNode : public rclcpp::Node
{
public:
  IMUTfNode() : Node("imu_tf_node")
  {
    // Subscribir al topic IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_raw", 10,
      std::bind(&IMUTfNode::imu_callback, this, std::placeholders::_1));

    // Crear el broadcaster de TF
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_INFO(this -> get_logger(), "TF2 Frame send. Wait for next value"); 
    // Crear el mensaje de TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "base_link";  // marco padre
    t.child_frame_id = "imu_link";    // marco hijo

    // De ejemplo: usar orientación del IMU como TF puro
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation = msg->orientation;

    // Publicar el TF
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUTfNode>());
  rclcpp::shutdown();
  return 0;
}
