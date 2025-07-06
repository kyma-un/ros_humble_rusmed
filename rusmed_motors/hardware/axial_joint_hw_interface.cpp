

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <pigpiod_if2.h>
#include <cstring>

#include <rclcpp/executors.hpp>
#include <rusmed_motors/axial_joint_hw_interface.hpp>

// variables
int bus;

namespace
{

} // namespace

namespace rusmed_motors
{

  static constexpr std::size_t POSITION_INTERFACE_INDEX = 0;
  static constexpr std::size_t VELOCITY_INTERFACE_INDEX = 1;
  // JointState doesn't contain an acceleration field, so right now it's not used
  static constexpr std::size_t EFFORT_INTERFACE_INDEX = 3;

  CallbackReturn AxialJointHW::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // /* Open i2c bus /dev/i2c-0 */
    //   if ((bus =  open("/dev/i2c-0", O_RDWR)) == -1) {
    //         return CallbackReturn::ERROR;
    // 	/* Error process */
    // }

    pi = pigpio_start(nullptr, nullptr);

    pic_handle = i2c_open(pi, 1, 0x42, 0);
    if (pic_handle < 0)
    {
      std::cerr << "No se pudo abrir el esclavo I2C\n";
      pigpio_stop(pi);
    }

    // Initialize storage for all joints' standard interfaces, regardless of their existence and set all values to nan
    joint_commands_.resize(standard_interfaces_.size());
    joint_states_.resize(standard_interfaces_.size());
    for (auto i = 0u; i < standard_interfaces_.size(); i++)
    {
      joint_commands_[i].resize(info_.joints.size(), 0.0);
      joint_states_[i].resize(info_.joints.size(), 0.0);
    }

    // Initial command values
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      const auto &component = info_.joints[i];
      for (const auto &interface : component.state_interfaces)
      {
        auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name);
        // If interface name is found in the interfaces list
        if (it != standard_interfaces_.end())
        {
          auto index = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
          // Check the initial_value param is used
          if (!interface.initial_value.empty())
          {
            joint_commands_[index][i] = std::stod(interface.initial_value);
          }
        }
      }
    }

    // Search for mimic joints
    for (auto i = 0u; i < info_.joints.size(); ++i)
    {
      const auto &joint = info_.joints.at(i);
      if (joint.parameters.find("mimic") != joint.parameters.cend())
      {
        const auto mimicked_joint_it = std::find_if(
            info_.joints.begin(), info_.joints.end(),
            [&mimicked_joint = joint.parameters.at("mimic")](const hardware_interface::ComponentInfo &joint_info)
            {
              return joint_info.name == mimicked_joint;
            });
        if (mimicked_joint_it == info_.joints.cend())
        {
          throw std::runtime_error(std::string("Mimicked joint '") + joint.parameters.at("mimic") + "' not found");
        }
        MimicJoint mimic_joint;
        mimic_joint.joint_index = i;
        mimic_joint.mimicked_joint_index =
            static_cast<std::size_t>(std::distance(info_.joints.begin(), mimicked_joint_it));
        auto param_it = joint.parameters.find("multiplier");
        if (param_it != joint.parameters.end())
        {
          mimic_joint.multiplier = std::stod(joint.parameters.at("multiplier"));
        }
        mimic_joints_.push_back(mimic_joint);
      }
    }

    const auto get_hardware_parameter = [this](const std::string &parameter_name, const std::string &default_value)
    {
      if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end())
      {
        return it->second;
      }
      return default_value;
    };

    // Add random ID to prevent warnings about multiple publishers within the same node
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=rusmed_motors_" + info_.name});

    node_ = rclcpp::Node::make_shared("_", options);

    if (auto it = info_.hardware_parameters.find("trigger_joint_command_threshold"); it != info_.hardware_parameters.end())
    {
      trigger_joint_command_threshold_ = std::stod(it->second);
    }

    topic_based_joint_commands_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        get_hardware_parameter("joint_commands_topic", "/robot_joint_commands"), rclcpp::QoS(1));
    topic_based_joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        get_hardware_parameter("joint_states_topic", "/topic_based_joint_commands"), rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr joint_state)
        { latest_joint_state_ = *joint_state; });

    // if the values on the `joint_states_topic` are wrapped between -2*pi and 2*pi (like they are in Isaac Sim)
    // sum the total joint rotation returned on the `joint_states_` interface
    if (get_hardware_parameter("sum_wrapped_joint_states", "false") == "true")
    {
      sum_wrapped_joint_states_ = true;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> AxialJointHW::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Joints' state interfaces
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      const auto &joint = info_.joints[i];
      for (const auto &interface : joint.state_interfaces)
      {
        // Add interface: if not in the standard list then use "other" interface list
        if (!getInterface(joint.name, interface.name, i, joint_states_, state_interfaces))
        {
          throw std::runtime_error("Interface is not found in the standard list.");
        }
      }
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> AxialJointHW::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Joints' state interfaces
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      const auto &joint = info_.joints[i];
      for (const auto &interface : joint.command_interfaces)
      {
        if (!getInterface(joint.name, interface.name, i, joint_commands_, command_interfaces))
        {
          throw std::runtime_error("Interface is not found in the standard list.");
        }
      }
    }

    return command_interfaces;
  }

  hardware_interface::return_type AxialJointHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (rclcpp::ok())
    {
      rclcpp::spin_some(node_);
    }


    char buffer[4];

    // Suponiendo que el esclavo envía 4 bytes en Little Endian

    char reg = 0x00;
   i2c_write_device(pi, pic_handle, &reg, 1);

    i2c_read_device(pi, pic_handle, (char *)buffer, 4);

    // Interpretar bytes como float:
    _Float32 value;
    std::memcpy(&value, buffer, sizeof(float));

    // RCLCPP_INFO(node_->get_logger(), "Value raw: %.3f, in rad: %.3f", value, value * M_PI / 180.0);

    joint_states_[POSITION_INTERFACE_INDEX][0] = value;

    return hardware_interface::return_type::OK;
  }

  template <typename HandleType>
  bool AxialJointHW::getInterface(const std::string &name, const std::string &interface_name,
                                  const size_t vector_index, std::vector<std::vector<double>> &values,
                                  std::vector<HandleType> &interfaces)
  {
    auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface_name);
    if (it != standard_interfaces_.end())
    {
      auto j = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
      interfaces.emplace_back(name, *it, &values[j][vector_index]);
      return true;
    }
    return false;
  }

  hardware_interface::return_type AxialJointHW::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    //    
    unsigned char value = joint_commands_[POSITION_INTERFACE_INDEX][0] > 50.0f ? 0x02 : 0x03;
    int result = i2c_write_device(pi, pic_handle, (char *) &value, 1);
    return hardware_interface::return_type::OK;
  }
} // end namespace topic_based_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rusmed_motors::AxialJointHW, hardware_interface::SystemInterface)