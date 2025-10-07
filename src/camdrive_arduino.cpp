#include "camdrive_arduino/camdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


CamDriveArduino::CamDriveArduino()
  : logger_(rclcpp::get_logger("CamDriveArduino"))
{}

return_type CamDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name    = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name   = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate          = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device             = info_.hardware_parameters["device"];
  cfg_.baud_rate          = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout            = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
 
  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);
    
  status_ = hardware_interface::status::CONFIGURED;
  RCLCPP_INFO(logger_, "Finished Configuration");
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> CamDriveArduino::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CamDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));
  return command_interfaces;
}

return_type CamDriveArduino::start()
{
  RCLCPP_INFO(logger_, "Starting Controller (CamDriveArduino) ...");

  // === Tambahan Baru: Subscriber line_reference ===
  node_ = std::make_shared<rclcpp::Node>("camdrive_arduino_node");
  
  line_ref_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
    "/line_reference", 10,
    [this](const geometry_msgs::msg::Point::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(ref_mutex_);
      latest_ref_ = *msg;
      has_new_ref_ = true;
    });

  std::thread([this]() {
      RCLCPP_INFO(logger_, "Spinning CamDriveArduino internal node...");
      rclcpp::spin(node_);
      RCLCPP_INFO(logger_, "Node spin stopped.");  
      }).detach();
  
  RCLCPP_INFO(logger_, "Subscribed to /line_reference topic");
  // =================================================

  arduino_.sendEmptyMsg();
  last_send_time_ = std::chrono::steady_clock::now();
  // arduino.setPidValues(9,7,0,100);
  // arduino.setPidValues(14,7,0,100);
  //arduino_.setPidValues(30, 20, 0, 100);

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}


return_type CamDriveArduino::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

hardware_interface::return_type CamDriveArduino::read()
{
    
  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;
 
  if (!arduino_.connected()) {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000, "Arduino not connected");
    return return_type::ERROR;
  }
  /*
  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }*/

  arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

  double pos_prev = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type CamDriveArduino::write()
{

  if (!arduino_.connected()) {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000, "Arduino not connected");
    return return_type::ERROR;
  }
  auto now = std::chrono::steady_clock::now();
  const auto min_interval = std::chrono::milliseconds(40); // 25 Hz max
  if (now - last_send_time_ < min_interval)
    return return_type::OK;
    
  /*
  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }
  */
  
  if (has_new_ref_)
  {
    std::lock_guard<std::mutex> lock(ref_mutex_);
    arduino_.setLineReference(latest_ref_.x, latest_ref_.y);
    has_new_ref_ = false;
    last_send_time_ = now;
  }

  return return_type::OK; 
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  CamDriveArduino,
  hardware_interface::SystemInterface
)
