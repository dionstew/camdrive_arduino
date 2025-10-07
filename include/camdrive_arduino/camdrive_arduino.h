#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

#include <cstring>
#include <mutex>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point.hpp>

//#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
//#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "config.h"
#include "wheel.h"
#include "arduino_comms.h"


using hardware_interface::return_type;

class CamDriveArduino : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{

public:
  CamDriveArduino();
  return_type configure(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  return_type start() override;
  return_type stop() override;
  return_type read() override;
  return_type write() override;

private:
  Config cfg_;
  ArduinoComms arduino_;
  Wheel l_wheel_;
  Wheel r_wheel_;

  rclcpp::Logger logger_;
  std::chrono::time_point<std::chrono::system_clock> time_;
  
   // === Tambahan Baru ===
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr line_ref_sub_;
  geometry_msgs::msg::Point latest_ref_;
  std::mutex ref_mutex_;
  bool has_new_ref_ = false;
  std::chrono::steady_clock::time_point last_send_time_;
};


#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H
