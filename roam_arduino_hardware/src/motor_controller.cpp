#include "roam_arduino_hardware/motor_controller.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <termios.h>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace roam_arduino_hardware
{
hardware_interface::CallbackReturn ROAMARD::on_init(const hardware_interface::HardwareInfo & info)
{
  // Get Robot Info
  info_ = info;
  init_variables();
  

  // Show How Many Joints it reads
 

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Resize variables for number of wheels
void ROAMARD::init_variables() {
  // resize vectors
  int num_joints = info_.joints.size();
  RCLCPP_INFO(rclcpp::get_logger("ROAM_ARDUINO"), "Joint Count: %i!", num_joints);

  velocity_commands_.resize(num_joints);
  prev_velocity_commands_.resize(num_joints);
  velocity_states_.resize(num_joints);
  position_states_.resize(num_joints);

}

// Activate and Deactivate
hardware_interface::CallbackReturn ROAMARD::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Starting robot hardware ...");
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn ROAMARD::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if(SerialPort == -1)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    tcflush(SerialPort, TCIFLUSH);
    close(SerialPort);
    return hardware_interface::CallbackReturn::SUCCESS;
}

// Get State And Command Interfaces
std::vector<hardware_interface::StateInterface> ROAMARD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;


  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> ROAMARD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

//Write and Read Init
int ROAMARD::WriteToSerial(const unsigned char* buf, int nBytes)
{
    return ::write(SerialPort, const_cast<unsigned char*>(buf), nBytes);
}

int ROAMARD::ReadSerial(unsigned char* buf, int nBytes)
{
    auto t_start = std::chrono::high_resolution_clock::now();
    int n = 0;
    while(n < nBytes)
    {
        int ret = ::read(SerialPort, &buf[n], 1);
        if(ret < 0)
        {
            return ret;
        }

        n+=ret;
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed_time_ms > 10000)
        {
            break;
        }
    }
    return n;
}

// Write To Serial
hardware_interface::return_type ROAMARD::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing");

  try{
    // Get the four velocity commands
    float rpmValue1 = static_cast<float>(velocity_commands_.at(0));
    int dirValue1 = (rpmValue1 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    float rpmValue2 = static_cast<float>(velocity_commands_.at(1));
    int dirValue2 = (rpmValue2 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    float rpmValue3 = static_cast<float>(velocity_commands_.at(2));
    int dirValue3 = (rpmValue3 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    float rpmValue4 = static_cast<float>(velocity_commands_.at(3));
    int dirValue4 = (rpmValue4 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    // Create a string with the command data
    std::string data = std::to_string(rpmValue1) + " " + std::to_string(dirValue1) + " " +
                       std::to_string(rpmValue2) + " " + std::to_string(dirValue2) + " " +
                       std::to_string(rpmValue3) + " " + std::to_string(dirValue3) + " " +
                       std::to_string(rpmValue4) + " " + std::to_string(dirValue4) + "\n";

    // Write the command data to the serial port
    WriteToSerial(reinterpret_cast<const unsigned char*>(data.c_str()), data.length());
    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing %s", data.c_str());

    // Throttle the data transfer to avoid overwhelming the Arduino
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust the sleep duration as needed
  }
  catch (const std::exception& e) {
    // Handle any exceptions that occur during the write process
    RCLCPP_FATAL(rclcpp::get_logger("arduino_actuator_interface"), "Error: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  // Log the left and right joint commands for debugging purposes
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Motor 1: %.2f, Direction: %d", velocity_commands_.at(0), (velocity_commands_.at(0) >= 0) ? 0 : 1);
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Motor 2: %.2f, Direction: %d", velocity_commands_.at(1), (velocity_commands_.at(1) >= 0) ? 0 : 1);
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Motor 3: %.2f, Direction: %d", velocity_commands_.at(2), (velocity_commands_.at(2) >= 0) ? 0 : 1);
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Motor 4: %.2f, Direction: %d", velocity_commands_.at(3), (velocity_commands_.at(3) >= 0) ? 0 : 1);

  return hardware_interface::return_type::OK;
}

// Read From Serial (Maybe)
hardware_interface::return_type ROAMARD::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}

}  // namespace roam_arduino_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roam_arduino_hardware::ROAMARD, hardware_interface::SystemInterface)
