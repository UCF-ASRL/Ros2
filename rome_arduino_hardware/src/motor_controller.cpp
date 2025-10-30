#include "rome_arduino_hardware/motor_controller.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <termios.h>
#include <iostream>
#include <fcntl.h>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rome_arduino_hardware
{
hardware_interface::CallbackReturn ROMEARD::on_init(const hardware_interface::HardwareInfo & info)
{
  // Get Robot Info
  info_ = info;
  init_variables();

  //Add this
  try
  {
  

    std::string port = "/dev/ttyACM1";
    SerialPort = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (SerialPort < 0)
    {

        RCLCPP_WARN(rclcpp::get_logger("arduino_controller_interface"), 
                    "Unable to open serial port %s. Error: %s", port.c_str(), strerror(errno));
        RCLCPP_WARN(rclcpp::get_logger("arduino_controller_interface"), 
                    "Controller will run in simulation mode.");
        return CallbackReturn::SUCCESS;  // Continue even if Arduino is not connected
    }

    if (tcgetattr(SerialPort, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("arduino_controller_interface"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return CallbackReturn::ERROR;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    speed_t speed = B115200;
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tcflush(SerialPort, TCIFLUSH);
    if (tcsetattr(SerialPort, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CustomHardware"), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("CustomHardware"), "SERIAL PORT OPENED: %d! WAITING...", SerialPort);

    //Wait
    std::this_thread::sleep_for(std::chrono::seconds(3));

  }

  catch(std::exception &e)
  {

    RCLCPP_WARN(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error during initialization: %s. Running in simulation mode.", e.what()
    );
    return CallbackReturn::SUCCESS;  // Continue even if there's an error
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Resize variables for number of wheels
void ROMEARD::init_variables() {
  // resize vectors
  int num_joints = info_.joints.size();
  RCLCPP_INFO(rclcpp::get_logger("ROME_ARDUINO"), "Joint Count: %i!", num_joints);

  velocity_commands_.resize(num_joints);
  prev_velocity_commands_.resize(num_joints);
  velocity_states_.resize(num_joints);
  position_states_.resize(num_joints);

}

// Activate and Deactivate
hardware_interface::CallbackReturn ROMEARD::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Starting robot hardware ...");
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn ROMEARD::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
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
std::vector<hardware_interface::StateInterface> ROMEARD::export_state_interfaces()
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


std::vector<hardware_interface::CommandInterface> ROMEARD::export_command_interfaces()
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
int ROMEARD::WriteToSerial(const unsigned char* buf, int nBytes)
{
    return ::write(SerialPort, const_cast<unsigned char*>(buf), nBytes);
}

int ROMEARD::ReadSerial(unsigned char* buf, int nBytes)
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
// Read From Serial (Maybe)
hardware_interface::return_type ROMEARD::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // For now, assume perfect control (simulated feedback)
    for (size_t i = 0; i < velocity_commands_.size(); i++) {
        velocity_states_[i] = velocity_commands_[i]; // pretend measured = commanded
        position_states_[i] += velocity_commands_[i] * period.seconds(); // integrate position
    }
    return hardware_interface::return_type::OK;
}

// Write To Serial
hardware_interface::return_type ROMEARD::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  //RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing");

  try{
    // Get the four velocity commands
    //Right now, ROS 1 = Physical 3
    //           ROS 2 = Physical 2
    //           ROS 3 = Physical 1
    //           ROS 4 = Physical 4
    float rpmValue3 = static_cast<float>(velocity_commands_.at(0));
    //int dirValue1 = (rpmValue1 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    float rpmValue2 = static_cast<float>(velocity_commands_.at(1));
    //int dirValue2 = (rpmValue2 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    float rpmValue1 = static_cast<float>(velocity_commands_.at(2));
    //int dirValue3 = (rpmValue3 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    float rpmValue4 = static_cast<float>(velocity_commands_.at(3));
    //int dirValue4 = (rpmValue4 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    // Create a string with the command data
    std::ostringstream ss;
    //ss << std::fixed << std:: setprecision(2)
    //  << rpmValue1 << " " << dirValue1 << " "
    //  << rpmValue2 << " " << dirValue2 << " "
    //  << rpmValue3 << " " << dirValue3 << " "
    //  << rpmValue4 << " " << dirValue4 << "\n";
    ss << std::fixed << std::setprecision(1) << rpmValue1 << " " << rpmValue2 << " " << rpmValue3 << " " << rpmValue4 << "\n";

    std::string data = ss.str();


    //std::string data = std::to_string(rpmValue1) + " " + std::to_string(dirValue1) + " " +
    //                   std::to_string(rpmValue2) + " " + std::to_string(dirValue2) + " " +
    //                   std::to_string(rpmValue3) + " " + std::to_string(dirValue3) + " " +
    //                   std::to_string(rpmValue4) + " " + std::to_string(dirValue4) + "\n";

    // Write the command data to the serial port
    WriteToSerial(reinterpret_cast<const unsigned char*>(data.c_str()), data.length());
    tcdrain(SerialPort);
    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing %s", data.c_str());

  }
  catch (const std::exception& e) {
    // Handle any exceptions that occur during the write process
    RCLCPP_FATAL(rclcpp::get_logger("arduino_actuator_interface"), "Error: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  // Log the left and right joint commands for debugging purposes
  //RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Motor 1: %.2f, Direction: %d", velocity_commands_.at(0), (velocity_commands_.at(0) >= 0) ? 0 : 1);
  //RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Motor 2: %.2f, Direction: %d", velocity_commands_.at(1), (velocity_commands_.at(1) >= 0) ? 0 : 1);
  //RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Motor 3: %.2f, Direction: %d", velocity_commands_.at(2), (velocity_commands_.at(2) >= 0) ? 0 : 1);
  //RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Motor 4: %.2f, Direction: %d", velocity_commands_.at(3), (velocity_commands_.at(3) >= 0) ? 0 : 1);

  return hardware_interface::return_type::OK;
}

}  // namespace rome_arduino_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rome_arduino_hardware::ROMEARD, hardware_interface::SystemInterface)
