#include "annin_ar4_driver/teensy_driver.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#define FW_VERSION "2.0.1"

namespace annin_ar4_driver {

bool TeensyDriver::init(std::string ar_model, std::string port, int baudrate,
                        int num_joints, bool velocity_control_enabled) {
  // @TODO read version from config
  version_ = FW_VERSION;
  ar_model_ = ar_model;

  // establish connection with teensy board
  boost::system::error_code ec;
  serial_port_.open(port, ec);

  if (ec) {
    RCLCPP_WARN(logger_, "Failed to connect to serial port %s", port.c_str());
    return false;
  } else {
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(
        static_cast<uint32_t>(baudrate)));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    RCLCPP_INFO(logger_, "Successfully connected to serial port %s",
                port.c_str());
  }

  initialised_ = false;
  std::string msg = "STA" + version_ + "B" + ar_model_ + "\n";

  while (!initialised_) {
    RCLCPP_INFO(logger_, "Waiting for response from Teensy on port %s",
                port.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    exchange(msg);
  }
  RCLCPP_INFO(logger_, "Successfully initialised driver on port %s",
              port.c_str());

  // initialise joint and encoder calibration
  num_joints_ = num_joints;
  joint_positions_deg_.resize(num_joints_);
  joint_velocities_deg_.resize(num_joints_);
  enc_calibrations_.resize(num_joints_);
  velocity_control_enabled_ = velocity_control_enabled;
  is_estopped_ = false;
  return true;
}

TeensyDriver::TeensyDriver() : serial_port_(io_service_) {}

// Update between hardware interface and hardware driver
void TeensyDriver::update(std::vector<double>& pos_commands,
                          std::vector<double>& vel_commands,
                          std::vector<double>& joint_positions,
                          std::vector<double>& joint_velocities) {
  // log pos_commands
  std::string logInfo = "Joint Pos Cmd: ";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointPositionStm;
    jointPositionStm << std::fixed << std::setprecision(2) << pos_commands[i];
    logInfo += std::to_string(i) + ": " + jointPositionStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());

  // log vel_commands
  logInfo = "Joint Vel Cmd: ";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointVelocityStm;
    jointVelocityStm << std::fixed << std::setprecision(2) << vel_commands[i];
    logInfo += std::to_string(i) + ": " + jointVelocityStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());

  std::string outMsg = "";
  // construct update message
  if (velocity_control_enabled_) {
    outMsg += "MV";
    for (int i = 0; i < num_joints_; ++i) {
      outMsg += 'A' + i;
      outMsg += std::to_string(vel_commands[i]);
    }
    for (int i = 0; i < num_joints_; ++i) {
      outMsg += 'A' + i + num_joints_;
      outMsg += std::to_string(pos_commands[i]);
    }
  } else {
    outMsg += "MT";
    for (int i = 0; i < num_joints_; ++i) {
      outMsg += 'A' + i;
      outMsg += std::to_string(pos_commands[i]);
    }
  }
  outMsg += "\n";

  // run the communication with board
  exchange(outMsg);

  joint_positions = joint_positions_deg_;
  joint_velocities = joint_velocities_deg_;

  // print joint_positions
  logInfo = "Joint Pos: ";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointPositionStm;
    jointPositionStm << std::fixed << std::setprecision(2)
                     << joint_positions[i];
    logInfo += std::to_string(i) + ": " + jointPositionStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());

  // print joint_velocities
  logInfo = "Joint Vel: ";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointVelocityStm;
    jointVelocityStm << std::fixed << std::setprecision(2)
                     << joint_velocities[i];
    logInfo += std::to_string(i) + ": " + jointVelocityStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
}

bool TeensyDriver::calibrateJoints() {
  std::string outMsg = "JC\n";
  return sendCommand(outMsg);
}

void TeensyDriver::getJointPositions(std::vector<double>& joint_positions) {
  // get current joint positions
  std::string msg = "JP\n";
  exchange(msg);
  joint_positions = joint_positions_deg_;
}

bool TeensyDriver::resetEStop() {
  std::string msg = "RE\n";
  exchange(msg);
  return !is_estopped_;
}

bool TeensyDriver::isEStopped() { return is_estopped_; }

void TeensyDriver::getJointVelocities(std::vector<double>& joint_velocities) {
  // get current joint velocities
  std::string msg = "JV\n";
  exchange(msg);
  joint_velocities = joint_velocities_deg_;
}

bool TeensyDriver::sendCommand(std::string outMsg) { return exchange(outMsg); }

// Send msg to board and collect data
bool TeensyDriver::exchange(std::string outMsg) {
  std::string inMsg;
  std::string errTransmit = "";

  // RCLCPP_INFO(logger_, "Sending message: %s", outMsg.c_str());
  if (!transmit(outMsg, errTransmit)) {
    RCLCPP_ERROR(logger_, "Error in transmit: %s", errTransmit.c_str());
    return false;
  }

  while (true) {
    receive(inMsg);
    std::string header = inMsg.substr(0, 2);

    if (header == "DB") {
      // debug message
      RCLCPP_DEBUG(logger_, "Debug message: %s", inMsg.c_str());
    } else {
      if (header == "ST") {
        // init acknowledgement
        checkInit(inMsg);
      } else if (header == "JC") {
        // encoder calibration values
        updateEncoderCalibrations(inMsg);
      } else if (header == "JP") {
        // encoder steps
        updateJointPositions(inMsg);
      } else if (header == "JV") {
        // encoder steps
        updateJointVelocities(inMsg);
      } else if (header == "ES") {
        // estop status
        updateEStopStatus(inMsg);
      } else if (header == "ER") {
        // error message
        RCLCPP_INFO(logger_, "ERROR message: %s", inMsg.c_str());
        return false;
      } else {
        // unknown header
        RCLCPP_WARN(logger_, "Unknown header %s", header.c_str());
        return false;
      }
      return true;
    }
  }
  return true;
}

bool TeensyDriver::transmit(std::string msg, std::string& err) {
  boost::system::error_code ec;
  const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());

  boost::asio::write(serial_port_, sendBuffer, ec);

  if (!ec) {
    return true;
  } else {
    err = ec.message();
    return false;
  }
}

void TeensyDriver::receive(std::string& inMsg) {
  char c;
  std::string msg = "";
  bool eol = false;
  while (!eol) {
    boost::asio::read(serial_port_, boost::asio::buffer(&c, 1));
    switch (c) {
      case '\r':
        break;
      case '\n':
        eol = true;
        break;
      default:
        msg += c;
    }
  }
  inMsg = msg;
}

void TeensyDriver::checkInit(std::string msg) {
  std::size_t ack_idx = msg.find("A", 2) + 1;
  std::size_t version_idx = msg.find("B", 2) + 1;
  std::size_t ar_model_matched_idx = msg.find("C", 2) + 1;
  std::size_t ar_model_idx = msg.find("D", 2) + 1;
  int ack = std::stoi(msg.substr(ack_idx, version_idx));
  int ar_model_matched =
      std::stoi(msg.substr(ar_model_matched_idx, ar_model_idx));
  if (!ack) {
    std::string version = msg.substr(version_idx);
    RCLCPP_ERROR(logger_, "Firmware version mismatch %s", version.c_str());
  }
  if (!ar_model_matched) {
    std::string ar_model = msg.substr(ar_model_idx);
    RCLCPP_ERROR(logger_, "Model mismatch %s", ar_model.c_str());
  }
  if (ack && ar_model_matched) {
    initialised_ = true;
  }
}

void TeensyDriver::updateJointPositions(const std::string msg) {
  parseValuesToVector(msg, joint_positions_deg_);
}

void TeensyDriver::updateJointVelocities(const std::string msg) {
  parseValuesToVector(msg, joint_velocities_deg_);
}

void TeensyDriver::updateEStopStatus(std::string msg) {
  is_estopped_ = msg.substr(2) == "1" ? true : false;
}

void TeensyDriver::updateEncoderCalibrations(const std::string msg) {
  parseValuesToVector(msg, enc_calibrations_);
}

template <typename T>
void TeensyDriver::parseValuesToVector(const std::string msg,
                                       std::vector<T>& values) {
  values.clear();
  size_t prevIdx = msg.find('A', 2) + 1;

  for (size_t i = 1;; ++i) {
    char currentIdentifier = 'A' + i;
    size_t currentIdx = msg.find(currentIdentifier, 2);

    try {
      if (currentIdx == std::string::npos) {
        if constexpr (std::is_same<T, int>::value) {
          values.push_back(std::stoi(msg.substr(prevIdx)));
        } else if constexpr (std::is_same<T, double>::value) {
          values.push_back(std::stod(msg.substr(prevIdx)));
        }
        break;
      }
      if constexpr (std::is_same<T, int>::value) {
        values.push_back(std::stoi(msg.substr(prevIdx, currentIdx - prevIdx)));
      } else if constexpr (std::is_same<T, double>::value) {
        values.push_back(std::stod(msg.substr(prevIdx, currentIdx - prevIdx)));
      }
    } catch (const std::invalid_argument&) {
      RCLCPP_WARN(logger_, "Invalid argument, can't parse %s", msg.c_str());
    }
    prevIdx = currentIdx + 1;
  }
}

}  // namespace annin_ar4_driver
