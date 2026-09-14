// My main node for validation testing

#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <sstream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

// Control table addresses for XM430
#define ADDR_OPERATING_MODE      11
#define ADDR_CURRENT_LIMIT       38
#define ADDR_TORQUE_ENABLE       64
#define ADDR_HARDWARE_ERROR      70
#define ADDR_GOAL_CURRENT       102
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_CURRENT    126
#define ADDR_PRESENT_POSITION   132

#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0

// Current-based position control mode
#define CURRENT_BASED_POSITION_CONTROL_MODE 5

// XM430 position resolution is 4096 ticks per mechanical revolution.
#define TICKS_PER_REVOLUTION    4096
#define COMMAND_MOVE_DEGREES    70   // 70 degree for AP gripper, 60 for TF gripper
#define COMMAND_STEP_TICKS      static_cast<int>((TICKS_PER_REVOLUTION * COMMAND_MOVE_DEGREES) / 360.0)
#define COMMAND_MOVE_DEGREES1   20
#define COMMAND_STEP_TICKS1     static_cast<int>((TICKS_PER_REVOLUTION * COMMAND_MOVE_DEGREES1) / 360.0)

#define PROTOCOL_VERSION        2.0

#define DXL_ID                  0
#define BAUDRATE                57600
#define DEVICE_NAME             "/dev/ttyUSB0"

// MoveIt/URDF gripper joint
#define GRIPPER_JOINT_NAME      "slider1_joint"

// URDF prismatic joint limits
#define GRIPPER_CLOSED_POS      0.06
#define GRIPPER_OPEN_POS        0.0

// XM430 current unit
#define CURRENT_UNIT_MA         2.69

// Approximate torque constant for XM430-W350-R
#define TORQUE_CONSTANT_NM_PER_A 1.78

// Safety limit for commanded current units
#define MAX_CURRENT_UNITS       300
#define DEFAULT_CURRENT_UNITS   100

class DynamixelPositionNode : public rclcpp::Node
{
public:
  DynamixelPositionNode()
  : Node("dynamixel_position_node"),
    dxl_id_(DXL_ID),
    baudrate_(BAUDRATE),
    gripper_position_(GRIPPER_CLOSED_POS),
    gripper_target_(GRIPPER_CLOSED_POS),
    commanded_current_units_(DEFAULT_CURRENT_UNITS)
  {
    RCLCPP_INFO(this->get_logger(), "Starting Dynamixel Position Node");

    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler_->openPort()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open port");
      throw std::runtime_error("Failed to open port");
    }

    if (!portHandler_->setBaudRate(baudrate_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to set baudrate");
      throw std::runtime_error("Failed to set baudrate");
    }

    uint16_t model_number = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->ping(
      portHandler_, dxl_id_, &model_number, &dxl_error);

    if (!check_comm_result(dxl_comm_result, dxl_error, "ping")) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not ping ID %u at baud %d. Trying auto-detect...",
        dxl_id_,
        baudrate_);

      if (!auto_detect_motor()) {
        throw std::runtime_error("ping");
      }

      dxl_error = 0;
      dxl_comm_result = packetHandler_->ping(
        portHandler_, dxl_id_, &model_number, &dxl_error);
      check_comm_result_or_throw(dxl_comm_result, dxl_error, "ping after auto-detect");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Connected to Dynamixel ID %u, model: %u",
      dxl_id_,
      model_number);

    // Disable torque before changing mode
    dxl_comm_result = packetHandler_->write1ByteTxRx(
      portHandler_, dxl_id_, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    check_comm_result_or_throw(dxl_comm_result, dxl_error, "disable torque before mode set");

    // Set current-based position control mode
    dxl_comm_result = packetHandler_->write1ByteTxRx(
      portHandler_,
      dxl_id_,
      ADDR_OPERATING_MODE,
      CURRENT_BASED_POSITION_CONTROL_MODE,
      &dxl_error);
    check_comm_result_or_throw(dxl_comm_result, dxl_error, "set current-based position control mode");

    // Set maximum allowed current
    dxl_comm_result = packetHandler_->write2ByteTxRx(
      portHandler_,
      dxl_id_,
      ADDR_CURRENT_LIMIT,
      MAX_CURRENT_UNITS,
      &dxl_error);
    check_comm_result_or_throw(dxl_comm_result, dxl_error, "set current limit");

    // Set default goal current
    dxl_comm_result = packetHandler_->write2ByteTxRx(
      portHandler_,
      dxl_id_,
      ADDR_GOAL_CURRENT,
      DEFAULT_CURRENT_UNITS,
      &dxl_error);
    check_comm_result_or_throw(dxl_comm_result, dxl_error, "set default goal current");

    // Enable torque
    dxl_comm_result = packetHandler_->write1ByteTxRx(
      portHandler_, dxl_id_, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    check_comm_result_or_throw(dxl_comm_result, dxl_error, "enable torque");

    RCLCPP_INFO(this->get_logger(), "Torque enabled");
    RCLCPP_INFO(this->get_logger(), "Current limit: %d units", MAX_CURRENT_UNITS);

    command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/motor_control",
      10,
      std::bind(&DynamixelPositionNode::on_command, this, std::placeholders::_1));

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states",
      10);

    joint_state_timer_ = this->create_wall_timer(
      50ms,
      std::bind(&DynamixelPositionNode::publish_gripper_joint_state, this));

    current_print_timer_ = this->create_wall_timer(
      500ms,
      std::bind(&DynamixelPositionNode::read_and_print_present_current, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Listening on /motor_control. Use: 'forward', 'backward', 'forward1', or 'backward1'");
  }

  ~DynamixelPositionNode()
  {
    uint8_t dxl_error = 0;
    packetHandler_->write1ByteTxRx(
      portHandler_, dxl_id_, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    portHandler_->closePort();
  }

private:
  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string direction;
    int requested_current = DEFAULT_CURRENT_UNITS;

    std::stringstream ss(msg->data);
    ss >> direction;

    if (ss >> requested_current) {
      requested_current = std::clamp(requested_current, 0, MAX_CURRENT_UNITS);
    } else {
      requested_current = DEFAULT_CURRENT_UNITS;
    }

    if (direction != "forward" && direction != "backward" &&
      direction != "forward1" && direction != "backward1") {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring unknown command: '%s'. Use 'forward', 'backward', 'forward1', or 'backward1'",
        msg->data.c_str());
      return;
    }

    commanded_current_units_ = requested_current;

    uint8_t dxl_error = 0;
    int dxl_comm_result = 0;

    dxl_comm_result = packetHandler_->write2ByteTxRx(
      portHandler_,
      dxl_id_,
      ADDR_GOAL_CURRENT,
      commanded_current_units_,
      &dxl_error);

    if (!check_comm_result(dxl_comm_result, dxl_error, "write goal current")) {
      return;
    }

    uint32_t present_position = 0;
    dxl_comm_result = packetHandler_->read4ByteTxRx(
      portHandler_, dxl_id_, ADDR_PRESENT_POSITION, &present_position, &dxl_error);

    if (!check_comm_result(dxl_comm_result, dxl_error, "read present position")) {
      return;
    }

    const bool is_forward = (direction == "forward" || direction == "forward1");
    const int step_ticks = (direction == "forward1" || direction == "backward1")
      ? COMMAND_STEP_TICKS1
      : COMMAND_STEP_TICKS;

    int32_t goal_position = 0;

    if (is_forward) {
      goal_position = static_cast<int32_t>(present_position) + step_ticks;
      gripper_target_ = GRIPPER_CLOSED_POS;
    } else {
      goal_position = static_cast<int32_t>(present_position) - step_ticks;
      gripper_target_ = GRIPPER_OPEN_POS;
    }

    dxl_comm_result = packetHandler_->write4ByteTxRx(
      portHandler_, dxl_id_, ADDR_GOAL_POSITION, goal_position, &dxl_error);

    if (!check_comm_result(dxl_comm_result, dxl_error, "write goal position")) {
      return;
    }

    uint8_t hw_error = 0;
    dxl_comm_result = packetHandler_->read1ByteTxRx(
      portHandler_, dxl_id_, ADDR_HARDWARE_ERROR, &hw_error, &dxl_error);

    if (!check_comm_result(dxl_comm_result, dxl_error, "read hardware error status")) {
      return;
    }

    if (hw_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "Hardware error status is non-zero: 0x%02X", hw_error);
    }

    double commanded_current_A = current_units_to_ampere(commanded_current_units_);
    double commanded_torque_Nm = current_to_torque_nm(commanded_current_A);

    RCLCPP_INFO(
      this->get_logger(),
      "Command: %s | Goal position: %d | Present position: %u | Commanded current: %d units, %.3f A | Approx commanded motor torque: %.3f Nm | Target slider1_joint: %.3f | HW err: 0x%02X",
      direction.c_str(),
      goal_position,
      present_position,
      commanded_current_units_,
      commanded_current_A,
      commanded_torque_Nm,
      gripper_target_,
      hw_error);
  }

  void publish_gripper_joint_state()
  {
    const double step = 0.002;

    if (gripper_position_ < gripper_target_) {
      gripper_position_ = std::min(gripper_position_ + step, gripper_target_);
    } else if (gripper_position_ > gripper_target_) {
      gripper_position_ = std::max(gripper_position_ - step, gripper_target_);
    }

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->now();
    joint_msg.name.push_back(GRIPPER_JOINT_NAME);
    joint_msg.position.push_back(gripper_position_);
    joint_msg.velocity.push_back(0.0);
    joint_msg.effort.push_back(0.0);

    joint_state_publisher_->publish(joint_msg);
  }

  void read_and_print_present_current()
  {
    uint8_t dxl_error = 0;
    uint16_t raw_current = 0;

    int dxl_comm_result = packetHandler_->read2ByteTxRx(
      portHandler_,
      dxl_id_,
      ADDR_PRESENT_CURRENT,
      &raw_current,
      &dxl_error);

    if (!check_comm_result(dxl_comm_result, dxl_error, "read present current")) {
      return;
    }

    int16_t signed_current_units = static_cast<int16_t>(raw_current);

    double present_current_A = current_units_to_ampere(signed_current_units);
    double present_torque_Nm = current_to_torque_nm(present_current_A);

    RCLCPP_INFO(
      this->get_logger(),
      "Present Current: %d units | %.3f A | Approx motor torque: %.3f Nm",
      signed_current_units,
      present_current_A,
      present_torque_Nm);
  }

  double current_units_to_ampere(int current_units)
  {
    return static_cast<double>(current_units) * CURRENT_UNIT_MA / 1000.0;
  }

  double current_to_torque_nm(double current_A)
  {
    return current_A * TORQUE_CONSTANT_NM_PER_A;
  }

  bool auto_detect_motor()
  {
    const int baud_candidates[] = {57600, 115200, 1000000, 2000000, 3000000, 4000000};

    for (const int baud : baud_candidates) {
      if (!portHandler_->setBaudRate(baud)) {
        continue;
      }

      for (uint8_t id = 0; id <= 20; ++id) {
        uint16_t model_number = 0;
        uint8_t dxl_error = 0;

        const int dxl_comm_result = packetHandler_->ping(
          portHandler_, id, &model_number, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
          dxl_id_ = id;
          baudrate_ = baud;

          RCLCPP_INFO(
            this->get_logger(),
            "Auto-detect success: ID %u at baud %d, model: %u",
            dxl_id_,
            baudrate_,
            model_number);
          return true;
        }
      }
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "Auto-detect failed. Checked IDs 0-20 at common baud rates.");
    return false;
  }

  bool check_comm_result(int dxl_comm_result, uint8_t dxl_error, const char * action)
  {
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "%s failed: %s",
        action,
        packetHandler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    if (dxl_error != 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "%s failed with DXL error: %s",
        action,
        packetHandler_->getRxPacketError(dxl_error));
      return false;
    }

    return true;
  }

  void check_comm_result_or_throw(int dxl_comm_result, uint8_t dxl_error, const char * action)
  {
    if (!check_comm_result(dxl_comm_result, dxl_error, action)) {
      throw std::runtime_error(action);
    }
  }

  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;

  uint8_t dxl_id_;
  int baudrate_;

  double gripper_position_;
  double gripper_target_;

  int commanded_current_units_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr current_print_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<DynamixelPositionNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}