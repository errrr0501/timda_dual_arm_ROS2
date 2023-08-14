#include "linear_motion/slide.h"
#include <ctime>

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


modbus_t* init_modbus_rtu(int id, std::string port, int baud_rate)
{
    modbus_t* ct = modbus_new_rtu(port.c_str(), baud_rate, 'E', 8, id);
    modbus_set_slave(ct, id);
    if (modbus_connect(ct) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n",
            modbus_strerror(errno));
        std::cout << "Error connect" << std::endl;
        modbus_free(ct);
        return nullptr;
    }
    std::cout << "Init success" << std::endl;
    // modbus_set_debug(ct, true);

    return ct;
}

namespace slider_hardware
{
constexpr const char * kSliderHardware = "SliderHardware";
// constexpr uint8_t kGoalPositionIndex = 0;
// constexpr uint8_t kGoalVelocityIndex = 1;
// constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
// constexpr const char * kGoalPositionItem = "Goal_Position";
// constexpr const char * kGoalVelocityItem = "Goal_Velocity";
// constexpr const char * kMovingSpeedItem = "Moving_Speed";
// constexpr const char * kPresentPositionItem = "Present_Position";
// constexpr const char * kPresentVelocityItem = "Present_Velocity";
// constexpr const char * kPresentSpeedItem = "Present_Speed";
// constexpr const char * kPresentCurrentItem = "Present_Current";
// constexpr const char * kPresentLoadItem = "Present_Load";
// constexpr const char * const kExtraJointParameters[] = {
//   "Profile_Velocity",
//   "Profile_Acceleration",
//   "Position_P_Gain",
//   "Position_I_Gain",
//   "Position_D_Gain",
//   "Velocity_P_Gain",
//   "Velocity_I_Gain",
// };

CallbackReturn SliderHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kSliderHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    RCLCPP_INFO(rclcpp::get_logger(kSliderHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }
//   joints_.state.position = std::numeric_limits<double>::quiet_NaN();
//   joints_.state.velocity = std::numeric_limits<double>::quiet_NaN();
//   joints_.state.effort = std::numeric_limits<double>::quiet_NaN();
//   joints_.command.position = std::numeric_limits<double>::quiet_NaN();
//   joints_.command.velocity = std::numeric_limits<double>::quiet_NaN();
//   joints_.command.effort = std::numeric_limits<double>::quiet_NaN();
//   joints_.prev_command.position = joints_.command.position;
//   joints_.prev_command.velocity = joints_.command.velocity;
//   joints_.prev_command.effort = joints_.command.effort;
//   RCLCPP_INFO(rclcpp::get_logger(kSliderHardware), "joint_id %d: %d", i, joint_ids_[i]);

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto side = info_.hardware_parameters.at("side");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kSliderHardware), "connect_port: %s", (usb_port + side).c_str());
  RCLCPP_INFO(rclcpp::get_logger(kSliderHardware), "baud_rate: %d", baud_rate);


  int id = side.c_str() == "right" ? 3 : 2;
  ct = init_modbus_rtu(id, (usb_port + side).c_str(), baud_rate);


  if (!ct)
    {
        std::cout << "Connect " + usb_port + " fail!!!" << std::endl;
        return CallbackReturn::ERROR;
    }

//   for (uint i = 0; i < info_.joints.size(); ++i) {
//     uint16_t model_number = 0;
//     if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
//       RCLCPP_FATAL(rclcpp::get_logger(kSliderHardware), "%s", log);
//       return CallbackReturn::ERROR;
//     }
//   }


//   read_feedback();
  
  goal_pos = curr_pos;

//   set_control_mode(ControlMode::Position, true);
//   set_joint_params();


//   control_items_[kGoalPositionItem] = goal_position;
//   control_items_[kGoalVelocityItem] = goal_velocity;
//   control_items_[kPresentPositionItem] = present_position;
//   control_items_[kPresentVelocityItem] = present_velocity;
//   control_items_[kPresentCurrentItem] = present_current;

//   if (!dynamixel_workbench_.addSyncWriteHandler(
//         control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
//         &log)) {
//     RCLCPP_FATAL(rclcpp::get_logger(kSliderHardware), "%s", log);
//     return CallbackReturn::ERROR;
//   }

//   if (!dynamixel_workbench_.addSyncWriteHandler(
//         control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
//         &log)) {
//     RCLCPP_FATAL(rclcpp::get_logger(kSliderHardware), "%s", log);
//     return CallbackReturn::ERROR;
//   }

//   uint16_t start_address = std::min(
//     control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
//   uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
//                          control_items_[kPresentVelocityItem]->data_length +
//                          control_items_[kPresentCurrentItem]->data_length + 2;
//   if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
//     RCLCPP_FATAL(rclcpp::get_logger(kSliderHardware), "%s", log);
//     return CallbackReturn::ERROR;
//   }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SliderHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kSliderHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SliderHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kSliderHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  return command_interfaces;
}

CallbackReturn SliderHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kSliderHardware), "start");
//   for (uint i = 0; i < joints_.size(); i++) {
//     if (use_dummy_ && std::isnan(joints_[i].state.position)) {
//       joints_[i].state.position = 0.0;
//       joints_[i].state.velocity = 0.0;
//       joints_[i].state.effort = 0.0;
//     }
//   }
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));
  write_command();

  return CallbackReturn::SUCCESS;
}

CallbackReturn SliderHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kSliderHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type SliderHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{

  int rc = modbus_read_registers(ct, ADDRESS_FDB, FDB_LENGTH, fdb_val);
  if (rc != FDB_LENGTH)
  {
      fprintf(stderr, "modbus read failed: %d %s\n", errno, modbus_strerror(errno));
      errno = 0;
  }

    curr_pos   = fdb_val[0]<<16 | fdb_val[1];
    curr_speed = fdb_val[4]<<16 | fdb_val[5];



  return return_type::OK;
}

return_type SliderHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{


//   for (uint i = 0; i < ids.size(); i++) {
//     joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]);
//     joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
//     joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
//   }

//   if (use_dummy_) {
//     for (auto & joint : joints_) {
//       joint.prev_command.position = joint.command.position;
//       joint.state.position = joint.command.position;
//     }
//     return return_type::OK;
//   }
  goal_pos = (double)100000.0*(joints_[0].state.position+0.8);
  // goal_pos = -1*(double)100000.0*(msg->pos);
  if(goal_pos > 80000.0) goal_pos = 80000;
  if(goal_pos < 0.0) goal_pos = 0;

  // Velocity control
  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.velocity != j.prev_command.velocity; })) {
    // set_control_mode(ControlMode::Velocity);
    if(mode_changed_)
    {
    //   set_joint_params();
    }
    // set_joint_velocities();
    return return_type::OK;
  }
  
  // Position control
  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.position != j.prev_command.position; })) {
    // set_control_mode(ControlMode::Position);
    if(mode_changed_)
    {
    //   set_joint_params();
    }
    // set_joint_positions();
    return return_type::OK;
  }

  // Effort control
  if (std::any_of(
               joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.effort != 0.0; })) {
    RCLCPP_ERROR(rclcpp::get_logger(kSliderHardware), "Effort control is not implemented");
    return return_type::ERROR;
  }

  // if all command values are unchanged, then remain in existing control mode and set corresponding command values
  switch (control_mode_) {
    case ControlMode::Velocity:
    //   set_joint_velocities();
      return return_type::OK;
      break;
    case ControlMode::Position:
    //   set_joint_positions();
      return return_type::OK;
      break;
    default: // effort, etc
      RCLCPP_ERROR(rclcpp::get_logger(kSliderHardware), "Control mode not implemented");
    //   return return_type::ERROR;
      break;
  }

}

return_type SliderHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;
  mode_changed_ = false;

//   if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
//     bool torque_enabled = torque_enabled_;
//     if (torque_enabled) {
//       enable_torque(false);
//     }

//     for (uint i = 0; i < joint_ids_.size(); ++i) {
//       if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
//         RCLCPP_FATAL(rclcpp::get_logger(kSliderHardware), "%s", log);
//         return return_type::ERROR;
//       }
//     }
//     RCLCPP_INFO(rclcpp::get_logger(kSliderHardware), "Velocity control");
//     if(control_mode_ != ControlMode::Velocity)
//     {
//       mode_changed_ = true;
//       control_mode_ = ControlMode::Velocity;
//     }

//     if (torque_enabled) {
//       enable_torque(true);
//     }
//     return return_type::OK;
//   }

//   if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
//     bool torque_enabled = torque_enabled_;
//     if (torque_enabled) {
//       enable_torque(false);
//     }

//     for (uint i = 0; i < joint_ids_.size(); ++i) {
//       if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
//         RCLCPP_FATAL(rclcpp::get_logger(kSliderHardware), "%s", log);
//         return return_type::ERROR;
//       }
//     }
//     RCLCPP_INFO(rclcpp::get_logger(kSliderHardware), "Position control");
//     if(control_mode_ != ControlMode::Position)
//     {
//       mode_changed_ = true;
//       control_mode_ = ControlMode::Position;
//     }

//     if (torque_enabled) {
//       enable_torque(true);
//     }
//     return return_type::OK;
//   }
  
//   if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
//     RCLCPP_FATAL(
//       rclcpp::get_logger(kSliderHardware), "Only position/velocity control are implemented");
//     return return_type::ERROR;
//   }

  return return_type::OK;
}

return_type SliderHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

return_type SliderHardware::write_command()
{

  if (goal_pos != curr_pos){
    int diff_pos = goal_pos - curr_pos;
    int speed_tmp = float(diff_pos) / (smp_time*20);
    // speed_tmp = (speed_tmp < MAX_SPEED) ? speed_tmp : MAX_SPEED;
    if(speed_tmp > MAX_SPEED) speed_tmp = MAX_SPEED;
    if(speed_tmp < MIN_SPEED) speed_tmp = MIN_SPEED;
    // cmd_speed = (speed_tmp < cmd_speed) ? (
    //     ((cmd_speed - speed_tmp) < smp_deleration) ? speed_tmp : cmd_speed - smp_deleration
    //     ) : speed_tmp;
    // cmd_speed = (speed_tmp < cmd_speed) ? cmd_speed : speed_tmp;
    // cmd_speed = speed_tmp;
    // cmd_arr[6] = cmd_speed>>16;
    // if(diff_pos > 3) goal_pos = goal_pos + (diff_pos - 3) * 2;
    // if(diff_pos < -3) goal_pos = goal_pos + (diff_pos + 3) * 2;
    // if(goal_pos > 80000.0) goal_pos = 80000;
    // if(goal_pos < 0.0) goal_pos = 0;
    if(abs(diff_pos) < float(abs(curr_pos))/MAX_SPEED * 100)
    {
        cmd_arr[3] = 1;
        cmd_speed = abs(speed_tmp);
    }
    else
    {
        cmd_arr[3] = 7;
        cmd_speed = speed_tmp;
    }
    int diff_speed = abs(cmd_speed - curr_speed);
    int acc = 3 * diff_speed / (smp_time) + 1;
    // cmd_speed = abs(speed_tmp);
    // cmd_arr[3] = 1;
    cmd_arr[4] = goal_pos>>16;
    cmd_arr[5] = goal_pos;
    cmd_arr[6] = cmd_speed>>16;
    cmd_arr[7] = cmd_speed;
    // cmd_arr[9]  = exp((cmd_speed / MAX_SPEED)*4 - 2) * 5410;
    // cmd_arr[9] = (3*abs(cmd_speed) > ACCELERATION) ? ACCELERATION : 3*abs(cmd_speed);
    // cmd_arr[11] = (3*abs(cmd_speed) > DECELERATION) ? DECELERATION : 3*abs(cmd_speed);
    cmd_arr[9] = (acc > ACCELERATION) ? ACCELERATION : acc;
    cmd_arr[11] = (acc > DECELERATION) ? DECELERATION : acc;
    int rc = modbus_write_registers(ct, ADDRESS_CMD, CMD_LENGTH, cmd_arr);
    if (rc != CMD_LENGTH)
    {
        fprintf(stderr, "modbus write failed: %d %s\n", errno, modbus_strerror(errno));
        errno = 0;
    }
  }
  else{
    cmd_speed = 0;
  }
  return return_type::OK;
}

CallbackReturn SliderHardware::set_joint_positions()
{

  const char * log = nullptr;
//   std::vector<int32_t> commands(info_.joints.size(), 0);
//   std::vector<uint8_t> ids(info_.joints.size(), 0);

//   std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
//   for (uint i = 0; i < ids.size(); i++) {
//     joints_[i].prev_command.position = joints_[i].command.position;
//     commands[i] = dynamixel_workbench_.convertRadian2Value(
//       ids[i], static_cast<float>(joints_[i].command.position));
//   }
//   if (!dynamixel_workbench_.syncWrite(
//         kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
//     RCLCPP_ERROR(rclcpp::get_logger(kSliderHardware), "%s", log);
//   }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SliderHardware::set_joint_velocities()
{
//   const char * log = nullptr;
//   std::vector<int32_t> commands(info_.joints.size(), 0);
//   std::vector<uint8_t> ids(info_.joints.size(), 0);

//   std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
//   for (uint i = 0; i < ids.size(); i++) {
//     joints_[i].prev_command.velocity = joints_[i].command.velocity;
//     commands[i] = dynamixel_workbench_.convertVelocity2Value(
//       ids[i], static_cast<float>(joints_[i].command.velocity));
//   }
//   if (!dynamixel_workbench_.syncWrite(
//         kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
//     RCLCPP_ERROR(rclcpp::get_logger(kSliderHardware), "%s", log);
//   }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SliderHardware::set_joint_params()
{
//   const char * log = nullptr;
//   for (uint i = 0; i < info_.joints.size(); ++i) {
//     for (auto paramName : kExtraJointParameters) {
//       if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
//         auto value = std::stoi(info_.joints[i].parameters.at(paramName));
//         if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
//           RCLCPP_FATAL(rclcpp::get_logger(kSliderHardware), "%s", log);
//           return CallbackReturn::ERROR;
//         }
//         RCLCPP_INFO(rclcpp::get_logger(kSliderHardware), "%s set to %d for joint %d", paramName, value, i);
//       }
//     }
//   }
  return CallbackReturn::SUCCESS;
}

}  // namespace slider_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(slider_hardware::SliderHardware, hardware_interface::SystemInterface)
