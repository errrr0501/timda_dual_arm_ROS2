#ifndef SLIDER_HARDWARE__SLIDER_HARDWARE_HPP_
#define SLIDER_HARDWARE__SLIDER_HARDWARE_HPP_

#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <boost/thread.hpp>
#include <math.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <map>
#include <vector>

#include "linear_motion/visiblity_control.h"
#include "rclcpp/macros.hpp"
#include <modbus.h>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

#define ADDRESS_CMD  88
#define ADDRESS_FDB  204
#define MAX_SPEED    20000
#define MIN_SPEED    -20000
#define ACCELERATION 40000
#define DECELERATION 40000

#define CMD_LENGTH 16
#define FDB_LENGTH 6

namespace slider_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
};

enum class ControlMode {
  Position,
  Velocity,
  Torque,
  Currrent,
  ExtendedPosition,
  MultiTurn,
  CurrentBasedPosition,
  PWM,
};

class SliderHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SliderHardware)

  SLIDER_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  SLIDER_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SLIDER_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SLIDER_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SLIDER_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SLIDER_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SLIDER_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  uint16_t cmd_arr[CMD_LENGTH] = {0, 0, 0, 1, 0, 0, 0, 0, 0, ACCELERATION, 0, DECELERATION, 0, 800, 0, 1};
  uint16_t fdb_val[FDB_LENGTH] = {0, 0, 0, 0, 0, 0};
  
  modbus_t *ct    = nullptr;
  int goal_pos    = 0;
  int curr_pos    = 0;
  int curr_speed  = 0;
  int cmd_speed   = 0;
  double smp_time = 0.008;
  double smp_deleration = 0;
//   boost::thread  *com_driver_thread_;

private:
  return_type enable_torque(const bool enabled);

  return_type set_control_mode(const ControlMode & mode, const bool force_set = false);

  return_type reset_command();

  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  CallbackReturn set_joint_params();

  return_type write_command();

//   DynamixelWorkbench dynamixel_workbench_;
//   std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
//   bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
  bool mode_changed_{false};
//   bool use_dummy_{false};
};
}  // namespace slider_hardware

#endif  // SLIDER_HARDWARE__SLIDER_HARDWARE_HPP_
