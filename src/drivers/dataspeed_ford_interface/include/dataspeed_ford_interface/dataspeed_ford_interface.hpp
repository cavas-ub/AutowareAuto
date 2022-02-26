#ifndef DATASPEED_FORD_INTERFACE__DATASPEED_FORD_INTERFACE_HPP_
#define DATASPEED_FORD_INTERFACE__DATASPEED_FORD_INTERFACE_HPP_

#include <dataspeed_ford_interface/visibility_control.hpp>
#include <dataspeed_ford_interface/pid_controller.hpp>

#include <common/types.hpp>
#include <vehicle_interface/dbw_state_machine.hpp>
#include <vehicle_interface/platform_interface.hpp>

#include <dataspeed_dbw_msgs/msg/brake_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/gear_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/misc_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/steering_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/throttle_cmd.hpp>

#include <dataspeed_dbw_msgs/msg/brake_report.hpp>
#include <dataspeed_dbw_msgs/msg/gear_report.hpp>
#include <dataspeed_dbw_msgs/msg/misc_report.hpp>
#include <dataspeed_dbw_msgs/msg/steering_report.hpp>
#include <dataspeed_dbw_msgs/msg/throttle_report.hpp>

#include <dataspeed_dbw_msgs/msg/gear.hpp>
#include <dataspeed_dbw_msgs/msg/gear_reject.hpp>
#include <dataspeed_dbw_msgs/msg/turn_signal.hpp>
#include <dataspeed_dbw_msgs/msg/watchdog_counter.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_control_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/headlights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/wipers_command.hpp>
#include <autoware_auto_vehicle_msgs/srv/autonomy_mode_change.hpp>

#include <std_msgs/msg/bool.hpp>
#include <motion_common/motion_common.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::PI;
using autoware::common::types::TAU;

using dataspeed_dbw_msgs::msg::BrakeCmd;
using dataspeed_dbw_msgs::msg::GearCmd;
using dataspeed_dbw_msgs::msg::MiscCmd;
using dataspeed_dbw_msgs::msg::SteeringCmd;
using dataspeed_dbw_msgs::msg::ThrottleCmd;

using dataspeed_dbw_msgs::msg::BrakeReport;

// Need to distinguish between dataspeed_dbw_msgs::msg::GearReport and
// autoware_auto_vehicle_msg::msg::GearReport
namespace dbw_ford
{
using dataspeed_dbw_msgs::msg::GearReport;
}  // namespace dbw_ford

using dataspeed_dbw_msgs::msg::MiscReport;
using dataspeed_dbw_msgs::msg::SteeringReport;
using dataspeed_dbw_msgs::msg::ThrottleReport;

using dataspeed_dbw_msgs::msg::Gear;
using dataspeed_dbw_msgs::msg::GearReject;
using dataspeed_dbw_msgs::msg::TurnSignal;
using dataspeed_dbw_msgs::msg::WatchdogCounter;

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_control_msgs::msg::HighLevelControlCommand;
using autoware_auto_vehicle_msgs::msg::HeadlightsCommand;
using autoware_auto_vehicle_msgs::msg::RawControlCommand;
using autoware_auto_vehicle_msgs::msg::VehicleControlCommand;
using autoware_auto_vehicle_msgs::msg::VehicleStateCommand;
using autoware_auto_vehicle_msgs::msg::WipersCommand;

using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;

using autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using autoware_auto_vehicle_msgs::msg::VehicleOdometry;
using autoware_auto_vehicle_msgs::msg::VehicleStateReport;

using autoware_auto_vehicle_msgs::srv::AutonomyModeChange;
using ModeChangeRequest = autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Request;
using ModeChangeResponse = autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Response;

using autoware::drivers::vehicle_interface::DbwState;
using autoware::drivers::vehicle_interface::DbwStateMachine;
using namespace std::chrono_literals;  // NOLINT

namespace autoware
{
namespace dataspeed_ford_interface
{
static constexpr float32_t KPH_TO_MPS_RATIO = 1000.0F / (60.0F * 60.0F);
static constexpr float32_t DEGREES_TO_RADIANS = PI / 180.0F;
static constexpr int64_t CLOCK_1_SEC = 1000;  // duration in milliseconds
/// \brief Class for interfacing with Dataspeed Ford DBW
class DATASPEED_FORD_INTERFACE_PUBLIC DataspeedFordInterface
: public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  /// \brief Default constructor.
  /// \param[in] node Reference to node
  /// \param[in] ecu_build_num ECU build #
  /// \param[in] front_axle_to_cog Distance from front axle to center-of-gravity in meters
  /// \param[in] rear_axle_to_cog Distance from rear axle to center-of-gravity in meters
  /// \param[in] steer_to_tire_ratio Ratio of steering angle / car tire angle
  /// \param[in] max_steer_angle Maximum steering wheel turn angle
  /// \param[in] acceleration_limit m/s^2, zero = no limit
  /// \param[in] deceleration_limit m/s^2, zero = no limit
  /// \param[in] acceleration_positive_jerk_limit m/s^3
  /// \param[in] deceleration_negative_jerk_limit m/s^3
  /// \param[in] pub_period message publishing period, in milliseconds
  /// \param[in] accel_control_kp P-gain of the throttle/brake PID controller
  /// \param[in] accel_control_ki I-gain of the throttle/brake PID controller
  /// \param[in] accel_control_kd D-gain of the throttle/brake PID controller
  /// \param[in] accel_control_deadzone_min lower bound of the deadzone of throttle/brake controller
  /// \param[in] accel_control_deadzone_max upper bound of the deadzone of throttle/brake controller
  /// \param[in] autobrake_threshold velocity threshold for autobrake
  /// \param[in] autobrake_brake_pedal_stress pedal stress for the brake for autobrake
  explicit DataspeedFordInterface(
    rclcpp::Node & node,
    uint16_t ecu_build_num,
    float32_t front_axle_to_cog,
    float32_t rear_axle_to_cog,
    float32_t steer_to_tire_ratio,
    float32_t max_steer_angle,
    float32_t acceleration_limit,
    float32_t deceleration_limit,
    float32_t acceleration_positive_jerk_limit,
    float32_t deceleration_negative_jerk_limit,
    uint32_t pub_period,
    float32_t accel_control_kp,
    float32_t accel_control_ki,
    float32_t accel_control_kd,
    float32_t accel_control_deadzone_min,
    float32_t accel_control_deadzone_max,
    float32_t autobrake_threshold,
    float32_t autobrake_brake_pedal_stress);

  /// \brief Default destructor
  ~DataspeedFordInterface() noexcept override = default;

  /// \brief Try to receive data from the vehicle platform, and update StateReport and Odometry.
  ///   Exceptions may be thrown on errors
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \return True if data was received before the timeout, false otherwise
  bool8_t update(std::chrono::nanoseconds timeout) override;

  /// \brief Send the state command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The state command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_state_command(const VehicleStateCommand & msg) override;

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const HighLevelControlCommand & msg);

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const RawControlCommand & msg) override;

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  ///   Steering -> tire angle conversion is linear except for extreme angles
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const VehicleControlCommand & msg) override;

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  ///   Steering -> tire angle conversion is linear except for extreme angles
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const AckermannControlCommand & msg) override;

  /// \brief Handle a request from the user to enable or disable the DBW system.
  ///   Exceptions may be thrown on errors
  /// \param[in] request The requested autonomy mode
  /// \return false only if enabling the DBW system actually failed, true otherwise
  bool8_t handle_mode_change_request(ModeChangeRequest::SharedPtr request) override;

  /// \brief Send a headlights command to the vehicle platform.
  /// \param[in] msg The headlights command to send to the vehicle
  void send_headlights_command(const HeadlightsCommand & msg) override;

  /// \brief Send a horn command to the vehicle platform.
  /// \param[in] msg The horn command to send to the vehicle
  void send_horn_command(const HornCommand & msg) override;

  /// \brief Send a wipers command to the vehicle platform.
  /// \param[in] msg The wipers command to send to the vehicle
  void send_wipers_command(const WipersCommand & msg) override;

  /// \brief Set acceleration PID control kp value.
  /// \param[in] kp
  void set_acceleration_control_kp(const float32_t kp);

  /// \brief Set acceleration PID control ki value.
  /// \param[in] ki
  void set_acceleration_control_ki(const float32_t ki);

  /// \brief Set acceleration PID control kd value.
  /// \param[in] kd
  void set_acceleration_control_kd(const float32_t kd);


private:
  /// \brief Send out command packets periodically
  void cmdCallback();

  // Publishers (to Dataspeed Ford DBW)
  rclcpp::Publisher<ThrottleCmd>::SharedPtr m_throttle_cmd_pub;
  rclcpp::Publisher<BrakeCmd>::SharedPtr m_brake_cmd_pub;
  rclcpp::Publisher<GearCmd>::SharedPtr m_gear_cmd_pub;
  rclcpp::Publisher<MiscCmd>::SharedPtr m_misc_cmd_pub;
  rclcpp::Publisher<SteeringCmd>::SharedPtr m_steer_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_dbw_enable_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_dbw_disable_cmd_pub;

  // Subscribers (from Dataspeed Ford DBW)
  rclcpp::SubscriptionBase::SharedPtr m_brake_info_rpt_sub, m_gear_rpt_sub, m_misc_rpt_sub,
    m_steering_rpt_sub, m_wheel_spd_rpt_sub, m_dbw_enable_sub;

  rclcpp::Logger m_logger;
  uint16_t m_ecu_build_num;
  float32_t m_front_axle_to_cog;
  float32_t m_rear_axle_to_cog;
  float32_t m_steer_to_tire_ratio;
  float32_t m_max_steer_angle;
  float32_t m_acceleration_limit;
  float32_t m_deceleration_limit;
  float32_t m_acceleration_positive_jerk_limit;
  float32_t m_deceleration_negative_jerk_limit;
  std::chrono::milliseconds m_pub_period;
  std::unique_ptr<DbwStateMachine> m_dbw_state_machine;
  rclcpp::Clock m_clock;
  rclcpp::TimerBase::SharedPtr m_timer;

  // for throttle/brake control
  rclcpp::Time m_prev_tick;
  PIDController m_throttle_pid_controller;
  float32_t m_accel_control_deadzone_min;
  float32_t m_accel_control_deadzone_max;
  float32_t m_target_speed;
  float32_t m_autobrake_velocity_threshold;  // using mps
  float32_t m_autobrake_brake_stress;        // using percent

  ThrottleCmd m_throttle_cmd{};
  BrakeCmd m_brake_cmd{};
  GearCmd m_gear_cmd{};
  MiscCmd m_misc_cmd{};
  SteeringCmd m_steer_cmd{};

  bool8_t m_seen_brake_info_rpt{false};
  bool8_t m_seen_gear_rpt{false};
  bool8_t m_seen_misc_rpt{false};
  bool8_t m_seen_steering_rpt{false};
  bool8_t m_seen_wheel_spd_rpt{false};
  bool8_t m_seen_vehicle_state_cmd{false};
  float32_t m_travel_direction{0.0F};

  /** \brief Receives the brake info report from the vehicle platform.
   * Gets parking brake status for VehicleStateReport.
   *
   * \param[in] msg The report received from the vehicle
   */
  void on_brake_info_report(const BrakeInfoReport::SharedPtr & msg);

  /** \brief Receives the gear state report from the vehicle platform.
   * Gets PRNDL status for VehicleStateReport.
   *
   * \param[in] msg The report received from the vehicle
   */
  void on_gear_report(const dbw_ford::GearReport::SharedPtr & msg);

  /** \brief Receives the miscellaneous state report from the vehicle platform.
   * Gets vehicle speed for VehicleOdometry and VehicleKinematicState.
   * Gets fuel level for VehicleStateReport.
   * Calls kinematic_bicycle_model() to calculate VehicleKinematicState.
   * Publishes VehicleKinematicState.
   *
   * \param[in] msg The report received from the vehicle
   */
  void on_misc_report(const Misc1Report::SharedPtr & msg);

  /** \brief Receives the steering state report from the vehicle platform.
   * Converts steering angle to tire angle for VehicleOdometry and VehicleStateReport.
   * Publishes VehicleOdometry.
   *
   * \param[in] msg The report received from the vehicle
   */
  void on_steering_report(const SteeringReport::SharedPtr & msg);

  /** \brief Receives the wheel speed state report from the vehicle platform.
   * Checks travel direction for VehicleOdometry and VehicleStateReport
   *
   * \param[in] msg The report received from the vehicle
   */
  void on_wheel_spd_report(const WheelSpeedReport::SharedPtr & msg);

  /** \brief Receives the enable message from the vehicle platform.
  * Update state machine by calling dbw_feedback()
  * 
  * \param[in] msg The enable message received from the vehicle
  */
  void on_dbw_enable(const std_msgs::msg::Bool::SharedPtr & msg);

};  // class DataspeedFordInterface
}  // namespace dataspeed_ford_interface
}  // namespace autoware
#endif  // DATASPEED_FORD_INTERFACE__DATASPEED_FORD_INTERFACE_HPP_
