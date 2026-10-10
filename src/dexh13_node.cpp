#include "dexh13_hand/dexh13_node.hpp"

#include "dexh13_hand/constants.hpp"
#include "dexh13_hand/dexh13_sdk.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "dexh13_hand/msg/fault.hpp"
#include "dexh13_hand/msg/hand_status.hpp"
#include "dexh13_hand/msg/motor_command.hpp"
#include "dexh13_hand/msg/motor_state.hpp"
#include "dexh13_hand/msg/press_force.hpp"
#include "dexh13_hand/msg/tactile.hpp"
#include "dexh13_hand/msg/tactile_detail.hpp"
#include "dexh13_hand/msg/upgrade_progress.hpp"
#include "dexh13_hand/srv/compute_force_command.hpp"
#include "dexh13_hand/srv/connect_ethercat.hpp"
#include "dexh13_hand/srv/forward_kinematics.hpp"
#include "dexh13_hand/srv/get_camera_calibration.hpp"
#include "dexh13_hand/srv/get_fault.hpp"
#include "dexh13_hand/srv/get_joint_angles.hpp"
#include "dexh13_hand/srv/get_version.hpp"
#include "dexh13_hand/srv/inverse_kinematics.hpp"
#include "dexh13_hand/srv/save_image.hpp"
#include "dexh13_hand/srv/scan_ethercat.hpp"
#include "dexh13_hand/srv/set_camera_config.hpp"
#include "dexh13_hand/srv/set_control_mode.hpp"
#include "dexh13_hand/srv/set_force_params.hpp"
#include "dexh13_hand/srv/set_joint_angles.hpp"
#include "dexh13_hand/srv/set_motor_limits.hpp"
#include "dexh13_hand/srv/upgrade_driveboard.hpp"
#include "dexh13_hand/srv/upgrade_mainboard.hpp"

namespace dexh13_hand
{
namespace
{

using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;

rclcpp::QoS statusQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
}

std::string faultKey(bool fault, const std::vector<FaultItem> & items)
{
  std::string key = fault ? "1" : "0";
  for (const FaultItem & item : items) {
    key += "|";
    key += item.finger_name;
    key += ":";
    key += std::to_string(item.motor_id);
    key += ":";
    key += item.error_code;
  }
  return key;
}

}  // namespace

class Dexh13Node : public rclcpp::Node
{
public:
  Dexh13Node()
  : rclcpp::Node("dexh13_hand")
  {
    const std::string side_text = declare_parameter<std::string>("side", "left");
    if (!parseSide(side_text, expected_side_)) {
      throw std::invalid_argument("参数 side 必须是 left、right 或 auto");
    }
    const std::string mode_text = declare_parameter<std::string>("control_mode", "position");
    if (!parseMode(mode_text, startup_mode_)) {
      throw std::invalid_argument("参数 control_mode 必须是 position、speed 或 torque");
    }

    hand_port_ = declare_parameter<std::string>("hand_port", "");
    camera_port_ = declare_parameter<std::string>("camera_port", "none");
    prefix_ = declare_parameter<std::string>("prefix", "");
    if (prefix_.empty() && expected_side_ != Side::kAuto) {
      prefix_ = std::string(toString(expected_side_)) + "_";
    }
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    if (frame_id_.empty()) {
      frame_id_ = std::string("dexh13_") +
        (expected_side_ == Side::kAuto ? "hand" : toString(expected_side_)) + "_palm";
    }

    connect_on_start_ = declare_parameter<bool>("connect_on_start", true);
    enable_on_start_ = declare_parameter<bool>("enable_on_start", true);
    init_on_start_ = declare_parameter<bool>("init_on_start", false);
    clamp_ = declare_parameter<bool>("clamp_commands", true);
    publish_undistorted_ = declare_parameter<bool>("publish_undistorted", false);
    max_current_ma_ = declare_parameter<int>("max_current_ma", kMaxCurrentMax);
    max_speed_rpm_ = declare_parameter<int>("max_speed_rpm", kMaxSpeedMax);
    camera_width_ = declare_parameter<int>("camera_width", 640);
    camera_height_ = declare_parameter<int>("camera_height", 480);
    camera_fps_ = declare_parameter<double>("camera_fps", 30.0);
    state_hz_ = declare_parameter<double>("state_hz", 20.0);
    tactile_hz_ = declare_parameter<double>("tactile_hz", 10.0);
    tactile_detail_hz_ = declare_parameter<double>("tactile_detail_hz", 0.0);
    press_force_hz_ = declare_parameter<double>("press_force_hz", 0.0);
    camera_hz_ = declare_parameter<double>("camera_hz", 15.0);
    force_hz_ = declare_parameter<double>("force_control_hz", 30.0);

    joint_names_.reserve(kJointCount);
    for (int i = 0; i < kJointCount; ++i) {
      joint_names_.push_back(prefix_ + kJointSuffix[static_cast<size_t>(i)]);
    }

    sdk_ = std::make_unique<Dexh13Sdk>();
    sdk_->setExpectedSide(expected_side_);

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "~/joint_position_cmd", command_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { onJointCommand(std::move(msg)); });
    speed_cmd_sub_ = create_subscription<msg::MotorCommand>(
      "~/motor_speed_cmd", command_qos,
      [this](msg::MotorCommand::SharedPtr msg) { onMotorSpeed(std::move(msg)); });
    current_cmd_sub_ = create_subscription<msg::MotorCommand>(
      "~/motor_current_cmd", command_qos,
      [this](msg::MotorCommand::SharedPtr msg) { onMotorCurrent(std::move(msg)); });
    motor_position_cmd_sub_ = create_subscription<msg::MotorCommand>(
      "~/motor_position_cmd", command_qos,
      [this](msg::MotorCommand::SharedPtr msg) { onMotorPosition(std::move(msg)); });

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    motor_pub_ = create_publisher<msg::MotorState>("~/motor_state", 10);
    status_pub_ = create_publisher<msg::HandStatus>("~/status", statusQos());
    tactile_pub_ = create_publisher<msg::Tactile>("~/tactile", 10);
    detail_pub_ = create_publisher<msg::TactileDetail>("~/tactile_detail", 10);
    press_pub_ = create_publisher<msg::PressForce>("~/press_force", 10);
    fault_pub_ = create_publisher<msg::Fault>("~/fault", statusQos());
    upgrade_pub_ = create_publisher<msg::UpgradeProgress>("~/upgrade_progress", 10);
    if (cameraEnabled()) {
      const auto image_qos = rclcpp::SensorDataQoS();
      image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", image_qos);
      info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", image_qos);
      if (publish_undistorted_) {
        rect_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_rect", image_qos);
      }
    }

    connect_srv_ = create_service<Trigger>(
      "~/connect", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        onConnect(response);
      });
    disconnect_srv_ = create_service<Trigger>(
      "~/disconnect",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        onDisconnect(response);
      });
    enable_srv_ = create_service<SetBool>(
      "~/enable_motor",
      [this](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
        onEnableMotor(request, response);
      });
    init_srv_ = create_service<Trigger>(
      "~/init_motor_position",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        stopForce();
        fill(sdk_->initMotorPosition(), response);
      });
    mode_srv_ = create_service<srv::SetControlMode>(
      "~/set_control_mode",
      [this](const srv::SetControlMode::Request::SharedPtr request,
      srv::SetControlMode::Response::SharedPtr response) {
        onSetMode(request, response);
      });
    limits_srv_ = create_service<srv::SetMotorLimits>(
      "~/set_motor_limits",
      [this](const srv::SetMotorLimits::Request::SharedPtr request,
      srv::SetMotorLimits::Response::SharedPtr response) {
        onSetLimits(request, response);
      });
    clear_fault_srv_ = create_service<Trigger>(
      "~/clear_fault",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        fill(sdk_->clearFault(), response);
      });
    fault_srv_ = create_service<srv::GetFault>(
      "~/get_fault",
      [this](const srv::GetFault::Request::SharedPtr, srv::GetFault::Response::SharedPtr response) {
        onGetFault(response);
      });
    version_srv_ = create_service<srv::GetVersion>(
      "~/get_version",
      [this](const srv::GetVersion::Request::SharedPtr,
      srv::GetVersion::Response::SharedPtr response) {
        onGetVersion(response);
      });
    calibrate_srv_ = create_service<Trigger>(
      "~/calibrate_tactile",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        fill(sdk_->calibrateTactile(), response);
      });
    fk_srv_ = create_service<srv::ForwardKinematics>(
      "~/forward_kinematics",
      [this](const srv::ForwardKinematics::Request::SharedPtr request,
      srv::ForwardKinematics::Response::SharedPtr response) {
        MotorValues motors;
        motors.values = request->motor_position;
        const ApiResult result = sdk_->forwardKinematics(motors, response->joint_radians);
        fill(result, response);
      });
    ik_srv_ = create_service<srv::InverseKinematics>(
      "~/inverse_kinematics",
      [this](const srv::InverseKinematics::Request::SharedPtr request,
      srv::InverseKinematics::Response::SharedPtr response) {
        MotorValues motors;
        const ApiResult result = sdk_->inverseKinematics(request->joint_radians, motors);
        response->motor_position.assign(motors.values.begin(), motors.values.end());
        fill(result, response);
      });
    set_deg_srv_ = create_service<srv::SetJointAngles>(
      "~/set_joint_angles_deg",
      [this](const srv::SetJointAngles::Request::SharedPtr request,
      srv::SetJointAngles::Response::SharedPtr response) {
        onSetDegrees(request, response);
      });
    get_deg_srv_ = create_service<srv::GetJointAngles>(
      "~/get_joint_angles_deg",
      [this](const srv::GetJointAngles::Request::SharedPtr,
      srv::GetJointAngles::Response::SharedPtr response) {
        const ApiResult result = sdk_->readJointDegrees(response->degrees);
        fill(result, response);
      });
    force_param_srv_ = create_service<srv::SetForceParams>(
      "~/set_force_params",
      [this](const srv::SetForceParams::Request::SharedPtr request,
      srv::SetForceParams::Response::SharedPtr response) {
        std::vector<bool> enable;
        enable.reserve(request->enable.size());
        for (const auto value : request->enable) {
          enable.push_back(static_cast<bool>(value));
        }
        fill(sdk_->setForceParams(
          request->impedance_factor, request->reference_force_factor, enable), response);
      });
    force_compute_srv_ = create_service<srv::ComputeForceCommand>(
      "~/compute_force_command",
      [this](const srv::ComputeForceCommand::Request::SharedPtr request,
      srv::ComputeForceCommand::Response::SharedPtr response) {
        const ApiResult result = sdk_->computeForceCommand(
          request->current_radians, response->command_radians);
        fill(result, response);
      });
    force_enable_srv_ = create_service<SetBool>(
      "~/enable_force_control",
      [this](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
        onEnableForce(request, response);
      });
    scan_srv_ = create_service<srv::ScanEthercat>(
      "~/scan_ethercat",
      [this](const srv::ScanEthercat::Request::SharedPtr request,
      srv::ScanEthercat::Response::SharedPtr response) {
        const ApiResult result = sdk_->scanEthercat(request->interface_name, response->addresses);
        fill(result, response);
      });
    ethercat_srv_ = create_service<srv::ConnectEthercat>(
      "~/connect_ethercat",
      [this](const srv::ConnectEthercat::Request::SharedPtr request,
      srv::ConnectEthercat::Response::SharedPtr response) {
        onConnectEthercat(request, response);
      });
    camera_cfg_srv_ = create_service<srv::SetCameraConfig>(
      "~/set_camera_config",
      [this](const srv::SetCameraConfig::Request::SharedPtr request,
      srv::SetCameraConfig::Response::SharedPtr response) {
        onSetCamera(request, response);
      });
    camera_cal_srv_ = create_service<srv::GetCameraCalibration>(
      "~/get_camera_calibration",
      [this](const srv::GetCameraCalibration::Request::SharedPtr,
      srv::GetCameraCalibration::Response::SharedPtr response) {
        onGetCamera(response);
      });
    save_image_srv_ = create_service<srv::SaveImage>(
      "~/save_image",
      [this](const srv::SaveImage::Request::SharedPtr request,
      srv::SaveImage::Response::SharedPtr response) {
        onSaveImage(request, response);
      });
    all_tactile_srv_ = create_service<Trigger>(
      "~/read_all_tactile",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        onReadAllTactile(response);
      });
    mainboard_srv_ = create_service<srv::UpgradeMainboard>(
      "~/upgrade_mainboard",
      [this](const srv::UpgradeMainboard::Request::SharedPtr request,
      srv::UpgradeMainboard::Response::SharedPtr response) {
        onUpgradeMainboard(request, response);
      });
    driveboard_srv_ = create_service<srv::UpgradeDriveboard>(
      "~/upgrade_driveboard",
      [this](const srv::UpgradeDriveboard::Request::SharedPtr request,
      srv::UpgradeDriveboard::Response::SharedPtr response) {
        onUpgradeDriveboard(request, response);
      });

    state_timer_ = makeTimer(state_hz_, [this] { onStateTimer(); });
    tactile_timer_ = makeTimer(tactile_hz_, [this] { onTactileTimer(); });
    detail_timer_ = makeTimer(tactile_detail_hz_, [this] { onDetailTimer(); });
    press_timer_ = makeTimer(press_force_hz_, [this] { onPressTimer(); });
    if (cameraEnabled()) {
      camera_timer_ = makeTimer(camera_hz_, [this] { onCameraTimer(); });
    }

    publishStatus(false, false, Mode::kNone, false);
    if (connect_on_start_) {
      if (hand_port_.empty()) {
        RCLCPP_ERROR(get_logger(), "connect_on_start 为 true，但 hand_port 为空，未连接");
      } else {
        const ApiResult result = bringUpSerial();
        if (!result.success) {
          RCLCPP_ERROR(get_logger(), "启动连接失败: %s", result.message.c_str());
        }
      }
    }

    RCLCPP_INFO(
      get_logger(), "DexH13 side=%s port=%s camera=%s prefix=%s undistort=%s",
      toString(expected_side_), hand_port_.c_str(), camera_port_.c_str(), prefix_.c_str(),
      sdk_->undistortAvailable() ? "ready" : "unavailable");
  }

  ~Dexh13Node() override
  {
    state_timer_.reset();
    tactile_timer_.reset();
    detail_timer_.reset();
    press_timer_.reset();
    camera_timer_.reset();
    stopForce();
    while (inflight_.load() > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (sdk_) {
      sdk_->waitUntilUpgradeDone();
      sdk_->shutdown();
    }
  }

private:
  template<typename Callback>
  rclcpp::TimerBase::SharedPtr makeTimer(double hz, Callback callback)
  {
    if (hz <= 0.0) {
      return nullptr;
    }
    const auto period = std::chrono::duration<double>(1.0 / hz);
    return create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::move(callback));
  }

  bool cameraEnabled() const
  {
    return !camera_port_.empty() && camera_port_ != "none";
  }

  int jointIndex(const std::string & name) const
  {
    for (int i = 0; i < kJointCount; ++i) {
      if (name == joint_names_[static_cast<size_t>(i)] ||
        name == kJointSuffix[static_cast<size_t>(i)])
      {
        return i;
      }
    }
    return -1;
  }

  template<typename Response>
  void fill(const ApiResult & result, const Response & response) const
  {
    response->success = result.success;
    response->message = result.message;
  }

  void warnThrottle(const char * text)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", text);
  }

  bool acceptMotion(Mode required, bool block_in_force, std::string & why)
  {
    const CommandContext ctx = sdk_->context();
    if (ctx.upgrading) {
      why = "固件升级进行中";
      return false;
    }
    if (!ctx.connected) {
      why = "灵巧手未连接";
      return false;
    }
    if (!ctx.motor_enabled) {
      why = "电机未使能";
      return false;
    }
    if (ctx.mode != required) {
      why = std::string("当前控制模式是 ") + toString(ctx.mode);
      return false;
    }
    if (block_in_force && force_active_.load()) {
      why = "力控环正在运行";
      return false;
    }
    return true;
  }

  void clampRadians(std::vector<double> & radians) const
  {
    if (!clamp_) {
      return;
    }
    for (int i = 0; i < kJointCount; ++i) {
      radians[static_cast<size_t>(i)] = std::clamp(
        radians[static_cast<size_t>(i)], jointLowerRad(i), jointUpperRad(i));
    }
  }

  void clampDegrees(std::vector<double> & degrees) const
  {
    if (!clamp_) {
      return;
    }
    for (int i = 0; i < kJointCount; ++i) {
      degrees[static_cast<size_t>(i)] = std::clamp(
        degrees[static_cast<size_t>(i)], jointLowerDeg(i), jointUpperDeg(i));
    }
  }

  void clampMotors(std::array<int16_t, kMotorCount> & values, int lower, int upper) const
  {
    if (!clamp_) {
      return;
    }
    for (int16_t & value : values) {
      value = static_cast<int16_t>(std::clamp(static_cast<int>(value), lower, upper));
    }
  }

  bool parseJoints(
    const sensor_msgs::msg::JointState & msg, std::vector<double> & radians, std::string & error) const
  {
    if (msg.name.empty()) {
      if (msg.position.size() != static_cast<size_t>(kJointCount)) {
        error = "position 长度必须是 16";
        return false;
      }
      radians.assign(msg.position.begin(), msg.position.end());
    } else {
      if (msg.name.size() != msg.position.size()) {
        error = "name 和 position 长度不一致";
        return false;
      }
      radians.assign(kJointCount, 0.0);
      std::array<bool, kJointCount> seen{};
      for (size_t i = 0; i < msg.name.size(); ++i) {
        const int index = jointIndex(msg.name[i]);
        if (index < 0) {
          error = "未知关节 " + msg.name[i];
          return false;
        }
        radians[static_cast<size_t>(index)] = msg.position[i];
        seen[static_cast<size_t>(index)] = true;
      }
      for (int i = 0; i < kJointCount; ++i) {
        if (!seen[static_cast<size_t>(i)]) {
          error = "缺少关节 " + joint_names_[static_cast<size_t>(i)];
          return false;
        }
      }
    }
    clampRadians(radians);
    return true;
  }

  void publishJoints(const std::vector<double> & radians)
  {
    if (radians.size() != static_cast<size_t>(kJointCount)) {
      return;
    }
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.name = joint_names_;
    msg.position = radians;
    joint_pub_->publish(msg);
  }

  void publishMotors(const DeviceSnapshot & snapshot)
  {
    msg::MotorState msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.position = snapshot.position.values;
    msg.speed_rpm = snapshot.speed_rpm.values;
    msg.current_ma = snapshot.current_ma.values;
    motor_pub_->publish(msg);
  }

  void publishTactile(const TactileSample & sample)
  {
    msg::Tactile msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.name = sample.name;
    msg.x = sample.x;
    msg.y = sample.y;
    msg.z = sample.z;
    msg.temp_name = sample.temp_name;
    msg.sensor_id = sample.sensor_id;
    msg.temperature = sample.temperature;
    tactile_pub_->publish(msg);
  }

  void publishDetail(const TactileDetail & detail)
  {
    msg::TactileDetail msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.x = detail.x;
    msg.y = detail.y;
    msg.z = detail.z;
    detail_pub_->publish(msg);
  }

  void publishPress(const PressSample & sample)
  {
    msg::PressForce msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.model_name = sample.model_name;
    msg.x = sample.x;
    msg.y = sample.y;
    msg.z = sample.z;
    msg.force = sample.force;
    press_pub_->publish(msg);
  }

  void publishFault(bool fault, const std::vector<FaultItem> & items)
  {
    const std::string key = faultKey(fault, items);
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      if (key == fault_key_) {
        return;
      }
      fault_key_ = key;
    }
    msg::Fault msg;
    msg.fault = fault;
    msg.finger_name.reserve(items.size());
    msg.motor_id.reserve(items.size());
    msg.error_code.reserve(items.size());
    for (const FaultItem & item : items) {
      msg.finger_name.push_back(item.finger_name);
      msg.motor_id.push_back(item.motor_id);
      msg.error_code.push_back(item.error_code);
    }
    fault_pub_->publish(msg);
  }

  void publishStatus(bool connected, bool enabled, Mode mode, bool fault)
  {
    std::string sdk_version;
    std::string firmware_version;
    std::string side_name;
    const bool force = force_active_.load();
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      sdk_version = sdk_version_;
      firmware_version = firmware_version_;
      side_name = connected ? side_name_ : "none";
    }
    const std::string key = std::string(connected ? "1" : "0") + side_name +
      (enabled ? "1" : "0") + toString(mode) + (fault ? "1" : "0") + sdk_version +
      firmware_version + (force ? "1" : "0");
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      if (key == status_key_) {
        return;
      }
      status_key_ = key;
    }
    msg::HandStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.connected = connected;
    msg.hand_type = side_name;
    msg.motor_enabled = enabled;
    msg.control_mode = toString(mode);
    msg.fault = fault;
    msg.sdk_version = sdk_version;
    msg.firmware_version = firmware_version;
    msg.force_control_active = force;
    status_pub_->publish(msg);
  }

  void rememberConnection()
  {
    const VersionInfo versions = sdk_->readVersions();
    const CommandContext ctx = sdk_->context();
    std::lock_guard<std::mutex> lock(cache_mutex_);
    if (!versions.sdk.empty()) {
      sdk_version_ = versions.sdk;
    }
    firmware_version_ = versions.firmware;
    side_name_ = toString(ctx.side);
  }

  void configureCamera()
  {
    if (!cameraEnabled()) {
      return;
    }
    const ApiResult config = sdk_->setCameraConfig(
      camera_width_, camera_height_, static_cast<float>(camera_fps_));
    if (!config.success) {
      RCLCPP_WARN(get_logger(), "设置相机参数失败: %s", config.message.c_str());
    }
    CameraModel model;
    const ApiResult calibration = sdk_->readCameraModel(camera_width_, camera_height_, model);
    if (!calibration.success) {
      RCLCPP_WARN(get_logger(), "读取相机标定失败: %s", calibration.message.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(cache_mutex_);
    camera_model_ = std::move(model);
  }

  ApiResult finishBringUp(const ApiResult & connected)
  {
    if (!connected.success) {
      publishStatus(false, false, Mode::kNone, false);
      return connected;
    }
    rememberConnection();
    if (init_on_start_) {
      const ApiResult init = sdk_->initMotorPosition();
      if (init.success) {
        RCLCPP_INFO(get_logger(), "%s", init.message.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "初始化电机位置失败: %s", init.message.c_str());
      }
    }
    const ApiResult mode = sdk_->setMode(startup_mode_);
    if (!mode.success) {
      RCLCPP_ERROR(get_logger(), "设置控制模式失败: %s", mode.message.c_str());
    }
    int current = max_current_ma_;
    int speed = max_speed_rpm_;
    if (clamp_) {
      current = std::clamp(current, kMaxCurrentMin, kMaxCurrentMax);
      speed = std::clamp(speed, kMaxSpeedMin, kMaxSpeedMax);
    }
    const ApiResult current_result = sdk_->setMaxCurrent(current);
    const ApiResult speed_result = sdk_->setMaxSpeed(speed);
    if (!current_result.success) {
      RCLCPP_WARN(get_logger(), "%s", current_result.message.c_str());
    }
    if (!speed_result.success) {
      RCLCPP_WARN(get_logger(), "%s", speed_result.message.c_str());
    }
    bool enabled = false;
    if (enable_on_start_) {
      const ApiResult enable = sdk_->enable();
      enabled = enable.success;
      if (!enable.success) {
        RCLCPP_ERROR(get_logger(), "使能失败: %s", enable.message.c_str());
      }
    }
    configureCamera();
    const CommandContext ctx = sdk_->context();
    publishStatus(true, enabled || ctx.motor_enabled, ctx.mode, false);
    RCLCPP_INFO(get_logger(), "%s", connected.message.c_str());
    return connected;
  }

  ApiResult bringUpSerial()
  {
    stopForce();
    return finishBringUp(sdk_->connectSerial(hand_port_, camera_port_, expected_side_));
  }

  void stopForce()
  {
    force_active_.store(false);
    force_timer_.reset();
  }

  void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr & msg)
  {
    std::string why;
    if (!acceptMotion(Mode::kPosition, true, why)) {
      warnThrottle(why.c_str());
      return;
    }
    std::vector<double> radians;
    std::string error;
    if (!parseJoints(*msg, radians, error)) {
      warnThrottle(error.c_str());
      return;
    }
    const ApiResult result = sdk_->setJointRadians(radians);
    if (!result.success) {
      warnThrottle(result.message.c_str());
    }
  }

  void onMotorSpeed(const msg::MotorCommand::SharedPtr & msg)
  {
    std::string why;
    if (!acceptMotion(Mode::kSpeed, true, why)) {
      warnThrottle(why.c_str());
      return;
    }
    MotorValues values;
    values.values = msg->values;
    clampMotors(values.values, kSpeedCommandMin, kSpeedCommandMax);
    const ApiResult result = sdk_->setMotorSpeed(values);
    if (!result.success) {
      warnThrottle(result.message.c_str());
    }
  }

  void onMotorCurrent(const msg::MotorCommand::SharedPtr & msg)
  {
    std::string why;
    if (!acceptMotion(Mode::kTorque, false, why)) {
      warnThrottle(why.c_str());
      return;
    }
    MotorValues values;
    values.values = msg->values;
    clampMotors(values.values, kCurrentCommandMin, kCurrentCommandMax);
    const ApiResult result = sdk_->setMotorCurrent(values);
    if (!result.success) {
      warnThrottle(result.message.c_str());
    }
  }

  void onMotorPosition(const msg::MotorCommand::SharedPtr & msg)
  {
    std::string why;
    if (!acceptMotion(Mode::kPosition, true, why)) {
      warnThrottle(why.c_str());
      return;
    }
    MotorValues values;
    values.values = msg->values;
    const ApiResult result = sdk_->setMotorPosition(values);
    if (!result.success) {
      warnThrottle(result.message.c_str());
    }
  }

  void onStateTimer()
  {
    const auto poll = sdk_->tryPollState();
    if (!poll) {
      return;
    }
    if (!poll->snapshot.connected) {
      publishStatus(false, false, Mode::kNone, false);
      return;
    }
    if (!poll->message.empty()) {
      warnThrottle(poll->message.c_str());
    }
    publishJoints(poll->snapshot.joint_radians);
    publishMotors(poll->snapshot);
    publishFault(poll->snapshot.fault, poll->snapshot.faults);
    publishStatus(
      true, poll->snapshot.motor_enabled, poll->snapshot.mode, poll->snapshot.fault);
  }

  void onTactileTimer()
  {
    std::string error;
    const auto sample = sdk_->tryReadTactile(error);
    if (!error.empty()) {
      warnThrottle(error.c_str());
    }
    if (sample) {
      publishTactile(*sample);
    }
  }

  void onDetailTimer()
  {
    std::string error;
    const auto detail = sdk_->tryReadTactileDetail(error);
    if (!error.empty()) {
      warnThrottle(error.c_str());
    }
    if (detail) {
      publishDetail(*detail);
    }
  }

  void onPressTimer()
  {
    std::string error;
    const auto sample = sdk_->tryReadPress(error);
    if (!error.empty()) {
      warnThrottle(error.c_str());
    }
    if (sample) {
      publishPress(*sample);
    }
  }

  void onCameraTimer()
  {
    CameraModel model;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      model = camera_model_;
    }
    const auto frames = sdk_->tryGrab(publish_undistorted_, model.matrix, model.dist_coeffs);
    if (!frames) {
      return;
    }
    if (!frames->message.empty()) {
      warnThrottle(frames->message.c_str());
    }
    if (!frames->raw.empty() && image_pub_) {
      image_pub_->publish(toImage(frames->raw));
    }
    if (frames->rect_valid && rect_pub_) {
      rect_pub_->publish(toImage(frames->rect));
    }
    if (info_pub_ && model.matrix.size() == 9) {
      info_pub_->publish(toCameraInfo(model));
    }
  }

  void onForceTimer()
  {
    if (!force_active_.load()) {
      return;
    }
    const auto step = sdk_->tryForceStep();
    if (!step) {
      return;
    }
    if (!step->ok) {
      warnThrottle(step->message.c_str());
      return;
    }
    publishJoints(step->joint_radians);
    publishTactile(step->tactile);
  }

  sensor_msgs::msg::Image toImage(const RawImage & image) const
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(image.height);
    msg.width = static_cast<uint32_t>(image.width);
    msg.encoding = image.encoding;
    msg.is_bigendian = false;
    msg.step = image.step;
    msg.data = image.data;
    return msg;
  }

  sensor_msgs::msg::CameraInfo toCameraInfo(const CameraModel & model) const
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = now();
    info.header.frame_id = frame_id_;
    info.height = static_cast<uint32_t>(model.height);
    info.width = static_cast<uint32_t>(model.width);
    info.distortion_model = "plumb_bob";
    info.d = model.dist_coeffs;
    for (size_t i = 0; i < info.k.size() && i < model.matrix.size(); ++i) {
      info.k[i] = model.matrix[i];
    }
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {
      model.matrix[0], model.matrix[1], model.matrix[2], 0.0,
      model.matrix[3], model.matrix[4], model.matrix[5], 0.0,
      model.matrix[6], model.matrix[7], model.matrix[8], 0.0};
    return info;
  }

  void onConnect(const Trigger::Response::SharedPtr & response)
  {
    if (sdk_->context().upgrading) {
      response->success = false;
      response->message = "固件升级进行中";
      return;
    }
    fill(bringUpSerial(), response);
  }

  void onDisconnect(const Trigger::Response::SharedPtr & response)
  {
    stopForce();
    fill(sdk_->disconnect(), response);
    publishStatus(false, false, Mode::kNone, false);
  }

  void onEnableMotor(
    const SetBool::Request::SharedPtr & request, const SetBool::Response::SharedPtr & response)
  {
    if (!request->data) {
      stopForce();
      fill(sdk_->disable(), response);
      return;
    }
    fill(sdk_->enable(), response);
  }

  void onSetMode(
    const srv::SetControlMode::Request::SharedPtr & request,
    const srv::SetControlMode::Response::SharedPtr & response)
  {
    Mode mode = Mode::kNone;
    if (!parseMode(request->mode, mode)) {
      response->success = false;
      response->message = "mode 必须是 position、speed 或 torque";
      return;
    }
    if (force_active_.load() && mode != Mode::kPosition) {
      stopForce();
    }
    fill(sdk_->setMode(mode), response);
  }

  void onSetLimits(
    const srv::SetMotorLimits::Request::SharedPtr & request,
    const srv::SetMotorLimits::Response::SharedPtr & response)
  {
    if (!request->apply_current && !request->apply_speed) {
      response->success = false;
      response->message = "apply_current 和 apply_speed 都为 false";
      return;
    }
    std::string message;
    bool success = true;
    if (request->apply_current) {
      int current = request->max_current_ma;
      if (clamp_) {
        current = std::clamp(current, kMaxCurrentMin, kMaxCurrentMax);
      }
      const ApiResult result = sdk_->setMaxCurrent(current);
      success = success && result.success;
      message += result.message;
    }
    if (request->apply_speed) {
      int speed = request->max_speed_rpm;
      if (clamp_) {
        speed = std::clamp(speed, kMaxSpeedMin, kMaxSpeedMax);
      }
      const ApiResult result = sdk_->setMaxSpeed(speed);
      success = success && result.success;
      if (!message.empty()) {
        message += "；";
      }
      message += result.message;
    }
    response->success = success;
    response->message = message;
  }

  void onGetFault(const srv::GetFault::Response::SharedPtr & response)
  {
    std::vector<FaultItem> items;
    bool fault = false;
    const ApiResult result = sdk_->readFault(fault, items);
    fill(result, response);
    response->fault = fault;
    for (const FaultItem & item : items) {
      response->finger_name.push_back(item.finger_name);
      response->motor_id.push_back(item.motor_id);
      response->error_code.push_back(item.error_code);
    }
    if (result.success) {
      publishFault(fault, items);
    }
  }

  void onGetVersion(const srv::GetVersion::Response::SharedPtr & response)
  {
    const VersionInfo info = sdk_->readVersions();
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      if (!info.sdk.empty()) {
        sdk_version_ = info.sdk;
      }
      if (!info.firmware.empty()) {
        firmware_version_ = info.firmware;
      }
    }
    response->success = info.success;
    response->message = info.message;
    response->sdk_version = info.sdk;
    response->firmware_version = info.firmware;
  }

  void onSetDegrees(
    const srv::SetJointAngles::Request::SharedPtr & request,
    const srv::SetJointAngles::Response::SharedPtr & response)
  {
    std::string why;
    if (!acceptMotion(Mode::kPosition, true, why)) {
      response->success = false;
      response->message = why;
      return;
    }
    if (request->degrees.size() != static_cast<size_t>(kJointCount)) {
      response->success = false;
      response->message = "关节角度数量必须是 16";
      return;
    }
    std::vector<double> degrees = request->degrees;
    clampDegrees(degrees);
    fill(sdk_->setJointDegrees(degrees), response);
  }

  void onEnableForce(
    const SetBool::Request::SharedPtr & request, const SetBool::Response::SharedPtr & response)
  {
    if (!request->data) {
      stopForce();
      response->success = true;
      response->message = "已停止力控环";
      publishStatusFromContext();
      return;
    }
    const CommandContext ctx = sdk_->context();
    if (!ctx.connected) {
      response->success = false;
      response->message = "灵巧手未连接";
      return;
    }
    if (!ctx.motor_enabled) {
      response->success = false;
      response->message = "电机未使能";
      return;
    }
    if (ctx.mode != Mode::kPosition) {
      response->success = false;
      response->message = "力控环需要位置模式";
      return;
    }
    if (!ctx.force_params_ready) {
      response->success = false;
      response->message = "先调用 set_force_params";
      return;
    }
    if (force_hz_ <= 0.0) {
      response->success = false;
      response->message = "force_control_hz 必须大于 0";
      return;
    }
    force_active_.store(true);
    if (!force_timer_) {
      force_timer_ = makeTimer(force_hz_, [this] { onForceTimer(); });
    }
    response->success = true;
    response->message = "力控环已启动";
    publishStatusFromContext();
  }

  void publishStatusFromContext()
  {
    const CommandContext ctx = sdk_->context();
    publishStatus(ctx.connected, ctx.motor_enabled, ctx.mode, false);
  }

  void onConnectEthercat(
    const srv::ConnectEthercat::Request::SharedPtr & request,
    const srv::ConnectEthercat::Response::SharedPtr & response)
  {
    if (sdk_->context().upgrading) {
      response->success = false;
      response->message = "固件升级进行中";
      return;
    }
    stopForce();
    const std::string previous_camera = camera_port_;
    if (!request->camera_port.empty()) {
      camera_port_ = request->camera_port;
    }
    const ApiResult result = finishBringUp(sdk_->connectEthercat(
      request->address, request->interface_name, camera_port_, expected_side_));
    if (!result.success) {
      camera_port_ = previous_camera;
    }
    response->hand_type = result.success ? sideName() : "none";
    fill(result, response);
  }

  std::string sideName()
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return side_name_;
  }

  void onSetCamera(
    const srv::SetCameraConfig::Request::SharedPtr & request,
    const srv::SetCameraConfig::Response::SharedPtr & response)
  {
    if (!cameraEnabled()) {
      response->success = false;
      response->message = "camera_port 为 none，节点没有打开相机";
      return;
    }
    camera_width_ = request->width;
    camera_height_ = request->height;
    camera_fps_ = request->fps;
    const ApiResult config = sdk_->setCameraConfig(
      camera_width_, camera_height_, static_cast<float>(camera_fps_));
    if (!config.success) {
      fill(config, response);
      return;
    }
    CameraModel model;
    const ApiResult calibration = sdk_->readCameraModel(camera_width_, camera_height_, model);
    if (calibration.success) {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      camera_model_ = std::move(model);
    }
    fill(calibration.success ? config : calibration, response);
  }

  void onGetCamera(const srv::GetCameraCalibration::Response::SharedPtr & response)
  {
    CameraModel model;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      model = camera_model_;
    }
    if (model.matrix.size() != 9) {
      const ApiResult calibration = sdk_->readCameraModel(camera_width_, camera_height_, model);
      if (!calibration.success) {
        fill(calibration, response);
        return;
      }
      std::lock_guard<std::mutex> lock(cache_mutex_);
      camera_model_ = model;
    }
    response->success = true;
    response->message = "已读取相机标定";
    response->width = model.width;
    response->height = model.height;
    response->camera_matrix = model.matrix;
    response->dist_coeffs = model.dist_coeffs;
    response->hand_eye_matrix = model.hand_eye;
  }

  void onSaveImage(
    const srv::SaveImage::Request::SharedPtr & request,
    const srv::SaveImage::Response::SharedPtr & response)
  {
    CameraModel model;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      model = camera_model_;
    }
    fill(sdk_->saveImage(request->path, request->undistort, model.matrix, model.dist_coeffs), response);
  }

  void onReadAllTactile(const Trigger::Response::SharedPtr & response)
  {
    AllTactile sample;
    const ApiResult result = sdk_->readAllTactile(sample);
    if (result.success) {
      publishTactile(sample.resultant);
      publishDetail(sample.detail);
    }
    fill(result, response);
  }

  void publishUpgrade(const std::vector<UpgradeSample> & samples)
  {
    for (const UpgradeSample & sample : samples) {
      msg::UpgradeProgress msg;
      msg.upgrade_type = sample.type;
      msg.part = sample.part;
      msg.percent = sample.percent;
      msg.status = sample.status;
      upgrade_pub_->publish(msg);
    }
  }

  template<typename Function>
  UpgradeOutcome runUpgrade(Function && function)
  {
    const int held = inflight_.fetch_add(1);
    (void)held;
    struct Guard
    {
      std::atomic<int> & counter;
      ~Guard() { counter.fetch_sub(1); }
    } guard{inflight_};
    std::atomic<bool> stop{false};
    std::thread progress([this, &stop] {
      rclcpp::Rate rate(2.0);
      while (!stop.load() && rclcpp::ok()) {
        publishUpgrade(sdk_->readUpgradeInfo());
        rate.sleep();
      }
    });
    UpgradeOutcome outcome = function();
    stop.store(true);
    if (progress.joinable()) {
      progress.join();
    }
    publishUpgrade(sdk_->readUpgradeInfo());
    return outcome;
  }

  void onUpgradeMainboard(
    const srv::UpgradeMainboard::Request::SharedPtr & request,
    const srv::UpgradeMainboard::Response::SharedPtr & response)
  {
    if (!request->confirm) {
      response->success = false;
      response->message = "升级主控会改写固件，请求里必须 confirm: true";
      return;
    }
    stopForce();
    const UpgradeOutcome outcome = runUpgrade([this, &request] {
      return sdk_->upgradeMainboard(request->file_names);
    });
    response->success = outcome.success;
    response->message = outcome.message;
    response->error_code = outcome.error_code;
    response->error_name = outcome.error_name;
  }

  void onUpgradeDriveboard(
    const srv::UpgradeDriveboard::Request::SharedPtr & request,
    const srv::UpgradeDriveboard::Response::SharedPtr & response)
  {
    if (!request->confirm) {
      response->success = false;
      response->message = "升级电驱会改写固件，请求里必须 confirm: true";
      return;
    }
    std::vector<UpgradeFileSpec> files;
    files.reserve(request->files.size());
    for (const auto & file : request->files) {
      UpgradeFileSpec spec;
      spec.file_path = file.file_path;
      spec.parts = file.parts;
      files.push_back(std::move(spec));
    }
    stopForce();
    const UpgradeOutcome outcome = runUpgrade([this, files = std::move(files)] {
      return sdk_->upgradeDriveboard(files);
    });
    response->success = outcome.success;
    response->message = outcome.message;
    response->error_code = outcome.error_code;
    response->error_name = outcome.error_name;
  }

  std::unique_ptr<Dexh13Sdk> sdk_;
  Side expected_side_{Side::kLeft};
  Mode startup_mode_{Mode::kPosition};
  std::string hand_port_;
  std::string camera_port_;
  std::string prefix_;
  std::string frame_id_;
  std::vector<std::string> joint_names_;
  bool connect_on_start_{true};
  bool enable_on_start_{true};
  bool init_on_start_{false};
  bool clamp_{true};
  bool publish_undistorted_{false};
  int max_current_ma_{kMaxCurrentMax};
  int max_speed_rpm_{kMaxSpeedMax};
  int camera_width_{640};
  int camera_height_{480};
  double camera_fps_{30.0};
  double state_hz_{20.0};
  double tactile_hz_{10.0};
  double tactile_detail_hz_{0.0};
  double press_force_hz_{0.0};
  double camera_hz_{15.0};
  double force_hz_{30.0};

  std::mutex cache_mutex_;
  std::string sdk_version_;
  std::string firmware_version_;
  std::string side_name_{"none"};
  std::string fault_key_;
  std::string status_key_;
  CameraModel camera_model_;
  std::atomic<bool> force_active_{false};
  std::atomic<int> inflight_{0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<msg::MotorCommand>::SharedPtr speed_cmd_sub_;
  rclcpp::Subscription<msg::MotorCommand>::SharedPtr current_cmd_sub_;
  rclcpp::Subscription<msg::MotorCommand>::SharedPtr motor_position_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<msg::MotorState>::SharedPtr motor_pub_;
  rclcpp::Publisher<msg::HandStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<msg::Tactile>::SharedPtr tactile_pub_;
  rclcpp::Publisher<msg::TactileDetail>::SharedPtr detail_pub_;
  rclcpp::Publisher<msg::PressForce>::SharedPtr press_pub_;
  rclcpp::Publisher<msg::Fault>::SharedPtr fault_pub_;
  rclcpp::Publisher<msg::UpgradeProgress>::SharedPtr upgrade_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  rclcpp::Service<Trigger>::SharedPtr connect_srv_;
  rclcpp::Service<Trigger>::SharedPtr disconnect_srv_;
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<Trigger>::SharedPtr init_srv_;
  rclcpp::Service<srv::SetControlMode>::SharedPtr mode_srv_;
  rclcpp::Service<srv::SetMotorLimits>::SharedPtr limits_srv_;
  rclcpp::Service<Trigger>::SharedPtr clear_fault_srv_;
  rclcpp::Service<srv::GetFault>::SharedPtr fault_srv_;
  rclcpp::Service<srv::GetVersion>::SharedPtr version_srv_;
  rclcpp::Service<Trigger>::SharedPtr calibrate_srv_;
  rclcpp::Service<srv::ForwardKinematics>::SharedPtr fk_srv_;
  rclcpp::Service<srv::InverseKinematics>::SharedPtr ik_srv_;
  rclcpp::Service<srv::SetJointAngles>::SharedPtr set_deg_srv_;
  rclcpp::Service<srv::GetJointAngles>::SharedPtr get_deg_srv_;
  rclcpp::Service<srv::SetForceParams>::SharedPtr force_param_srv_;
  rclcpp::Service<srv::ComputeForceCommand>::SharedPtr force_compute_srv_;
  rclcpp::Service<SetBool>::SharedPtr force_enable_srv_;
  rclcpp::Service<srv::ScanEthercat>::SharedPtr scan_srv_;
  rclcpp::Service<srv::ConnectEthercat>::SharedPtr ethercat_srv_;
  rclcpp::Service<srv::SetCameraConfig>::SharedPtr camera_cfg_srv_;
  rclcpp::Service<srv::GetCameraCalibration>::SharedPtr camera_cal_srv_;
  rclcpp::Service<srv::SaveImage>::SharedPtr save_image_srv_;
  rclcpp::Service<Trigger>::SharedPtr all_tactile_srv_;
  rclcpp::Service<srv::UpgradeMainboard>::SharedPtr mainboard_srv_;
  rclcpp::Service<srv::UpgradeDriveboard>::SharedPtr driveboard_srv_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr tactile_timer_;
  rclcpp::TimerBase::SharedPtr detail_timer_;
  rclcpp::TimerBase::SharedPtr press_timer_;
  rclcpp::TimerBase::SharedPtr camera_timer_;
  rclcpp::TimerBase::SharedPtr force_timer_;
};

std::shared_ptr<rclcpp::Node> createNode()
{
  return std::make_shared<Dexh13Node>();
}

}  // namespace dexh13_hand
