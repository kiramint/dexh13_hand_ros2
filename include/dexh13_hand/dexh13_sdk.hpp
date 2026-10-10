#pragma once

#include "dexh13_hand/constants.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace dexh13_hand
{

struct ApiResult
{
  bool success{false};
  std::string message;
};

enum class Side
{
  kNone,
  kLeft,
  kRight,
  kAuto,
};

enum class Mode
{
  kNone,
  kTorque,
  kSpeed,
  kPosition,
};

inline const char * toString(Side side)
{
  switch (side) {
    case Side::kLeft:
      return "left";
    case Side::kRight:
      return "right";
    case Side::kAuto:
      return "auto";
    case Side::kNone:
    default:
      return "none";
  }
}

inline const char * toString(Mode mode)
{
  switch (mode) {
    case Mode::kTorque:
      return "torque";
    case Mode::kSpeed:
      return "speed";
    case Mode::kPosition:
      return "position";
    case Mode::kNone:
    default:
      return "none";
  }
}

inline bool parseSide(const std::string & text, Side & side)
{
  if (text == "left") {
    side = Side::kLeft;
    return true;
  }
  if (text == "right") {
    side = Side::kRight;
    return true;
  }
  if (text == "auto") {
    side = Side::kAuto;
    return true;
  }
  return false;
}

inline bool parseMode(const std::string & text, Mode & mode)
{
  if (text == "position") {
    mode = Mode::kPosition;
    return true;
  }
  if (text == "speed") {
    mode = Mode::kSpeed;
    return true;
  }
  if (text == "torque") {
    mode = Mode::kTorque;
    return true;
  }
  return false;
}

struct MotorValues
{
  std::array<int16_t, kMotorCount> values{};
};

struct FaultItem
{
  std::string finger_name;
  int motor_id{0};
  std::string error_code;
};

struct TactileSample
{
  std::vector<std::string> name;
  std::vector<int32_t> x;
  std::vector<int32_t> y;
  std::vector<int32_t> z;
  std::vector<std::string> temp_name;
  std::vector<int32_t> sensor_id;
  std::vector<float> temperature;
};

struct TactileDetail
{
  std::vector<int32_t> x;
  std::vector<int32_t> y;
  std::vector<int32_t> z;
};

struct PressSample
{
  std::vector<std::string> model_name;
  std::vector<int32_t> x;
  std::vector<int32_t> y;
  std::vector<int32_t> z;
  std::vector<uint32_t> force;
};

struct AllTactile
{
  TactileSample resultant;
  TactileDetail detail;
};

struct DeviceSnapshot
{
  std::vector<double> joint_radians;
  MotorValues position;
  MotorValues speed_rpm;
  MotorValues current_ma;
  bool motor_enabled{false};
  Mode mode{Mode::kNone};
  bool fault{false};
  std::vector<FaultItem> faults;
  bool connected{false};
};

struct PollResult
{
  bool ok{false};
  std::string message;
  DeviceSnapshot snapshot;
};

struct ForceStep
{
  bool ok{false};
  std::string message;
  std::vector<double> joint_radians;
  std::vector<double> command_radians;
  TactileSample tactile;
};

struct RawImage
{
  int width{0};
  int height{0};
  std::string encoding;
  uint32_t step{0};
  std::vector<uint8_t> data;

  bool empty() const
  {
    return data.empty() || width <= 0 || height <= 0;
  }
};

struct CameraFrames
{
  RawImage raw;
  RawImage rect;
  bool rect_valid{false};
  std::string message;
};

struct CameraModel
{
  int width{0};
  int height{0};
  std::vector<double> matrix;
  std::vector<double> dist_coeffs;
  std::vector<double> hand_eye;
};

struct VersionInfo
{
  bool success{false};
  std::string message;
  std::string sdk;
  std::string firmware;
};

struct UpgradeSample
{
  uint8_t type{0};
  int16_t part{0};
  int32_t percent{0};
  std::string status;
};

struct UpgradeOutcome
{
  bool success{false};
  std::string message;
  int error_code{0};
  std::string error_name;
};

struct CommandContext
{
  bool connected{false};
  bool upgrading{false};
  bool motor_enabled{false};
  Mode mode{Mode::kNone};
  Side side{Side::kNone};
  bool force_params_ready{false};
};

struct UpgradeFileSpec
{
  std::string file_path;
  std::vector<int16_t> parts;
};

struct Dexh13SdkImpl;

// One process owns one hand. Every SDK call takes the same mutex, except the
// blocking firmware upgrade: the SDK requires getUpgradeInfo on another thread
// while upgradeMainboard or upgradeDriveboard is running.
class Dexh13Sdk
{
public:
  Dexh13Sdk();
  ~Dexh13Sdk();

  Dexh13Sdk(const Dexh13Sdk &) = delete;
  Dexh13Sdk & operator=(const Dexh13Sdk &) = delete;

  void setExpectedSide(Side side);

  ApiResult connectSerial(
    const std::string & hand_port, const std::string & camera_port, Side expected);
  ApiResult connectEthercat(
    uint16_t address, const std::string & interface_name, const std::string & camera_port,
    Side expected);
  ApiResult disconnect();
  void shutdown();
  void waitUntilUpgradeDone();

  CommandContext context() const;
  VersionInfo readVersions();

  ApiResult setMode(Mode mode);
  ApiResult setMaxCurrent(int milliamps);
  ApiResult setMaxSpeed(int rpm);
  ApiResult enable();
  ApiResult disable();
  ApiResult initMotorPosition();
  ApiResult clearFault();
  ApiResult calibrateTactile();
  ApiResult readFault(bool & fault, std::vector<FaultItem> & items);

  ApiResult setJointRadians(const std::vector<double> & radians);
  ApiResult setJointDegrees(const std::vector<double> & degrees);
  ApiResult readJointDegrees(std::vector<double> & degrees);
  ApiResult setMotorSpeed(const MotorValues & speed);
  ApiResult setMotorCurrent(const MotorValues & current);
  ApiResult setMotorPosition(const MotorValues & position);

  std::optional<PollResult> tryPollState();
  std::optional<TactileSample> tryReadTactile(std::string & error);
  std::optional<TactileDetail> tryReadTactileDetail(std::string & error);
  std::optional<PressSample> tryReadPress(std::string & error);
  ApiResult readAllTactile(AllTactile & sample);

  ApiResult forwardKinematics(const MotorValues & motors, std::vector<double> & radians);
  ApiResult inverseKinematics(const std::vector<double> & radians, MotorValues & motors);
  ApiResult setForceParams(
    const std::vector<double> & impedance, const std::vector<double> & reference_force,
    const std::vector<bool> & enable);
  ApiResult computeForceCommand(
    const std::vector<double> & current_radians, std::vector<double> & command_radians);
  std::optional<ForceStep> tryForceStep();

  ApiResult scanEthercat(const std::string & interface_name, std::vector<uint16_t> & addresses);

  bool undistortAvailable() const;
  ApiResult setCameraConfig(int width, int height, float fps);
  ApiResult readCameraModel(int width, int height, CameraModel & model);
  std::optional<CameraFrames> tryGrab(
    bool undistort, const std::vector<double> & matrix, const std::vector<double> & dist_coeffs);
  ApiResult saveImage(const std::string & path, bool undistort,
    const std::vector<double> & matrix, const std::vector<double> & dist_coeffs);

  UpgradeOutcome upgradeMainboard(const std::vector<std::string> & file_names);
  UpgradeOutcome upgradeDriveboard(const std::vector<UpgradeFileSpec> & files);
  std::vector<UpgradeSample> readUpgradeInfo();

private:
  std::unique_ptr<Dexh13SdkImpl> impl_;
};

}  // namespace dexh13_hand
