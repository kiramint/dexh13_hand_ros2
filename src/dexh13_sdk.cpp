#include "dexh13_hand/dexh13_sdk.hpp"

#include <dexh13/dexh13_control.h>
#include <dexh13/dexh13_force_control.h>
#include <dexh13/dexh13_kinematic.h>

#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <utility>

#include <dlfcn.h>

namespace dexh13_hand
{
namespace
{

using paxini::bot::dexh13::ControlMode;
using paxini::bot::dexh13::Dex13MotorCurrent;
using paxini::bot::dexh13::Dex13MotorPosition;
using paxini::bot::dexh13::Dex13MotorSpeed;
using paxini::bot::dexh13::ErrorCode;
using paxini::bot::dexh13::FingerAngle;
using paxini::bot::dexh13::FingerTemp;
using paxini::bot::dexh13::ForcePoint;
using paxini::bot::dexh13::HandType;
using paxini::bot::dexh13::PressForcePoint;
using paxini::bot::dexh13::UpgradeErrorCode;
using paxini::bot::dexh13::UpgradeFileAndPart;

ApiResult okResult(std::string message = {})
{
  ApiResult result;
  result.success = true;
  result.message = std::move(message);
  return result;
}

ApiResult failResult(std::string message)
{
  ApiResult result;
  result.success = false;
  result.message = std::move(message);
  return result;
}

ApiResult fromBool(bool value, const char * action)
{
  if (value) {
    return okResult(action);
  }
  return failResult(std::string(action) + " 失败");
}

ApiResult fromInt(int value, const char * action)
{
  if (value == 1) {
    return okResult(action);
  }
  return failResult(std::string(action) + " 失败");
}

Side sideFromSdk(int type)
{
  if (type == static_cast<int>(HandType::LEFT_HAND)) {
    return Side::kLeft;
  }
  if (type == static_cast<int>(HandType::RIGHT_HAND)) {
    return Side::kRight;
  }
  return Side::kNone;
}

HandType sideToSdk(Side side)
{
  if (side == Side::kLeft) {
    return HandType::LEFT_HAND;
  }
  if (side == Side::kRight) {
    return HandType::RIGHT_HAND;
  }
  return HandType::NONE_HAND;
}

Mode modeFromSdk(ControlMode mode)
{
  switch (mode) {
    case ControlMode::TORQUE_CONTROL_MODE:
      return Mode::kTorque;
    case ControlMode::SPEED_CONTROL_MODE:
      return Mode::kSpeed;
    case ControlMode::POSITION_CONTROL_MODE:
      return Mode::kPosition;
    case ControlMode::NONE:
    default:
      return Mode::kNone;
  }
}

ControlMode modeToSdk(Mode mode)
{
  switch (mode) {
    case Mode::kTorque:
      return ControlMode::TORQUE_CONTROL_MODE;
    case Mode::kSpeed:
      return ControlMode::SPEED_CONTROL_MODE;
    case Mode::kPosition:
      return ControlMode::POSITION_CONTROL_MODE;
    case Mode::kNone:
    default:
      return ControlMode::NONE;
  }
}

std::string errorCodeName(ErrorCode code)
{
  switch (code) {
    case ErrorCode::OVER_VOLT:
      return "OVER_VOLT";
    case ErrorCode::UNDER_VOLT:
      return "UNDER_VOLT";
    case ErrorCode::OVER_CURRENTU:
      return "OVER_CURRENTU";
    case ErrorCode::OVER_CURRENTV:
      return "OVER_CURRENTV";
    case ErrorCode::OVER_CURRENTW:
      return "OVER_CURRENTW";
    case ErrorCode::OVER_TEMPERATURE:
      return "OVER_TEMPERATURE";
    case ErrorCode::ADC_CALIB:
      return "ADC_CALIB";
    case ErrorCode::FOC_ERR:
      return "FOC_ERR";
    case ErrorCode::OVER_IQ:
      return "OVER_IQ";
    case ErrorCode::OPD_U:
      return "OPD_U";
    case ErrorCode::OPD_V:
      return "OPD_V";
    case ErrorCode::OPD_W:
      return "OPD_W";
    case ErrorCode::POS_REF_OVER:
      return "POS_REF_OVER";
    case ErrorCode::POS_REF_UNDER:
      return "POS_REF_UNDER";
    case ErrorCode::ABZ_ERR:
      return "ABZ_ERR";
    case ErrorCode::ABZ_Z_ERR:
      return "ABZ_Z_ERR";
    case ErrorCode::DISCONNECT:
      return "DISCONNECT";
    default: {
      const int value = static_cast<int>(code);
      if (value >= static_cast<int>(ErrorCode::RESERVED0) &&
        value <= static_cast<int>(ErrorCode::RESERVED14))
      {
        return "RESERVED_" + std::to_string(value);
      }
      return "UNKNOWN_" + std::to_string(value);
    }
  }
}

std::string upgradeErrorName(UpgradeErrorCode code)
{
  switch (code) {
    case UpgradeErrorCode::UPGRADE_CHECK_OK:
      return "UPGRADE_CHECK_OK";
    case UpgradeErrorCode::UPGRADE_FAIL:
      return "UPGRADE_FAIL";
    case UpgradeErrorCode::UPGRADE_SUCCESS:
      return "UPGRADE_SUCCESS";
    case UpgradeErrorCode::FILE_FORMAT_ERROR:
      return "FILE_FORMAT_ERROR";
    case UpgradeErrorCode::FILE_SIZE_ERROR:
      return "FILE_SIZE_ERROR";
    case UpgradeErrorCode::FILE_MATCH_ERROR:
      return "FILE_MATCH_ERROR";
    case UpgradeErrorCode::FILE_NAME_ERROR:
      return "FILE_NAME_ERROR";
    case UpgradeErrorCode::FILE_COUNT_ERROR:
      return "FILE_COUNT_ERROR";
    case UpgradeErrorCode::FILE_EXIT_ERROR:
      return "FILE_EXIT_ERROR";
    case UpgradeErrorCode::FILE_VERSION_ERROR:
      return "FILE_VERSION_ERROR";
    case UpgradeErrorCode::PARAM_PART_ERROR:
      return "PARAM_PART_ERROR";
    case UpgradeErrorCode::HAND_FAULT_ERROR:
      return "HAND_FAULT_ERROR";
    case UpgradeErrorCode::HAND_ENABLE_ERROR:
      return "HAND_ENABLE_ERROR";
    case UpgradeErrorCode::SEND_FILE_OPERATE_ERROR:
      return "SEND_FILE_OPERATE_ERROR";
    case UpgradeErrorCode::SEND_FRAME_1_ERROR:
      return "SEND_FRAME_1_ERROR";
    case UpgradeErrorCode::SEND_FRAME_N_ERROR:
      return "SEND_FRAME_N_ERROR";
    case UpgradeErrorCode::SEND_FRAME_LAST_ERROR:
      return "SEND_FRAME_LAST_ERROR";
    case UpgradeErrorCode::SEND_EXTRA_ACTION_ERROR:
      return "SEND_EXTRA_ACTION_ERROR";
    case UpgradeErrorCode::FILE_SEND_SUCCESS:
      return "FILE_SEND_SUCCESS";
    case UpgradeErrorCode::UPGRADE_REQUEST_ERROR:
      return "UPGRADE_REQUEST_ERROR";
    case UpgradeErrorCode::UPGRADE_EXECUTE_ERROR:
      return "UPGRADE_EXECUTE_ERROR";
    case UpgradeErrorCode::UPGRADE_TYPE_ERROR:
      return "UPGRADE_TYPE_ERROR";
    default:
      return "UNKNOWN_" + std::to_string(static_cast<int>(code));
  }
}

MotorValues fromCurrent(const Dex13MotorCurrent & current)
{
  MotorValues out;
  out.values = {
    current.motor1_current, current.motor2_current, current.motor3_current,
    current.motor4_current, current.motor5_current, current.motor6_current,
    current.motor7_current, current.motor8_current, current.motor9_current,
    current.motor10_current, current.motor11_current, current.motor12_current,
    current.motor13_current};
  return out;
}

MotorValues fromSpeed(const Dex13MotorSpeed & speed)
{
  MotorValues out;
  out.values = {
    speed.motor1_speed, speed.motor2_speed, speed.motor3_speed, speed.motor4_speed,
    speed.motor5_speed, speed.motor6_speed, speed.motor7_speed, speed.motor8_speed,
    speed.motor9_speed, speed.motor10_speed, speed.motor11_speed, speed.motor12_speed,
    speed.motor13_speed};
  return out;
}

MotorValues fromPosition(const Dex13MotorPosition & position)
{
  MotorValues out;
  out.values = {
    position.motor1_pos, position.motor2_pos, position.motor3_pos, position.motor4_pos,
    position.motor5_pos, position.motor6_pos, position.motor7_pos, position.motor8_pos,
    position.motor9_pos, position.motor10_pos, position.motor11_pos, position.motor12_pos,
    position.motor13_pos};
  return out;
}

Dex13MotorCurrent toCurrent(const MotorValues & values)
{
  Dex13MotorCurrent current;
  current.motor1_current = values.values[0];
  current.motor2_current = values.values[1];
  current.motor3_current = values.values[2];
  current.motor4_current = values.values[3];
  current.motor5_current = values.values[4];
  current.motor6_current = values.values[5];
  current.motor7_current = values.values[6];
  current.motor8_current = values.values[7];
  current.motor9_current = values.values[8];
  current.motor10_current = values.values[9];
  current.motor11_current = values.values[10];
  current.motor12_current = values.values[11];
  current.motor13_current = values.values[12];
  return current;
}

Dex13MotorSpeed toSpeed(const MotorValues & values)
{
  Dex13MotorSpeed speed;
  speed.motor1_speed = values.values[0];
  speed.motor2_speed = values.values[1];
  speed.motor3_speed = values.values[2];
  speed.motor4_speed = values.values[3];
  speed.motor5_speed = values.values[4];
  speed.motor6_speed = values.values[5];
  speed.motor7_speed = values.values[6];
  speed.motor8_speed = values.values[7];
  speed.motor9_speed = values.values[8];
  speed.motor10_speed = values.values[9];
  speed.motor11_speed = values.values[10];
  speed.motor12_speed = values.values[11];
  speed.motor13_speed = values.values[12];
  return speed;
}

Dex13MotorPosition toPosition(const MotorValues & values)
{
  Dex13MotorPosition position;
  position.motor1_pos = values.values[0];
  position.motor2_pos = values.values[1];
  position.motor3_pos = values.values[2];
  position.motor4_pos = values.values[3];
  position.motor5_pos = values.values[4];
  position.motor6_pos = values.values[5];
  position.motor7_pos = values.values[6];
  position.motor8_pos = values.values[7];
  position.motor9_pos = values.values[8];
  position.motor10_pos = values.values[9];
  position.motor11_pos = values.values[10];
  position.motor12_pos = values.values[11];
  position.motor13_pos = values.values[12];
  return position;
}

std::vector<FingerAngle> degreesToFingers(const std::vector<double> & degrees)
{
  std::vector<FingerAngle> fingers(4);
  for (int finger = 0; finger < 4; ++finger) {
    fingers[static_cast<size_t>(finger)].joint1 = degrees[static_cast<size_t>(finger * 4)];
    fingers[static_cast<size_t>(finger)].joint2 = degrees[static_cast<size_t>(finger * 4 + 1)];
    fingers[static_cast<size_t>(finger)].joint3 = degrees[static_cast<size_t>(finger * 4 + 2)];
    fingers[static_cast<size_t>(finger)].joint4 = degrees[static_cast<size_t>(finger * 4 + 3)];
  }
  return fingers;
}

std::vector<double> fingersToDegrees(const std::vector<FingerAngle> & fingers)
{
  std::vector<double> degrees(kJointCount);
  for (int finger = 0; finger < 4; ++finger) {
    degrees[static_cast<size_t>(finger * 4)] = fingers[static_cast<size_t>(finger)].joint1;
    degrees[static_cast<size_t>(finger * 4 + 1)] = fingers[static_cast<size_t>(finger)].joint2;
    degrees[static_cast<size_t>(finger * 4 + 2)] = fingers[static_cast<size_t>(finger)].joint3;
    degrees[static_cast<size_t>(finger * 4 + 3)] = fingers[static_cast<size_t>(finger)].joint4;
  }
  return degrees;
}

void fillForces(const std::vector<ForcePoint> & forces, std::vector<int32_t> & x,
  std::vector<int32_t> & y, std::vector<int32_t> & z)
{
  x.resize(forces.size());
  y.resize(forces.size());
  z.resize(forces.size());
  for (size_t i = 0; i < forces.size(); ++i) {
    x[i] = forces[i].x;
    y[i] = forces[i].y;
    z[i] = forces[i].z;
  }
}

TactileSample makeTactile(
  const std::vector<ForcePoint> & forces, const std::vector<FingerTemp> & temperatures)
{
  TactileSample sample;
  fillForces(forces, sample.x, sample.y, sample.z);
  if (temperatures.size() == forces.size()) {
    sample.name.reserve(temperatures.size());
  }
  sample.temp_name.reserve(temperatures.size());
  sample.sensor_id.reserve(temperatures.size());
  sample.temperature.reserve(temperatures.size());
  for (const FingerTemp & temperature : temperatures) {
    if (temperatures.size() == forces.size()) {
      sample.name.push_back(temperature.finger_name);
    }
    sample.temp_name.push_back(temperature.finger_name);
    sample.sensor_id.push_back(temperature.sensor_ID);
    sample.temperature.push_back(temperature.temperature);
  }
  return sample;
}

TactileDetail makeDetail(const std::vector<ForcePoint> & forces)
{
  TactileDetail detail;
  fillForces(forces, detail.x, detail.y, detail.z);
  return detail;
}

std::vector<double> copyMatrix(const cv::Mat & mat)
{
  std::vector<double> values;
  if (mat.data == nullptr || mat.rows <= 0 || mat.cols <= 0) {
    return values;
  }
  const int type = mat.flags & CV_MAT_TYPE_MASK;
  const int depth = CV_MAT_DEPTH(type);
  const int channels = CV_MAT_CN(type);
  const size_t step = mat.step[0];
  const int row_values = mat.cols * channels;
  values.reserve(static_cast<size_t>(mat.rows * row_values));
  if (depth == CV_64F) {
    for (int row = 0; row < mat.rows; ++row) {
      const auto * data = reinterpret_cast<const double *>(mat.data + step * static_cast<size_t>(row));
      values.insert(values.end(), data, data + row_values);
    }
  } else if (depth == CV_32F) {
    for (int row = 0; row < mat.rows; ++row) {
      const auto * data = reinterpret_cast<const float *>(mat.data + step * static_cast<size_t>(row));
      for (int col = 0; col < row_values; ++col) {
        values.push_back(static_cast<double>(data[col]));
      }
    }
  }
  return values;
}

RawImage copyImage(const cv::Mat & mat)
{
  RawImage image;
  if (mat.data == nullptr || mat.rows <= 0 || mat.cols <= 0) {
    return image;
  }
  const int type = mat.flags & CV_MAT_TYPE_MASK;
  if (CV_MAT_DEPTH(type) != CV_8U) {
    return image;
  }
  const int channels = CV_MAT_CN(type);
  if (channels == 3) {
    image.encoding = "bgr8";
  } else if (channels == 1) {
    image.encoding = "mono8";
  } else if (channels == 4) {
    image.encoding = "bgra8";
  } else {
    return image;
  }
  const size_t step = mat.step[0];
  const size_t row_bytes = static_cast<size_t>(mat.cols * channels);
  image.width = mat.cols;
  image.height = mat.rows;
  image.step = static_cast<uint32_t>(row_bytes);
  image.data.resize(row_bytes * static_cast<size_t>(mat.rows));
  for (int row = 0; row < mat.rows; ++row) {
    std::memcpy(
      image.data.data() + row_bytes * static_cast<size_t>(row),
      mat.data + step * static_cast<size_t>(row), row_bytes);
  }
  return image;
}

cv::Mat matrixFromDoubles(const std::vector<double> & values, int rows, int cols)
{
  cv::Mat mat(rows, cols, CV_64F);
  if (mat.data != nullptr && values.size() == static_cast<size_t>(rows * cols)) {
    std::memcpy(mat.data, values.data(), values.size() * sizeof(double));
  }
  return mat;
}

const char * kFisheyeSymbol =
  "_ZN2cv7fisheye23initUndistortRectifyMapERKNS_11_InputArrayES3_S3_S3_RKNS_5Size_IiEEiRKNS_12_"
  "OutputArrayESA_";

}  // namespace

struct Dexh13SdkImpl
{
  mutable std::mutex mutex;
  std::condition_variable upgrade_cv;
  bool upgrading{false};
  bool connected{false};
  bool serial{false};
  bool force_params_ready{false};
  bool undistort_ready{false};
  Side expected{Side::kNone};
  Side detected{Side::kNone};
  std::unique_ptr<paxini::bot::dexh13::DexH13Control> control;
  std::unique_ptr<paxini::bot::dexh13::DexH13Kinematic> kinematic;
  std::unique_ptr<paxini::bot::dexh13::DexH13ForceControl> force;

  Side resolvedSide() const
  {
    if (detected != Side::kNone) {
      return detected;
    }
    if (expected == Side::kLeft || expected == Side::kRight) {
      return expected;
    }
    return Side::kNone;
  }
};

namespace
{

void safeDisconnect(Dexh13SdkImpl & impl)
{
  try {
    impl.control->disconnectHandy();
  } catch (...) {
  }
  impl.connected = false;
  impl.serial = false;
}

template<typename F>
ApiResult invoke(Dexh13SdkImpl & impl, bool require_connection, F && function)
{
  std::lock_guard<std::mutex> lock(impl.mutex);
  if (impl.upgrading) {
    return failResult("固件升级进行中");
  }
  if (require_connection && !impl.connected) {
    return failResult("灵巧手未连接");
  }
  try {
    return function(impl);
  } catch (const std::exception & ex) {
    return failResult(ex.what());
  } catch (...) {
    return failResult("SDK 调用出现未知异常");
  }
}

class UpgradeSession
{
public:
  explicit UpgradeSession(Dexh13SdkImpl & impl)
  : impl_(impl)
  {
  }

  ~UpgradeSession()
  {
    if (!active_) {
      return;
    }
    std::lock_guard<std::mutex> lock(impl_.mutex);
    impl_.upgrading = false;
    impl_.upgrade_cv.notify_all();
  }

  ApiResult begin()
  {
    std::lock_guard<std::mutex> lock(impl_.mutex);
    if (impl_.upgrading) {
      return failResult("固件升级进行中");
    }
    if (!impl_.connected) {
      return failResult("灵巧手未连接");
    }
    impl_.upgrading = true;
    active_ = true;
    return okResult();
  }

private:
  Dexh13SdkImpl & impl_;
  bool active_{false};
};

}  // namespace

Dexh13Sdk::Dexh13Sdk()
: impl_(std::make_unique<Dexh13SdkImpl>())
{
  impl_->control = std::make_unique<paxini::bot::dexh13::DexH13Control>();
  impl_->kinematic = std::make_unique<paxini::bot::dexh13::DexH13Kinematic>();
  impl_->force = std::make_unique<paxini::bot::dexh13::DexH13ForceControl>();
  impl_->undistort_ready = dlsym(RTLD_DEFAULT, kFisheyeSymbol) != nullptr;
}

Dexh13Sdk::~Dexh13Sdk()
{
  shutdown();
}

void Dexh13Sdk::setExpectedSide(Side side)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->expected = side;
}

ApiResult Dexh13Sdk::connectSerial(
  const std::string & hand_port, const std::string & camera_port, Side expected)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->upgrading) {
    return failResult("固件升级进行中");
  }
  if (hand_port.empty()) {
    return failResult("hand_port 为空");
  }
  if (impl_->connected) {
    safeDisconnect(*impl_);
  }
  const std::string camera = camera_port.empty() ? "none" : camera_port;
  int type = static_cast<int>(HandType::NONE_HAND);
  try {
    type = impl_->control->activeHandy(hand_port, camera);
  } catch (const std::exception & ex) {
    safeDisconnect(*impl_);
    return failResult(ex.what());
  } catch (...) {
    safeDisconnect(*impl_);
    return failResult("activeHandy 失败");
  }
  impl_->serial = true;
  const Side detected = sideFromSdk(type);
  if (detected == Side::kNone) {
    safeDisconnect(*impl_);
    return failResult("设备没有回报左手或右手");
  }
  if (expected != Side::kAuto && expected != detected) {
    safeDisconnect(*impl_);
    return failResult(
      std::string("设备回报 ") + toString(detected) + "，参数 side 是 " + toString(expected));
  }
  bool online = false;
  try {
    online = impl_->control->isConnectHandy();
  } catch (const std::exception & ex) {
    safeDisconnect(*impl_);
    return failResult(ex.what());
  }
  if (!online) {
    safeDisconnect(*impl_);
    return failResult("isConnectHandy 返回未连接");
  }
  impl_->connected = true;
  impl_->detected = detected;
  return okResult(std::string("已连接，手型为 ") + toString(detected));
}

ApiResult Dexh13Sdk::connectEthercat(
  uint16_t address, const std::string & interface_name, const std::string & camera_port,
  Side expected)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->upgrading) {
    return failResult("固件升级进行中");
  }
  if (interface_name.empty()) {
    return failResult("网卡名为空");
  }
  if (impl_->connected) {
    safeDisconnect(*impl_);
  }
  const std::string camera = camera_port.empty() ? "none" : camera_port;
  int type = static_cast<int>(HandType::NONE_HAND);
  try {
    type = impl_->control->connectHandByEthercat(address, interface_name, camera);
  } catch (const std::exception & ex) {
    safeDisconnect(*impl_);
    return failResult(ex.what());
  } catch (...) {
    safeDisconnect(*impl_);
    return failResult("connectHandByEthercat 失败");
  }
  impl_->serial = false;
  const Side detected = sideFromSdk(type);
  if (detected == Side::kNone) {
    safeDisconnect(*impl_);
    return failResult("设备没有回报左手或右手");
  }
  if (expected != Side::kAuto && expected != detected) {
    safeDisconnect(*impl_);
    return failResult(
      std::string("设备回报 ") + toString(detected) + "，参数 side 是 " + toString(expected));
  }
  impl_->connected = true;
  impl_->detected = detected;
  return okResult(std::string("已通过 EtherCAT 连接，手型为 ") + toString(detected));
}

ApiResult Dexh13Sdk::disconnect()
{
  return invoke(*impl_, false, [](Dexh13SdkImpl & impl) {
    if (!impl.connected) {
      return okResult("灵巧手未连接");
    }
    const int rc = impl.control->disconnectHandy();
    impl.connected = false;
    impl.serial = false;
    return fromInt(rc, "断开连接");
  });
}

void Dexh13Sdk::shutdown()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->upgrading || !impl_->connected) {
    return;
  }
  try {
    impl_->control->disableMotor();
  } catch (...) {
  }
  safeDisconnect(*impl_);
}

void Dexh13Sdk::waitUntilUpgradeDone()
{
  std::unique_lock<std::mutex> lock(impl_->mutex);
  impl_->upgrade_cv.wait(lock, [this] { return !impl_->upgrading; });
}

CommandContext Dexh13Sdk::context() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  CommandContext ctx;
  ctx.connected = impl_->connected;
  ctx.upgrading = impl_->upgrading;
  ctx.side = impl_->resolvedSide();
  ctx.force_params_ready = impl_->force_params_ready;
  if (!impl_->connected || impl_->upgrading) {
    return ctx;
  }
  try {
    ctx.motor_enabled = impl_->control->isMotorEnabled();
    ctx.mode = modeFromSdk(impl_->control->getMotorControlMode());
  } catch (...) {
  }
  return ctx;
}

VersionInfo Dexh13Sdk::readVersions()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  VersionInfo info;
  try {
    info.sdk = impl_->control->getSDKVersion();
  } catch (const std::exception & ex) {
    info.message = ex.what();
    return info;
  }
  if (!impl_->connected) {
    info.success = !info.sdk.empty();
    info.message = info.success ? "未连接，没有固件版本" : "读取 SDK 版本失败";
    return info;
  }
  try {
    info.firmware = impl_->control->getFirmwareVersion();
    info.success = true;
    info.message = "已读取版本";
  } catch (const std::exception & ex) {
    info.message = ex.what();
  }
  return info;
}

ApiResult Dexh13Sdk::setMode(Mode mode)
{
  return invoke(*impl_, true, [mode](Dexh13SdkImpl & impl) {
    if (mode == Mode::kNone) {
      return failResult("控制模式无效");
    }
    return fromBool(impl.control->setMotorControlMode(modeToSdk(mode)), "设置控制模式");
  });
}

ApiResult Dexh13Sdk::setMaxCurrent(int milliamps)
{
  return invoke(*impl_, true, [milliamps](Dexh13SdkImpl & impl) {
    return fromBool(impl.control->setMotorMaxCurrent(milliamps), "设置最大电流");
  });
}

ApiResult Dexh13Sdk::setMaxSpeed(int rpm)
{
  return invoke(*impl_, true, [rpm](Dexh13SdkImpl & impl) {
    return fromBool(impl.control->setMotorMaxSpeed(rpm), "设置最大速度");
  });
}

ApiResult Dexh13Sdk::enable()
{
  return invoke(*impl_, true, [](Dexh13SdkImpl & impl) {
    return fromBool(impl.control->enableMotor(), "使能电机");
  });
}

ApiResult Dexh13Sdk::disable()
{
  return invoke(*impl_, true, [](Dexh13SdkImpl & impl) {
    return fromBool(impl.control->disableMotor(), "下使能电机");
  });
}

ApiResult Dexh13Sdk::initMotorPosition()
{
  return invoke(*impl_, true, [](Dexh13SdkImpl & impl) {
    return fromInt(impl.control->initMotorPosition(), "初始化电机位置");
  });
}

ApiResult Dexh13Sdk::clearFault()
{
  return invoke(*impl_, true, [](Dexh13SdkImpl & impl) {
    return fromBool(impl.control->clearFaultCode(), "清除故障");
  });
}

ApiResult Dexh13Sdk::calibrateTactile()
{
  return invoke(*impl_, true, [](Dexh13SdkImpl & impl) {
    return fromBool(impl.control->calibrateSensor(), "触觉标定");
  });
}

ApiResult Dexh13Sdk::readFault(bool & fault, std::vector<FaultItem> & items)
{
  return invoke(*impl_, true, [&fault, &items](Dexh13SdkImpl & impl) {
    fault = impl.control->isFault();
    items.clear();
    if (!fault) {
      return okResult("无故障");
    }
    const auto codes = impl.control->getFaultCode();
    items.reserve(codes.size());
    for (const auto & code : codes) {
      FaultItem item;
      item.finger_name = code.finger_name;
      item.motor_id = code.motor_ID;
      item.error_code = errorCodeName(code.error_code);
      items.push_back(std::move(item));
    }
    return okResult("已读取故障");
  });
}

ApiResult Dexh13Sdk::setJointRadians(const std::vector<double> & radians)
{
  return invoke(*impl_, true, [&radians](Dexh13SdkImpl & impl) {
    if (radians.size() != static_cast<size_t>(kJointCount)) {
      return failResult("关节弧度数量必须是 16");
    }
    return fromInt(impl.control->setJointPositionsRadian(radians), "设置关节弧度");
  });
}

ApiResult Dexh13Sdk::setJointDegrees(const std::vector<double> & degrees)
{
  return invoke(*impl_, true, [&degrees](Dexh13SdkImpl & impl) {
    if (degrees.size() != static_cast<size_t>(kJointCount)) {
      return failResult("关节角度数量必须是 16");
    }
    return fromInt(
      impl.control->setJointPositionsAngle(degreesToFingers(degrees)), "设置关节角度");
  });
}

ApiResult Dexh13Sdk::readJointDegrees(std::vector<double> & degrees)
{
  return invoke(*impl_, true, [&degrees](Dexh13SdkImpl & impl) {
    std::vector<FingerAngle> angles;
    impl.control->getJointPositionsAngle(angles);
    if (angles.size() != 4) {
      return failResult("SDK 返回的手指数不是 4");
    }
    degrees = fingersToDegrees(angles);
    return okResult("已读取关节角度");
  });
}

ApiResult Dexh13Sdk::setMotorSpeed(const MotorValues & speed)
{
  return invoke(*impl_, true, [&speed](Dexh13SdkImpl & impl) {
    return fromInt(impl.control->setMotorTargetSpeed(toSpeed(speed)), "设置电机速度");
  });
}

ApiResult Dexh13Sdk::setMotorCurrent(const MotorValues & current)
{
  return invoke(*impl_, true, [&current](Dexh13SdkImpl & impl) {
    return fromInt(impl.control->setMotorTargetCurrent(toCurrent(current)), "设置电机电流");
  });
}

ApiResult Dexh13Sdk::setMotorPosition(const MotorValues & position)
{
  return invoke(*impl_, true, [&position](Dexh13SdkImpl & impl) {
    return fromInt(impl.control->setMotorTargetPosition(toPosition(position)), "设置电机位置");
  });
}

std::optional<PollResult> Dexh13Sdk::tryPollState()
{
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock() || impl_->upgrading) {
    return std::nullopt;
  }
  PollResult result;
  result.snapshot.connected = impl_->connected;
  if (!impl_->connected) {
    result.ok = false;
    result.message = "灵巧手未连接";
    return result;
  }
  try {
    if (impl_->serial && !impl_->control->isConnectHandy()) {
      impl_->connected = false;
      result.snapshot.connected = false;
      result.ok = false;
      result.message = "isConnectHandy 返回未连接";
      return result;
    }
    impl_->control->getJointPositionsRadian(result.snapshot.joint_radians);
    result.snapshot.position = fromPosition(impl_->control->getMotorPosition());
    Dex13MotorSpeed speed;
    if (impl_->control->getMotorActualSpeed(speed) == 1) {
      result.snapshot.speed_rpm = fromSpeed(speed);
    } else {
      result.message = "读取电机速度失败";
    }
    Dex13MotorCurrent current;
    if (impl_->control->getMotorActualCurrent(current) == 1) {
      result.snapshot.current_ma = fromCurrent(current);
    } else {
      if (!result.message.empty()) {
        result.message += "；";
      }
      result.message += "读取电机电流失败";
    }
    result.snapshot.motor_enabled = impl_->control->isMotorEnabled();
    result.snapshot.mode = modeFromSdk(impl_->control->getMotorControlMode());
    result.snapshot.fault = impl_->control->isFault();
    result.snapshot.connected = true;
    if (result.snapshot.fault) {
      const auto codes = impl_->control->getFaultCode();
      for (const auto & code : codes) {
        FaultItem item;
        item.finger_name = code.finger_name;
        item.motor_id = code.motor_ID;
        item.error_code = errorCodeName(code.error_code);
        result.snapshot.faults.push_back(std::move(item));
      }
    }
    result.ok = result.message.empty();
  } catch (const std::exception & ex) {
    result.ok = false;
    result.message = ex.what();
  } catch (...) {
    result.ok = false;
    result.message = "读取状态出现未知异常";
  }
  return result;
}

std::optional<TactileSample> Dexh13Sdk::tryReadTactile(std::string & error)
{
  error.clear();
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock() || impl_->upgrading || !impl_->connected) {
    return std::nullopt;
  }
  try {
    const auto forces = impl_->control->getFingerTactile();
    const auto temperatures = impl_->control->getTemperature();
    return makeTactile(forces, temperatures);
  } catch (const std::exception & ex) {
    error = ex.what();
    return std::nullopt;
  } catch (...) {
    error = "读取触觉失败";
    return std::nullopt;
  }
}

std::optional<TactileDetail> Dexh13Sdk::tryReadTactileDetail(std::string & error)
{
  error.clear();
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock() || impl_->upgrading || !impl_->connected) {
    return std::nullopt;
  }
  try {
    return makeDetail(impl_->control->getFingerTactileDetail());
  } catch (const std::exception & ex) {
    error = ex.what();
    return std::nullopt;
  } catch (...) {
    error = "读取分布力失败";
    return std::nullopt;
  }
}

std::optional<PressSample> Dexh13Sdk::tryReadPress(std::string & error)
{
  error.clear();
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock() || impl_->upgrading || !impl_->connected) {
    return std::nullopt;
  }
  try {
    const auto points = impl_->control->getPressPositionForce();
    PressSample sample;
    sample.model_name.reserve(points.size());
    sample.x.reserve(points.size());
    sample.y.reserve(points.size());
    sample.z.reserve(points.size());
    sample.force.reserve(points.size());
    for (const PressForcePoint & point : points) {
      sample.model_name.push_back(point.model_name);
      sample.x.push_back(point.x);
      sample.y.push_back(point.y);
      sample.z.push_back(point.z);
      sample.force.push_back(point.force_value);
    }
    return sample;
  } catch (const std::exception & ex) {
    error = ex.what();
    return std::nullopt;
  } catch (...) {
    error = "读取按压力失败";
    return std::nullopt;
  }
}

ApiResult Dexh13Sdk::readAllTactile(AllTactile & sample)
{
  return invoke(*impl_, true, [&sample](Dexh13SdkImpl & impl) {
    std::vector<ForcePoint> forces;
    std::vector<ForcePoint> detail;
    std::vector<FingerTemp> temperatures;
    impl.control->getAllTactileData(forces, detail, temperatures);
    sample.resultant = makeTactile(forces, temperatures);
    sample.detail = makeDetail(detail);
    return okResult("已读取全部触觉");
  });
}

ApiResult Dexh13Sdk::forwardKinematics(const MotorValues & motors, std::vector<double> & radians)
{
  return invoke(*impl_, false, [&motors, &radians](Dexh13SdkImpl & impl) {
    const Side side = impl.resolvedSide();
    if (side == Side::kNone) {
      return failResult("还不能确定左右手，无法做正运动学");
    }
    std::vector<int16_t> motor_pos(motors.values.begin(), motors.values.end());
    std::vector<double> parsed;
    const int rc = impl.kinematic->forwardKinematicParse(sideToSdk(side), motor_pos, parsed);
    if (rc != 1) {
      return failResult("正运动学失败");
    }
    radians = std::move(parsed);
    return okResult("正运动学完成");
  });
}

ApiResult Dexh13Sdk::inverseKinematics(const std::vector<double> & radians, MotorValues & motors)
{
  return invoke(*impl_, false, [&radians, &motors](Dexh13SdkImpl & impl) {
    if (radians.size() != static_cast<size_t>(kJointCount)) {
      return failResult("关节弧度数量必须是 16");
    }
    const Side side = impl.resolvedSide();
    if (side == Side::kNone) {
      return failResult("还不能确定左右手，无法做逆运动学");
    }
    std::vector<int16_t> parsed;
    const int rc = impl.kinematic->inverseKinematicParse(sideToSdk(side), radians, parsed);
    if (rc != 1 || parsed.size() != static_cast<size_t>(kMotorCount)) {
      return failResult("逆运动学失败");
    }
    std::copy(parsed.begin(), parsed.end(), motors.values.begin());
    return okResult("逆运动学完成");
  });
}

ApiResult Dexh13Sdk::setForceParams(
  const std::vector<double> & impedance, const std::vector<double> & reference_force,
  const std::vector<bool> & enable)
{
  return invoke(*impl_, false, [&](Dexh13SdkImpl & impl) {
    if (impedance.size() != static_cast<size_t>(kForceDof) ||
      reference_force.size() != static_cast<size_t>(kForceDof) ||
      enable.size() != static_cast<size_t>(kForceDof))
    {
      return failResult("力控参数长度必须是 8");
    }
    for (double value : impedance) {
      if (value < 1.0 || value > 100.0) {
        return failResult("阻抗因子超出 [1, 100]");
      }
    }
    for (double value : reference_force) {
      if (value < 1.0 || value > 10.0) {
        return failResult("参考力系数超出 [1, 10]");
      }
    }
    const Side side = impl.resolvedSide();
    if (side == Side::kNone) {
      return failResult("还不能确定左右手，无法设置力控参数");
    }
    paxini::bot::dexh13::ForceCtlParam param;
    param.impedance_factor = impedance;
    param.reference_force_factor = reference_force;
    param.enable_flag = enable;
    const int rc = impl.force->setForceCtlParam(sideToSdk(side), param);
    impl.force_params_ready = rc == 1;
    return fromInt(rc, "设置力控参数");
  });
}

ApiResult Dexh13Sdk::computeForceCommand(
  const std::vector<double> & current_radians, std::vector<double> & command_radians)
{
  return invoke(*impl_, true, [&](Dexh13SdkImpl & impl) {
    if (!impl.force_params_ready) {
      return failResult("还没有设置力控参数");
    }
    const Side side = impl.resolvedSide();
    if (side == Side::kNone) {
      return failResult("还不能确定左右手");
    }
    std::vector<double> current = current_radians;
    if (current.empty()) {
      impl.control->getJointPositionsRadian(current);
    }
    if (current.size() != static_cast<size_t>(kJointCount)) {
      return failResult("关节弧度数量必须是 16");
    }
    const auto forces = impl.control->getFingerTactile();
    std::vector<double> command;
    const int rc = impl.force->computeSerialTargetCommand(sideToSdk(side), current, forces, command);
    if (rc != 1 || command.size() != static_cast<size_t>(kJointCount)) {
      return failResult("计算力控指令失败");
    }
    command_radians = std::move(command);
    return okResult("已计算，未下发");
  });
}

std::optional<ForceStep> Dexh13Sdk::tryForceStep()
{
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock() || impl_->upgrading) {
    return std::nullopt;
  }
  ForceStep step;
  if (!impl_->connected) {
    step.message = "灵巧手未连接";
    return step;
  }
  if (!impl_->force_params_ready) {
    step.message = "还没有设置力控参数";
    return step;
  }
  const Side side = impl_->resolvedSide();
  if (side == Side::kNone) {
    step.message = "还不能确定左右手";
    return step;
  }
  try {
    impl_->control->getJointPositionsRadian(step.joint_radians);
    const auto forces = impl_->control->getFingerTactile();
    const auto temperatures = impl_->control->getTemperature();
    step.tactile = makeTactile(forces, temperatures);
    if (step.joint_radians.size() != static_cast<size_t>(kJointCount)) {
      step.message = "当前关节弧度数量不是 16";
      return step;
    }
    const int computed = impl_->force->computeSerialTargetCommand(
      sideToSdk(side), step.joint_radians, forces, step.command_radians);
    if (computed != 1 || step.command_radians.size() != static_cast<size_t>(kJointCount)) {
      step.message = "计算力控指令失败";
      return step;
    }
    if (impl_->control->setJointPositionsRadian(step.command_radians) != 1) {
      step.message = "下发力控弧度失败";
      return step;
    }
    step.ok = true;
  } catch (const std::exception & ex) {
    step.message = ex.what();
  } catch (...) {
    step.message = "力控循环出现未知异常";
  }
  return step;
}

ApiResult Dexh13Sdk::scanEthercat(
  const std::string & interface_name, std::vector<uint16_t> & addresses)
{
  return invoke(*impl_, false, [&](Dexh13SdkImpl & impl) {
    if (interface_name.empty()) {
      return failResult("网卡名为空");
    }
    addresses = impl.control->scanDevice(
      paxini::bot::dexh13::CommunicationProtocol::PROTOCOL_ETHERCAT, interface_name);
    return okResult("扫描完成，地址数量 " + std::to_string(addresses.size()));
  });
}

bool Dexh13Sdk::undistortAvailable() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->undistort_ready;
}

ApiResult Dexh13Sdk::setCameraConfig(int width, int height, float fps)
{
  return invoke(*impl_, true, [&](Dexh13SdkImpl & impl) {
    return fromBool(impl.control->setCameraConfig(width, height, fps), "设置相机参数");
  });
}

ApiResult Dexh13Sdk::readCameraModel(int width, int height, CameraModel & model)
{
  return invoke(*impl_, true, [&](Dexh13SdkImpl & impl) {
    const auto intrinsic = impl.control->getIntrinsicMatrixAndDistCoeffs(width, height);
    model.width = width;
    model.height = height;
    model.matrix = copyMatrix(intrinsic.first);
    model.dist_coeffs = copyMatrix(intrinsic.second);
    model.hand_eye = copyMatrix(impl.control->getHandEyeMatrix());
    if (model.matrix.size() != 9 || model.dist_coeffs.empty()) {
      return failResult("相机内参为空或不是 3x3");
    }
    return okResult("已读取相机标定");
  });
}

std::optional<CameraFrames> Dexh13Sdk::tryGrab(
  bool undistort, const std::vector<double> & matrix, const std::vector<double> & dist_coeffs)
{
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock() || impl_->upgrading || !impl_->connected) {
    return std::nullopt;
  }
  CameraFrames frames;
  try {
    const cv::Mat raw = impl_->control->getFrame();
    frames.raw = copyImage(raw);
    if (frames.raw.empty()) {
      frames.message = "相机没有返回图像";
      return frames;
    }
    if (!undistort) {
      return frames;
    }
    if (!impl_->undistort_ready) {
      frames.message = "缺少 OpenCV 4.5 calib3d，不能去畸变";
      return frames;
    }
    if (matrix.size() != 9 || dist_coeffs.empty()) {
      frames.message = "还没有相机内参";
      return frames;
    }
    const cv::Mat camera_matrix = matrixFromDoubles(matrix, 3, 3);
    const cv::Mat distortion = matrixFromDoubles(dist_coeffs, 1, static_cast<int>(dist_coeffs.size()));
    frames.rect = copyImage(impl_->control->undistortImage(raw, camera_matrix, distortion));
    frames.rect_valid = !frames.rect.empty();
    if (!frames.rect_valid) {
      frames.message = "去畸变结果为空";
    }
  } catch (const std::exception & ex) {
    frames.message = ex.what();
  } catch (...) {
    frames.message = "读取图像失败";
  }
  return frames;
}

ApiResult Dexh13Sdk::saveImage(
  const std::string & path, bool undistort, const std::vector<double> & matrix,
  const std::vector<double> & dist_coeffs)
{
  return invoke(*impl_, true, [&](Dexh13SdkImpl & impl) {
    if (path.empty()) {
      return failResult("图片路径为空");
    }
    cv::Mat frame = impl.control->getFrame();
    if (frame.data == nullptr || frame.rows <= 0 || frame.cols <= 0) {
      return failResult("相机没有返回图像");
    }
    if (undistort) {
      if (!impl.undistort_ready) {
        return failResult("缺少 OpenCV 4.5 calib3d，不能去畸变");
      }
      if (matrix.size() != 9 || dist_coeffs.empty()) {
        return failResult("还没有相机内参");
      }
      const cv::Mat camera_matrix = matrixFromDoubles(matrix, 3, 3);
      const cv::Mat distortion = matrixFromDoubles(dist_coeffs, 1, static_cast<int>(dist_coeffs.size()));
      frame = impl.control->undistortImage(frame, camera_matrix, distortion);
    }
    impl.control->saveImage(path, frame);
    return okResult("已保存图像");
  });
}

UpgradeOutcome Dexh13Sdk::upgradeMainboard(const std::vector<std::string> & file_names)
{
  UpgradeOutcome outcome;
  if (file_names.empty()) {
    outcome.message = "固件文件列表为空";
    return outcome;
  }
  UpgradeSession session(*impl_);
  const ApiResult started = session.begin();
  if (!started.success) {
    outcome.message = started.message;
    return outcome;
  }
  try {
    const UpgradeErrorCode code = impl_->control->upgradeMainboard(file_names);
    outcome.error_code = static_cast<int>(code);
    outcome.error_name = upgradeErrorName(code);
    outcome.success = code == UpgradeErrorCode::UPGRADE_SUCCESS;
    outcome.message = outcome.success ? "主控升级完成" : "主控升级失败";
  } catch (const std::exception & ex) {
    outcome.error_code = -1;
    outcome.error_name = "EXCEPTION";
    outcome.message = ex.what();
  } catch (...) {
    outcome.error_code = -1;
    outcome.error_name = "EXCEPTION";
    outcome.message = "主控升级出现未知异常";
  }
  return outcome;
}

UpgradeOutcome Dexh13Sdk::upgradeDriveboard(const std::vector<UpgradeFileSpec> & files)
{
  UpgradeOutcome outcome;
  if (files.empty()) {
    outcome.message = "电驱固件列表为空";
    return outcome;
  }
  std::vector<UpgradeFileAndPart> specs;
  specs.reserve(files.size());
  for (const UpgradeFileSpec & file : files) {
    if (file.file_path.empty() || file.parts.empty()) {
      outcome.message = "每个电驱固件都需要路径和部位号";
      return outcome;
    }
    UpgradeFileAndPart spec;
    spec.file_path = file.file_path;
    spec.upgrade_part.insert(file.parts.begin(), file.parts.end());
    specs.push_back(std::move(spec));
  }
  UpgradeSession session(*impl_);
  const ApiResult started = session.begin();
  if (!started.success) {
    outcome.message = started.message;
    return outcome;
  }
  try {
    const UpgradeErrorCode code = impl_->control->upgradeDriveboard(specs);
    outcome.error_code = static_cast<int>(code);
    outcome.error_name = upgradeErrorName(code);
    outcome.success = code == UpgradeErrorCode::UPGRADE_SUCCESS;
    outcome.message = outcome.success ? "电驱升级完成" : "电驱升级失败";
  } catch (const std::exception & ex) {
    outcome.error_code = -1;
    outcome.error_name = "EXCEPTION";
    outcome.message = ex.what();
  } catch (...) {
    outcome.error_code = -1;
    outcome.error_name = "EXCEPTION";
    outcome.message = "电驱升级出现未知异常";
  }
  return outcome;
}

std::vector<UpgradeSample> Dexh13Sdk::readUpgradeInfo()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::vector<UpgradeSample> samples;
  try {
    const auto info = impl_->control->getUpgradeInfo();
    samples.reserve(info.size());
    for (const auto & item : info) {
      UpgradeSample sample;
      sample.type = static_cast<uint8_t>(item.upgrade_type);
      sample.part = item.part;
      sample.percent = item.process_data;
      sample.status = item.status;
      samples.push_back(std::move(sample));
    }
  } catch (...) {
  }
  return samples;
}

}  // namespace dexh13_hand
