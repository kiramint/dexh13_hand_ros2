#include <dexh13/dexh13_control.h>
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "dexh13_hand/dexh13_node.hpp"
#include "dexh13_hand/dexh13_sdk.hpp"

namespace
{
using paxini::bot::dexh13::ControlMode;
using dexh13_hand::Mode;
using dexh13_hand::Side;

struct FakeDevice
{
  bool connected{false};
  bool enabled{false};
  bool mode_success{true};
  bool disable_success{true};
  ControlMode hardware_mode{ControlMode::POSITION_CONTROL_MODE};
  ControlMode user_mode{ControlMode::NONE};
  int mode_calls{0};
  int enable_calls{0};
  std::vector<std::string> events;
};

FakeDevice device;
}  // namespace

// Interpose the vendor's public I/O methods in this test executable. The real
// wrapper and node run unchanged; none of these methods opens a physical port.
namespace paxini::bot::dexh13
{
int DexH13Control::activeHandy(const std::string &, const std::string &)
{
  device.connected = true;
  device.user_mode = ControlMode::NONE;
  return static_cast<int>(HandType::LEFT_HAND);
}

int DexH13Control::disconnectHandy()
{
  device.connected = false;
  return 1;
}

bool DexH13Control::isConnectHandy() {return device.connected;}
bool DexH13Control::isMotorEnabled() {return device.enabled;}
ControlMode DexH13Control::getMotorControlMode() {return device.hardware_mode;}

bool DexH13Control::setMotorControlMode(ControlMode mode)
{
  ++device.mode_calls;
  device.events.push_back("mode");
  if (device.enabled || !device.mode_success) {
    return false;
  }
  device.hardware_mode = mode;
  device.user_mode = mode;
  return true;
}

bool DexH13Control::enableMotor()
{
  ++device.enable_calls;
  device.events.push_back("enable");
  device.enabled = true;
  return true;
}

bool DexH13Control::disableMotor()
{
  device.events.push_back("disable");
  if (device.disable_success) {
    device.enabled = false;
  }
  return device.disable_success;
}

int DexH13Control::setJointPositionsRadian(const std::vector<double> &)
{
  return device.enabled && device.user_mode == ControlMode::POSITION_CONTROL_MODE ? 1 : -1;
}

int DexH13Control::initMotorPosition()
{
  device.events.push_back("init");
  return device.user_mode == ControlMode::POSITION_CONTROL_MODE ? 1 : 0;
}

bool DexH13Control::setMotorMaxCurrent(int) {return true;}
bool DexH13Control::setMotorMaxSpeed(int) {return true;}
const std::string DexH13Control::getSDKVersion() {return "test";}
const std::string DexH13Control::getFirmwareVersion() {return "test";}
}  // namespace paxini::bot::dexh13

namespace
{
class SdkModeTest : public ::testing::Test
{
protected:
  void SetUp() override {device = FakeDevice{};}
};

TEST_F(SdkModeTest, InitializesUserModeEvenWhenHardwareAlreadyInPositionMode)
{
  dexh13_hand::Dexh13Sdk sdk;
  ASSERT_TRUE(sdk.connectSerial("fake", "none", Side::kLeft).success);
  EXPECT_EQ(sdk.context().mode, Mode::kNone);
  EXPECT_FALSE(sdk.enable().success);
  EXPECT_EQ(device.enable_calls, 0);
  ASSERT_TRUE(sdk.setMode(Mode::kPosition).success);
  EXPECT_EQ(device.mode_calls, 1);
  EXPECT_EQ(sdk.context().mode, Mode::kPosition);
  ASSERT_TRUE(sdk.enable().success);
  EXPECT_TRUE(sdk.setJointRadians(std::vector<double>(16, 0.0)).success);
}

TEST_F(SdkModeTest, RepeatedModeRequestWhileEnabledRemainsIdempotent)
{
  dexh13_hand::Dexh13Sdk sdk;
  ASSERT_TRUE(sdk.connectSerial("fake", "none", Side::kLeft).success);
  ASSERT_TRUE(sdk.setMode(Mode::kPosition).success);
  ASSERT_TRUE(sdk.enable().success);
  EXPECT_TRUE(sdk.setMode(Mode::kPosition).success);
  EXPECT_EQ(device.mode_calls, 1);
  EXPECT_FALSE(sdk.setMode(Mode::kSpeed).success);
  EXPECT_TRUE(device.enabled);
}

TEST_F(SdkModeTest, RequiresDisableToInitializeAnAlreadyEnabledDevice)
{
  dexh13_hand::Dexh13Sdk sdk;
  device.enabled = true;
  ASSERT_TRUE(sdk.connectSerial("fake", "none", Side::kLeft).success);
  EXPECT_FALSE(sdk.setMode(Mode::kPosition).success);
  EXPECT_FALSE(sdk.enable().success);
  ASSERT_TRUE(sdk.disable().success);
  EXPECT_TRUE(sdk.setMode(Mode::kPosition).success);
  EXPECT_EQ(device.mode_calls, 1);
}

TEST_F(SdkModeTest, FailedSetterDoesNotPermitEnableOrReportReady)
{
  dexh13_hand::Dexh13Sdk sdk;
  ASSERT_TRUE(sdk.connectSerial("fake", "none", Side::kLeft).success);
  device.mode_success = false;
  EXPECT_FALSE(sdk.setMode(Mode::kPosition).success);
  EXPECT_FALSE(sdk.enable().success);
  EXPECT_EQ(sdk.context().mode, Mode::kNone);
  EXPECT_EQ(device.enable_calls, 0);
  device.mode_success = true;
  EXPECT_TRUE(sdk.setMode(Mode::kPosition).success);
  EXPECT_EQ(device.mode_calls, 2);
}

TEST_F(SdkModeTest, ReconnectInitializesUserModeAgain)
{
  dexh13_hand::Dexh13Sdk sdk;
  ASSERT_TRUE(sdk.connectSerial("fake", "none", Side::kLeft).success);
  ASSERT_TRUE(sdk.setMode(Mode::kPosition).success);
  ASSERT_TRUE(sdk.disconnect().success);
  ASSERT_TRUE(sdk.connectSerial("fake", "none", Side::kLeft).success);
  EXPECT_EQ(sdk.context().mode, Mode::kNone);
  EXPECT_FALSE(sdk.enable().success);
  EXPECT_TRUE(sdk.setMode(Mode::kPosition).success);
  EXPECT_EQ(device.mode_calls, 2);
}

TEST_F(SdkModeTest, HardwareModeChangeRequiresExplicitSetupAgain)
{
  dexh13_hand::Dexh13Sdk sdk;
  ASSERT_TRUE(sdk.connectSerial("fake", "none", Side::kLeft).success);
  ASSERT_TRUE(sdk.setMode(Mode::kPosition).success);
  device.hardware_mode = ControlMode::SPEED_CONTROL_MODE;
  EXPECT_EQ(sdk.context().mode, Mode::kNone);
  EXPECT_FALSE(sdk.enable().success);
  ASSERT_TRUE(sdk.setMode(Mode::kSpeed).success);
  EXPECT_EQ(device.mode_calls, 2);
  EXPECT_EQ(sdk.context().mode, Mode::kSpeed);
}

TEST_F(SdkModeTest, InvalidModeIsRejected)
{
  dexh13_hand::Dexh13Sdk sdk;
  ASSERT_TRUE(sdk.connectSerial("fake", "none", Side::kLeft).success);
  EXPECT_FALSE(sdk.setMode(Mode::kNone).success);
  EXPECT_EQ(device.mode_calls, 0);
}

class NodeStartupTest : public SdkModeTest
{
protected:
  void SetUp() override
  {
    SdkModeTest::SetUp();
    const char * args[] = {
      "test", "--ros-args", "-p", "hand_port:=fake", "-p",
      initOnStart() ? "init_on_start:=true" : "init_on_start:=false"};
    rclcpp::init(6, args);
  }

  void TearDown() override {rclcpp::shutdown();}
  virtual bool initOnStart() const {return false;}
};

TEST_F(NodeStartupTest, DisablesRetainedEnableBeforeInitializingMode)
{
  device.enabled = true;
  auto node = dexh13_hand::createNode();
  EXPECT_EQ(device.events, (std::vector<std::string>{"disable", "mode", "enable"}));
  EXPECT_EQ(device.user_mode, ControlMode::POSITION_CONTROL_MODE);
  EXPECT_TRUE(device.enabled);
}

TEST_F(NodeStartupTest, ModeFailureStopsBeforeEnable)
{
  device.mode_success = false;
  auto node = dexh13_hand::createNode();
  EXPECT_EQ(device.mode_calls, 1);
  EXPECT_EQ(device.enable_calls, 0);
  EXPECT_FALSE(device.enabled);
}

class NodeInitStartupTest : public NodeStartupTest
{
protected:
  bool initOnStart() const override {return true;}
};

TEST_F(NodeInitStartupTest, SelectsSdkModeBeforeExplicitInitialization)
{
  auto node = dexh13_hand::createNode();
  EXPECT_EQ(device.events, (std::vector<std::string>{"mode", "init", "enable"}));
  EXPECT_TRUE(device.enabled);
}

TEST_F(NodeInitStartupTest, ModeFailureStopsBeforeExplicitInitialization)
{
  device.mode_success = false;
  auto node = dexh13_hand::createNode();
  EXPECT_EQ(device.events, (std::vector<std::string>{"mode"}));
  EXPECT_FALSE(device.enabled);
}

TEST_F(NodeStartupTest, DisableFailureStopsBeforeModeAndEnable)
{
  device.enabled = true;
  device.disable_success = false;
  auto node = dexh13_hand::createNode();
  EXPECT_EQ(device.mode_calls, 0);
  EXPECT_EQ(device.enable_calls, 0);
}
}  // namespace
