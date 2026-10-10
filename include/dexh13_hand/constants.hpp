#pragma once

#include <array>
#include <cstddef>

namespace dexh13_hand
{

constexpr int kJointCount = 16;
constexpr int kMotorCount = 13;
constexpr int kForceDof = 8;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

constexpr int kCurrentCommandMin = -400;
constexpr int kCurrentCommandMax = 400;
constexpr int kSpeedCommandMin = -24000;
constexpr int kSpeedCommandMax = 24000;
constexpr int kMaxCurrentMin = 0;
constexpr int kMaxCurrentMax = 500;
constexpr int kMaxSpeedMin = 0;
constexpr int kMaxSpeedMax = 24000;

constexpr std::array<const char *, kJointCount> kJointSuffix{{
  "index_joint1", "index_joint2", "index_joint3", "index_joint4",
  "middle_joint1", "middle_joint2", "middle_joint3", "middle_joint4",
  "ring_joint1", "ring_joint2", "ring_joint3", "ring_joint4",
  "thumb_joint1", "thumb_joint2", "thumb_joint3", "thumb_joint4",
}};

inline bool abductionJoint(int index)
{
  return index % 4 == 0;
}

inline double jointLowerDeg(int index)
{
  return abductionJoint(index) ? -20.0 : 0.0;
}

inline double jointUpperDeg(int index)
{
  return abductionJoint(index) ? 20.0 : 90.0;
}

inline double jointLowerRad(int index)
{
  return jointLowerDeg(index) * kDegToRad;
}

inline double jointUpperRad(int index)
{
  return jointUpperDeg(index) * kDegToRad;
}

}  // namespace dexh13_hand
