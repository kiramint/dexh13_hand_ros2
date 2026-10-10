cmake_minimum_required(VERSION 3.20)
project(dexh13_hand)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(DexHandSDK REQUIRED)

# The installed DexHand library is built for Ubuntu 22.04 SONAMEs. Ubuntu 24.04
# does not provide them, so this package ships that closure in compat/ and
# uses it only on 24.04. Other releases link the installed SDK.
set(DEXH13_USE_BUNDLED_COMPAT OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND EXISTS "/etc/os-release")
  file(READ "/etc/os-release" DEXH13_OS_RELEASE)
  if(DEXH13_OS_RELEASE MATCHES "(^|\n)VERSION_ID=\"24\\.04\"")
    set(DEXH13_USE_BUNDLED_COMPAT ON)
  endif()
endif()
set(DEXH13_COMPAT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/compat")
if(DEXH13_USE_BUNDLED_COMPAT AND NOT EXISTS "${DEXH13_COMPAT_DIR}/libdexhand_control_cpp.so")
  message(FATAL_ERROR "DexH13 compat library is missing: ${DEXH13_COMPAT_DIR}/libdexhand_control_cpp.so")
endif()
if(DEXH13_USE_BUNDLED_COMPAT)
  message(STATUS "dexh13_hand: Ubuntu 24.04, linking bundled compat libraries")
else()
  message(STATUS "dexh13_hand: linking the installed DexHand SDK")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/HandStatus.msg"
  "msg/Tactile.msg"
  "msg/TactileDetail.msg"
  "msg/PressForce.msg"
  "msg/Fault.msg"
  "msg/UpgradeProgress.msg"
  "msg/UpgradeFile.msg"
  "srv/SetControlMode.srv"
  "srv/SetMotorLimits.srv"
  "srv/GetFault.srv"
  "srv/GetVersion.srv"
  "srv/ForwardKinematics.srv"
  "srv/InverseKinematics.srv"
  "srv/SetJointAngles.srv"
  "srv/GetJointAngles.srv"
  "srv/SetForceParams.srv"
  "srv/ComputeForceCommand.srv"
  "srv/ScanEthercat.srv"
  "srv/ConnectEthercat.srv"
  "srv/SetCameraConfig.srv"
  "srv/GetCameraCalibration.srv"
  "srv/SaveImage.srv"
  "srv/UpgradeMainboard.srv"
  "srv/UpgradeDriveboard.srv"
  DEPENDENCIES std_msgs
)

add_executable(hand_node
  src/main.cpp
  src/dexh13_node.cpp
  src/dexh13_sdk.cpp
)
target_include_directories(hand_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(hand_node SYSTEM PRIVATE
  ${DEXHANDSDK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hand_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_srvs::std_srvs
  "${cpp_typesupport_target}"
)
if(DEXH13_USE_BUNDLED_COMPAT)
  # RUNPATH only resolves this binary's direct NEEDED entries. calib3d's
  # own dependencies have no RUNPATH, so they must be linked here too.
  # imshow stays unresolved. The node never calls showImage.
  target_link_libraries(hand_node
    -Wl,--no-as-needed
    "${DEXH13_COMPAT_DIR}/libdexhand_control_cpp.so"
    "${DEXH13_COMPAT_DIR}/libboost_filesystem.so.1.74.0"
    "${DEXH13_COMPAT_DIR}/libspdlog.so.1"
    "${DEXH13_COMPAT_DIR}/libglog.so.0"
    "${DEXH13_COMPAT_DIR}/libfmt.so.8"
    "${DEXH13_COMPAT_DIR}/libyaml-cpp.so.0.7"
    "${DEXH13_COMPAT_DIR}/libgsl.so.25"
    "${DEXH13_COMPAT_DIR}/libopencv_core.so.4.5d"
    "${DEXH13_COMPAT_DIR}/libopencv_imgproc.so.4.5d"
    "${DEXH13_COMPAT_DIR}/libopencv_flann.so.4.5d"
    "${DEXH13_COMPAT_DIR}/libopencv_features2d.so.4.5d"
    "${DEXH13_COMPAT_DIR}/libopencv_calib3d.so.4.5d"
    -Wl,--as-needed
  )
  target_link_options(hand_node PRIVATE
    "LINKER:--allow-shlib-undefined"
    "LINKER:-rpath-link,${DEXH13_COMPAT_DIR}"
  )
  set_target_properties(hand_node PROPERTIES
    BUILD_RPATH "${DEXH13_COMPAT_DIR}"
    INSTALL_RPATH "\$ORIGIN/compat"
    BUILD_WITH_INSTALL_RPATH OFF
  )
else()
  if(NOT EXISTS "${DEXHANDSDK_LIBRARY_DIRS}/libdexhand_control_cpp.so")
    message(FATAL_ERROR
      "Installed DexHand library is missing: ${DEXHANDSDK_LIBRARY_DIRS}/libdexhand_control_cpp.so")
  endif()
  target_link_libraries(hand_node
    "${DEXHANDSDK_LIBRARY_DIRS}/libdexhand_control_cpp.so"
    ${OpenCV_LIBS}
  )
endif()

install(TARGETS hand_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
if(DEXH13_USE_BUNDLED_COMPAT)
  install(DIRECTORY compat/
    DESTINATION lib/${PROJECT_NAME}/compat
    USE_SOURCE_PERMISSIONS
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
