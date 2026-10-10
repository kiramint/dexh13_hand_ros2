# dexh13_hand

ROS 2 Jazzy 下的第三方帕西尼 DexH13 C++ 驱动。一个进程管一只手。左手和右手使用同一个可执行文件，各自指定 `side` 和 `hand_port`。

> [!TIP]
>
> * 系统支持：Ubuntu 22.04（amd64，官方SDK支持），Ubuntu 24.04（amd64，使用了一些trick）

## 构建

先安装本仓库自带的 SDK：`docs/DexHandSDK-2.1.1-Linux-nonlopt.deb`。Ubuntu 22.04 使用 ROS 2 Humble，Ubuntu 24.04 使用 ROS 2 Jazzy。

```bash
sudo apt install ./docs/DexHandSDK-2.1.1-Linux-nonlopt.deb
source /opt/ros/jazzy/setup.zsh   # Ubuntu 22.04 改为 humble
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.zsh
```

官方 `libdexhand_control_cpp.so` 按 Ubuntu 22.04 的 SONAME 链接。Ubuntu 24.04 上，节点使用包内 `compat/` 里的这一套库：去掉 `highgui` 依赖的 DexHand 库，以及 jammy 的 boost、spdlog、glog、fmt、yaml-cpp、gsl、OpenCV 4.5 和它带上的视频库。安装后的 `RPATH` 是 `$ORIGIN/compat`，也就是 `lib/dexh13_hand/compat`。`calib3d` 的 `imgproc`、`flann`、`features2d` 没有自己的 `RUNPATH`，CMake 用 `--no-as-needed` 把它们和 `core`、`calib3d` 一起直接链接。

这个兼容目录只在 `/etc/os-release` 的 `VERSION_ID` 为 `24.04` 时参与链接和安装。其他版本链接已安装的 DexHand SDK 和系统 OpenCV。

24.04 上节点链接 `compat/` 里的 OpenCV 4.5，不链接系统 OpenCV 4.6，也不使用 `cv_bridge`。节点不调用 `showImage`。

GitHub Actions 会在 Ubuntu 22.04 和 24.04 上各编译一次，并用 `connect_on_start:=false` 启动节点，确认进程能加载且不打开串口。22.04 的任务使用 Humble，并检查可执行文件没有链到包内 `compat/`。24.04 的任务使用 Jazzy，并检查 `RPATH` 为 `$ORIGIN/compat`。22.04 的托管镜像预装了 LLVM 的 `libunwind`，和 deb 依赖的 `libgoogle-glog-dev` 冲突，所以 22.04 的 Action 只解包 SDK，再安装运行所需的库；Jammy 没有 `libgsl.so.25`，这一份从 `compat/` 拷到 `/usr/local/lib`。source ROS 环境时要关掉 `nounset`，`setup.bash` 会读取尚未赋值的 `AMENT_*` 变量。

## 启动

`hand_port` 默认为空。下面的端口只是示例，启动前确认它确实是要驱动的那只 DexH13。

```bash
ros2 launch dexh13_hand hand.launch.py side:=left name:=dexh13_left hand_port:=/dev/ttyUSB2
```

以后右手单独起一个进程：

```bash
ros2 launch dexh13_hand hand.launch.py side:=right name:=dexh13_right hand_port:=/dev/ttyUSB3
```

`side` 取 `left`、`right` 或 `auto`。取 `left` 或 `right` 时，设备回报的手型必须一致，否则节点断开连接并失败。`auto` 接受设备回报的手型。`prefix` 为空时由 `side` 生成，左手是 `left_`，`auto` 保持空前缀。

`connect_on_start` 为 true 且 `hand_port` 为空时，节点保持运行并记录错误，不打开串口。`side` 或 `control_mode` 无法识别时，节点在启动阶段直接退出。

默认传输是 RS485。EtherCAT 使用 `scan_ethercat` 和 `connect_ethercat`。

## 关节与电机

DexH13 这个名字里的 13 是自由度：13 个电机驱动 13 个主动关节。接口仍然收发 16 个串联关节角，顺序是食指、中指、无名指、拇指，每指 `joint1` 到 `joint4`。`joint1` 是侧摆，范围 [-20°, +20°]；`joint2`、`joint3`、`joint4` 是屈曲，范围 [0°, 90°]。

食指、中指、无名指的 `joint4` 是被动关节，没有独立电机。拇指的四个关节都是主动关节。位置指令仍要带满 16 个数，下面示例里三个被动关节写成 `1`：

```bash
ros2 topic pub --once /dexh13_left/joint_position_cmd sensor_msgs/msg/JointState \
  "{position: [0,0,0,1, 0,0,0,1, 0,0,0,1, 0,0,0,0]}"
```

`joint_states.position` 同样是 16 个数。被动关节也会返回当前角度，读到的是该关节此刻的实际角度。

话题里的关节量用弧度。`set_joint_angles_deg` 和 `get_joint_angles_deg` 用度。`joint_states` 的 `velocity` 不填写电机转速，转速在 `motor_state.speed_rpm`。

关节名是前缀加上下列后缀：

`index_joint1` … `index_joint4`，`middle_joint1` … `middle_joint4`，`ring_joint1` … `ring_joint4`，`thumb_joint1` … `thumb_joint4`。

位置指令可以写完整名，也可以只写后缀。`name` 为空且 `position` 长度为 16 时按上述顺序解释。电机数组固定 13 路，顺序是 SDK 的 `motor1` 到 `motor13`。

`clamp_commands` 为 true 时，关节指令限制在上面的角度范围内，电流指令限制在 [-400, 400] mA，速度指令限制在 [-24000, 24000] rpm，电流上限限制在 [0, 500] mA，速度上限限制在 [0, 24000] rpm。

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `side` | `left` | `left`、`right`、`auto` |
| `hand_port` | 空 | RS485 串口。空字符串不连接 |
| `camera_port` | `none` | 掌心相机的 V4L2 路径，例如 `/dev/video0`。`none` 时不打开相机、不创建图像话题 |
| `prefix` | 空 | 空则按 `side` 生成 |
| `frame_id` | 空 | 空则是 `dexh13_<side>_palm`；`auto` 时是 `dexh13_hand_palm` |
| `connect_on_start` | true | 启动时按 `hand_port` 连接 |
| `enable_on_start` | true | 连接后使能电机 |
| `init_on_start` | false | 为 true 时调用 `initMotorPosition`，手会运动，大约 5 秒 |
| `control_mode` | `position` | `position`、`speed`、`torque` |
| `max_current_ma` | 500 | 电机电流上限，范围 [0, 500] |
| `max_speed_rpm` | 24000 | 电机速度上限，范围 [0, 24000] |
| `state_hz` | 20 | `joint_states` 与 `motor_state` |
| `tactile_hz` | 10 | 11 路合力与温度 |
| `tactile_detail_hz` | 0 | 分布力。0 表示不周期读取 |
| `press_force_hz` | 0 | 按压力。0 表示不周期读取 |
| `camera_hz` | 15 | 相机周期。仅在启动时相机已启用时生效 |
| `camera_width` / `camera_height` / `camera_fps` | 640 / 480 / 30 | 连接后写给相机 |
| `publish_undistorted` | false | 为 true 且相机启用时发布 `image_rect` |
| `clamp_commands` | true | 按上一节限制指令 |
| `force_control_hz` | 30 | 力控环频率。启用前必须大于 0 |

话题、服务都在节点私有命名空间下，例如 `/dexh13_left/joint_states`。

## 订阅

指令 QoS 为 `KeepLast(1)`。模式不匹配、未连接、未使能或正在升级时，指令被丢弃，并大约每 2 秒警告一次。

| 话题 | 类型 | 条件 |
| --- | --- | --- |
| `~/joint_position_cmd` | `sensor_msgs/JointState` | 位置模式，16 个弧度。力控环运行时丢弃 |
| `~/motor_speed_cmd` | `dexh13_hand/MotorCommand` | 速度模式，单位 rpm。力控环运行时丢弃 |
| `~/motor_current_cmd` | `dexh13_hand/MotorCommand` | 力矩模式，单位 mA。力控环运行时仍接受 |
| `~/motor_position_cmd` | `dexh13_hand/MotorCommand` | 位置模式，电机原始刻度。力控环运行时丢弃 |

## 发布

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/joint_states` | `sensor_msgs/JointState` | 16 个关节弧度。食指、中指、无名指的 `joint4` 是被动关节，这里仍返回它们的当前角度 |
| `~/motor_state` | `dexh13_hand/MotorState` | 13 路位置、转速 rpm、电流 mA |
| `~/status` | `dexh13_hand/HandStatus` | `transient_local`。断线时 `hand_type` 为 `none` |
| `~/tactile` | `dexh13_hand/Tactile` | 11 个触觉模块合力。`x`、`y`、`z` 是三轴力，单位 0.1 N，另有温度。名称只在温度数组长度与合力一致时填写 |
| `~/tactile_detail` | `dexh13_hand/TactileDetail` | 分布力，完整读取 1140 点，三轴力，单位 0.1 N。`tactile_detail_hz` 为 0 时不周期发布 |
| `~/press_force` | `dexh13_hand/PressForce` | 接触点坐标和标量力。11 个模组各 6 个槽。见下文 |
| `~/fault` | `dexh13_hand/Fault` | `transient_local`。故障内容变化时发布 |
| `~/upgrade_progress` | `dexh13_hand/UpgradeProgress` | 仅升级过程中发布 |
| `~/image_raw` | `sensor_msgs/Image` | 启动时 `camera_port` 不是 `none` 才创建。这是原始图 |
| `~/camera_info` | `sensor_msgs/CameraInfo` | 与 `image_raw` 一起创建，`plumb_bob` |
| `~/image_rect` | `sensor_msgs/Image` | 还要 `publish_undistorted:=true` |

中指节模块（食指、中指、无名指的 middle）各 60 个触觉点，其余 8 个模块各 120 个点。周期触觉读取不调用 `getAllTactileData`；一次性完整读取使用服务 `read_all_tactile`，它会同时发布 `tactile` 和 `tactile_detail`。

## 服务

除特别注明外，应答都含 `success` 和 `message`。

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `~/connect` | `std_srvs/Trigger` | 用当前 `hand_port` 重连。先停止力控环 |
| `~/disconnect` | `std_srvs/Trigger` | 失能并断开。已识别的手型会保留给运动学 |
| `~/enable_motor` | `std_srvs/SetBool` | 使能或失能。失能会停止力控环 |
| `~/init_motor_position` | `std_srvs/Trigger` | 回到初始位。手会运动，大约 5 秒，并停止力控环 |
| `~/set_control_mode` | `dexh13_hand/SetControlMode` | `position`、`speed`、`torque`。离开位置模式会停止力控环 |
| `~/set_motor_limits` | `dexh13_hand/SetMotorLimits` | `apply_current` / `apply_speed` 分别决定是否写入 |
| `~/clear_fault` | `std_srvs/Trigger` | 清故障 |
| `~/get_fault` | `dexh13_hand/GetFault` | 读当前故障 |
| `~/get_version` | `dexh13_hand/GetVersion` | SDK 版本和固件版本 |
| `~/calibrate_tactile` | `std_srvs/Trigger` | 触觉标定。启动时不自动执行 |
| `~/read_all_tactile` | `std_srvs/Trigger` | 读全部触觉并发布合力与分布力 |
| `~/forward_kinematics` | `dexh13_hand/ForwardKinematics` | 13 路电机刻度到 16 个关节弧度。`side` 为 left/right 时未连接也可计算 |
| `~/inverse_kinematics` | `dexh13_hand/InverseKinematics` | 16 个关节弧度到电机刻度 |
| `~/set_joint_angles_deg` | `dexh13_hand/SetJointAngles` | 按度写位置。要求位置模式 |
| `~/get_joint_angles_deg` | `dexh13_hand/GetJointAngles` | 读当前角度，单位度 |
| `~/set_force_params` | `dexh13_hand/SetForceParams` | 8 路：食指、中指、无名指、拇指的近端再远端。阻抗 [1, 100]，参考力 [1, 10] |
| `~/compute_force_command` | `dexh13_hand/ComputeForceCommand` | 只计算目标弧度，不发送。`current_radians` 为空时从设备读取 |
| `~/enable_force_control` | `std_srvs/SetBool` | 启停主机侧力控环 |
| `~/scan_ethercat` | `dexh13_hand/ScanEthercat` | 扫描网卡上的从站地址 |
| `~/connect_ethercat` | `dexh13_hand/ConnectEthercat` | 按地址连接。`camera_port` 传给 SDK |
| `~/set_camera_config` | `dexh13_hand/SetCameraConfig` | 宽、高、帧率 |
| `~/get_camera_calibration` | `dexh13_hand/GetCameraCalibration` | 从手上读取内参、畸变和手眼矩阵。分辨率只接受 640×480、1280×720、1920×1080 |
| `~/save_image` | `dexh13_hand/SaveImage` | 保存一帧。`undistort` 为 true 时先矫正 |
| `~/upgrade_mainboard` | `dexh13_hand/UpgradeMainboard` | 主板升级。应答另有 `error_code`、`error_name` |
| `~/upgrade_driveboard` | `dexh13_hand/UpgradeDriveboard` | 驱动板升级。文件用 `UpgradeFile` |

`side` 为 `auto` 且从未连接成功时，正运动学、逆运动学和力控参数会被拒绝。连接成功后，断开连接仍保留已识别手型，这两类计算可以继续。`status.hand_type` 在未连接时是 `none`。

## 力控

力控在主机上运行，由 `DexH13ForceControl::computeSerialTargetCommand` 计算目标，再调用 `setJointPositionsRadian`。它不是设备控制模式。

启动条件是：已经 `set_force_params`、电机已使能、当前为位置模式、`force_control_hz` 大于 0。环运行时丢弃关节位置、电机速度和电机原始位置指令，电流指令仍可写入。停止力控环不改变使能状态和控制模式。重连、回初始位、失能，或切出位置模式，都会停止力控环。

## 触觉和按压力

`~/tactile` 是 11 个触觉模块的合力，`x`、`y`、`z` 是三个方向的力，单位 0.1 N。`~/tactile_detail` 是同一批传感器的分布力，同样是三轴力。

`~/press_force` 来自 `getPressPositionForce`，模组名和触觉相同，记录的是接触点。每条里的 `x`、`y`、`z` 是模组上的坐标，设备上各占一个有符号字节；`force` 才是该点的力，单位 0.1 N。

SDK 固定返回 11 个模组、每个模组 6 个接触槽，共 66 条。力为 0 的槽也会留下。顺序是：

`thumb_bottom`、`thumb_tip`、`index_bottom`、`index_middle`、`index_tip`、`middle_bottom`、`middle_middle`、`middle_tip`、`ring_bottom`、`ring_middle`、`ring_tip`。

`thumb_bottom` 是这 66 条里的前 6 条。看到 5 个有力的点时，第 6 个槽仍在数组里，它的 `force` 为 0。

串口上这两帧是：寄存器 `0x4040`、长度 `0x78`（前 10 个模组，240 字节），寄存器 `0x40B7`、长度 `0x0C`（最后的 `ring_tip`，24 字节）。每个槽 4 字节，依次是 `x`、`y`、`z`、`force`。

## 相机

`camera_port` 是掌心相机的 V4L2 设备路径，例如 `/dev/video0`。可用 `ls /dev/video*` 查看。SDK 以 OpenCV `CAP_V4L2` 打开它。`none` 和空字符串表示不打开相机。路径打不开时，官方 `activeHandy` 抛出「打开摄像头失败」，这次连接失败。

```bash
ros2 launch dexh13_hand hand.launch.py side:=left name:=dexh13_left \
  hand_port:=/dev/ttyUSB2 camera_port:=/dev/video6
```

图像话题只在进程启动时创建。启动参数 `camera_port:=none` 时，之后调用 `set_camera_config`、`get_camera_calibration`、`save_image` 会失败，节点不会临时打开相机。EtherCAT 连接可以把相机端口传给 SDK，但 ROS 话题不会因此补建。要发布图像，启动时就写上 `camera_port`。

`image_raw` 是原始图。矫正结果在 `image_rect`，需要 `publish_undistorted:=true`。鱼眼矫正符号不可用时，原始图仍可发布，矫正请求返回错误。

内参、畸变和手眼矩阵存在手上。串口连接时，`getIntrinsicMatrixAndDistCoeffs` 和 `getHandEyeMatrix` 都读 Modbus 寄存器 `0x4400`，长度 `0x7A`（122 字节）。SDK 要求这块数据能解析成 61 个浮点数，分辨率只接受 `640x480`、`1280x720`、`1920x1080`。读标定不需要相机设备已经打开。EtherCAT 走另一条 `getCameraParam`。当前节点的 `~/get_camera_calibration` 在启动时 `camera_port` 为 `none` 时不会发出这帧读请求。

## 固件升级

两个升级服务都要求 `confirm:=true`，调用会阻塞到结束。进度由另一条线程在升级期间发布到 `~/upgrade_progress`。官方流程要求升级前先断电再上电。

主板升级需要同时给出 CM7 和 CM4 固件路径。驱动板用 `UpgradeFile`：`file_path` 加 `parts`。部件号：

| 部件 | id |
| --- | --- |
| 食指双电机 | `0x0201` |
| 食指单电机 | `0x0101` |
| 中指双电机 | `0x0202` |
| 中指单电机 | `0x0102` |
| 无名指双电机 | `0x0203` |
| 无名指单电机 | `0x0103` |
| 侧摆双电机 | `0x0204` |
| 拇指单电机 | `0x0104` |
| 拇指指尖 | `0x0105` |

升级进行时，运动指令和其他会占用设备的服务返回失败。进程退出会等升级调用返回，升级未完成时不断开设备。
