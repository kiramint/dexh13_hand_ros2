# AGENTS.md — dexh13_hand

本包只驱动一只帕西尼 DexH13。不要改 `qrby_hand`，不要在其他包里加两种手的切换。

- 一个进程一只手。左右手用同一个可执行文件起两个进程，靠 `side` 和 `hand_port` 区分。
- 设备回报的手型必须和 `side` 一致，`auto` 除外。正运动学、逆运动学和力控都使用这个手型。
- 节点不直接调用 `paxini::bot::dexh13`。SDK 调用放在 `Dexh13Sdk`，同一把锁串行。固件升级期间例外：升级函数不占着锁，进度线程读 `getUpgradeInfo`。
- 不要在启动时调用 `initMotorPosition`。默认 `init_on_start` 为 false。这个调用会让手回到初始位，最多约 5 秒。
- 不要调用 `showImage`。
- 关节指令和 `joint_states` 用弧度。16 个关节顺序是食指、中指、无名指、拇指，每指 `joint1` 到 `joint4`。`joint1` 是侧摆。主动自由度是 13。食指、中指、无名指的 `joint4` 是被动关节，没有独立电机；`joint_states` 仍返回这三个关节的当前角度。拇指四个关节都是主动关节。
- 触觉分布力默认不读。`tactile_detail_hz` 为 0。
- Ubuntu 24.04 使用包内 `compat/`。官方 `.so` 依赖 Ubuntu 22.04 的 SONAME，安装后的 RPATH 是 `$ORIGIN/compat`。OpenCV 4.5 的 core、imgproc、flann、features2d、calib3d 从该目录直接链接：可执行文件的 RUNPATH 不解析这些库的传递依赖。其他系统版本链接已安装的 DexHand SDK，不使用这份 compat。
- 目标发行版是 ROS 2 Jazzy。改完至少运行 `colcon build --packages-select dexh13_hand` 和 `git diff --check`。
