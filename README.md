# ESP32-D0WD / ROS 2 Humble 遥控底盘

原生 ESP-IDF、`app_main()` + FreeRTOS，全部应用代码保留在
[`main/remoteESP.c`](main/remoteESP.c)。无 Arduino、循线、OpenMV 或 `/odom`。

链路：`/cmd_vel` (`geometry_msgs/msg/Twist`) → WiFi UDP4 Agent:8888 →
`/esp32_car` → 左右轮目标 PPS → 原有 PCNT + PI → LEDC 电机输出。
订阅 QoS 为 **best effort、volatile、keep last 1**，兼容常见的 reliable
`ros2 topic pub` / `teleop_twist_keyboard` 发布者，避免持久化旧指令。

**当前构建已启用电机，PWM 上限为 360/1023。首次运行必须架空车轮。** 默认网络配置仍需
设置 WiFi 和 Agent 地址；方向测试模式无需轮距、轮径或每圈脉冲数，但 Twist 数值只作为方向
及强弱输入，不代表实际 m/s、rad/s。当前版本尚未完成实车验证。

## 1. SDK 与组件

本机 SDK 是 `/home/clbritheking/esp/esp-idf`，版本 **v5.2.7**。ESP32 交叉工具链和
IDF Python 环境已安装；新环境构建 micro-ROS 前需按下文安装组件的 Python 依赖。

官方 Humble `micro_ros_espidf_component` **22.0.0** 已以源码集成到 `components/`，
固定提交 `4ddd8c26e721662319ed8af981cb7cdc9ae05382`；组件声明 `idf >=5.2`。
版本、来源及构建限制见 [`components/UPSTREAM.md`](components/UPSTREAM.md)。
源码核对时发现组件 CMake 依赖了未定义的 `libmicroros_project` target，已移除该依赖，
并显式指定已由配置阶段同步构建的静态库路径；没有修改组件运行时代码。
其官方说明列出 ESP-IDF v5.2 支持：
[Humble 官方 README](https://github.com/micro-ROS/micro_ros_espidf_component/blob/4ddd8c26e721662319ed8af981cb7cdc9ae05382/README.md)。
应用使用与官方示例相同的原生 NVS/netif/event/WiFi STA 初始化流程，自己管理持续重连，
没有调用会耗尽重试次数并打印密码的示例网络辅助函数。

首次 CMake 配置会由组件下载并交叉构建 ROS 依赖，**连 `menuconfig` 都可能触发该过程**。
应用已经声明组件依赖，不需要另行创建占位头文件或安装 Arduino 库。
组件内部的 Humble 依赖由官方构建脚本拉取，其分支并非全部固定到提交。

## 2. 配置、编译、烧录（由你执行）

使用没有 source ROS 2 的干净 shell 构建固件；ROS 2 的命令另开终端执行。

```bash
source /home/clbritheking/esp/esp-idf/export.sh
cd /mnt/ROS2Workspace/esp32/remoteESP
python -m pip install catkin_pkg colcon-common-extensions lark 'empy<4'
idf.py set-target esp32
idf.py menuconfig
```

在 `micro-ROS Settings` 中选择：

- middleware：`micro-ROS over eProsima Micro XRCE-DDS`
- network：`WLAN interface`
- `WiFi Configuration`：你的 **2.4 GHz** SSID 和密码
- `micro-ROS Agent IP`：Linux Mint 在该局域网的 IPv4，不能用 `127.0.0.1` 或默认 `0.0.0.0`
- `micro-ROS Agent Port`：`8888`

在 `ESP32 remote chassis` 中配置电机使能、映射方式、限幅、符号等。
当前 `Enable real motor output` 已启用；没有新鲜的非零命令时不会主动输出 PWM。
真实密码仅放本机 `sdkconfig`；它已被 `.gitignore` 忽略，默认配置没有密码。
不要分享包含凭据的 `sdkconfig`、构建产物或整目录压缩包。

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

按实际串口改设备名。分区表为至少 2 MiB flash 的单 factory 应用，无 OTA。
若实际二进制超过分区大小，应按开发板真实 flash 容量调整 `partitions.csv` 和 flash 设置。
升级组件或更改它的 `colcon.meta` 后可执行 `idf.py clean-microros` 再构建。

## 3. 不依赖轮距的第一阶段

默认 `Direction test` 模式：

```text
forward = linear.x / 0.300 * 3000 PPS
turn    = angular.z / 2.000 * 2000 PPS
left    = forward - turn
right   = forward + turn
```

所有比例及限制均可在 menuconfig 调整；上面是方向测试比例，**不是测量得到的底盘参数**。
左右目标超过 `Maximum wheel PPS` 时等比例缩小，保留差速比例。

| 指令 | 左轮逻辑目标 | 右轮逻辑目标 | 预期方向 |
|---|---:|---:|---|
| `v > 0, w = 0` | + | + | 前进 |
| `v < 0, w = 0` | − | − | 后退 |
| `v = 0, w > 0` | − | + | 原地左转 |
| `v = 0, w < 0` | + | − | 原地右转 |
| `v = 0, w = 0` | 0 | 0 | PWM 立即归零（下次控制周期内） |
| `v != 0, w != 0` | `forward-turn` | `forward+turn` | 弧线/差速转弯 |

默认接受范围 `|linear.x| <= 0.3`、`|angular.z| <= 2.0`；越界、NaN、Infinity
会作废原指令并停车，其他 Twist 分量不参与运动，但也必须有限。

## 4. 后续切换真实单位

测量后填写 `Measured wheel diameter`、`Measured wheel center spacing`（mm）、
`Measured PCNT ... ticks per wheel revolution`，再选 `Physical differential drive`。
参数初始均为 **0（未知）**。任一缺失时不会驱动电机，仍可接收消息，目标 PPS 显示 0。
方向测试模式不要求填写这些参数。

```text
left_mps  = v - w * track_m / 2
right_mps = v + w * track_m / 2
wheel_PPS = wheel_mps * ticks_per_rev / (pi * wheel_diameter_m)
```

每圈脉冲数必须使用本工程 **A 相上升沿、B 相判方向** 的实际车轮一圈计数，不能套用
编码器四倍频参数。默认编码器符号左 `+1`、右 `−1`；电机符号独立，初始均为 `+1`。
符号配置只允许 `+1/-1`，0 无效并禁止电机输出。

PCNT 连续运行，通过两次读数差和真实 `dt` 计算 PPS；高低限 watch point 配合
`accum_count` 处理硬件计数溢出，应用还处理 32 位计数回绕。保留左右 64 位带符号累计
脉冲、实际 dt、目标/渐变参考/实测 PPS，供未来 `/odom` 使用。当前不发布里程计。

## 5. Agent、命令与键盘遥控

Linux Mint ROS 终端：

```bash
source /opt/ros/humble/setup.bash
# 如 Agent 在自建工作区，还需 source 对应 install/setup.bash
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

ESP32 使用默认 domain 0。Mint 与 ESP32 必须局域网互通，允许 UDP 8888，关闭 AP 客户端隔离。
日志应依次出现 `WiFi connected`、`Agent reachable`、节点 ready。
在另一个 ROS 终端检查/发送（下列指令可在电机禁用时使用）：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 node list
ros2 topic info /cmd_vel --verbose
# 前进，持续 10 Hz 刷新，Ctrl-C 后 400ms 左右触发超时停车
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}, angular: {z: 0.0}}'
# 后退 / 左转 / 右转：分别替换为 (-0.10, 0)、(0, +0.5)、(0, -0.5)
# 零指令：重复发布，便于确认连接且清除运动指令
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

键盘包未安装时先装，然后以低于固件输入上限的参数启动：

```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.50 -r cmd_vel:=/cmd_vel
```

本机已安装 2.4.1，源码与 [Humble 官方键盘节点](https://github.com/ros2/teleop_twist_keyboard/blob/humble/teleop_twist_keyboard.py)
均只在读到按键时发布，没有 `repeat_rate` / `key_timeout` 功能。需要持续按住方向键，
由系统键盘重复事件刷新；重复间隔和首次重复延迟应小于 400 ms（可在 Mint 键盘设置调整）。
否则短按只运动一个看门狗周期，长按开始时也可能先停一下。松手后由 ESP32 看门狗停车。
`i/,` 前后、`j/l` 原地转、`u/o` 弧线、`k` 停止；按键窗口要有焦点。
不要让其他发布者同时向 `/cmd_vel` 发指令，也不要把键盘增速调到固件范围以外。
这些上位机命令本次没有运行。

## 6. 首次架空检查与调参

1. 架空两轮、准备切断电机电源；先发布零速度，确认 PWM 为零，再观察目标 PPS 和超时日志。
2. 零速度时手动分别转动轮子：逻辑前进方向应让左右累计 ticks 都增加，倒转都减少。
   不符时调整对应 encoder sign；保持 PCNT 边沿模式不变。
3. 使用当前已启用电机的方向测试模式与 360 PWM 上限。重启本身不应转动；
   只有收到新有效非零命令后允许输出。先发低幅前进、零、后退、零、左转、零、右转。
4. 对照车轮机械方向、逻辑 PPS 与 PWM。驱动中的明显反号反馈会锁定故障，需修正后复位。
   断电/架空状态下核对接线及两个独立符号，不要靠增加 PI 来掩盖反馈反号。
5. 默认低速死区/前馈均为 0，PWM cap=360；启动死区仍需实测，**可能只嗡鸣或不转**。
   需分别测定两轮、正反转启动 PWM 与低速关系，再调整四个死区前馈及左右斜率、PI。
   死区参数作为带方向的前馈偏置加入，最终仍受 PWM/上升斜率限幅。
6. 检查停止发布、关闭 Agent、断开 WiFi、Agent 重启、WiFi 恢复、非法输入以及正反切换。
   恢复连接后应等待新消息，不能恢复缓存指令。最后才考虑落地低速检查。

保留原 PI 初值：左 Kp/Ki=`0.040/0.300`，右=`0.038/0.290`，源文件顶部可调整；
积分限幅仍为 6000。高速 48000 PPS 的局部前馈模型已删除，不能声称低速或反向已经标定。
无编码器反馈时 PI 仍可能上升到保守上限，首次必须架空并观察计数，不能以它证明电机堵转安全。

## 7. 安全机制与硬件边界

- 上电应用最早将四个输入拉低，LEDC 初始 duty 为 0；无延迟后自动启动。
- 50 Hz 控制任务独立执行默认 400 ms 命令看门狗；停车清零 PWM、参考及积分，零指令绕过渐变。
- Agent 每 200 ms 检测一次、单次 ping 超时 100 ms；失败作废命令并重建实体。
  控制任务还独立检查 500 ms Agent 存活期限，通信任务阻塞不能无限保持运动。
- WiFi 断线/IP 丢失立即作废指令，控制任务最迟下一周期关输出；每 2 秒尝试连接，不耗尽重试。
  WiFi 会话代次防止快速断开/重连绕过清零；Agent 实体每次重建，等待新 Twist。
- PWM 正反向统一限幅，按目标方向计算 PI 幅值，输出不会因积分/死区跨到相反符号；
  结合幅值/上升速率饱和判断是否接受积分。默认参考斜率 10000 PPS/s，PWM 上升 500/s。
- 换向先关 PWM，滑行至编码器 `|PPS| <= 150` 连续 200 ms 后再渐增反向输出；
  未减速到阈值则持续等待。两 H 桥输入不会被软件同时主动驱动。
- 独立高优先级 `esp_timer` 在写入非零 PWM 前布防，停止输出后解除；输出期间控制任务超过
  100 ms 未完成控制周期，永久断开 LEDC
  到引脚的路由并拉低四脚，复位前不恢复。PCNT/LEDC 错误、明显反号反馈、测速异常或任务创建
  失败也锁定输出；网络短暂丢失则正常重连。
- `Twist` 没有发送时间戳，看门狗基于**本机接收时间**；QoS/清空会话防止持久化重放，
  不能识别仍在网络中延迟到达的单个新包。后续如需端到端时效应增加带时间戳协议。
- GPIO15 的现有连接保留。ROM/启动配置采样阶段早于 `app_main()`，软件无法保证这段时间
  外部 H 桥的输入状态；需确认现有电路在上电/复位时安全。全系统挂起或失去供电时同样不能
  只靠软件看门狗，应由驱动 EN/硬件急停提供最终保障。

引脚保持：电机 `13/15/25/14` → LEDC 低速 Timer0、通道 0/1/2/3，1 kHz，10 bit；
编码器左 `16/17`、右 `18/19`，毛刺滤波 1000 ns。没有初始化或读取旧 ADC GPIO27/33/32/35/34，
没有配置 OpenMV GPIO22/23，GPIO27 的 ADC2/WiFi 冲突业务已移除。

## 8. 修改与验证记录

| 原有内容 | 处理 |
|---|---|
| LEDC 引脚、频率、分辨率、通道、正反向和全零停车 | 保留，补充错误锁定与限幅 |
| PCNT 单边沿计数/B 判方向、滤波、左右符号 | 保留，改连续计数差分、增加溢出累计 |
| 20 ms、实际 dt、独立左右 PI、积分限幅与抗饱和思路 | 保留并修复有符号输出、停止清积分、上升限速 |
| 速度前馈/死区补偿思路 | 保留可调入口，低速参数默认 0，不复用高速拟合 |
| ADC、归一化、line_data_t、循线 PD、TrackState/PID_Profile | 删除 |
| OpenMV UART/解析、圈数、赛道距离/事件、旧调试与废弃注释 | 删除 |
| WiFi、micro-ROS 节点/订阅、重连、共享锁、安全监控 | 新增，全部仍在单个 C 文件 |

已完成的是**文件/API/控制路径的人工源码核对**：SDK/组件兼容性声明、引脚、消息类型、
方向公式、PPS 单位、零输出路径、限幅、超时失效和重连清理。它不等于构建或运行验证。

2026-10-29 使用 ESP-IDF 5.2.7 执行 `idf.py build` 已通过，当前构建启用电机、PWM 上限 360。
本轮尚未烧录或实车验证。完成上述架空检查、实际编码器符号/脉冲数确认、双向低速 PI/前馈
标定、超时/断网/换向实测及上电电气检查后，才能确认实际运动效果。
