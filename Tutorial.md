
# 第一阶段考核任务：完整实施教程

**目标环境**：Ubuntu 22.04 (x86_64) + ROS 2 Humble + Gazebo Classic 11 + TurtleBot3 (burger)

**怎么用这份教程**：每一章末尾都有「验收标准」和「常见坑」。做完一章、通过验收，再进下一章。卡住的时候，把你卡在哪一章哪一步告诉我，附上完整报错。

---

## 目录

- [第 0 章 概念速成：你到底在搭什么](#第-0-章-概念速成你到底在搭什么)
- [第 1 章 环境准备](#第-1-章-环境准备)
- [第 2 章 建立仓库和 ROS 2 功能包](#第-2-章-建立仓库和-ros-2-功能包)
- [第 3 章 自定义仿真世界（子任务 1）](#第-3-章-自定义仿真世界子任务-1)
- [第 4 章 SLAM 建图（子任务 2）](#第-4-章-slam-建图子任务-2)
- [第 5 章 Nav2 自主导航（子任务 3）](#第-5-章-nav2-自主导航子任务-3)
- [第 6 章 Python 多目标点导航节点（子任务 4）](#第-6-章-python-多目标点导航节点子任务-4)
- [第 7 章 实验日志](#第-7-章-实验日志)
- [第 8 章 附加挑战 1：RGB-D 相机](#第-8-章-附加挑战-1给-turtlebot3-加-rgb-d-相机)
- [第 9 章 附加挑战 2：自定义路径规划器](#第-9-章-附加挑战-2自定义路径规划器a--dijkstra)
- [第 10 章 附加挑战 3：视觉感知](#第-10-章-附加挑战-3视觉感知目标检测)
- [第 11 章 附加挑战 4：Frontier 自主探索](#第-11-章-附加挑战-4frontier-based-自主探索)
- [第 12 章 实验报告撰写](#第-12-章-实验报告撰写)
- [附录 A 排错速查](#附录-a-排错速查)
- [附录 B 验收自检表](#附录-b-验收自检表)

---

# 第 0 章 概念速成：你到底在搭什么

> 这一章不动手，全是概念。你说你对 ROS 和 SLAM 完全不懂，那这 20 分钟是整份教程里回报率最高的。后面每一步出问题，答案基本都在这一章里。

## 0.1 ROS 2 不是操作系统

名字骗人。ROS 2 实际上是三样东西：

1. **一套进程间通信机制**。你的程序被拆成很多个独立进程（叫 **node，节点**），它们通过网络协议互相发消息。激光雷达驱动是一个节点，SLAM 算法是一个节点，路径规划是一个节点。
2. **一套构建系统**（`colcon` + `ament`）。管理这些节点怎么编译、怎么安装、依赖谁。
3. **一套命令行工具**（`ros2 topic`、`ros2 node`、`ros2 launch`……）。让你能在运行时观察和干预这些进程。

**为什么要拆成这么多进程？** 因为机器人系统里，每个模块的失效方式不一样。雷达驱动崩了不该带着规划器一起死；你想换一个 SLAM 算法，不该重新编译整个系统。进程隔离是代价（通信开销、调试变复杂），换来的是模块可替换性。

## 0.2 节点之间怎么说话：三种通信方式

| 方式                      | 语义                                            | 类比              | 本项目中的例子                            |
| ------------------------- | ----------------------------------------------- | ----------------- | ----------------------------------------- |
| **Topic（话题）**   | 单向、异步、多对多、连续流                      | 广播电台          | `/scan` 激光数据、`/cmd_vel` 速度指令 |
| **Service（服务）** | 请求-应答、一问一答、瞬时                       | 函数调用 / HTTP   | 保存地图、切换模式                        |
| **Action（动作）**  | 请求-应答 + 中途反馈 + 可取消，**长耗时** | 异步任务 + 进度条 | `navigate_to_pose` 导航到某点           |

**为什么导航要用 Action 而不是 Service？** 因为"走到 (3, 2) 去"这件事要花 30 秒。Service 是阻塞的一问一答，30 秒里你什么都不知道，也没法中途反悔。Action 允许服务端持续回传反馈（"还剩 1.2 米"），也允许客户端中途 cancel。子任务 4 明确要求用 `navigate_to_pose` 动作客户端，原因就在这。

**快速自查命令**（后面你会用无数次）：

```bash
ros2 node list                      # 现在有哪些节点在跑
ros2 topic list                     # 现在有哪些话题
ros2 topic echo /scan --once        # 看一眼 /scan 上真的有数据吗
ros2 topic hz /scan                 # 数据频率是多少
ros2 topic info /scan --verbose     # 谁在发、谁在收
ros2 action list                    # 有哪些动作服务端
```

## 0.3 TF：机器人身上的坐标系树

这是新手最容易忽略、又最容易导致"一切看起来都对但就是不工作"的东西。

激光雷达测到"前方 2 米有个障碍物"。这个"前方 2 米"是**相对雷达自己**说的。但路径规划器需要知道的是"这个障碍物在**地图上的哪个位置**"。要做这个换算，你必须知道：雷达装在机器人身上的哪里、机器人现在在地图的哪里。

**TF（Transform）就是维护这一整套坐标变换关系的系统**。它是一棵树：

```
map                    ← 全局地图坐标系（固定不动）
 └── odom              ← 里程计坐标系（连续但会漂移）
      └── base_footprint    ← 机器人在地面上的投影点
           └── base_link         ← 机器人本体中心
                ├── base_scan        ← 激光雷达
                ├── wheel_left_link
                └── wheel_right_link
```

**每一段变换由谁发布，这个必须记住：**

| 变换                                          | 发布者                                 | 含义                         |
| --------------------------------------------- | -------------------------------------- | ---------------------------- |
| `map` → `odom`                           | SLAM（建图时）或 AMCL（导航时）        | 修正里程计的累积漂移         |
| `odom` → `base_footprint`                | Gazebo 的`diff_drive` 插件（仿真中） | 机器人从起点走了多远（会漂） |
| `base_footprint` → `base_link` → 各部件 | `robot_state_publisher`（读 URDF）   | 机器人自身固定的几何结构     |

**这解释了一个你后面一定会问的问题**：为什么 launch 文件里既要启动 Gazebo 插件，又要单独启动 `robot_state_publisher`？因为它们负责树的不同段。少了任何一段，树就断了，TF 查询失败，SLAM 和 Nav2 全部瘫痪。

调试命令：

```bash
ros2 run tf2_tools view_frames -o tf_tree   # 生成 tf_tree.pdf，看整棵树
ros2 run tf2_ros tf2_echo map base_link # 实时打印这两个坐标系之间的变换
```

## 0.4 仿真里的三个角色

```
┌──────────────┐   /scan /odom /clock /tf   ┌──────────────┐
│              │ ─────────────────────────► │              │
│    Gazebo    │                            │   ROS 2 节点 │
│  (物理世界)  │ ◄───────────────────────── │  (机器人大脑)│
│              │          /cmd_vel          │              │
└──────────────┘                            └──────────────┘
                                                    │
                                                    │ 订阅一切
                                                    ▼
                                             ┌──────────────┐
                                             │    RViz2     │
                                             │  (显示器)    │
                                             └──────────────┘
```

三句话记住它们的分工：

- **Gazebo = 假的现实**。它跑物理引擎，负责"如果轮子转了，机器人会移动到哪"、"激光打到墙上会返回什么距离"。**它不是可视化工具**，它是物理仿真器（虽然它自带一个 3D 窗口）。
- **ROS 2 节点 = 大脑**。SLAM、定位、规划、控制都在这里。它们完全不知道自己接的是仿真还是真机 —— 这正是仿真的价值。
- **RViz2 = 显示器**。它**什么都不产生**，只订阅话题然后画出来。RViz 里看不到东西，99% 是数据本身没发出来，而不是 RViz 坏了。

> **新手最常见的误解**：以为 RViz 是"控制界面"。不是。RViz 里的 "2D Goal Pose" 按钮只是往一个话题上发了一条消息而已。

## 0.5 `use_sim_time`：本项目最高频的翻车点

现实世界里时间自己走。仿真里不是 —— 仿真时间由 Gazebo 决定，可以比现实快、比现实慢、可以暂停。

Gazebo 会把它的时间发布到 `/clock` 话题上。ROS 2 节点有一个参数 `use_sim_time`：

- `false`（默认）：用系统墙上时钟
- `true`：忽略系统时钟，只认 `/clock` 上的时间

**如果你的节点里有一部分用仿真时间、另一部分用系统时间，会发生什么？**

TF 系统会认为"这个变换是 1800 秒之前的，太旧了，不可信"，直接拒绝查询。你会看到这类报错：

```
Lookup would require extrapolation into the past
Could not transform from [odom] to [base_footprint]
Message Filter dropping message: frame 'base_scan' at time ... for reason 'discarding message because the queue is full'
```

**铁律：仿真中，所有节点都必须 `use_sim_time:=true`。一个都不能漏，包括 RViz2。**

自查：

```bash
ros2 topic echo /clock --once                          # Gazebo 在发时间吗
ros2 param get /slam_toolbox use_sim_time              # 逐个节点确认
```

## 0.6 SLAM 是什么，为什么它难

**SLAM = Simultaneous Localization and Mapping，同时定位与建图。**

先看这个鸡生蛋问题：

- 想知道**我在哪**（定位），需要有**地图**做参照
- 想**画地图**（建图），需要知道**我在哪**，否则不知道把这一帧激光数据画到纸上的哪个位置

SLAM 就是同时解这两个未知数。核心步骤：

1. **扫描匹配（scan matching）**：新来一帧激光，跟已有地图对齐，对齐得最好的那个位姿就是"我现在最可能在哪"。
2. **累积误差**。每一次匹配都有小误差，走一圈下来误差累加，地图开始扭曲 —— 这就是**漂移（drift）**。
3. **回环检测（loop closure）**：走了一圈回到起点，算法认出"这个地方我来过"，于是知道了一个强约束：起点和现在应该是同一个位置。
4. **位姿图优化（pose graph optimization）**：把这个约束"摊平"到整条轨迹上，一次性修正整张地图。

**关键直觉**：提高传感器采样率**不能**消除漂移。漂移是随机游走误差的累积，是结构性问题，只能靠回环 + 优化这种全局约束来解决。

**这对你实操的直接影响**：建图时**一定要多绕几圈、多回到起点**，不要只跑一遍就保存。这是地图质量的决定因素。

你会用的工具是 **slam_toolbox**。注意：考核任务只要求你**会用**它，不要求你实现 SLAM 算法。

## 0.7 Nav2 是什么

Nav2（Navigation2）是 ROS 2 的导航栈。你给它一张地图和一个目标点，它负责让机器人开过去。内部主要组件：

| 组件                                 | 职责                                                           |
| ------------------------------------ | -------------------------------------------------------------- |
| **AMCL**                       | 粒子滤波定位。已知地图，用激光匹配算出"我在地图的哪"           |
| **Global Costmap**             | 全局代价地图。地图 + 障碍物膨胀                                |
| **Planner Server**             | 全局规划器。从当前位置到目标点算一条完整路径                   |
| **Local Costmap**              | 局部代价地图。只关注机器人周围几米，实时更新动态障碍           |
| **Controller Server**          | 局部控制器。跟踪全局路径，同时实时避障，输出`/cmd_vel`       |
| **Recovery / Behavior Server** | 恢复行为。卡住了就原地转圈、后退、清空代价地图                 |
| **BT Navigator**               | 行为树。用一棵树把上面这些编排起来：规划失败怎么办、卡住怎么办 |

**代价地图（costmap）的核心概念 —— 膨胀（inflation）**：

机器人不是一个点，它有半径。如果规划器把路径贴着墙画，机器人一定会撞。所以代价地图会把每个障碍物"膨胀"一圈，膨胀半径至少要大于机器人半径。

```
原始障碍       膨胀后
   ██          ▒▒▒▒▒▒
               ▒▒██▒▒     ← 灰色区域规划器会尽量避开
               ▒▒▒▒▒▒
```

`inflation_radius` 调太小 → 蹭墙、卡住；调太大 → 窄门过不去。这是你后面调参的主战场。TurtleBot3 burger 半径约 **0.105 m**。

## 0.8 完整数据流（把上面全串起来）

**建图阶段（子任务 2）：**

```
Gazebo ──/scan──┐
       ──/odom──┼──► slam_toolbox ──► /map (地图) + tf(map→odom)
       ──/clock─┘
键盘 ──► teleop ──/cmd_vel──► Gazebo
```

**导航阶段（子任务 3、4）：**

```
map.yaml ──► map_server ──/map──┐
Gazebo ──/scan──────────────────┼──► AMCL ──► tf(map→odom)
                                │
                                └──► Costmaps ──► Planner ──► Controller ──/cmd_vel──► Gazebo
                                                      ▲
你的 Python 节点 ──navigate_to_pose action──► BT Navigator
```

看懂这两张图，你就知道任何一个环节断了该去查哪个话题。

---

# 第 1 章 环境准备

## 1.1 先看清现状

```bash
# ROS 2 版本
echo $ROS_DISTRO                       # 应该输出 humble

# 现在装了哪些 Gazebo
which gazebo gzserver gzclient 2>/dev/null    # Classic 的可执行文件
which gz 2>/dev/null && gz sim --versions     # 新 Gazebo
dpkg -l | grep -E '^ii' | grep -E 'gazebo|gz-|ignition' | awk '{print $2, $3}'
```

你现在应该只有 `gz-*`（Harmonic）和/或 `ignition-*`，没有 `gazebo11`。

## 1.2 安装 Gazebo Classic（先模拟，再执行）

⚠️ 新版 Gazebo 和 Classic 在 apt 层面可能冲突，装 Classic 时 apt 可能要卸载你的 Harmonic。**先模拟一遍看清楚**：

```bash
sudo apt update
sudo apt install -s ros-humble-gazebo-ros-pkgs
```

`-s` 是 simulate，只打印计划不真的执行。**仔细读输出里的这两段**：

- `The following packages will be REMOVED:` ← 它要删什么
- `The following NEW packages will be installed:` ← 它要装什么

分三种情况：

**情况 A：只装不删** —— 直接执行，两套 Gazebo 共存。

**情况 B：要删掉 `gz-*` / `ignition-*`** —— 对本项目来说可以接受。Harmonic 以后想装回来一条命令的事，而你这四周都不会用到它。执行即可。

**情况 C：报依赖冲突装不上** —— 把完整输出发给我。

确认后正式安装：

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

这个包会自动拉进来 `gazebo11`（Ubuntu 22.04 的 universe 仓库里就有）。

**顺手把后面要用的都装了：**

```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-rosdep
```

**验证 Classic 装好了：**

```bash
gazebo --version        # 应该是 11.x
gzserver --version
```

**验证 ROS 桥接插件装好了**（这一条更重要）：

```bash
ls /opt/ros/humble/lib/ | grep gazebo_ros
```

必须看到这四个（还会有十几个其他的）：

```
libgazebo_ros_diff_drive.so
libgazebo_ros_ray_sensor.so
libgazebo_ros_imu_sensor.so
libgazebo_ros_joint_state_publisher.so
```

**这就是第 0 章说的那些插件，它们的存在是整个项目能跑起来的前提。**

> **⚠️ 别找错目录。** Classic 环境下有两套插件，分别装在两个地方：
>
> | 插件                   | 来自                                                      | 装在哪                                           | 例子                                                                   |
> | ---------------------- | --------------------------------------------------------- | ------------------------------------------------ | ---------------------------------------------------------------------- |
> | Gazebo 原生插件        | `gazebo11`（Ubuntu universe）                           | `/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/` | `libCameraPlugin.so`、`libRayPlugin.so`、`libDiffDrivePlugin.so` |
> | **ROS 桥接插件** | `ros-humble-gazebo-plugins` / `ros-humble-gazebo-ros` | **`/opt/ros/humble/lib/`**               | `libgazebo_ros_diff_drive.so`、`libgazebo_ros_ray_sensor.so`       |
>
> `gazebo_ros_pkgs` 是一组 **ament 包**，ament 包的库统一装进 `<prefix>/lib/`，不会塞进 Gazebo 自己的插件目录。去 `gazebo-11/plugins/` 里 grep `gazebo_ros` 永远是空的。
>
> 另外注意 Gazebo 原生也有一个 `libDiffDrivePlugin.so` —— 它只驱动轮子，不认识 ROS 话题。带 `_ros_` 的那个才是既驱动轮子、又订阅 `/cmd_vel`、又发布 `/odom` 的版本。

**如果 `grep` 结果为空**，说明 `ros-humble-gazebo-ros-pkgs` 没真正装上。逐步排查：

```bash
# ① 哪些 gazebo 相关的 ROS 包装了
dpkg -l | grep ros-humble-gazebo | awk '{print $2, $3}'
#   期望看到 5 个：gazebo-dev / gazebo-msgs / gazebo-plugins / gazebo-ros / gazebo-ros-pkgs
#   其中 ros-humble-gazebo-plugins 才是提供上面那四个插件的包

# ② 缺了就补装
sudo apt install ros-humble-gazebo-plugins ros-humble-gazebo-ros

# ③ 还找不到就全盘搜
sudo find / -name 'libgazebo_ros_diff_drive.so' 2>/dev/null
```

> 文件存在只是**必要条件**。Gazebo 运行时能否加载到它们还取决于 `GAZEBO_PLUGIN_PATH` / `LD_LIBRARY_PATH`（由 source `/opt/ros/humble/setup.bash` 时的环境钩子设置）。真正的判据是 1.5 节的冒烟测试 —— `ros2 topic hz /scan` 能跑出约 5 Hz，才算这一环真的通了。

## 1.3 安装 TurtleBot3（apt 路径）

```bash
# TurtleBot3 核心（metapackage，含 description / navigation2 / teleop 等）
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-msgs

# Gazebo 仿真包（含 model.sdf、worlds、launch）
sudo apt install ros-humble-turtlebot3-gazebo
```

> **不需要装 `ros-humble-gazebo-ros2-control`。** 那个包是给用 `ros2_control` 框架的机器人准备的。TurtleBot3 走的是 `libgazebo_ros_diff_drive.so` 插件那条路，完全不经过 ros2_control。装了不影响运行，但会让你的 `package.xml` 多一条无用依赖。
>
> **也不需要单独装 `gazebo11`。** `ros-humble-gazebo-ros-pkgs`（1.2 节已装）会自动把它拉进来。

### 二进制包确实是 Gazebo Classic 版本

这一点值得说明，因为网上很多教程会告诉你"apt 包不可靠，必须源码构建"。**实测不是这样。**

ROS 二进制包是从 `ros2-gbp/turtlebot3_simulations-release` 仓库构建的。Humble 下最新发布版本是 **2.3.8**，其内容：

```
package.xml 依赖：   <depend>gazebo_ros_pkgs</depend>
model.sdf 插件：     libgazebo_ros_imu_sensor.so
                     libgazebo_ros_ray_sensor.so
                     libgazebo_ros_diff_drive.so
                     libgazebo_ros_joint_state_publisher.so
gz-sim system 插件： 无
launch 文件：        robot_state_publisher.launch.py / spawn_turtlebot3.launch.py / turtlebot3_world.launch.py ...
```

和 GitHub `humble` 分支的源码一致。所以本教程第 3 章的 launch 文件（复用 `robot_state_publisher.launch.py` 和 `spawn_turtlebot3.launch.py`）在 apt 安装下同样能用。

### 立刻验证

```bash
grep -o 'libgazebo_ros[a-z_]*\.so' \
  /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
```

期望输出（4 行，顺序可能不同）：

```
libgazebo_ros_imu_sensor.so
libgazebo_ros_ray_sensor.so
libgazebo_ros_diff_drive.so
libgazebo_ros_joint_state_publisher.so
```

**如果输出为空**，说明 ROBOTIS 已经把新 Gazebo 版本发布进 Humble 了，本教程后半部分需要调整 —— 停下来找我。

### 记录版本号 + 锁版本

```bash
dpkg -l | grep -E 'turtlebot3|gazebo' | awk '{print $2, $3}'
```

把输出抄进 `docs/logbook.md`。

然后**锁住版本**：

```bash
sudo apt-mark hold ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
# 想解锁：sudo apt-mark unhold ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
```

**为什么要锁？** `humble` 分支上还挂着一个 `feature-gazebo-sim-migration` 分支没合并。万一在你做项目的这四周里 ROBOTIS 合并并发了新版，你某次随手 `apt upgrade` 就可能把仿真包换成新 Gazebo 版本，`/scan` 突然消失，而你完全想不到是 apt 干的。

这一条 `apt-mark hold` + logbook 里的版本记录，就是"可复现性"在实践中的样子 —— 写进实验报告的"环境管理"一节很有说服力。

### 什么时候需要改成源码构建

apt 装的东西在 `/opt/ros/humble/` 下，**是只读的，不能改**。

但**"要改机器人模型"通常不意味着要覆盖上游包**。更好的做法是在你自己的包里做一份**派生模型**：

- 复制那两个 XML（`model.sdf` + `model.config`，一共 32 KB）到你的包
- 网格 `.stl` 通过 `model://turtlebot3_common/...` 引用，仍然由 apt 装的包提供，不用复制
- 所有改动都在你的 Git 仓库里，别人 clone 即可复现

**第 8 章（RGB-D 相机）就是这么做的**，那里有完整步骤。

真正需要 clone 上游包覆盖的情况只有：想读源码、想调试上游包本身、或者要改的东西多到复制不现实。方法是：

```bash
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && colcon build --symlink-install
# 在 .bashrc 里 /opt/ros/humble 之后 source 它
```

> **overlay 机制**：`AMENT_PREFIX_PATH` 是一串前缀路径，包查找按顺序遍历、返回**第一个**命中。每次 `source .../setup.bash` 把自己的前缀**插到最前面**（prepend），所以后 source 的赢。
>
> ```bash
> echo $AMENT_PREFIX_PATH | tr ':' '\n'
> ros2 pkg prefix turtlebot3_gazebo
> ```
>
> **但能不用就不用** —— 它把你的改动散落在仓库之外，破坏可复现性。

> **⚠️ 如果你在别处看到 `-b humble-devel`，那是过时的。** 该分支已被 ROBOTIS 从远端删除，clone 会报 `fatal: 远程分支 humble-devel 在上游 origin 未发现`。现存分支只有 `noetic / humble / jazzy / main`（`git ls-remote --heads` 实测）。
>
> 另外，ROBOTIS 官方 e-Manual 的 Simulation 页面写着"本仿真使用 ros-gz 包"，那描述的是 jazzy/main，**和 humble 的实际代码不符**。以代码为准，不以文档为准。

## 1.4 配置环境变量

```bash
cat >> ~/.bashrc << 'EOF'

# ===== ROS 2 / TurtleBot3 =====
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_DOMAIN_ID=30
EOF

source ~/.bashrc
```

逐条解释：

| 变量                                  | 作用                                                                                   | 不设会怎样                                                                             |
| ------------------------------------- | -------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| `source /opt/ros/humble/setup.bash` | 把 ROS 2 加入搜索路径                                                                  | `ros2` 命令都不存在                                                                  |
| `source /usr/share/gazebo/setup.sh` | 设置`GAZEBO_MODEL_PATH` / `GAZEBO_RESOURCE_PATH` / `GAZEBO_PLUGIN_PATH` 的基础值 | **gzserver 找不到 `ground_plane`、`sun` 等自带模型，转去联网拉取，然后卡死** |
| `TURTLEBOT3_MODEL=burger`           | 选机器人型号                                                                           | launch 文件报`KeyError`                                                              |
| `GAZEBO_MODEL_PATH`                 | 追加 TurtleBot3 的模型目录，供`model://` 解析                                        | 机器人模型加载不出来                                                                   |
| `GAZEBO_MODEL_DATABASE_URI=""`      | 禁用在线模型数据库                                                                     | 本地缺模型时**静默卡死**而非报错                                                 |
| `ROS_DOMAIN_ID`                     | DDS 分组，避免和同一局域网的其他人串话                                                 | 可能收到别人的话题                                                                     |

> **⚠️ 第 2 章建完包后，记得再补一行**（很多人漏掉，症状是新终端里 `Package 'tb3_stage1' not found: searching: ['/opt/ros/humble']`）：
>
> ```bash
> echo 'source ~/tb3_nav_stage1/install/setup.bash' >> ~/.bashrc
> ```
>
> 必须用**绝对路径**，写相对路径的话换个目录开终端就失效。位置放在 `source /opt/ros/humble/setup.bash` 之后（overlay 机制：后 source 的覆盖先 source 的）。

**关于 `GAZEBO_MODEL_PATH` 的顺序。** `setup.sh` 是**直接赋值**（覆盖），不是追加。所以它必须写在 `export GAZEBO_MODEL_PATH=...` **之前**，否则你追加的 TurtleBot3 路径会被冲掉。配好后应该长这样：

```
/usr/share/gazebo-11/models : : /opt/ros/humble/share/turtlebot3_gazebo/models
└─ setup.sh 加的（ground_plane、sun 在这）    └─ 你加的（turtlebot3 模型在这）
```

**关于 `GAZEBO_MODEL_DATABASE_URI=""`。** Gazebo Classic 解析 `model://xxx` 的顺序是：先扫 `GAZEBO_MODEL_PATH`，本地找不到就去在线数据库 `models.gazebosim.org` 下载。

问题是这个站随 Gazebo 项目迁移到 Fuel 之后基本荒废了，国内网络下更是直接超时。一旦触发回退，gzserver 会阻塞在一个连不通的 HTTP 请求上 —— **世界加载停住，`/spawn_entity` 服务不会被注册**，于是你看到的是 `Was Gazebo started with GazeboRosFactory?`。那句报错报的是症状，不是病因。

设成空串后，本地缺模型会**立刻报错**说找不到 `model://xxx`，而不是挂几分钟。把"静默卡死"变成"明确报错"，排查时价值很大。本项目的自定义世界只用 `ground_plane`、`sun` 和纯几何体，全在本地，禁用在线库没有任何副作用。

> **⚠️ `.bashrc` 只对终端生效，桌面图标启动的程序拿不到这些变量。**
>
> 典型症状：终端里敲 `gazebo` 一切正常，**点桌面图标启动则中间 3D 画布全黑**（菜单栏、侧边面板反而正常）。
>
> **原因**：`~/.bashrc` 是交互式非登录 bash shell 才读的文件。点图标时程序由桌面会话（GNOME Shell / systemd --user）直接 fork，**整条链路没有 bash**，自然读不到。
>
> **为什么恰好是"画布黑、菜单正常"**：菜单栏是 Qt 画的，3D 画布是 Ogre 渲染的。Ogre 靠 `GAZEBO_RESOURCE_PATH` 找材质脚本和着色器，找不到就什么都渲染不出来 —— 两个渲染系统，两种命运。
>
> **自己验证**（`/proc/<PID>/environ` 存的是进程启动那一刻继承到的环境）：
>
> ```bash
> tr '\0' '\n' < /proc/$(pgrep -n gzserver)/environ | grep -E '^GAZEBO_'
> ```
>
> 分别在"图标启动"和"终端启动"两种情况下跑，对比输出。
>
> **建议不修。** 本项目全程用 `ros2 launch`，不会点这个图标；而且即使修好 `GAZEBO_*`，图标启动的 Gazebo 依然没有 ROS 环境，加载不了 `libgazebo_ros_*.so` —— 没有 `/scan`、没有 `/cmd_vel`，对项目毫无用处。
>
> 真要修，改一份自己的 `.desktop`（不会被 apt 升级冲掉）：
>
> ```bash
> mkdir -p ~/.local/share/applications
> cp /usr/share/applications/gazebo.desktop ~/.local/share/applications/
> # 编辑该文件，把 Exec= 改成：
> # Exec=bash -lc "source /usr/share/gazebo/setup.sh && source /opt/ros/humble/setup.bash && exec gazebo"
> ```

### ⚠️ 还要屏蔽 Fuel 在线模型库（否则 Gazebo 会周期性卡死）

`GAZEBO_MODEL_DATABASE_URI=""` **只管住了老的 model database，管不住 Fuel。**

Gazebo Classic 11 内置了 ign-fuel-tools，gzclient 的 **Insert 面板会去 `fuel.ignitionrobotics.org` 拉在线模型列表**。这个域名在国内连不通，请求会挂在 TCP 超时上（默认可能 30~130 秒）。

**后果比想象中严重**：gzserver 阻塞在网络 I/O 上 → 答不了 gzclient 的同步请求 → GUI 线程在 `transport::request()` 里永久等待 → 整个窗口冻结。一个连不通的 HTTPS 请求加一个无超时的同步 IPC，两者叠加就是"Gazebo 随机卡死"的完整链条。

**重要事实：只有 gzclient 访问 Fuel，gzserver 不碰网络。**

用抓包工具或代理日志能确认，发起请求的进程名是 `gzclient-11.10.2`。这决定了下面方案的优先级。

**方案 A（推荐）：有代理就让它成功。**

Fuel 的模型列表只有几十 KB，走代理一秒就回来，gzclient 拿到响应后不会阻塞。先测：

```bash
curl -m 10 -sSI https://fuel.ignitionrobotics.org/1.0/models | head -3
```

秒回 HTTP 200/30x 就说明通了。不通的话在代理软件里把 `ignitionrobotics.org` 和 `gazebosim.org` 两个域名后缀规则改成走代理节点。

**这是最干净的方案** —— 不改 Gazebo、不改系统，让它按设计正常工作。

**方案 B（保底，本教程默认）：不开 GUI。**

```bash
gzkill
ros2 launch tb3_stage1 stage1_world.launch.py gui:=false
```

既然只有 gzclient 访问 Fuel，不开它这个问题就**完全不存在**。gzserver 干净地跑物理和传感器，RViz 也不碰 Fuel。

**方案 C：从配置层面彻底关掉 Fuel（想保留 GUI 但代理不可用时）。**

读 `ign-fuel-tools` 的 `ClientConfig::LoadConfig` 源码可以确认：**`AddServer()` 只在解析到 `url` 字段时才被调用，没有任何兜底逻辑注入默认服务器。** 所以写一份不含 `servers` 条目的配置，客户端就是零服务器，不会发起任何网络请求。

```bash
mkdir -p ~/.ignition/fuel
[ -f ~/.ignition/fuel/config.yaml ] && cp ~/.ignition/fuel/config.yaml ~/.ignition/fuel/config.yaml.bak

cat > ~/.ignition/fuel/config.yaml << 'EOF'
---
# 故意留空 servers：不写 url 字段 = 零服务器 = 不发任何请求。
# 本地模型不受影响（走 GAZEBO_MODEL_PATH，与 Fuel 无关）。

cache:
  path: /home/你的用户名/.ignition/fuel
EOF
```

> `cache.path` 必须是绝对路径，`~` 在这个 YAML 里不会展开。

**代价**：gzclient 左侧 Insert 面板的在线模型库会是空的。本项目完全不需要它 —— 自定义世界用的是本地基本几何体，机器人模型来自 `GAZEBO_MODEL_PATH`。

回滚：`mv ~/.ignition/fuel/config.yaml.bak ~/.ignition/fuel/config.yaml`

**方案选择建议**：先试 A（成本最低，且保留 Insert 面板功能）；A 不可行且需要 GUI 就用 C；不需要 GUI 直接用 B。

> **⚠️ 不要用 `/etc/hosts` 把这些域名指到 `127.0.0.1`。**
>
> 直觉上"立刻 ECONNREFUSED 好过慢慢超时"，但实测**会让情况更糟** —— 秒失败会让 ign-fuel-tools 进入密集重试循环，从"随机卡"变成"一打开就卡死"。
>
> **教训：快失败不总是优于慢失败。** 这取决于调用方的重试策略。前面 `GAZEBO_MODEL_DATABASE_URI=""` 那一招之所以有效，是因为它让代码**根本不发起请求**，而不是让请求快速失败 —— 两者性质完全不同。

**验证配置生效：**

```bash
env | grep '^GAZEBO_'
```

应该看到 `GAZEBO_MODEL_PATH`（两段）、`GAZEBO_RESOURCE_PATH`、`GAZEBO_PLUGIN_PATH`、`GAZEBO_MASTER_URI`、`GAZEBO_MODEL_DATABASE_URI`（空）。

```bash
ls /usr/share/gazebo-11/models/ | grep -E 'ground_plane|sun'
```

必须两个都在。

**为什么选 burger 而不是 waffle？** burger 只有 2D 激光雷达，waffle 多一个深度相机。相机的渲染开销在仿真里非常大，而本任务的四个子任务**完全不需要相机**。除非你要做附加挑战里的视觉感知题，否则 burger 是正确选择。

## 1.5 冒烟测试

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Gazebo 窗口应该弹出，里面有一堆六边形柱子和一个小机器人。

**新开一个终端**验证数据流（这是你要养成的习惯 —— 眼睛看到不算数，话题上有数据才算）：

```bash
ros2 topic list
ros2 topic hz /scan          # 期望约 5 Hz
ros2 topic hz /odom          # 期望约 30 Hz
ros2 topic echo /clock --once
ros2 run tf2_tools view_frames -o tf_tree    # 生成 tf_tree.pdf
```

再开一个终端遥控它动起来：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

按 `i` 前进、`,` 后退、`j`/`l` 转向、`k` 停。**这个终端必须保持焦点**才能接收按键。

### 专条：gzclient 卡死 / 拖不动视角（已定位为 Gazebo 自身缺陷）

**症状**：世界正常加载，终端输出完美（插件全加载、spawn 成功、`/scan` `/odom` 都有），但 Gazebo 3D 窗口拖不动、转不了视角，桌面提示"无响应"。gzclient **没有死**，只是不响应。

> **✅ 已知根因（2026-09 实测）**：冻结来自 gzclient 的两个问题叠加 —— 它会去访问 Fuel 在线模型库（国内常连不通），同时它的 GUI 线程存在**无超时的同步 IPC**。
>
> **gzserver 完全不受影响**，物理、传感器、TF 全程正常。所以最省事的做法是 `gui:=false`（见 1.4 节方案 B）；有可用代理的话可以试方案 A 让 Fuel 请求正常完成。下面保留完整排查过程作为方法论参考。

**根因（gdb 栈回溯实测）**：

```
QCoreApplication::exec()                     ← Qt 主事件循环（GUI 线程）
 └→ GLWidget::mouseReleaseEvent()            ← 松开鼠标
     └→ GLWidget::OnMouseReleaseNormal()     ← 处理"选中实体"
         └→ JointControlWidget::SetModelName()   ← 加载该模型的关节列表
             └→ gazebo::transport::request()     ← 向 gzserver 发同步请求
                 └→ pthread_cond_wait(abstime=0x0)   ← 阻塞等待，无超时
```

`abstime=0x0` 意味着**永久等待，没有超时**。

链条是：3D 视图里松开鼠标 → Gazebo 认为选中了实体 → 关节控制面板去问 gzserver "这个模型有哪些关节" → **同步阻塞请求，直接跑在 Qt 的 GUI 线程上** → gzserver 未及时应答 → GUI 线程永久冻结。

**在 UI 线程上做无超时的同步 IPC，是教科书级的反模式。** 任何一次响应丢失都会让界面永久冻结，而不是超时报错。

这解释了全部相关现象：转视角就卡（拖拽结束必然触发 mouseRelease）、Shift 松开后卡、放着不动也会卡（同一阻塞请求的其他触发路径）、时好时坏（取决于那次请求有没有被应答，本质是竞态）、GPU 处于 P8 只占 27MiB 显存（压根没在渲染，是在等锁）。

**和显卡、内存、驱动、网络、虚拟化全都无关。**

**自己抓栈的方法**（卡死当下执行）：

```bash
sudo apt install -y gdb
sudo gdb -p $(pgrep -n gzclient) -batch -ex "bt" 2>/dev/null | grep -v "New LWP"
```

> ⚠️ 不要用 `thread apply all bt` 再 `head`，gzclient 有几十个线程，输出会被 `[New LWP ...]` 刷满。要全部线程就重定向到文件再看。

**解法：不用 gzclient，用 RViz。**

**RViz 不走 Gazebo 的 transport** —— 它订阅的是 ROS 话题（`/scan`、`/map`、`/tf`、`/plan`），和 `libgazebo_transport.so` 毫无关系，上面那条 bug 路径在 RViz 里不存在。

更重要的是，**四个子任务要看的东西本来就在 RViz 里，不在 Gazebo 里**：

| 章节    | 要观察什么                 | Gazebo 能看到吗                                |
| ------- | -------------------------- | ---------------------------------------------- |
| 第 3 章 | 激光有没有勾出房间轮廓     | ❌ Gazebo 只显示 3D 模型，**看不到激光** |
| 第 4 章 | 地图在增长、有没有重影     | ❌ Gazebo**完全不显示地图**              |
| 第 5 章 | 全局路径、代价地图、粒子云 | ❌ Gazebo**一个都不显示**                |
| 第 6 章 | 航点执行状态               | ❌ 看终端日志                                  |

**Gazebo 显示"真实世界"，RViz 显示"机器人以为的世界"。做 SLAM 和导航，你需要看的恰恰是后者。** 官方教程两个窗口都开只是为了直观对比，不是必需。

所以从第 3 章起默认这样跑：

```bash
gzkill
ros2 launch tb3_stage1 stage1_world.launch.py gui:=false
```

### ⚠️ 如果你装了 Tailscale / VPN / Clash TUN：先检查 Publicized address

**Gazebo 的 gzserver ↔ gzclient 之间是真正的 TCP 通信**（不是共享内存）。gzserver 广播自己的地址，gzclient 连过去。如果它挑中了 VPN 的虚拟网卡，所有 Gazebo 内部消息都要穿过 tun 设备，延迟不可控。

单独启动 gzclient 能看到这一行：

```bash
gzclient --verbose
```

```
[Msg] Publicized address: 100.79.8.130      ← 100.x.x.x 是 Tailscale 的 CGNAT 段！
```

**佐证**：同时会出现

```
[Wrn] [Publisher.cc:135] Queue limit reached for topic /gazebo/default/user_camera/pose, deleting message.
```

相机位姿话题在拖视角时高频发布，队列堆积到丢消息 = **传输通道已经堵了**。

完整因果链：

```
transport 走 VPN 虚拟网卡
  → 消息积压、延迟不可控
    → JointControlWidget::SetModelName() 的同步请求收不到应答
      → pthread_cond_wait(abstime=0x0) 永久阻塞
        → GUI 冻结 → GNOME 弹"无响应" → 强制退出 → exit -9
```

> **顺带纠正一个易错判断**：`exit code -9`（SIGKILL）**不一定是 OOM**。GNOME 的"应用程序无响应 → 强制退出"发的也是 SIGKILL。判据是 `dmesg -T | grep -i "killed process"` —— 没输出就不是 OOM，是被强制退出，而冻结才是真正的问题。

**修复：强制走回环。**

```bash
cat >> ~/.bashrc << 'EOF'
# 强制 Gazebo transport 走回环，避免选中 Tailscale/VPN 虚拟网卡
export GAZEBO_IP=127.0.0.1
export GAZEBO_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
EOF
source ~/.bashrc
```

验证：

```bash
gzkill
ros2 launch tb3_stage1 stage1_world.launch.py 2>&1 | grep -i "publicized"
# 应输出 127.0.0.1
```

对照实验（临时关掉 VPN，测完 `sudo tailscale up` 恢复）：

```bash
sudo tailscale down
gzkill && ros2 launch tb3_stage1 stage1_world.launch.py
```

### ⭐ 确定性触发条件：不要「选中模型」

**实测确认的触发动作只有一个：选中一个模型。** 不是打开 Gazebo，不是拖视角，就是"选中"这个动作。

对应栈里的 `OnMouseReleaseNormal → JointControlWidget::SetModelName` —— 选中实体后，关节控制面板会去问 gzserver "这个模型有哪些关节"，这个同步请求无超时。

**所以 Gazebo GUI 其实是可用的**，只要避开一个动作：

| 操作                               | 安全吗                                      |
| ---------------------------------- | ------------------------------------------- |
| 鼠标**拖拽**移动/旋转视角    | ✅ 安全（拖拽被识别为移动相机，不触发选中） |
| 滚轮缩放                           | ✅ 安全                                     |
| 顶部工具栏的预设视角按钮           | ✅ 安全                                     |
| View 菜单                          | ✅ 安全                                     |
| **左侧 Models 树里点模型名** | ❌**必卡**                            |
| **3D 视区里单击某个模型**    | ❌**必卡**                            |
| 左侧 Insert 标签页                 | ⚠️ 触发 Fuel 网络请求，见 1.4 节          |

**规则一句话：只拖，不点。**

已经卡住了只能重启（`gzkill` 后重新 launch），没有恢复办法 —— 因为主线程在无超时等待，不会自己醒。

**整个项目只有一处需要点模型**：第 5 章用鼠标拖箱子做动态避障演示。用命令行代替即可，效果一样且更容易录进视频：

```bash
gz model -m obstacle_box_small -x 1.0 -y 0.5 -z 0.175
```

### 关于 3D 视区的补充

而且本教程的世界文件里已经设好了俯视机位：

```xml
<gui fullscreen="0">
  <camera name="user_camera">
    <pose>0 -12 10 0 0.7 1.5708</pose>
  </camera>
</gui>
```

**开局就是斜上方俯瞰全场的视角，本来就不需要调相机。** 所以：

```bash
gzkill
ros2 launch tb3_stage1 stage1_world.launch.py      # 带 GUI

# 窗口弹出后，双手离开它，不点不拖不滚轮
# 另开终端遥控，只看不摸
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
```

要换视角就用顶部工具栏的预设视角按钮或 View 菜单，避开在画布上拖鼠标。

**如果全程不碰视区能稳定运行，说明我们对根因的判断是对的** —— 这本身就是一个干净的验证实验。

**Gazebo 3D 窗口在整个项目里只有一处曾经需要**：第 5 章用鼠标拖箱子做动态避障演示。用命令行代替即可，效果一样：

```bash
gz model -m obstacle_box_small -x 1.0 -y 0.5 -z 0.175
```

### RViz 首次配置（⚠️ 空白画布多半是这个原因）

**RViz2 默认 Fixed Frame 是 `map`。** 第 3 章还没跑 SLAM，`map` 坐标系不存在 → 什么都画不出来，画布一片空白，左上角 Global Status 是红的。**这不是 bug。**

```bash
rviz2 --ros-args -p use_sim_time:=true
```

逐步操作：

1. 左侧面板顶部 **Global Options** → **Fixed Frame** → `map` 改成 **`odom`**
   → Global Status 应从红色变成绿色 `OK`
2. 左下角 **Add** → **LaserScan** → OK
   → 展开该项 → **Topic** 选 `/scan`
   → **Size (m)** 从 0.01 调到 **0.05**（点更大更清楚）
3. **Add** → **RobotModel** → **Description Topic** 填 `/robot_description`
4. **Add** → **TF**（可选，显示坐标系）

这时应该看到一圈红点勾出两个房间的墙、门洞和四个障碍物。

**配好后 File → Save Config As** → 存到 `~/tb3_nav_stage1/src/tb3_stage1/rviz/stage1.rviz`，以后 `rviz2 -d <路径>` 直接加载。记得 `colcon build` 让它装进 install。

（第 4 章建图时把 Fixed Frame 改回 `map` —— 那时 slam_toolbox 会发布这个坐标系。）

### 附：原先怀疑过但已排除的方向

留作记录，说明排查过程：

| 假设                  | 排除依据                                                       |
| --------------------- | -------------------------------------------------------------- |
| 软件渲染（llvmpipe）  | `glxinfo` 显示 RTX 3060 + direct rendering: Yes              |
| 内存不足 / OOM        | 31G 内存，19G 空闲，15G swap 全空                              |
| 虚拟机 3D 直通不完整  | `systemd-detect-virt` 返回 `none`（裸机）                  |
| Wayland/XWayland 兼容 | `nvidia-smi` 进程列表显示 `/usr/lib/xorg/Xorg`，是原生 X11 |
| NVIDIA 驱动故障       | `journalctl -k` 无 `Xid` / `NVRM` 错误                   |
| 在线模型库超时        | `GAZEBO_MODEL_DATABASE_URI` 已为空                           |
| EOL GUI 插件阻塞      | 读源码确认它只往菜单栏加了个 QLabel 超链接                     |

**教训：症状（界面冻结）能对应的原因太多，逐个猜代价很高。一旦确认进程是"阻塞"而非"忙碌"（GPU 空闲 + CPU 低），就该直接上栈回溯 —— 它一次给出答案，而不是排除一个假设。**

**先确认问题确实只在 gzclient**：

```bash
gz stats                       # 看 real-time factor，接近 1.0 说明物理在正常跑
ros2 topic hz /clock           # 仿真时间在推进吗
ros2 topic hz /scan            # 传感器在出数据吗
```

**这三条正常 = gzserver 没问题，纯粹是 3D 渲染跟不上。**

**查渲染方式（决定性）：**

```bash
systemd-detect-virt                        # 是虚拟机吗
sudo apt install -y mesa-utils
glxinfo | grep -E "OpenGL renderer|OpenGL version|direct rendering"
```

出现 **`llvmpipe`** 或 **`softpipe`** = 软件渲染，没有 GPU 加速，3D 全靠 CPU 逐像素算。Gazebo 的 Ogre 场景在软渲染下大概只有 1~3 FPS —— 主观感受就是完全卡死。虚拟机里 3D 直通不完整时最常见。

> 软渲染同时解释了另一个症状：**gzclient 被 OOM 杀掉（exit -9）**。软渲染要在内存里维护整个帧缓冲和纹理，内存开销远大于硬件渲染。**两种不同症状，同一个根因。**

**解法：不用 gzclient。**

这个项目四章里，你真正要看的东西是：

| 章节    | 看什么                       | 用什么看       |
| ------- | ---------------------------- | -------------- |
| 第 3 章 | `/scan` 有没有勾出房间轮廓 | **RViz** |
| 第 4 章 | 地图在长、有没有重影         | **RViz** |
| 第 5 章 | 全局路径、代价地图、粒子云   | **RViz** |
| 第 6 章 | 航点执行日志                 | **终端** |

**Gazebo 3D 窗口在整个项目里只有一处必需**：第 5 章用鼠标拖箱子做动态避障演示。那一次可以单独开忍几分钟，或者干脆用命令行移动模型，效果一样：

```bash
gz model -m obstacle_box_small -x 1.0 -y 0.5 -z 0.175
```

**RViz 比 Gazebo 轻得多** —— 它画的是点云、线条、栅格地图这类图元，不需要材质、光照、阴影、纹理。软渲染下 RViz 通常还能用，Gazebo 就不行。

所以从第 3 章起，默认这样跑：

```bash
ros2 launch tb3_stage1 stage1_world.launch.py gui:=false
```

### 专条：gzclient 被 OOM 杀掉

**先分清 gzserver 和 gzclient：**

| 进程         | 职责                                                            | 挂了会怎样                           |
| ------------ | --------------------------------------------------------------- | ------------------------------------ |
| `gzserver` | 物理引擎 + 传感器插件。发布`/scan`、`/odom`、`/clock`、TF | **一切停摆**                   |
| `gzclient` | 纯 3D 显示窗口                                                  | **只是看不见画面，仿真照常跑** |

所以 `[gzclient-2]: process has died ... exit code -9` **不影响验收**。`gzserver` 还活着，新开终端 `ros2 topic hz /scan` 照样有数据。

**退出码的含义**（排查时很有用）：

| 退出码  | 信号    | 含义                                          |
| ------- | ------- | --------------------------------------------- |
| `-2`  | SIGINT  | 你按了 Ctrl+C                                 |
| `-9`  | SIGKILL | **几乎总是内核的 OOM Killer**：内存不够 |
| `-11` | SIGSEGV | 程序自己段错误崩溃                            |
| `1`   | —      | 程序主动以错误码退出                          |

**确认是不是 OOM：**

```bash
free -h
swapon --show
dmesg -T | grep -iE 'out of memory|Killed process' | tail -5
```

打出 `Killed process ... (gzclient)` 就实锤了。

**顺带查渲染方式：**

```bash
sudo apt install -y mesa-utils
glxinfo | grep -E "OpenGL renderer|OpenGL version"
```

出现 **`llvmpipe`** 或 **`softpipe`** = 软件渲染，3D 全靠 CPU 算，又慢又吃内存。虚拟机或缺显卡驱动时的典型情况。

**缓解措施（按性价比排序）：**

**① 加 swap**（`swapon --show` 为空时必做）

```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
free -h
```

swap 慢，但**慢总比被杀好**。没有 swap 时内存一满就直接 OOM Kill，毫无缓冲。

**② 跑仿真时关掉浏览器。** Chrome 开十几个标签轻松吃 2~3 GB，是最容易忽略也最见效的一条。

**③ 用无头模式。**

⚠️ **官方的 `turtlebot3_world.launch.py` 无条件包含 gzclient，没有 `gui` 参数**，`gui:=false` 对它无效。

但**第 3 章你自己写的 launch 文件有这个开关**：

```bash
ros2 launch tb3_stage1 stage1_world.launch.py gui:=false
```

后面做 SLAM 和 Nav2，你真正要看的是 **RViz**（地图、激光、路径），不是 Gazebo 的 3D 窗口。Gazebo 窗口只在用鼠标拖障碍物做动态避障演示时才必需。平时关掉能省下一大半图形内存。

**④ 你自己的世界比官方世界轻得多。** 官方 `turtlebot3_world` 用了 12 个 `.dae` 网格模型（10 个六边形柱 + 2 面墙）；你的 `stage1_world` 全是 box 和 cylinder 基本几何体，零网格零纹理，并且已经关了阴影。所以别因为这次卡就对第 3 章没信心。

## 1.6 关掉仿真的正确姿势

```bash
# Ctrl+C 之后，务必确认进程真的死了
pgrep -af 'gzserver|gzclient'   # pgrep 只接受一个模式，必须用 -f + 正则
# 如果还有残留：
pkill -9 gzserver; pkill -9 gzclient
```

**Ctrl+C 经常杀不干净 `gzserver`。** 残留的 gzserver 会占着端口 11345，导致下次启动出现各种诡异现象（世界加载不出来、话题重复、机器人不动）。养成每次退出后 `pgrep` 一下的习惯，能省掉大量莫名其妙的调试时间。

建议加个带**验证**的清理函数（只 kill 不验证，你永远不知道有没有清干净）：

```bash
cat >> ~/.bashrc << 'EOF'
gzkill() {
  pkill -9 gzserver 2>/dev/null
  pkill -9 gzclient 2>/dev/null
  sleep 2
  if pgrep -af 'gzserver|gzclient'; then
    echo "⚠️  仍有残留，再执行一次"
  else
    echo "✓ Gazebo 进程已清理干净"
  fi
}
EOF
source ~/.bashrc
```

**每次启动仿真前先跑一次 `gzkill`。** 残留进程会导致两类互不相干的症状：

- 新 gzserver 抢不到 11345 端口 → **秒死，exit 255**
- 旧进程占着内存和 CPU → 新 gzclient 资源不足 → **卡死或被 OOM 杀掉**

两者都不会给出指向"残留进程"的报错，所以只能靠习惯预防。

> **⚠️ 症状时好时坏 = 资源临界，不是"已经好了"**
>
> 如果你发现同样的命令有时正常、有时卡死、有时被杀，**而你并没有改任何配置**，那不是随机故障，是系统刚好在够用与不够用的边界上 —— 成败取决于当时内存余量、page cache 命中、后台开了什么。
>
> 这类问题的危险在于它会在负载变大时才真正发作。对比一下各章负载：
>
> | 阶段                   | 同时运行                                           | 持续时长                     |
> | ---------------------- | -------------------------------------------------- | ---------------------------- |
> | 第 3 章                | gzserver + gzclient                                | 几分钟                       |
> | **第 4 章 SLAM** | gzserver + gzclient + slam_toolbox + RViz + teleop | **10~20 分钟不能中断** |
>
> 第 4 章中途崩溃 = 地图从头重建。**所以一旦观察到这种摇摆，立刻做诊断（`systemd-detect-virt` / `glxinfo` / `free -h`）并规划减负方案，不要因为"现在能跑"就往下走。**

## ✅ 第 1 章验收标准

- [ ] `gazebo --version` 输出 11.x
- [ ] `ls /opt/ros/humble/lib/ | grep gazebo_ros` 能看到 4 个桥接插件 `.so`（插件**装了**）
- [ ] `model.sdf` 里 grep 得到 4 个 `libgazebo_ros_*.so`（模型**用了**这些插件）
- [ ] 包版本已记入 logbook，且 `apt-mark hold` 已锁定
- [ ] `env | grep '^GAZEBO_'` 中 `GAZEBO_MODEL_PATH` 含 `/usr/share/gazebo-11/models` 和 turtlebot3 两段
- [ ] `turtlebot3_world.launch.py` 能弹出 Gazebo 并显示机器人
- [ ] `/scan` ≈ 5 Hz，`/odom` ≈ 30 Hz，`/clock` 有数据
  （**注意：gzclient 是否存活不是验收标准**。它挂了只是看不见画面，gzserver 仍在跑）
- [ ] `frames.pdf` 里能看到 `odom → base_footprint → base_link → base_scan` 完整链条
- [ ] 键盘遥控能让机器人移动

## 🔧 第 1 章常见坑

| 现象                                                                                             | 原因                                                                                                                 | 解决                                                                                                                                 |
| ------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?` + Gazebo 无响应 | **不是插件问题**。gzserver 卡在加载世界阶段（找不到本地模型转去联网），还没轮到开 `/spawn_entity` 服务       | `source /usr/share/gazebo/setup.sh`；`export GAZEBO_MODEL_DATABASE_URI=""` 让它立刻报错而非卡死；先 `pkill -9 gzserver` 清残留 |
| `package 'turtlebot3_gazebo' not found`                                                        | 没 source ROS，或包没装上                                                                                            | `source /opt/ros/humble/setup.bash`；`dpkg -l ros-humble-turtlebot3-gazebo`                                                      |
| Gazebo 窗口黑屏 / 卡死                                                                           | 显卡驱动或软渲染                                                                                                     | `glxinfo \| grep "OpenGL renderer"` 看是不是 llvmpipe（软渲染），见下方专条                                                         |
| `[gzclient-2]: process has died ... exit code -9`                                              | **`-9` = SIGKILL = 内存不足被 OOM Killer 杀掉**。死的只是 3D 显示窗口，gzserver 和所有传感器插件仍在正常运行 | 见下方"gzclient 被 OOM 杀掉"专条。**注意：这不算冒烟测试失败**                                                                 |
| `[Err] Waiting for master`                                                                     | 上次的 gzserver 没杀干净                                                                                             | `pkill -9 gzserver`                                                                                                                |
| `/scan` 没数据但 Gazebo 有机器人                                                               | 插件没加载 = 模型不是 Classic 版                                                                                     | 回 1.3 重跑`model.sdf` 的 grep                                                                                                     |
| 加载世界极慢（首次几分钟）                                                                       | Gazebo 在联网下载模型                                                                                                | 等一次即可，之后缓存在`~/.gazebo/models`                                                                                           |
| 编译自己的包时被 OOM Killer 杀掉                                                                 | 并行度太高                                                                                                           | `colcon build --parallel-workers 2`                                                                                                |

---

# 第 2 章 建立仓库和 ROS 2 功能包

考核要点明确写了：**"ROS2 功能包的创建与目录结构是否正确（package.xml、setup.py、launch/、config/ 等）"**。这一章直接对应这个得分点。

## 2.1 建 GitHub private repo

```bash
mkdir -p ~/tb3_nav_stage1/src
cd ~/tb3_nav_stage1
git init
git branch -M main
```

在 GitHub 上创建一个 **private** repository（任务书明确要求 private），然后：

```bash
git remote add origin git@github.com:<你的用户名>/tb3_nav_stage1.git
```

`.gitignore`：

```bash
cat > .gitignore << 'EOF'
build/
install/
log/
__pycache__/
*.pyc
.vscode/
*.swp
EOF
```

> **要不要提交 `.pgm` 地图文件？** 要。它是任务的交付物之一，只有几百 KB，而且后续导航必须加载它。Git 存二进制文件的痛点在于"频繁修改的大文件"，地图不属于这一类。但**不要**提交 `~/.gazebo/models` 下载的模型缓存，也不要提交 rosbag。

## 2.2 创建功能包

### 先读文档，别先抄命令

```bash
ros2 pkg create --help
```

花 30 秒扫一遍。除了 `--build-type`，还有 `--description`、`--maintainer-name`、`--maintainer-email`、`--dependencies`、`--node-name` —— **很多人手改 `package.xml` 和 `setup.py` 的内容，命令行本来就能直接指定。**

### 建包前先想：此刻能确定哪些依赖

这一步是流程的核心。建包这一刻你真正确定的只有：

- 要写一个 Python ROS 2 节点 → `rclpy`
- 它要调 Nav2 的 `navigate_to_pose` 动作 → `nav2_msgs`（动作定义）、`action_msgs`（`GoalStatus`）

你**还不知道**会用到 `gazebo_ros`、`slam_toolbox`、`nav2_bringup`、`rviz2` —— 那些要等第 3、4、5 章写 launch 文件时才浮现。

所以：**create 时只声明这三个，其余用到再加。** 这就是真实工程里的增量维护，而不是一次性写全。

### 建包

```bash
cd ~/tb3_nav_stage1/src

ros2 pkg create tb3_stage1 \
  --build-type ament_python \
  --license Apache-2.0 \
  --description "第一阶段考核：TurtleBot3 自建世界 SLAM 建图与自主导航" \
  --maintainer-name "你的名字" \
  --maintainer-email "你的真邮箱" \
  --dependencies rclpy nav2_msgs action_msgs \
  --node-name waypoint_navigator
```

`--maintainer-*` 填你自己的，建议和 GitHub 身份一致。

**`ament_python` vs `ament_cmake`**：前者是纯 Python 包（用 `setup.py`），后者是 C++ 包（用 `CMakeLists.txt`）。本项目只写 Python，选 `ament_python`。

### 跑完先看它生成了什么

```bash
cd tb3_stage1
find . -type f | sort
cat package.xml
cat setup.py
```

两处值得注意：

- **`package.xml` 里生成的是 `<depend>` 而不是 `<exec_depend>`。** `<depend>` 等于 `build_depend` + `build_export_depend` + `exec_depend` 三合一。对 ament_python 包来说没有真正的"构建期依赖"，所以两种写法功能上等价，`<exec_depend>` 更精确，`<depend>` 更常见。**保持生成的 `<depend>` 就行，不必改。**
- **`--node-name` 连 `setup.py` 的 `entry_points` 一起生成了**，还建了一个 hello-world 版的 `tb3_stage1/waypoint_navigator.py`。第 6 章你只需要覆盖这个文件的内容，不用再动 `entry_points`。

### `ros2 pkg create` 已经帮你生成了什么

这条命令**不是只建了个空文件夹**。它是个脚手架生成器（和 `cargo new`、`npm init`、`django-admin startproject` 一个道理），跑完之后目录里已经有：

```
tb3_stage1/
├── package.xml          ← 已生成，内容是模板
├── setup.py             ← 已生成，内容是模板
├── setup.cfg            ← 已生成（告诉 colcon 可执行文件装到哪）
├── resource/tb3_stage1  ← 已生成（空文件，ament 索引的标记）
├── tb3_stage1/
│   └── __init__.py      ← 已生成（空文件）
└── test/                ← 已生成（三个代码风格检查脚本）
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

**这个骨架已经是一个合法的、能 `colcon build` 通过的 ROS 2 包了**，只是什么功能都没有。它存在的意义就是保证上一节说的四个强制项一个不漏。

先自己看一眼生成的原始内容：

```bash
cd ~/tb3_nav_stage1/src/tb3_stage1
cat package.xml
```

大致长这样（版本号、描述、维护者是命令填的默认值）：

```xml
<package format="3">
  <name>tb3_stage1</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="你的用户名@todo.todo">你的用户名</maintainer>
  <license>Apache-2.0</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**所以下面 2.3、2.4 两节做的是"改"，不是"从零建"。** 看到文件里已经有内容是正常的，不是你操作错了。

补齐目录结构：

```bash
cd ~/tb3_nav_stage1/src/tb3_stage1
mkdir -p launch worlds config maps rviz
```

最终结构：

```
tb3_nav_stage1/
├── .gitignore
├── README.md
├── docs/
│   └── logbook.md              ← 实验日志
└── src/
    └── tb3_stage1/
        ├── package.xml          ← 生成的，2.3 节要改
        ├── setup.py             ← 生成的，2.4 节要改
        ├── setup.cfg            ← 生成的，不用动
        ├── resource/tb3_stage1  ← 生成的，不用动也别删
        ├── test/                ← 生成的，不用动
        ├── tb3_stage1/          ← Python 源码（唯一放节点的地方）
        │   ├── __init__.py
        │   └── waypoint_navigator.py    ← 第 6 章写
        ├── launch/              ← 你建的：launch 文件
        ├── worlds/              ← 你建的：自定义世界
        ├── config/              ← 你建的：YAML 参数
        ├── maps/                ← 你建的：SLAM 输出的地图
        └── rviz/                ← 你建的：RViz 配置
```

### 这是标准写法吗

**是约定，不是强制。** 对照两个官方包：

```
nav2_bringup/                    turtlebot3_navigation2/
├── package.xml                  ├── package.xml
├── CMakeLists.txt               ├── CMakeLists.txt
├── launch/     (8 个)           ├── launch/    (1 个)
├── params/     (4 个)           ├── param/     (burger.yaml 等 4 个)
├── maps/       (2 个)           ├── map/       (map.pgm + map.yaml)
├── rviz/       (2 个)           └── rviz/      (1 个)
├── worlds/     (2 个)
└── urdf/       (1 个)
```

分类模式几乎一致，但**名字并不统一** —— nav2 用 `params/` 复数，TurtleBot3 用 `param/` / `map/` 单数。所以"标准"的是这个**分类方式**，不是具体拼写。

ROS 2 真正**强制**的只有四样，其余全是约定：

| 路径                                  | 谁强制的                     | 少了会怎样                         |
| ------------------------------------- | ---------------------------- | ---------------------------------- |
| `package.xml`                       | ament 索引                   | `ros2 pkg list` 里找不到这个包   |
| `setup.py`（或 `CMakeLists.txt`） | colcon                       | 无法构建                           |
| `resource/<包名>`                   | ament 索引（一个空标记文件） | 能构建，但`ros2 launch` 找不到包 |
| `<包名>/__init__.py`                | Python                       | 模块导入失败                       |

理论上你把 `worlds/` 改名叫 `changjing/` 也能跑，只要 `setup.py` 的 `data_files` 里对应改掉。**约定的价值不在强制力，在于别人打开你的仓库时不用问"地图放哪了"。** 考核要点里的"目录结构是否正确"考的就是这个。

### 每个目录装什么

| 目录            | 内容                                         | 文件类型             | 本项目预计数量                                   |
| --------------- | -------------------------------------------- | -------------------- | ------------------------------------------------ |
| `tb3_stage1/` | **节点源码**（唯一放可执行代码的地方） | `.py`              | 1~2 个                                           |
| `launch/`     | 启动脚本：起哪些节点、喂什么参数、什么顺序   | `.launch.py`       | 4 个（world / slam / navigation / waypoint_nav） |
| `worlds/`     | Gazebo 场景描述                              | `.world`（SDF）    | 1 个                                             |
| `config/`     | 节点参数                                     | `.yaml`            | 3 个（slam / nav2 / waypoints）                  |
| `maps/`       | SLAM 建图产物                                | `.pgm` + `.yaml` | 1 组                                             |
| `rviz/`       | RViz 面板布局（存下来免得每次手配）          | `.rviz`            | 1~2 个                                           |

### ⚠️ 澄清一个常见误解：节点住在哪

**`launch/`、`config/`、`worlds/` 这些目录里一个节点都没有。**

- **节点（node）** = 一个运行中的进程。源码只在 `tb3_stage1/*.py`，通过 `setup.py` 的 `entry_points` 注册成可执行文件。
- **launch 文件不是节点。** 它是一段描述"要起哪些节点"的 Python 脚本，本身不参与机器人的运行逻辑。
- **`.yaml` / `.world` 是数据**，分别喂给节点和 Gazebo。Gazebo 本身甚至不是 ROS 节点（是它加载的 `libgazebo_ros_*.so` 插件在充当节点）。

这个区分决定了你这个项目**实际要写多少代码**：

| 阶段                | 运行中的节点数（大致）                                                                                                                      | 你自己写的     |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- | -------------- |
| 建图（第 4 章）     | ~6：gzserver 插件、robot_state_publisher、slam_toolbox、rviz2、teleop                                                                       | 0              |
| 导航（第 5 章）     | ~15：上面的 + map_server、amcl、planner_server、controller_server、behavior_server、bt_navigator、velocity_smoother、2 个 lifecycle_manager | 0              |
| 多点导航（第 6 章） | 同上 + 1                                                                                                                                    | **1 个** |

整个考核任务你要写的节点**就一个**（`waypoint_navigator`），其余全是现成节点。你的工作是用 launch 和 YAML 把它们正确地接起来。

这也解释了任务书的考核要点为什么反复强调"各节点能否正常启动并协同工作"、"目录结构是否正确"，而不是"算法实现得好不好" —— 这一阶段考的是**系统集成能力**，不是算法实现能力。

## 2.3 改 `package.xml`

### 先说个通则：这些文件分别从哪来

这份教程后面会出现很多长文件。**它们的来源完全不同**，别一律当成"要手打"。本项目全部 10 个产出物：

| 产出物                     | 谁生成                                          | 本教程对应 |
| -------------------------- | ----------------------------------------------- | ---------- |
| `package.xml`            | `ros2 pkg create` + 增量手改                  | 2.2 / 2.3  |
| `setup.py`               | 同上                                            | 2.4        |
| `*.launch.py`            | **纯手写**，没有任何生成器                | 3.4 起     |
| `stage1_world.world`     | **Gazebo GUI** 或手写                     | 3.3        |
| `slam_params.yaml`       | 抄 slam_toolbox 官方模板再改                    | 4.2        |
| `nav2_params.yaml`       | 抄 nav2_bringup 官方模板再改                    | 5.1        |
| `waypoints.yaml`         | 手写（坐标从 RViz 读出来）                      | 6.2        |
| `*.rviz`                 | **RViz 的 GUI**（File → Save Config As） | 4.4        |
| `map.pgm` / `map.yaml` | 跑 SLAM +`map_saver_cli` 产出                 | 4.7        |
| `waypoint_navigator.py`  | 纯手写                                          | 6.3        |

**10 个里只有 2 个来自 GUI，而且是两个不同的 GUI**（Gazebo 和 RViz）。

### 什么东西才会有 GUI

规律很清楚：**取决于产出物的本质是不是空间性/视觉性的**。

- **有 GUI**：世界布局（墙在哪、箱子多大）、RViz 面板配置（哪个窗格放哪个显示项）—— 用鼠标表达比用文字自然
- **没 GUI**：依赖关系、启动顺序、参数取值、控制逻辑 —— 本质是**结构和行为**，画不出来

`nav2_params.yaml` 里的 `inflation_radius: 0.30` 你没法用鼠标画出来，所以它只能抄模板 + 手改。

这个规律以后判断"某个东西有没有省力工具"很好用。而 `nav2_params.yaml` 有 800 多行 —— 没有任何人手写它，看到长文件先问一句"官方有没有模板可抄"。

### 现在还不用改

如果你按 2.2 的完整命令建的包，`<description>`、`<maintainer>`、`<license>` 和前三个依赖都已经就位了。**这一节你现在什么都不用做。**

真正要做的是**随着后面几章的进展往里加依赖**。下面这份是四周做完后 `package.xml` 该有的样子 —— 现在当参考看，不要提前抄进去。

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tb3_stage1</name>
  <version>0.1.0</version>
  <description>第一阶段考核：TurtleBot3 自建世界 SLAM 建图与自主导航</description>
  <maintainer email="你的邮箱">你的名字</maintainer>
  <license>Apache-2.0</license>

  <!-- 建包时就确定的（ros2 pkg create --dependencies 生成） -->
  <depend>rclpy</depend>
  <depend>nav2_msgs</depend>
  <depend>action_msgs</depend>

  <!-- 第 3 章：stage1_world.launch.py 引用 -->
  <depend>gazebo_ros</depend>
  <depend>turtlebot3_gazebo</depend>

  <!-- 第 4 章：slam.launch.py 引用 -->
  <depend>slam_toolbox</depend>
  <depend>rviz2</depend>
  <depend>nav2_map_server</depend>

  <!-- 第 5 章：navigation.launch.py 引用 -->
  <depend>nav2_bringup</depend>

  <!-- 第 6 章：waypoint_navigator.py 里 import yaml -->
  <depend>python3-yaml</depend>

  <!-- 全程手动运行的工具 -->
  <depend>teleop_twist_keyboard</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

> `package.xml` 是给 `rosdep` 看的。别人 clone 你的仓库后一句 `rosdep install --from-paths src --ignore-src -r -y` 就能装齐所有依赖 —— 这是"可复现性"的核心，也是考核要点里"目录结构是否正确"的实质内容。

### 增量节奏表

做到哪一章，就加哪几条。**不要提前加。**

| 什么时候 | 你写了什么                                                                                             | 加什么                                                    |
| -------- | ------------------------------------------------------------------------------------------------------ | --------------------------------------------------------- |
| 2.2 建包 | —                                                                                                     | `rclpy`、`nav2_msgs`、`action_msgs`（命令自动生成） |
| 第 3 章  | `stage1_world.launch.py` 里 `get_package_share_directory('gazebo_ros')`、`('turtlebot3_gazebo')` | `gazebo_ros`、`turtlebot3_gazebo`                     |
| 第 4 章  | `slam.launch.py` 引用 slam_toolbox；`Node(package='rviz2')`；用 `map_saver_cli` 存图             | `slam_toolbox`、`rviz2`、`nav2_map_server`          |
| 第 5 章  | `navigation.launch.py` 引用 nav2_bringup                                                             | `nav2_bringup`                                          |
| 第 6 章  | `waypoint_navigator.py` 里 `import yaml`                                                           | `python3-yaml`                                          |
| 全程     | 键盘遥控                                                                                               | `teleop_twist_keyboard`                                 |

### 每加完一批，跑一次审计闭环

**别靠记忆，靠 grep。** 完整循环是：写代码 → grep 审计 → 补 `package.xml` → `rosdep` 验证。

```bash
cd ~/tb3_nav_stage1/src/tb3_stage1

# Python 代码 import 了什么
grep -rhoP '^\s*(from|import)\s+\K[a-z_0-9]+' tb3_stage1/ | sort -u

# launch 文件引用了哪些包
grep -rhoP "get_package_share_directory\(['\"]\K[a-z_0-9]+" launch/ | sort -u
grep -rhoP "package=['\"]\K[a-z_0-9]+" launch/ | sort -u
```

输出的每一项都该在 `package.xml` 里有对应条目。然后验证声明的都能解析：

```bash
cd ~/tb3_nav_stage1
rosdep check --from-paths src --ignore-src
```

> **⚠️ 为什么必须靠工具而不是靠记忆：`colcon build` 抓不到缺失的依赖。**
>
> Python 是动态导入的，构建期不做任何检查。漏声明一个依赖，你自己的机器上照跑不误（因为本来就装着），只有在**别人的干净机器上**才会炸成 `ModuleNotFoundError`。而那通常就是你导师复现你项目的时候。
>
> `rosdep check` 只验证"你声明的能不能解析"，验证不了"你漏了什么"——漏的那部分只能靠上面的 grep 审计补。

### 直接依赖 vs 传递依赖

只声明**你自己的代码直接引用**的包。比如：

- 我们的 launch 引用了 `turtlebot3_gazebo` → 声明
- `turtlebot3_gazebo` 内部会启动 `robot_state_publisher` → **不声明**，那是它的依赖，不是我们的

多声明不会报错，但会让 `package.xml` 逐渐变成一份没人敢删的垃圾清单。少声明才是真出事。判断标准：**在你自己的文件里 grep 得到的，才写。**

## 2.4 改 `setup.py`（⚠️ 最容易踩的坑）

生成的骨架里，`maintainer`、`description`、`entry_points` 都已经对了（拜 2.2 那条完整命令所赐 —— `--node-name` 连 `console_scripts` 的注册一起写好了）。

**唯一要手动加的是 `data_files` 里那五行 glob**，因为 `launch/`、`worlds/`、`config/`、`maps/`、`rviz/` 是你自己建的目录，`ros2 pkg create` 不可能预知。

顺便把 `version` 从 `0.0.0` 改成 `0.1.0`（**要和 `package.xml` 里的保持一致**，不一致 colcon 会警告）。

补完之后的完整样子：

```python
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb3_stage1'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ↓↓↓ 只有这五行是你要加的 ↓↓↓
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.pgm') + glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='你的名字',
    maintainer_email='你的邮箱',
    description='第一阶段考核：TurtleBot3 自建世界 SLAM 建图与自主导航',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_navigator = tb3_stage1.waypoint_navigator:main'
        ],
    },
)
```

> **`setup.py` 不是给你执行的脚本。** 在 ROS 2 里它是一份**声明**，由 `colcon build` 读取。
>
> 千万别手动跑 `python3 setup.py install` —— 那会装进系统 Python 的 site-packages，**绕过 ament 索引**。结果是 `ros2 run` 找不到你的节点，但 `python3 -c "import tb3_stage1"` 又成功，制造出极难排查的矛盾状态。

### 为什么要"搬运"：源码树 vs 安装树

先回答一个更根本的问题：为什么这些文件不能待在原地被直接使用？

**因为 ROS 2 有一条硬性的源码树 / 安装树分离。**

```
源码树（你编辑的地方）                    安装树（ROS 运行时唯一读的地方）
~/tb3_nav_stage1/src/tb3_stage1/    ──►  ~/tb3_nav_stage1/install/tb3_stage1/
├── tb3_stage1/*.py                 ──►  ├── lib/python3.10/site-packages/tb3_stage1/
│                                        ├── lib/tb3_stage1/waypoint_navigator  ← 可执行文件
├── package.xml                     ──►  └── share/tb3_stage1/
├── launch/*.launch.py              ──►      ├── package.xml
├── worlds/*.world                  ──►      ├── launch/
├── config/*.yaml                   ──►      ├── worlds/
└── setup.py                                 └── config/
    （只是说明书，本身不搬）
```

**`ros2 launch` 和 `get_package_share_directory()` 从来不看你的源码目录，只看 `install/`。**

自己验证最直观 —— 看看 apt 装的 `turtlebot3_gazebo`：

```bash
ros2 pkg prefix turtlebot3_gazebo
# /opt/ros/humble

ls /opt/ros/humble/share/turtlebot3_gazebo/
# launch  models  rviz  urdf  worlds        ← 和你的包结构一模一样

find /opt/ros/humble -path "*turtlebot3_gazebo*" -name "*.cpp" | wc -l
# 0                                          ← 一行源码都没有
```

**apt 装的包只有安装树、没有源码树，照样能跑。** 你的包必须做到同样的事，因为 `get_package_share_directory('tb3_stage1')` 和 `get_package_share_directory('turtlebot3_gazebo')` 走的是同一套查找逻辑。

这道分离存在的三个理由：

1. **C++ 包必须编译。** `.cpp` 运行时毫无用处，只有编译产物有用。ROS 2 对所有包类型用**同一套**机制，所以 Python 包也走这条路，哪怕 Python 不需要编译
2. **运行时不能依赖源码位置。** 装好后 `install/` 是自包含的，可以打包拷走
3. **统一路径。** 不管包来自 apt 还是你的工作空间，`get_package_share_directory()` 返回的结构必须一致

### 那为什么 `.py` 不用写，`.world` 就要写

因为 **setuptools 只认识 Python**：

| 内容                                                | 谁负责搬                        | 要手写吗             |
| --------------------------------------------------- | ------------------------------- | -------------------- |
| `tb3_stage1/*.py`                                 | `packages=find_packages(...)` | ❌ 自动              |
| 可执行文件                                          | `entry_points`                | ❌ 自动              |
| `.launch.py` / `.world` / `.yaml` / `.rviz` | `data_files`                  | ✅**必须手写** |

注意 `.launch.py` 虽然后缀是 `.py`，但它**不是 Python 模块**（不会被 import），`find_packages` 抓不到它，一样要走 `data_files`。

对 setuptools 来说，一个 `.world` 文件就是项目目录里一个来路不明的普通文件，它没有任何理由认为这东西该被安装。**所以你必须明确声明"这是数据，要装"** —— `data_files` 就是字面意思。

> **这和目录名叫什么无关。** 就算你老老实实叫 `launch/`，不写进 `data_files` 也一样装不进去；反过来叫 `changjing/` 只要声明了也能用。约定俗成的目录名是给人看的，不是给构建系统看的。

### `entry_points`：从 .py 文件到 `ros2 run` 命令

`data_files` 管数据文件，`entry_points` 管**可执行节点**。

```python
'color_detector = tb3_stage1.color_detector:main'
 └── 命令名 ──┘   └─ 模块路径 ─┘ └─ 函数名
```

读作：**创建一个叫 `color_detector` 的可执行文件，它 import `tb3_stage1.color_detector` 模块并调用其中的 `main()`。**

有了它才能跑：

```bash
ros2 run tb3_stage1 color_detector
            └包名┘  └─ 命令名 ─┘
```

三个部分的自由度不同：

| 部分     | 对应什么                             | 能自己起名吗            |
| -------- | ------------------------------------ | ----------------------- |
| 命令名   | `ros2 run tb3_stage1 <这里>`       | ✅ 随便起               |
| 模块路径 | 文件`tb3_stage1/color_detector.py` | ❌ 必须和实际路径一致   |
| 函数名   | 文件里的`def main(...)`            | ❌ 必须和实际函数名一致 |

模块路径用**点号**、不带 `.py` 后缀 —— 那是 Python 的 import 语法：

```
文件系统：  src/tb3_stage1/tb3_stage1/color_detector.py
Python：                  tb3_stage1  .  color_detector
                          └ 包目录 ┘     └ 模块名 ┘
```

**⚠️ 注意两层同名目录**，`.py` 必须放内层：

```
src/tb3_stage1/              ← ROS 包目录（有 setup.py）
├── setup.py
└── tb3_stage1/              ← Python 模块目录（同名，不是笔误）
    ├── __init__.py
    └── color_detector.py    ← 放这里
```

**它实际生成了什么** —— build 之后自己看：

```bash
ls ~/tb3_nav_stage1/install/tb3_stage1/lib/tb3_stage1/
cat ~/tb3_nav_stage1/install/tb3_stage1/lib/tb3_stage1/waypoint_navigator
```

是个几行的胶水脚本：`from tb3_stage1.waypoint_navigator import main; sys.exit(main())`。setuptools 自动生成 —— 这就是"入口点"的字面含义：程序从这里进入。

它装在 `lib/tb3_stage1/` 而不是 `bin/`，是 `setup.cfg` 里那两行重定向的功劳。**`ros2 run` 只在 `lib/<包名>/` 下找可执行文件。**

### 后面每加一个新节点，都走这个流程

第 6、9、10、11 章各要加一个节点，流程完全一样：

```bash
cd ~/tb3_nav_stage1/src/tb3_stage1

ls tb3_stage1/新节点.py                       # ① 文件在内层目录
grep -n "^def main" tb3_stage1/新节点.py      # ② 有 main 函数
# ③ 编辑 setup.py，往 console_scripts 列表加一行（注意行末逗号）
python3 -m py_compile setup.py && echo OK    # ④ 语法检查

cd ~/tb3_nav_stage1
colcon build --symlink-install               # ⑤ 改了 setup.py 必须重新 build
source install/setup.bash
ls install/tb3_stage1/lib/tb3_stage1/        # ⑥ 关键验证：可执行文件生成了吗
ros2 run tb3_stage1 新节点                    # ⑦ 跑
```

**第 ⑥ 步是分水岭**：看不到文件 = 第 ③ 步没生效，不必去 debug 节点代码。

| 症状                                                      | 原因                                                                                        |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| `No executable found`                                   | 忘了重新 build；或`console_scripts` 那行拼错                                              |
| `ModuleNotFoundError: No module named 'tb3_stage1.xxx'` | `.py` 放错层了（在外层而非内层目录）                                                      |
| `cannot import name 'main'`                             | 文件里没有`def main()`，或拼成 `def Main()`                                             |
| 改了代码不生效                                            | 改`.py` 内容有 `--symlink-install` 会立即生效；**改 `setup.py` 必须重新 build** |

### 这几行分别在干什么

`data_files` 的每个元组就是**一条搬运指令**：`(目标目录, [要搬的源文件列表])`。

```python
(os.path.join('share', package_name, 'launch'),  glob('launch/*.launch.py'))
 └────────── 搬到哪 ──────────────────────────┘  └──── 搬什么 ────────┘
        'share/tb3_stage1/launch'                  ['launch/x.launch.py', ...]
```

**目标目录相对于 `install/tb3_stage1/`**，所以最终落在 `install/tb3_stage1/share/tb3_stage1/launch/`。

为什么要 `share/包名/` 套两层？因为这是 ament 的约定 —— `get_package_share_directory('tb3_stage1')` 返回的正是 `install/tb3_stage1/share/tb3_stage1`。你 launch 文件里那句 `get_package_share_directory(...)` 能找到 world 文件，全靠这个约定成立。

**两行 import 各服务于元组的一半：**

| import                    | 用在哪                         | 作用                 |
| ------------------------- | ------------------------------ | -------------------- |
| `import os`             | `os.path.join(...)`          | 拼**目标**路径 |
| `from glob import glob` | `glob('launch/*.launch.py')` | 找**源**文件   |

两个都不是非用不可：

- `os.path.join('share', package_name, 'launch')` 在 Linux 上等价于直接写 `'share/tb3_stage1/launch'`。用它是习惯 —— 跨平台，且复用 `package_name` 变量，改包名时只改一处
- `glob(...)` 可以换成手写文件列表。但那样**每加一个 launch 文件都要回来改 `setup.py`**，迟早会忘

五行的对应关系：

| 行     | 源                               | 目标                         | 什么时候才有内容 |
| ------ | -------------------------------- | ---------------------------- | ---------------- |
| launch | `launch/*.launch.py`           | `share/tb3_stage1/launch/` | 第 3 章起        |
| worlds | `worlds/*.world`               | `share/tb3_stage1/worlds/` | 第 3 章          |
| config | `config/*.yaml`                | `share/tb3_stage1/config/` | 第 4 章起        |
| maps   | `maps/*.pgm` + `maps/*.yaml` | `share/tb3_stage1/maps/`   | 第 4 章末        |
| rviz   | `rviz/*.rviz`                  | `share/tb3_stage1/rviz/`   | 第 4 章          |

刚加完时这五行全部匹配为空，**这是正常的**。空列表不报错，只是什么都不搬。

### 三个安装机制的分工

**`setup.py` 里其实有三个互相独立的安装机制**，`data_files` 只是其中一个：

| 参数                            | 管什么                                                   | 装到哪                                                          |
| ------------------------------- | -------------------------------------------------------- | --------------------------------------------------------------- |
| `packages=find_packages(...)` | **Python 模块**                                    | `install/tb3_stage1/lib/python3.10/site-packages/tb3_stage1/` |
| `entry_points=`               | **可执行入口脚本**                                 | `install/tb3_stage1/lib/tb3_stage1/waypoint_navigator`        |
| `data_files=`                 | **其他一切**（launch / world / yaml / map / rviz） | `install/tb3_stage1/share/tb3_stage1/...`                     |

所以不是"Python 代码不搬、其他要搬"，而是**Python 有人管、其他没人管**。自己看一眼：

```bash
cd ~/tb3_nav_stage1
find install/tb3_stage1 -type f -o -type l | grep -v ament_index | sort
cat src/tb3_stage1/setup.cfg
```

Python 模块躺在 `lib/python3.10/site-packages/` 下 —— 你没在 `data_files` 里写它，它照样进去了。而 `setup.cfg`（那个"不用动"的文件）干的是一件很具体的事：把 `entry_points` 生成的可执行文件重定向到 `lib/<包名>/` 而不是默认的 `bin/`。**这就是 `ros2 run tb3_stage1 waypoint_navigator` 能找到它的原因** —— `ros2 run` 只在 `lib/<包名>/` 里找。

**为什么非 Python 文件要手工声明？不是因为目录名是自定义的。** 不管你叫 `worlds/` 还是 `sdf_files/`，setuptools 都一样不认，没有一份"受祝福的目录名"清单。

真正的原因是历史性的：**setuptools 是给 Python 库设计的**。它知道 `.py` 该装进 site-packages，但完全不知道 `.world` 是什么、该放哪。ROS 2 选择复用 setuptools 而不是自己造轮子，代价就是非 Python 资源全得手工声明。（`ament_cmake` 包在 `CMakeLists.txt` 里写 `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})`，一回事，只是语法不同。）

### 为什么要有 install 这一层

**① 让运行时只依赖一个稳定位置。** 对比一下：

```
apt 装的：  /opt/ros/humble/share/turtlebot3_gazebo/models/
你构建的：  ~/tb3_nav_stage1/install/tb3_stage1/share/tb3_stage1/worlds/
```

**布局完全相同。** 所以第 3 章那个 launch 文件里，`get_package_share_directory('gazebo_ros')` 和 `get_package_share_directory('tb3_stage1')` 走的是同一套查找逻辑，尽管一个是 apt 装的、一个是你刚编的。代码不需要知道区别。

**② 源码布局是你的自由，安装布局是公共契约。** install 目录是**接口**，src 目录是**实现**。

**③ 构建产物和源码隔离。** `rm -rf build install log` 随时可以重来，源码一根汗毛不动 —— 这也是它们进 `.gitignore` 的原因。

### 完整的图

```
~/tb3_nav_stage1/
├── src/tb3_stage1/                    ← 你写的（Git 管这里）
│   ├── tb3_stage1/*.py    ──packages=──────┐
│   ├── launch/*.launch.py ──data_files=──┐ │
│   ├── worlds/*.world     ──data_files=──┤ │
│   ├── config/*.yaml      ──data_files=──┤ │
│   └── setup.py           （搬运规则）    │ │
│                                          │ │
├── build/                                 │ │  ← 中间产物，.gitignore
└── install/tb3_stage1/     ←──────────────┴─┘  ← colcon 生成，.gitignore
    ├── share/tb3_stage1/
    │   ├── launch/  worlds/  config/  maps/  rviz/
    │   └── package.xml
    └── lib/
        ├── python3.10/site-packages/tb3_stage1/   ← Python 模块
        └── tb3_stage1/waypoint_navigator          ← 可执行入口
```

**`ros2 launch` / `ros2 run` 只看 `install/`，从不看 `src/`。** 这就是为什么改了 `setup.py` 必须重新 build —— 搬运规则变了，得重新搬一次。

> `--symlink-install` 是个优化：install 里放的是指向 src 的符号链接而不是副本，所以**改**文件内容立刻生效。但**新增**文件仍要重新 build，因为 `glob()` 是在 build 那一刻执行的。

`data_files` 里没写的文件类型，**根本不会被安装**。所以你会遇到经典的：源码目录里明明有 `my_world.world`，但 launch 时报 `file not found`。

新手最常见的三种表现：

1. 新建了一个 `.world` 文件 → 它已在 glob 范围内，但**没有重新 `colcon build`** → 找不到
2. 加了一个新的文件类型（比如 `.sdf`）→ glob 没覆盖 → 永远找不到
3. 改了 `setup.py` → **必须重新 build**，`--symlink-install` 对 `data_files` 的新增项不生效

这也解释了 `--symlink-install` 的行为：它把"复制"换成"建符号链接"，所以**改**已有文件的内容立刻生效（链接指向源文件），但**新增**一个文件必须重新 build（链接还不存在）。

> **⚠️ glob 写错不会报错，只会静默漏文件。**
>
> 举个真实的例子：`glob('maps/*.[py][gs]*')` 看起来能同时匹配 `.pgm` 和 `.yaml`，实际不能——
>
> ```
> maps/*.[py][gs]*   ->  ['maps/stage1_map.pgm']              # .yaml 漏了
> maps/*.pgm+*.yaml  ->  ['maps/stage1_map.pgm', '...yaml']   # 正确
> ```
>
> 因为 `[gs]` 要求第二个字符是 g 或 s，而 `.yaml` 第二个是 a。结果 `.pgm` 装进去了、`.yaml` 没有，而 Nav2 加载地图要的恰恰是 `.yaml`——你要到第 5 章才会发现。
>
> **写不确定的 glob 时，先在 Python 里试一下再往 `setup.py` 里放。** 宁可写两个明确的 glob，也不要写一个自作聪明的。

### 改完怎么验证：四层，从便宜到贵

`setup.py` 不能直接执行安装，但查错完全有安全办法。**每一层抓的错误类型不同，别只用最后一层。**

**① 纯语法检查（不执行任何代码，最安全，30 毫秒）**

```bash
cd ~/tb3_nav_stage1/src/tb3_stage1
python3 -m py_compile setup.py && echo "语法 OK"
```

有错会精确指到行列：

```
  File "setup.py", line 8
    packages=find_packages(exclude=['test']),
            ^
SyntaxError: positional argument follows keyword argument
```

**② 检查能否真正执行完**

```bash
python3 setup.py --version
```

这个**会**从头执行 `setup.py`，但 `--version` 让 setuptools 打印元数据就退出，**不安装任何东西**，是安全的。输出应该是 `0.1.0`。

它能抓到 `py_compile` 抓不到的错 —— 变量名拼错、`os` 忘了 import 这种"语法合法但运行时炸"的问题。

**③ 验证 glob 到底匹配到了什么**（最容易出隐性错误的一层）

```bash
python3 -c "
from glob import glob
print('launch ->', glob('launch/*.launch.py'))
print('worlds ->', glob('worlds/*.world'))
print('config ->', glob('config/*.yaml'))
print('maps   ->', glob('maps/*.pgm') + glob('maps/*.yaml'))
print('rviz   ->', glob('rviz/*.rviz'))
"
```

**这一层专治上面那个 `[py][gs]*` 的 bug** —— 语法合法、执行正常，只是少匹配了一种扩展名。前两层都查不出来。

**④ 最终验证：构建 + 看产物**

```bash
cd ~/tb3_nav_stage1
colcon build --symlink-install
ls -R install/tb3_stage1/share/tb3_stage1/
```

看不到你的文件 = 没装进去，不是 launch 写错了。

> `colcon build` 本身也会报语法错误，但淹没在一堆构建输出里。前三层的价值是**在 30 毫秒内定位问题，而不是等一次完整构建**。这个"用最便宜的手段先排除最常见的错"的思路，后面调 SLAM 和 Nav2 参数时同样适用。

## 2.5 首次构建

```bash
cd ~/tb3_nav_stage1
colcon build --symlink-install
source install/setup.bash
```

把这个 source 也加进 `.bashrc`：

```bash
echo "source ~/tb3_nav_stage1/install/setup.bash" >> ~/.bashrc
```

> **source 的顺序有讲究**：先 `/opt/ros/humble`（底层），再 `tb3_nav_stage1`（你的工作空间）。后 source 的会**覆盖**先 source 的同名包，这叫 overlay（覆盖层）机制。
>
> 如果将来确实需要 clone 上游包来覆盖（第 1.3 节说明了什么时候才有必要），就 source 在这两者中间。不过第 8 章的相机是在自己包里做派生模型，不需要这一层。

## 2.6 首次提交

```bash
cd ~/tb3_nav_stage1
mkdir -p docs
cat > docs/logbook.md << 'EOF'
# 实验日志

格式：问题 → 根因 → 解决 → 教训

---
EOF

git add .
git commit -m "chore: 初始化 ROS2 功能包骨架"
git push -u origin main
```

**从现在开始，每完成一个子任务就提交一次**，commit message 写清楚做了什么。考核要点里有"Git 与实验记录习惯是否养成"，你的 commit history 本身就是证据。

## ✅ 第 2 章验收标准

- [ ] GitHub 上有 private repo，`git push` 成功
- [ ] `ros2 pkg list | grep tb3_stage1` 有输出
- [ ] `ls install/tb3_stage1/share/tb3_stage1/` 能看到 launch/ worlds/ config/ maps/
- [ ] `.gitignore` 排除了 build/ install/ log/

---

# 第 3 章 自定义仿真世界（子任务 1）

任务要求：**至少 2 个房间、至少 3 个不同尺寸障碍物、世界封闭、使用 TurtleBot3**。

## 3.1 SDF 是什么

SDF（Simulation Description Format）是 Gazebo 描述世界的 XML 格式。核心层级：

```
<world>              一个仿真世界
  └── <model>        一个物体（墙、箱子、机器人）
       └── <link>    物体的一个刚体部件
            ├── <collision>   碰撞体 —— 物理引擎用，决定"撞不撞得到"
            ├── <visual>      视觉体 —— 只管显示，激光雷达看不见它
            └── <inertial>    质量和转动惯量（static 模型不需要）
```

**`<collision>` 和 `<visual>` 必须理解的区别**：

- 激光雷达是通过和 `<collision>` 求交来测距的
- `<visual>` 只影响 3D 窗口里的显示

所以：**只写 visual 不写 collision，你在 Gazebo 里能看见墙，但激光会直接穿过去，SLAM 建出来的图是空的。** 这是"我明明建了墙但地图上没有"的标准答案。

`<static>true</static>` 表示这个物体固定不动，物理引擎不去计算它的动力学。所有墙和障碍物都该设为 static —— 省算力，也防止被机器人撞飞。

## 3.2 世界布局设计

```
        y
        ↑
  +3 ┌──────────────┬──────────────┐
     │              │              │
     │   房间 A     │   房间 B     │
     │              ▓              │   ▓ = 门洞 (宽 0.8 m)
     │    ○         │        □     │
     │              │              │
  -3 └──────────────┴──────────────┘ → x
    -4              0              +4
```

- 外围 8 m × 6 m 完全封闭
- 中间隔墙在 x=0，留一个 0.8 m 宽门洞（burger 直径仅 0.178 m，绰绰有余）
- 4 个不同尺寸障碍物（任务要求 ≥3，多放一个让 SLAM 的特征更丰富）

**为什么门洞开在 y≈1.0 而不是正中间？** 让两个房间不对称。完全对称的环境会让扫描匹配产生歧义（算法分不清自己在 A 房还是 B 房），这是 SLAM 里真实存在的问题。不对称布局能让你的地图更干净。

## 3.3 写世界文件

### 手写还是 GUI 画

Gazebo Classic 自带两个编辑器，世界文件确实可以全程用鼠标做出来：

- **Building Editor**（Edit → Building Editor）：在 2D 平面图上拉墙、开门窗，下方实时 3D 预览。画两个房间加一道门比手写快得多
- **Model Editor**（Edit → Model Editor）：用基本几何体拼模型
- 主界面工具栏可以直接插入 box / sphere / cylinder，拖动和缩放
- 最后 File → Save World As 导出 SDF

**本教程仍然手写，两个理由：**

1. **手写一遍你才会理解 `<collision>` 和 `<visual>` 的区别** —— 这是第 3 章最容易踩的坑（"能看见墙但激光穿过去"）。GUI 自动帮你生成两者，你就学不到这个机制
2. **GUI 导出的 SDF 会丢注释、重排结构，diff 很难看。** 我们的场景只有 6 面墙 + 4 个障碍物，手写文件干净可读，改布局就是改几个数字，还能写注释解释为什么这么放

这是建议不是规定。等你理解了机制，做更复杂的场景时用 GUI 提速完全合理。

> **⚠️ 如果你要走 GUI 路线，先知道这个坑：**
>
> Building Editor 默认把模型存到 **`~/building_editor_models/`**，而这个路径**不在 `GAZEBO_MODEL_PATH` 里**。于是你保存的 world 里那句 `<include><uri>model://你的建筑</uri></include>` 会解析失败。
>
> 两种解法：
>
> ```bash
> # 解法 A：把模型搬进你的包，跟着 Git 一起走（推荐）
> mkdir -p ~/tb3_nav_stage1/src/tb3_stage1/models
> cp -r ~/building_editor_models/你的建筑 ~/tb3_nav_stage1/src/tb3_stage1/models/
> # 然后 setup.py 的 data_files 里加一行 glob 装 models/
> # 并在 .bashrc 的 GAZEBO_MODEL_PATH 后面追加这个目录
>
> # 解法 B：直接把默认目录加进搜索路径（快，但模型不进版本控制）
> export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/building_editor_models
> ```
>
> 解法 A 更正确 —— 别人 clone 你的仓库才能复现。**注意这正是"可复现性"的一个具体体现：模型放在 home 目录下等于没提交。**
>
> 顺带一提，因为你在 1.4 节设了 `GAZEBO_MODEL_DATABASE_URI=""`，这个坑会以"找不到 model://xxx"的明确报错出现，而不是静默卡死几分钟。这就是那个配置的价值。

```bash
cd ~/tb3_nav_stage1/src/tb3_stage1/worlds
```

创建 `stage1_world.world`：

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="stage1_world">

    <!-- 光源与地面：Gazebo 自带模型 -->
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- 物理引擎参数 -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- 关阴影，弱显卡上能明显提速 -->
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <shadows>false</shadows>
    </scene>

    <!-- ================= 墙体 ================= -->
    <!-- 全部墙放在一个 model 里：静态几何体不需要拆分 -->
    <model name="arena_walls">
      <static>true</static>
      <link name="walls">

        <!-- 北墙 y=+3 -->
        <collision name="north_col">
          <pose>0 3 0.5 0 0 0</pose>
          <geometry><box><size>8.15 0.15 1.0</size></box></geometry>
        </collision>
        <visual name="north_vis">
          <pose>0 3 0.5 0 0 0</pose>
          <geometry><box><size>8.15 0.15 1.0</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script></material>
        </visual>

        <!-- 南墙 y=-3 -->
        <collision name="south_col">
          <pose>0 -3 0.5 0 0 0</pose>
          <geometry><box><size>8.15 0.15 1.0</size></box></geometry>
        </collision>
        <visual name="south_vis">
          <pose>0 -3 0.5 0 0 0</pose>
          <geometry><box><size>8.15 0.15 1.0</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script></material>
        </visual>

        <!-- 东墙 x=+4 -->
        <collision name="east_col">
          <pose>4 0 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry>
        </collision>
        <visual name="east_vis">
          <pose>4 0 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script></material>
        </visual>

        <!-- 西墙 x=-4 -->
        <collision name="west_col">
          <pose>-4 0 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry>
        </collision>
        <visual name="west_vis">
          <pose>-4 0 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script></material>
        </visual>

        <!-- 隔墙下段：y 从 -3 到 +0.6，长 3.6，中心 y=-1.2 -->
        <collision name="divider_lower_col">
          <pose>0 -1.2 0.5 0 0 0</pose>
          <geometry><box><size>0.15 3.6 1.0</size></box></geometry>
        </collision>
        <visual name="divider_lower_vis">
          <pose>0 -1.2 0.5 0 0 0</pose>
          <geometry><box><size>0.15 3.6 1.0</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script></material>
        </visual>

        <!-- 隔墙上段：y 从 +1.4 到 +3，长 1.6，中心 y=+2.2 -->
        <!-- 于是门洞在 y ∈ [0.6, 1.4]，宽 0.8 m -->
        <collision name="divider_upper_col">
          <pose>0 2.2 0.5 0 0 0</pose>
          <geometry><box><size>0.15 1.6 1.0</size></box></geometry>
        </collision>
        <visual name="divider_upper_vis">
          <pose>0 2.2 0.5 0 0 0</pose>
          <geometry><box><size>0.15 1.6 1.0</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script></material>
        </visual>

      </link>
    </model>

    <!-- ================= 障碍物 ================= -->

    <!-- 大立方体 0.6 m，房间 A -->
    <model name="obstacle_box_large">
      <static>true</static>
      <pose>-2.5 1.5 0.3 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>0.6 0.6 0.6</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>0.6 0.6 0.6</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
          </script></material>
        </visual>
      </link>
    </model>

    <!-- 高圆柱 r=0.25 h=1.0，房间 A -->
    <model name="obstacle_cylinder_tall">
      <static>true</static>
      <pose>-1.6 -1.6 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><cylinder><radius>0.25</radius><length>1.0</length></cylinder></geometry>
        </collision>
        <visual name="vis">
          <geometry><cylinder><radius>0.25</radius><length>1.0</length></cylinder></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Blue</name>
          </script></material>
        </visual>
      </link>
    </model>

    <!-- 小立方体 0.35 m，房间 B -->
    <model name="obstacle_box_small">
      <static>true</static>
      <pose>2.0 -1.5 0.175 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>0.35 0.35 0.35</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>0.35 0.35 0.35</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Green</name>
          </script></material>
        </visual>
      </link>
    </model>

    <!-- 矮圆柱 r=0.18 h=0.6，房间 B -->
    <model name="obstacle_cylinder_small">
      <static>true</static>
      <pose>2.6 1.6 0.3 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><cylinder><radius>0.18</radius><length>0.6</length></cylinder></geometry>
        </collision>
        <visual name="vis">
          <geometry><cylinder><radius>0.18</radius><length>0.6</length></cylinder></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Yellow</name>
          </script></material>
        </visual>
      </link>
    </model>

    <!-- 相机初始视角 -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 -12 10 0 0.7 1.5708</pose>
      </camera>
    </gui>

  </world>
</sdf>
```

**关键点说明**：

- **障碍物高度必须 > 激光雷达高度**。burger 的雷达装在约 0.18 m 处。所有障碍物最矮的是 0.35 m 立方体（顶面 0.35 m），高于雷达平面 —— 能被扫到。如果你放一个 0.1 m 的矮块，激光会从它上面飞过去，SLAM 完全看不见，但机器人会撞上。这是个很隐蔽的坑。
- **`pose` 的 z 坐标是几何中心**。0.6 m 的立方体要放在地面上，z 必须是 0.3；0.35 m 的立方体 z = 0.175。写成 0 的话物体一半会埋进地里。
- **`max_step_size` / `real_time_update_rate`**：如果你的机器跑不动（RTF 明显低于 1.0），把它们改成 `0.002` / `500`，物理迭代次数减半。代价是物理精度略降，对差速小车影响可以忽略。

## 3.4 写 launch 文件

创建 `launch/stage1_world.launch.py`：

```python
#!/usr/bin/env python3
"""启动自定义世界 + 生成 TurtleBot3。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_stage1 = get_package_share_directory('tb3_stage1')

    world_path = os.path.join(pkg_stage1, 'worlds', 'stage1_world.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-3.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')

    # 1) gzserver：物理引擎本体。必须挂 init/factory/force_system 插件，
    #    否则 /clock 不发布、spawn_entity 无法生成模型。
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items(),
    )

    # 2) gzclient：3D 图形界面。可以关掉以节省资源。
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    # 3) robot_state_publisher：读 URDF，发布机器人自身的 TF
    #    （base_footprint → base_link → base_scan ...）
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 4) spawn：把 TurtleBot3 的 model.sdf 塞进已经在跑的 Gazebo 里
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='使用 Gazebo 仿真时间'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 图形界面'),
        DeclareLaunchArgument('x_pose', default_value='-3.0',
                              description='机器人初始 x'),
        DeclareLaunchArgument('y_pose', default_value='-2.0',
                              description='机器人初始 y'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_tb3,
    ])
```

**这个 launch 文件复用了 `turtlebot3_gazebo` 的两个子 launch 文件**（`robot_state_publisher.launch.py` 和 `spawn_turtlebot3.launch.py`），只把世界换成你自己的。这样能最大限度减少出错面。

**先验证这两个子 launch 文件确实存在**：

```bash
ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/
```

应该看到 `robot_state_publisher.launch.py`、`spawn_turtlebot3.launch.py`、`turtlebot3_world.launch.py` 等。如果文件名不一样，把 `ls` 的输出发给我。

## 3.5 构建并运行

```bash
cd ~/tb3_nav_stage1
colcon build --symlink-install
source install/setup.bash

# 先确认世界文件真的被安装了
ls install/tb3_stage1/share/tb3_stage1/worlds/

ros2 launch tb3_stage1 stage1_world.launch.py
```

资源紧张时用无头模式（不开 3D 窗口，只跑物理）：

```bash
ros2 launch tb3_stage1 stage1_world.launch.py gui:=false
```

## 3.6 验证：这才是子任务 1 的真正考核点

任务书写的是"传感器是否正常发布数据"，所以**眼睛看到机器人不算完成**。逐条验证：

```bash
# 1) 话题都在吗
ros2 topic list | sort

# 2) 激光雷达在发数据吗（约 5 Hz）
ros2 topic hz /scan

# 3) 激光数据合理吗
ros2 topic echo /scan --once | head -20
#    检查 range_min / range_max（burger 是 0.12 / 3.5）
#    检查 ranges 数组里不全是 inf —— 全是 inf 说明周围没有 collision 体

# 4) 里程计在发吗（约 30 Hz）
ros2 topic hz /odom

# 5) 仿真时钟
ros2 topic hz /clock

# 6) TF 树完整吗
ros2 run tf2_tools view_frames -o tf_tree && xdg-open tf_tree.pdf
#    ⚠️ 必须加 -o。不加时输出文件名会带时间戳（frames_2026-09-01_17.19.06.pdf），
#       而日志却打印 "Generating graph in frames.pdf" —— 上游文本和实际文件名不一致。
#    ⚠️ 只生成了 .gv 没有 .pdf = graphviz 没装：sudo apt install -y graphviz

# 7) 遥控测试封闭性：开着到处撞，确认出不去
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
```

**第 3 条是关键**：`ranges` 数组里如果全是 `inf`，说明激光什么都没打到 —— 十有八九是你只写了 `<visual>` 忘了 `<collision>`。

RViz 里可视化确认（**推荐，而且比开 Gazebo 3D 窗口更可靠**）：

```bash
rviz2 --ros-args -p use_sim_time:=true
```

在 RViz 里：把 **Fixed Frame** 设为 `odom` → **Add** → **LaserScan** → topic 选 `/scan`。

### ⭐ Decay Time：让稀疏的激光点变成一张点云地图

burger 的雷达一帧只有 360 个点，单帧看起来又稀又抽象。**一个设置就能彻底改变这一点：**

展开左侧的 **LaserScan** 项 → 把 **Decay Time** 从 `0` 改成 **`60`**。

含义是"每帧激光点在屏幕上保留 60 秒再消失"。配合 Fixed Frame = `odom`（点停在世界坐标里，不跟着机器人跑），效果是：

**遥控走一圈，激光点不断累积，逐渐把整个房间的轮廓"画"出来** —— 从 360 个孤零零的点变成密密麻麻的点云。墙、门洞、四个障碍物的形状会非常清楚。

> 这其实就是 SLAM 建图的直观预演。第 4 章 slam_toolbox 做的事，本质上就是把这些累积的点对齐得更准，再转成栅格地图。先在这里建立直觉，第 4 章会顺很多。

其余几项调整：

| 设置                                    | 改成                            | 作用                 |
| --------------------------------------- | ------------------------------- | -------------------- |
| LaserScan →**Size (m)**          | `0.05`                        | 点更大，看得清       |
| LaserScan →**Style**             | `Spheres` 或 `Flat Squares` | 比默认的 Points 醒目 |
| LaserScan →**Color Transformer** | `FlatColor` + 亮色            | 对比度更高           |

再 **Add** 三个显示项：

1. **Grid** —— **Cell Size** = `1`，**Plane Cell Count** = `10`。1 米刻度参照网格，一眼读出房间尺寸对不对（应该是 8×6 米）
2. **RobotModel** —— Description Topic 填 `/robot_description`，看到机器人本体
3. **Odometry** —— Topic 选 `/odom`，**Keep** 设 `100`。画出走过的轨迹，验证封闭性时特别直观 —— 轨迹被墙挡住的地方一目了然

配完 **File → Save Config As** 存到 `~/tb3_nav_stage1/src/tb3_stage1/rviz/stage1.rviz`。

> **为什么 RViz 的激光点比 Gazebo 的 3D 画面更能证明世界是对的？**
>
> Gazebo 画面证明的是"视觉上有堵墙"（`<visual>` 存在）。RViz 里的激光点证明的是"激光真的打到了实体"（`<collision>` 存在且位置正确）。**后者才是 SLAM 和导航实际依赖的东西。**
>
> 这也是为什么 3.6 节的验收标准里没有任何一条是"Gazebo 窗口里能看到房间"。

> **如果 Gazebo 3D 窗口卡死或拖不动**，那是 gzclient 的渲染问题，和你的世界无关。用 `gui:=false` 跑无头模式，全程用 RViz 看 —— 验收标准一条都不受影响。详见第 1 章"专条：gzclient 卡死 / 拖不动视角"。

## ✅ 第 3 章验收标准

- [ ] 世界有 2 个由隔墙分开的房间，中间有门洞可通行
- [ ] 至少 3 个不同尺寸的障碍物，全部高于 0.2 m
- [ ] 遥控机器人撞遍四周，出不去世界
- [ ] `/scan` ≈ 5 Hz，`ranges` 不全是 inf
- [ ] `/odom` ≈ 30 Hz，`/clock` 有数据
- [ ] TF 树完整，`odom → base_footprint → base_link → base_scan` 无断裂
- [ ] RViz 里能看到激光点云勾勒出房间轮廓
- [ ] `git commit` 并推送

## 🔧 第 3 章常见坑

| 现象                              | 根因                                        | 解决                                                                                 |
| --------------------------------- | ------------------------------------------- | ------------------------------------------------------------------------------------ |
| 世界空白，只有地面和机器人        | SDF 语法错误，Gazebo 静默跳过               | `gz sdf -k stage1_world.world` 校验（Classic 里是 `gzsdf`）；看终端 `[Err]` 行 |
| 报`file not found` 找不到 world | 没重新 build，或`setup.py` 的 glob 没覆盖 | `ls install/.../worlds/` 确认；重新 `colcon build`                               |
| 能看见墙但激光穿过去              | 只写了`<visual>` 没写 `<collision>`     | 每面墙都要成对写                                                                     |
| 机器人生成后掉进地里 / 弹飞       | 初始位姿在墙体内部                          | 改`x_pose`/`y_pose` 到空地                                                       |
| 障碍物一半埋进地面                | `pose` 的 z 没设成高度的一半              | box 高 h → z = h/2                                                                  |
| 机器人能穿墙出去                  | 墙有缝，或墙太矮                            | 检查各段墙的起止坐标是否首尾相接                                                     |
| RTF 远低于 1.0，仿真很慢          | 机器性能不足                                | `gui:=false`；`shadows` 设 false；调大 `max_step_size`                         |

---

# 第 4 章 SLAM 建图（子任务 2）

## 4.1 slam_toolbox 在干什么

它订阅 `/scan` 和 TF 里的 `odom → base_footprint`，做三件事：

1. **扫描匹配**：新一帧激光和已有子图对齐，估计当前位姿
2. **构建位姿图**：把每个关键帧作为节点，帧间约束作为边
3. **回环检测 + 图优化**：认出走过的地方，加一条回环边，然后全局优化整张图

输出：`/map`（`nav_msgs/OccupancyGrid`）+ TF 的 `map → odom`。

**`map → odom` 这段变换的物理含义**：里程计（`odom → base_footprint`）是连续但会漂移的；SLAM 算出的位姿是准确但会跳变的。`map → odom` 这段就是"里程计漂了多少"的修正量。这个设计的好处是下游节点始终能拿到一个连续的 `odom` 参考系。

## 4.2 SLAM 参数配置

**先抄模板，别手打。** `slam_toolbox` 自带 5 份参数模板：

```bash
ls $(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/config/
# mapper_params_lifelong.yaml      长期建图
# mapper_params_localization.yaml  用已有图做定位
# mapper_params_offline.yaml       离线处理 rosbag
# mapper_params_online_async.yaml  ← 我们要的：在线异步建图
# mapper_params_online_sync.yaml   在线同步建图
```

复制过来当基线：

```bash
cp $(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/config/mapper_params_online_async.yaml \
   ~/tb3_nav_stage1/src/tb3_stage1/config/slam_params.yaml
```

然后改成下面这份。**建议用 `diff` 看看你改了什么** —— 这个 diff 本身就是实验报告里"参数选择与依据"一节的素材：

```bash
diff $(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/config/mapper_params_online_async.yaml \
     ~/tb3_nav_stage1/src/tb3_stage1/config/slam_params.yaml
```

最终的 `config/slam_params.yaml`：

```yaml
slam_toolbox:
  ros__parameters:
    # ---------- 坐标系 ----------
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_sim_time: true          # 仿真中必须为 true

    mode: mapping               # mapping = 建图；localization = 用已有图定位

    # ---------- 地图 ----------
    resolution: 0.05            # 每个栅格 5 cm。太小→内存爆炸、太大→细节丢失
    map_update_interval: 2.0    # 每 2 秒更新一次 /map（越小越卡）
    max_laser_range: 3.5        # burger 的 LDS-01 有效量程就是 3.5 m
    transform_publish_period: 0.02
    transform_timeout: 0.2
    tf_buffer_duration: 30.0

    # ---------- 关键帧触发 ----------
    # 走多远 / 转多少角度才插入一个新关键帧
    minimum_travel_distance: 0.2      # 单位 m
    minimum_travel_heading: 0.2       # 单位 rad (约 11.5°)
    scan_buffer_size: 20
    scan_buffer_maximum_scan_distance: 10.0

    # ---------- 回环检测（地图质量的关键）----------
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 8
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # ---------- 扫描匹配 ----------
    use_scan_matching: true
    use_scan_barycenter: true
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # ---------- 优化器 ----------
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None
```

**你最可能需要调的三个参数**：

| 参数                             | 调小                           | 调大                               |
| -------------------------------- | ------------------------------ | ---------------------------------- |
| `resolution`                   | 地图更细，内存和计算涨         | 地图粗糙，墙变厚                   |
| `minimum_travel_distance`      | 关键帧更密，地图更准，计算量大 | 关键帧稀疏，容易漏细节             |
| `loop_search_maximum_distance` | 回环更保守，漏检               | 回环更激进，可能误匹配导致地图撕裂 |

`max_laser_range` **不要设得比雷达实际量程大**。设成 10 而实际只有 3.5，算法会把 3.5 m 处的"无返回"当成"这里是空的"，在地图上刷出虚假的空白区域。

## 4.3 建图 launch 文件

创建 `launch/slam.launch.py`：

```python
#!/usr/bin/env python3
"""在自定义世界中运行 slam_toolbox 建图。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_stage1 = get_package_share_directory('tb3_stage1')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params = os.path.join(pkg_stage1, 'config', 'slam_params.yaml')

    # 启动世界 + 机器人
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stage1, 'launch', 'stage1_world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # slam_toolbox 异步在线建图节点
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        world,
        slam,
        rviz,
    ])
```

> **`online_async` vs `online_sync`**：async 版本不阻塞等待优化完成，实时性好、机器人可以一直跑；sync 版本每帧都等优化，精度略高但会掉帧。小场景 + 弱机器选 async。

## 4.4 开始建图

```bash
cd ~/tb3_nav_stage1 && colcon build --symlink-install && source install/setup.bash

# 终端 1
ros2 launch tb3_stage1 slam.launch.py

# 终端 2：键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
```

**RViz 配置**（第一次要手动配，配好后 File → Save Config As 存到 `rviz/slam.rviz`）：

1. **Fixed Frame** → `map`
2. Add → **Map**，topic `/map`
3. Add → **LaserScan**，topic `/scan`
4. Add → **TF**（看坐标系，确认 `map` 出现了）
5. Add → **RobotModel**，Description Topic `/robot_description`

## 4.5 怎么跑才能建出好图（这一步决定成败）

这是整个子任务 2 里最需要"手感"的部分。原则：

**① 慢。** 快速移动 = 扫描匹配失败 = 地图错位。线速度控制在 **0.1~0.15 m/s**，角速度 **0.3 rad/s** 以内。teleop 里按 `x` 减小线速度、`c` 减小角速度。

**② 转弯要特别慢。** 旋转时激光观测变化最剧烈，是匹配最容易失败的时刻。原地急转是"鬼影墙"的头号成因。

**③ 沿着墙走一圈，而不是在中间乱转。** 让激光始终能看到足够多的结构特征。

**④ 一定要多绕几圈，回到起点。** 这是触发回环检测的唯一方式。建议路线：

```
起点(-3,-2) → 沿房间A的墙走一圈 → 回到起点附近
            → 穿过门洞(0, 1.0) → 沿房间B的墙走一圈
            → 穿回房间A → 再沿墙走一圈 → 回到起点
```

**⑤ 边走边看 RViz。** 发现墙开始"重影"或者歪斜，说明刚才那段匹配失败了 —— 倒回去慢慢重走一遍那段，回环通常能修正回来。

### ⭐ 怎么知道自己在哪、怎么回到起点

**关键认识：`odom` 坐标系的原点就钉在机器人启动那一刻的脚下。** 所以"回到起点" = "让 `odom` 坐标回到 (0, 0)"。（注意区分：机器人在**世界坐标**里生成于 `(-3.0, -2.0)`，但在 `odom` 系里起点永远是原点。）

**① 终端实时打印相对起点的位置**

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

```
- Translation: [1.245, -0.331, 0.010]     ← 前两个数就是相对起点的 x, y
- Rotation: in RPY (radian) [0.0, 0.0, 1.571]
```

**② 看 SLAM 认为你在哪**

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

这是**算法估计的位置**。它和 ① 的差值就是 SLAM 修正掉的漂移量 —— 建图时对照着看，能直观感受到漂移正在发生。

**③ RViz 里画出轨迹（回起点最靠谱的办法）**

| 显示项             | 设置                                                                                 | 作用                                                     |
| ------------------ | ------------------------------------------------------------------------------------ | -------------------------------------------------------- |
| **Odometry** | Topic`/odom`，**Keep** = `500`，Shaft Length `0.1`，Head Length `0.05` | **画出走过的完整轨迹**，顺着面包屑倒着开就能回起点 |
| **TF**       | 展开 Frames，只勾`odom` 和 `base_footprint`                                      | `odom` 那组坐标轴**就钉在起点上**，是明确的靶心  |

（TF 默认画出所有坐标系会很乱，务必手动取消勾选不需要的。）

### ⭐ 驾驶策略：贴墙走，不要乱逛

"回不到起点"几乎都是因为在房间里随机游走。改成**右手贴墙法**：

```
起点(0,0) → 始终保持右手边是墙 → 沿墙走满一圈 → 自然回到起点
```

三个好处：

1. **必然闭合** —— 沿闭合边界走一圈一定回到出发点
2. **激光始终看得到结构** —— 墙面提供稳定的匹配特征，扫描匹配不易失败
3. **路径可复现** —— 第二圈轨迹几乎相同，回环检测更容易命中

具体路线：房间 A 沿墙一圈 → 回起点 → 过门洞 → 房间 B 沿墙一圈 → 原路返回 → 房间 A 再走一圈回起点。

### 地图歪了，要不要重来

| 现象                                                     | 处理                                             |
| -------------------------------------------------------- | ------------------------------------------------ |
| 墙有点毛糙、稍微不直                                     | **继续走**，回到起点触发回环，图优化会拉直 |
| 同一面墙出现**两条平行线**（重影）                 | 倒回去把那一段慢慢重走一遍，多半能修正           |
| 地图整体**扭曲成弧形**，或房间**分裂成两个** | **重来**。位姿图已经错得太远，回环救不回来 |

**重来的正确做法 —— 别只重启 SLAM：**

```bash
# 全部 Ctrl+C，然后
gzkill
ros2 launch tb3_stage1 slam.launch.py
```

> 只重启 slam_toolbox 而不重启 Gazebo 的话，机器人还停在上次的位置，新地图的原点会定在那儿，起点位置就乱了。**全部重来更干净，也就 30 秒。**

重来时把速度压得更低：teleop 里连按 `x` 和 `c`，线速度降到 **0.08~0.1**，角速度降到 **0.2 以下**。转弯时尤其慢。

**⑥ 障碍物周围要绕。** 只从一侧扫，障碍物在地图上会是个开口的弧，不是闭合轮廓。

**⑦ 全程别关 Gazebo 也别重启 SLAM。** 中途重启 = 从零开始。

## 4.6 判断地图质量

好地图长这样：

- 墙是**一条清晰的黑线**，不是模糊的灰带
- 房间轮廓**闭合**，没有裂口
- 没有**重影**（同一面墙出现两条平行线 —— 典型的漂移未修正）
- 障碍物是**闭合的方块/圆形**轮廓
- 房间内部是干净的白色（已知空闲），外部是灰色（未知）

不好就重来。**重建一次 5 分钟，将就一张烂图会让子任务 3 和 4 全程受苦。**

## 4.7 保存地图

**保持 SLAM 在运行**，新开终端：

```bash
mkdir -p ~/tb3_nav_stage1/src/tb3_stage1/maps
cd ~/tb3_nav_stage1/src/tb3_stage1/maps

ros2 run nav2_map_server map_saver_cli -f stage1_map --ros-args -p use_sim_time:=true
```

生成两个文件：

- **`stage1_map.pgm`** —— 灰度图。白=空闲，黑=障碍，灰(205)=未知
- **`stage1_map.yaml`** —— 元数据：

```yaml
image: stage1_map.pgm
mode: trinary
resolution: 0.05          # 每像素 5 cm
origin: [-5.2, -4.1, 0]   # 图像左下角对应的世界坐标 (x, y, yaw)
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

> **`origin` 是最容易搞错的字段。** 它是图像左下角像素在 `map` 坐标系里的位置。如果你手动移动了 `.pgm`、或者改了它却没改 `origin`，导航时机器人的位置会整体偏移。**别手动编辑这两个文件。**

看一眼地图：

```bash
eog stage1_map.pgm      # 或 xdg-open
```

提交：

```bash
cd ~/tb3_nav_stage1
colcon build --symlink-install     # 让 maps/ 装进 install
git add src/tb3_stage1/maps/ src/tb3_stage1/config/ src/tb3_stage1/launch/
git commit -m "feat(slam): 完成自建世界建图，保存 stage1_map"
git push
```

## ✅ 第 4 章验收标准

- [ ] slam_toolbox 正常启动，无 TF 报错
- [ ] RViz 中 `/map` 随机器人移动实时增长
- [ ] 两个房间和门洞都完整出现在地图上
- [ ] 墙体清晰、闭合，无明显重影和漂移
- [ ] 4 个障碍物都在地图上有闭合轮廓
- [ ] `.pgm` + `.yaml` 已保存并提交
- [ ] 日志记录了地图质量、遇到的问题和调参过程

## 🔧 第 4 章常见坑

| 现象                                     | 根因                                     | 解决                                                      |
| ---------------------------------------- | ---------------------------------------- | --------------------------------------------------------- |
| `Message Filter dropping message`      | `use_sim_time` 有节点没设 true         | 逐个`ros2 param get /<node> use_sim_time`               |
| `Could not transform from [odom]`      | TF 树断了或时间戳错乱                    | `view_frames` 检查；确认 `robot_state_publisher` 在跑 |
| 地图完全不更新                           | `/scan` 没数据，或 `scan_topic` 配错 | `ros2 topic hz /scan`                                   |
| 墙出现重影 / 鬼影墙                      | 移动太快，扫描匹配失败                   | 降速；倒回去重走那段触发回环                              |
| 地图整体扭曲                             | 长距离漂移未被回环修正                   | 一定要绕回起点；调大`loop_search_maximum_distance`      |
| 地图边缘有虚假空白                       | `max_laser_range` 设得比实际量程大     | 改回 3.5                                                  |
| 走廊里定位跳变                           | 长直走廊沿轴向缺乏特征（结构性问题）     | 在走廊里放障碍物提供特征                                  |
| RViz 里 Map 显示 "No transform from map" | SLAM 还没发布`map→odom`               | 稍等；或检查 SLAM 是否真的在跑                            |

---

# 第 5 章 Nav2 自主导航（子任务 3）

## 5.1 Nav2 参数配置

从 nav2 的默认参数出发，改成 burger 的尺寸：

```bash
cp $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml \
   ~/tb3_nav_stage1/src/tb3_stage1/config/nav2_params.yaml
```

打开 `config/nav2_params.yaml`，**必须改的地方**：

```yaml
# ① 全局搜索替换：把所有 use_sim_time 改成 true
#    grep -n "use_sim_time" config/nav2_params.yaml 找出所有位置

# ② AMCL 段
amcl:
  ros__parameters:
    use_sim_time: true
    base_frame_id: "base_footprint"     # TurtleBot3 用的是 base_footprint
    odom_frame_id: "odom"
    global_frame_id: "map"
    scan_topic: scan
    laser_max_range: 3.5                # burger 的量程
    laser_min_range: 0.12
    max_particles: 2000
    min_particles: 500

# ③ 局部代价地图
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_footprint
      robot_radius: 0.105               # burger 半径
      resolution: 0.05
      width: 3
      height: 3
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.30          # 小场景不要设太大

# ④ 全局代价地图
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_footprint
      robot_radius: 0.105
      resolution: 0.05
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.30

# ⑤ 局部控制器（DWB）—— burger 的速度上限
controller_server:
  ros__parameters:
    use_sim_time: true
    FollowPath:
      max_vel_x: 0.22          # burger 的物理上限就是 0.22 m/s
      min_vel_x: 0.0
      max_vel_theta: 1.82      # 上限 2.84，留余量
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      xy_goal_tolerance: 0.20   # 到达判定：位置误差 20 cm 内算成功
      yaw_goal_tolerance: 0.30  # 朝向误差 0.3 rad 内算成功
```

**`inflation_radius` 的取舍**（你调参的主战场）：

```
inflation_radius = 0.55（nav2 默认，给 waffle 用的）
  → 你的 0.8 m 门洞：两边各膨胀 0.55 m，中间完全被堵死，规划器认为过不去

inflation_radius = 0.30
  → 门洞剩余可通行宽度 0.8 - 0.6 = 0.2 m，burger 直径 0.178 m，勉强能过

inflation_radius = 0.20
  → 更容易通过，但机器人会贴着墙走，容易蹭到
```

如果导航一直规划不出穿过门洞的路径，**先怀疑 `inflation_radius`**。

## 5.2 导航 launch 文件

创建 `launch/navigation.launch.py`：

```python
#!/usr/bin/env python3
"""加载已有地图 + 启动 Nav2 导航栈。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_stage1 = get_package_share_directory('tb3_stage1')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_stage1, 'maps', 'stage1_map.yaml'))
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_stage1, 'config', 'nav2_params.yaml'))

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stage1, 'launch', 'stage1_world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 延迟 8 秒再起 Nav2：Gazebo 要先把 /clock 和 /scan 发出来，
    # 否则 Nav2 起来后拿不到 TF 会反复报错。
    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml,
                    'params_file': params_file,
                    'autostart': 'true',
                }.items(),
            )
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_yaml),
        DeclareLaunchArgument('params_file', default_value=params_file),
        world,
        nav2,
        rviz,
    ])
```

## 5.3 运行导航

```bash
cd ~/tb3_nav_stage1 && colcon build --symlink-install && source install/setup.bash
ros2 launch tb3_stage1 navigation.launch.py
```

**启动后必做的第一件事：设置初始位姿。**

Nav2 用 AMCL 定位，AMCL 启动时**不知道机器人在哪**。你必须告诉它：

1. RViz 顶部工具栏点 **"2D Pose Estimate"**
2. 在地图上机器人**实际所在的位置**按下鼠标
3. **拖动**箭头指向机器人**实际朝向**，松开

做对了的标志：RViz 里那一大团散开的粒子云（红色小箭头）会**迅速收敛**成一小簇，激光点云和地图上的墙**重合**。

如果粒子云不收敛、或者激光和墙对不上 —— 重新设一次，位置和朝向都要尽量准。

**然后发目标点：**

1. 点 **"2D Goal Pose"**
2. 在地图上想去的位置按下、拖动定朝向、松开
3. 应该出现一条彩色路径线，机器人开始移动

按任务要求，**至少测 3 个不同位置的目标点**，并记录成功/失败。建议：

| # | 目标点              | 考察什么                     |
| - | ------------------- | ---------------------------- |
| 1 | 房间 A 内绕过障碍物 | 基本全局规划 + 静态避障      |
| 2 | 穿过门洞到房间 B    | 窄通道通过能力（最容易失败） |
| 3 | 房间 B 最远角落     | 长距离规划                   |
| 4 | 返回起点            | 双向可达性                   |

**动态避障演示**（考核要点里写了"遇到动态/静态障碍物时能否重新规划"）：机器人走到一半时，在 Gazebo 里用鼠标拖一个箱子挡在它前面 —— 局部代价地图会更新，控制器应该绕开或重新规划。这个画面很适合放进演示视频。

## 5.4 排查思路：导航不动怎么办

按数据流顺序逐段查，不要乱猜：

```bash
# ① 地图加载了吗
ros2 topic echo /map --once | head -5

# ② 定位在工作吗（map → base_footprint 有变换吗）
ros2 run tf2_ros tf2_echo map base_footprint

# ③ Nav2 的生命周期节点都激活了吗
ros2 lifecycle get /planner_server        # 期望 active
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /amcl
ros2 lifecycle get /map_server

# ④ 全局规划出路径了吗
ros2 topic echo /plan --once | head -20

# ⑤ 速度指令发出来了吗
ros2 topic hz /cmd_vel

# ⑥ 动作服务端在吗
ros2 action list | grep navigate_to_pose
```

**这六条命令定位了整条链路**。第一条没数据 = 地图问题；第二条没数据 = 定位问题；第四条有路径但第五条没数据 = 控制器问题。

## ✅ 第 5 章验收标准

- [ ] Nav2 全部节点 lifecycle 状态为 `active`
- [ ] "2D Pose Estimate" 后粒子云收敛，激光与地图墙体重合
- [ ] 至少 3 个不同目标点导航成功
- [ ] 能穿过门洞在两个房间之间往返
- [ ] 全局路径平滑、不穿墙、绕开障碍物
- [ ] 动态放置障碍物时能重新规划
- [ ] 日志记录了每次导航的成功/失败和参数调整

## 🔧 第 5 章常见坑

| 现象                                      | 根因                                          | 解决                                                                                                                  |
| ----------------------------------------- | --------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| 粒子云不收敛                              | 初始位姿设得不准                              | 重设，位置和朝向都要对                                                                                                |
| `No goal checker` / 规划器立刻失败      | 目标点落在障碍物或膨胀区里                    | 换个空旷点；调小`inflation_radius`                                                                                  |
| 规划不出穿门洞的路径                      | `inflation_radius` 太大堵死门洞             | 改成 0.25~0.30                                                                                                        |
| 机器人原地打转不前进                      | 控制器参数不匹配 / 局部代价地图有幽灵障碍     | 检查`max_vel_x`；`ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap` |
| 一直报`Timed out waiting for transform` | `use_sim_time` 有漏                         | 全局 grep 参数文件，逐节点确认                                                                                        |
| Nav2 节点停在`unconfigured`             | `autostart` 没开或参数文件语法错            | 看 lifecycle_manager 的日志                                                                                           |
| 机器人贴着墙蹭过去                        | `robot_radius` 或 `inflation_radius` 太小 | 适当调大                                                                                                              |
| 地图和激光整体错位                        | 初始位姿错了，或地图`origin` 被改过         | 重设初始位姿；别手改 yaml                                                                                             |

---

# 第 6 章 Python 多目标点导航节点（子任务 4）

## 6.1 Action 的三段式交互

```
客户端                                        服务端 (BT Navigator)
  │                                               │
  ├── send_goal("去 (3, 2)") ───────────────────► │
  │                                               │
  │ ◄────────── goal accepted / rejected ─────────┤   ① 目标被接受了吗
  │                                               │
  │ ◄────────── feedback (还剩 2.3 m) ────────────┤   ② 持续反馈
  │ ◄────────── feedback (还剩 1.1 m) ────────────┤
  │ ◄────────── feedback (还剩 0.3 m) ────────────┤
  │                                               │
  │ ◄────────── result (SUCCEEDED) ───────────────┤   ③ 最终结果
```

**必须分清"目标被接受"和"目标完成"**。`send_goal_async()` 返回的 future 完成时，只代表服务端**收到并接受**了目标（或拒绝了），**不代表走到了**。要拿最终结果，必须再对 goal handle 调 `get_result_async()`。

新手最常见的 bug 就是把这两步混为一谈，导致节点一口气把 4 个目标全发出去然后立刻退出。

`NavigateToPose` 的接口：

```bash
ros2 interface show nav2_msgs/action/NavigateToPose
```

```
# Goal
geometry_msgs/PoseStamped pose
string behavior_tree
---
# Result
std_msgs/Empty result
---
# Feedback
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
```

## 6.2 航点配置文件

创建 `config/waypoints.yaml`：

```yaml
# 目标点序列。yaw 单位是弧度。
frame_id: "map"
goal_timeout_sec: 180.0       # 单个目标点的超时时间
settle_time_sec: 1.0          # 到达后停顿多久再发下一个

waypoints:
  - name: "房间A东北角"
    x: -1.2
    y: 2.2
    yaw: 0.0

  - name: "门洞"
    x: 0.0
    y: 1.0
    yaw: 0.0

  - name: "房间B中心"
    x: 2.5
    y: 0.5
    yaw: -1.5708

  - name: "房间B东南角"
    x: 3.2
    y: -2.0
    yaw: 3.1416

  - name: "返回起点"
    x: -3.0
    y: -2.0
    yaw: 0.0
```

> **坐标怎么定？** 在 RViz 里把鼠标移到目标位置，左下角会显示当前光标的 map 坐标。或者用 "2D Goal Pose" 点一下，然后 `ros2 topic echo /goal_pose --once` 读出精确坐标。**别凭 SDF 里的世界坐标猜** —— 地图坐标系的原点由 SLAM 建图时的起点决定，和世界坐标系可能有偏移。

## 6.3 节点代码

创建 `tb3_stage1/waypoint_navigator.py`：

```python
#!/usr/bin/env python3
"""
多目标点自主导航节点。

从 YAML 配置读取一串航点，依次通过 Nav2 的 navigate_to_pose 动作
接口发送，等待每一个到达后再发下一个，并输出完整的状态日志。
"""

import math
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float):
    """把绕 z 轴的偏航角（弧度）转成四元数 (z, w) 分量。

    绕单一轴 z 旋转时，四元数简化为:
        q = (0, 0, sin(yaw/2), cos(yaw/2))
    x 和 y 分量恒为 0，所以只返回 z 和 w。
    """
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WaypointNavigator(Node):
    """依次导航到一组预设航点。"""

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('waypoints_file', '')
        wp_file = self.get_parameter('waypoints_file').value
        if not wp_file:
            raise ValueError(
                '必须通过参数 waypoints_file 指定航点配置文件路径')

        self._config = self._load_config(wp_file)
        self._waypoints = self._config['waypoints']
        self._frame_id = self._config.get('frame_id', 'map')
        self._goal_timeout = float(self._config.get('goal_timeout_sec', 180.0))
        self._settle_time = float(self._config.get('settle_time_sec', 1.0))

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._last_feedback_log = 0.0

        self.get_logger().info(
            f'已加载 {len(self._waypoints)} 个航点，坐标系 [{self._frame_id}]')

    # ---------------- 配置加载 ----------------

    def _load_config(self, path: str) -> dict:
        """读取并校验 YAML 航点配置。"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f'航点配置文件不存在: {path}')
        except yaml.YAMLError as exc:
            raise ValueError(f'航点配置 YAML 解析失败: {exc}')

        if not isinstance(cfg, dict) or 'waypoints' not in cfg:
            raise ValueError('配置文件缺少顶层 waypoints 字段')

        wps = cfg['waypoints']
        if not isinstance(wps, list) or not wps:
            raise ValueError('waypoints 必须是非空列表')

        for i, wp in enumerate(wps):
            for key in ('x', 'y'):
                if key not in wp:
                    raise ValueError(f'第 {i + 1} 个航点缺少字段 "{key}"')
            wp.setdefault('yaw', 0.0)
            wp.setdefault('name', f'waypoint_{i + 1}')

        return cfg

    # ---------------- 动作交互 ----------------

    def wait_for_nav2(self, timeout_sec: float = 60.0) -> bool:
        """等待 Nav2 的 navigate_to_pose 动作服务端上线。"""
        self.get_logger().info('等待 Nav2 动作服务端 navigate_to_pose ...')
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(
                f'{timeout_sec:.0f} 秒内未等到 Nav2。'
                '请确认导航栈已启动且各节点处于 active 状态。')
            return False
        self.get_logger().info('Nav2 动作服务端已就绪')
        return True

    def _build_goal(self, wp: dict) -> NavigateToPose.Goal:
        """把一个航点字典转换成 NavigateToPose 目标消息。"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        goal.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(float(wp['yaw']))
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal

    def _on_feedback(self, feedback_msg):
        """限流打印导航反馈，避免刷屏。"""
        now = time.monotonic()
        if now - self._last_feedback_log < 2.0:
            return
        self._last_feedback_log = now

        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  剩余距离 {fb.distance_remaining:.2f} m | '
            f'已用时 {fb.navigation_time.sec} s | '
            f'恢复行为触发 {fb.number_of_recoveries} 次')

    def _spin_until(self, future, timeout_sec: float) -> bool:
        """自旋等待 future 完成，返回是否在超时前完成。"""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def go_to_waypoint(self, wp: dict, index: int, total: int) -> bool:
        """发送单个目标点并阻塞等待结果。成功返回 True。"""
        name = wp['name']
        self.get_logger().info(
            f'[{index}/{total}] 前往 "{name}" '
            f'-> x={wp["x"]:.2f}, y={wp["y"]:.2f}, yaw={wp["yaw"]:.2f}')

        goal = self._build_goal(wp)

        # 第一步：发送目标，等待服务端接受或拒绝
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        if not self._spin_until(send_future, timeout_sec=10.0):
            self.get_logger().error(f'  "{name}": 发送目标超时')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'  "{name}": 目标被 Nav2 拒绝。'
                '通常是目标点落在障碍物或膨胀区内。')
            return False

        self.get_logger().info(f'  "{name}": 目标已接受，导航中 ...')

        # 第二步：等待最终结果（这才是"走到了没有"）
        result_future = goal_handle.get_result_async()
        if not self._spin_until(result_future, timeout_sec=self._goal_timeout):
            self.get_logger().error(
                f'  "{name}": 超过 {self._goal_timeout:.0f} 秒未完成，取消目标')
            cancel_future = goal_handle.cancel_goal_async()
            self._spin_until(cancel_future, timeout_sec=5.0)
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'  ✓ "{name}": 已到达')
            return True

        reason = {
            GoalStatus.STATUS_ABORTED: '导航失败（规划或控制中止）',
            GoalStatus.STATUS_CANCELED: '目标被取消',
        }.get(status, f'未知状态码 {status}')
        self.get_logger().error(f'  ✗ "{name}": {reason}')
        return False

    # ---------------- 主流程 ----------------

    def run(self) -> int:
        """依次访问全部航点，返回失败数量。"""
        if not self.wait_for_nav2():
            return len(self._waypoints)

        total = len(self._waypoints)
        succeeded, failed = [], []

        for i, wp in enumerate(self._waypoints, start=1):
            ok = self.go_to_waypoint(wp, i, total)
            (succeeded if ok else failed).append(wp['name'])

            if not ok:
                self.get_logger().warn(
                    f'  跳过 "{wp["name"]}"，继续下一个目标点')

            # 停顿一下让机器人稳定，避免上一个目标的残余速度影响下一次规划
            if i < total:
                end = time.monotonic() + self._settle_time
                while rclpy.ok() and time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info('=' * 52)
        self.get_logger().info(f'导航任务结束：成功 {len(succeeded)}/{total}')
        if succeeded:
            self.get_logger().info(f'  成功: {", ".join(succeeded)}')
        if failed:
            self.get_logger().warn(f'  失败: {", ".join(failed)}')
        self.get_logger().info('=' * 52)
        return len(failed)


def main(args=None):
    rclpy.init(args=args)

    node = None
    exit_code = 0
    try:
        node = WaypointNavigator()
        exit_code = 1 if node.run() > 0 else 0
    except (ValueError, FileNotFoundError) as exc:
        print(f'[waypoint_navigator] 配置错误: {exc}', file=sys.stderr)
        exit_code = 2
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('收到中断信号，退出')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
```

**代码里几个值得注意的设计**（考核要点里有"代码质量：可读性、健壮性"）：

1. **配置校验前置**。`_load_config` 在启动时就检查 YAML 格式和必填字段，而不是等发目标时才崩。
2. **两级超时**。发送目标 10 秒超时，导航完成用配置里的 `goal_timeout_sec`，超时后主动 `cancel_goal_async()` 而不是傻等。
3. **反馈限流**。`distance_remaining` 反馈频率很高，不限流会把日志刷爆。
4. **失败不中断**。单个目标失败时记录并继续，最后统一汇总 —— 这样一次运行就能看到全部航点的成败情况，比失败即退出更适合做实验。
5. **退出码有语义**。0 = 全成功，1 = 有失败，2 = 配置错误。方便以后写自动化测试。

## 6.4 注册入口点

确认 `setup.py` 里有：

```python
entry_points={
    'console_scripts': [
        'waypoint_navigator = tb3_stage1.waypoint_navigator:main',
    ],
},
```

## 6.5 运行

```bash
cd ~/tb3_nav_stage1 && colcon build --symlink-install && source install/setup.bash

# 终端 1：启动导航栈
ros2 launch tb3_stage1 navigation.launch.py
# → RViz 里用 "2D Pose Estimate" 设置初始位姿，等粒子云收敛

# 终端 2：运行你的节点
ros2 run tb3_stage1 waypoint_navigator --ros-args \
  -p use_sim_time:=true \
  -p waypoints_file:=$HOME/tb3_nav_stage1/src/tb3_stage1/config/waypoints.yaml
```

期望日志：

```
[INFO] 已加载 5 个航点，坐标系 [map]
[INFO] 等待 Nav2 动作服务端 navigate_to_pose ...
[INFO] Nav2 动作服务端已就绪
[INFO] [1/5] 前往 "房间A东北角" -> x=-1.20, y=2.20, yaw=0.00
[INFO]   "房间A东北角": 目标已接受，导航中 ...
[INFO]     剩余距离 4.12 m | 已用时 2 s | 恢复行为触发 0 次
[INFO]     剩余距离 2.05 m | 已用时 4 s | 恢复行为触发 0 次
[INFO]   ✓ "房间A东北角": 已到达
[INFO] [2/5] 前往 "门洞" -> x=0.00, y=1.00, yaw=0.00
...
[INFO] ====================================================
[INFO] 导航任务结束：成功 5/5
[INFO] ====================================================
```

## 6.6 可选：一键 launch

创建 `launch/waypoint_nav.launch.py`，把导航栈和你的节点串起来：

```python
#!/usr/bin/env python3
"""一键启动：世界 + Nav2 + 多点导航节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_stage1 = get_package_share_directory('tb3_stage1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    waypoints = os.path.join(pkg_stage1, 'config', 'waypoints.yaml')

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stage1, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 留 25 秒给 Nav2 起来 + 你手动设初始位姿
    navigator = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='tb3_stage1',
                executable='waypoint_navigator',
                name='waypoint_navigator',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'waypoints_file': waypoints,
                }],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        nav,
        navigator,
    ])
```

> **提醒**：用这个一键版时，你只有 25 秒来手动设置初始位姿。演示时建议还是分两个终端，节奏可控。

## ✅ 第 6 章验收标准

- [ ] `ros2 run tb3_stage1 waypoint_navigator` 能正常启动
- [ ] 成功连接 `navigate_to_pose` 动作服务端
- [ ] 至少 3 个（建议 5 个）目标点按顺序依次到达
- [ ] 日志清楚显示当前目标、剩余距离、到达状态
- [ ] 航点从 YAML 读取，不是硬编码
- [ ] 目标被拒绝、超时、失败三种情况都有处理
- [ ] 代码有 docstring，函数职责单一，命名清晰

## 🔧 第 6 章常见坑

| 现象                               | 根因                                                | 解决                                               |
| ---------------------------------- | --------------------------------------------------- | -------------------------------------------------- |
| `wait_for_server` 永远超时       | Nav2 没起来或没 active                              | `ros2 action list \| grep navigate`               |
| 所有目标都被 rejected              | 目标点在障碍物/膨胀区里，或`frame_id` 写错        | 用 RViz 确认坐标；`frame_id` 必须是 `map`      |
| 机器人到了但状态是 ABORTED         | 容差太严                                            | 调大`xy_goal_tolerance` / `yaw_goal_tolerance` |
| 目标全部立刻返回，机器人没动       | 只等了`send_goal_async` 没等 `get_result_async` | 见 6.1                                             |
| 节点时间和 Nav2 对不上             | 忘了`-p use_sim_time:=true`                       | 加上                                               |
| `ModuleNotFoundError: nav2_msgs` | 没 source                                           | `source install/setup.bash`                      |
| `executable not found`           | `setup.py` 的 entry_points 没写或没重新 build     | 检查 + rebuild                                     |

---

# 第 7 章 实验日志

> 报告撰写的完整方法见**第 12 章**。本章只讲日志 —— 日志是报告的原材料，得先有料才能写。

## 7.1 实验日志格式

`docs/logbook.md`，用**问题 → 根因 → 解决 → 教训**四段式。这个格式的价值在于强迫你写"根因"，而不是停在"我换了个命令就好了"。

````markdown
## 2026-08-15 | 子任务 2 | SLAM 建图出现鬼影墙

**问题**
沿房间 A 走第二圈时，RViz 中南墙出现两条平行的黑线，间距约 0.3 m。

**根因**
第一圈转弯时用了 teleop 默认角速度（约 0.8 rad/s）。旋转时激光观测的
帧间变化过大，scan matching 找不到有效对应关系，位姿估计跳变，导致同
一面墙被记录到两个不同位置。由于当时还没完成第一个回环，位姿图里没有
足够的约束把这个错误拉回来。

**解决**
1. teleop 里连按 `c` 把角速度降到 0.3 rad/s 以下
2. 重新建图，转弯处刻意放慢
3. 完整绕回起点触发回环检测，图优化后重影消失

**教训**
- 扫描匹配的失败概率和帧间观测变化量正相关，旋转是最危险的动作
- 回环检测不是"锦上添花"，是修正累积漂移的**结构性**手段。只跑单程
  不回到起点，位姿图缺乏闭合约束，任何漂移都无法被修正
- 建图时应该边走边看 RViz，发现重影立刻倒回去重走，而不是走完再看
````

**建议记录频率**：每次遇到卡壳且花了 15 分钟以上解决的问题，都写一条。四周下来会有 15~30 条，这就是实验报告的素材库。

## 7.2 实验报告结构

```markdown
# 第一阶段考核实验报告

## 1. 引言
   1.1 任务背景与目标
   1.2 技术栈选型说明
       —— 重点写清楚为什么选 Gazebo Classic 而不是新 Gazebo
          （TurtleBot3 模型的插件依赖 + Humble 官方配对）
   1.3 报告结构

## 2. 系统架构
   2.1 整体数据流图（画出第 0.8 节那两张图）
   2.2 TF 坐标系树（贴 frames.pdf）
   2.3 ROS 2 功能包组织结构

## 3. 仿真环境搭建（子任务 1）
   3.1 世界设计思路（为什么门洞不在中间、障碍物高度为什么这样定）
   3.2 SDF 结构说明（collision vs visual）
   3.3 launch 文件设计
   3.4 传感器数据验证（贴 ros2 topic hz 截图）

## 4. SLAM 建图（子任务 2）
   4.1 slam_toolbox 原理简述
   4.2 关键参数选择与依据
   4.3 建图策略（速度、路径、回环）
   4.4 地图质量分析（贴最终地图）
   4.5 遇到的问题与解决（引用日志）

## 5. 自主导航（子任务 3）
   5.1 Nav2 架构说明
   5.2 参数调优过程（重点写 inflation_radius 的取舍）
   5.3 导航实验结果表（≥3 个目标点，成功/失败/耗时）
   5.4 动态避障验证

## 6. 多目标点导航节点（子任务 4）
   6.1 Action 通信机制说明
   6.2 节点设计（状态机、错误处理）
   6.3 实验结果

## 7. 问题与反思
   7.1 关键问题汇总（从日志里挑 3-5 个最有价值的）
   7.2 当前方案的局限
   7.3 后续改进方向

## 8. 附录
   A. 复现步骤（clone 到跑通的完整命令）
   B. 演示视频链接
   C. 参考资料
```

**写报告时最容易丢分的地方**：只写"我做了什么"，不写"为什么这样做"和"遇到了什么问题"。任务书里"考核要点"反复强调的是**理解**，不是**完成度**。把日志里那些踩坑记录整理进去，比多做一道附加题更有价值。

## 7.3 演示视频

建议 3~5 分钟，录屏工具 `sudo apt install simplescreenrecorder` 或 OBS。

内容顺序：

1. 启动世界，展示两个房间和障碍物（转一圈视角）
2. 遥控建图过程（快进）+ 最终地图
3. RViz 里手动发目标点，机器人自主避障到达
4. 中途放一个障碍物，展示重新规划
5. 运行 Python 节点，连续走完 5 个航点
6. 终端日志特写

## ✅ 第 7 章验收标准

- [ ] `docs/logbook.md` 有 15 条以上结构化记录
- [ ] Git commit history 清晰，能看出开发过程
- [ ] 实验报告涵盖全部 4 个子任务
- [ ] 报告里有原理说明，不只是操作步骤
- [ ] 有演示视频

---

---

# 第 8 章 附加挑战 1：给 TurtleBot3 加 RGB-D 相机

> 这一章是后面第 10 章（视觉感知）的前置。相机装不上，目标检测就无从谈起。

## 8.1 先搞懂：URDF 和 SDF 是什么关系

这是本章最容易绕晕的地方，先说清楚。

|                | URDF                                                 | SDF                                    |
| -------------- | ---------------------------------------------------- | -------------------------------------- |
| 全称           | Unified Robot Description Format                     | Simulation Description Format          |
| 谁用           | **ROS**（`robot_state_publisher` 读它发 TF） | **Gazebo**（物理引擎读它建世界） |
| 描述什么       | 机器人的连杆、关节、几何                             | 机器人 + 世界 + 传感器 + 插件          |
| 能描述传感器吗 | ❌ 原生不能                                          | ✅ 能                                  |

**URDF 没有"传感器"这个概念。** 它只描述"哪根杆连着哪根杆"。所以要在 URDF 里加相机，必须用 `<gazebo>` 扩展标签 —— 那是 URDF 规范里预留的"给仿真器的私货"，ROS 工具会忽略它，Gazebo 会读它。

回顾第 3 章那个 launch 文件，你会发现 TurtleBot3 同时用了两份文件：

```
turtlebot3_burger.urdf     → robot_state_publisher 读，发布 TF
models/turtlebot3_burger/model.sdf  → spawn_entity 读，Gazebo 建模型 + 挂插件
```

**两份文件描述同一个机器人，各自服务不同的消费者。** 所以加相机要改**两个地方**，改一个不够：

- URDF 加 `camera_link` 和关节 → 让 TF 树里有相机坐标系
- SDF 加 `<sensor>` + 插件 → 让 Gazebo 真的渲染图像并发布话题

## 8.2 设计决策：派生模型，而不是覆盖上游包

有两种做法，先说清楚为什么选后者。

| 做法                                | 结构                                                                                      | 问题                                                                           |
| ----------------------------------- | ----------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| **A. 覆盖上游包**             | clone`turtlebot3_simulations` 到另一个工作空间，改它的 URDF/SDF，靠 overlay 覆盖 apt 版 | **改动在你的 Git 仓库之外**，别人 clone 你的项目拿不到相机模型，无法复现 |
| **B. 派生模型（本教程采用）** | 在**你自己的包里**新建一份 URDF 和 SDF                                              | 全部在仓库内，`git clone` + `colcon build` 即可复现                        |

### 为什么 B 可行：网格文件是共享的

查看官方 burger 模型会发现：

```bash
grep -o '<uri>[^<]*</uri>' \
  /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf | sort -u
```

```
model://turtlebot3_common/meshes/bases/burger_base.stl
model://turtlebot3_common/meshes/sensors/lds.stl
model://turtlebot3_common/meshes/wheels/left_tire.stl
model://turtlebot3_common/meshes/wheels/right_tire.stl
```

**所有 `.stl` 网格都在独立的 `turtlebot3_common` 模型里，通过 `model://` 引用。** burger 模型目录本身只有两个 XML 文件、32 KB。

所以你的派生模型只要：

1. 复制那两个 XML 到自己的包
2. 加上相机的 `<sensor>` 块
3. 让 `GAZEBO_MODEL_PATH` 包含你的 models 目录

网格照样从 apt 装的 `turtlebot3_common` 解析 —— **零二进制文件复制，零上游覆盖。**

> **顺带解释 overlay 机制**（虽然本章不用它，但值得理解）：
>
> `AMENT_PREFIX_PATH` 是一串前缀路径，`get_package_share_directory()` 按顺序遍历、返回**第一个**命中。每次 `source .../setup.bash` 会把自己的前缀**插到最前面**（prepend）。所以后 source 的赢。
>
> ```bash
> echo $AMENT_PREFIX_PATH | tr ':' '\n'
> ros2 pkg prefix turtlebot3_gazebo
> ```
>
> 这就是 ROS 2 局部替换系统包的方式。但**能不用就不用** —— 它把你的改动散落在仓库之外。

## 8.3 建立派生模型

```bash
cd ~/tb3_nav_stage1/src/tb3_stage1
mkdir -p urdf models/turtlebot3_burger_rgbd

TB3=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo

# URDF：从官方带相机连杆的版本出发
cp $TB3/urdf/turtlebot3_burger_cam.urdf urdf/turtlebot3_burger_rgbd.urdf

# SDF：从官方 burger 模型出发
cp $TB3/models/turtlebot3_burger/model.sdf   models/turtlebot3_burger_rgbd/
cp $TB3/models/turtlebot3_burger/model.config models/turtlebot3_burger_rgbd/
```

改 `models/turtlebot3_burger_rgbd/model.config`，把 `<name>` 改成：

```xml
<name>turtlebot3_burger_rgbd</name>
```

`setup.py` 的 `data_files` 追加两行（**目录结构要保留，所以用 `models/*/*`**）：

```python
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'models', 'turtlebot3_burger_rgbd'),
            glob('models/turtlebot3_burger_rgbd/*')),
```

`.bashrc` 里给 `GAZEBO_MODEL_PATH` 追加你的 models 目录：

```bash
cat >> ~/.bashrc << 'EOF'
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/tb3_nav_stage1/install/tb3_stage1/share/tb3_stage1/models
EOF
source ~/.bashrc
```

> 更好的做法是在 launch 文件里用 `SetEnvironmentVariable` 动态设置，这样别人不改 `.bashrc` 也能跑。8.5 节的 launch 文件里我用了这个写法。

## 8.4 改 URDF：加相机连杆

打开 `urdf/turtlebot3_burger_rgbd.urdf`，官方版本已经包含：

```xml
  <joint name="camera_joint" type="fixed">
    <origin xyz="0.040 -0.000 0.110" rpy="0 0 0"/>   <!-- 装在车头前上方 -->
    <parent link="base_link"/>
    <child link="camera_link"/>
  </joint>
  <link name="camera_link">
    <collision>
      <origin xyz="0.005 0.000 0.013" rpy="0 0 0"/>
      <geometry><box size="0.015 0.030 0.027"/></geometry>
    </collision>
  </link>

  <joint name="camera_rgb_joint" type="fixed">
    <origin xyz="0.003 0.000 0.009" rpy="0 0 0"/>
    <parent link="camera_link"/>
    <child link="camera_rgb_frame"/>
  </joint>
  <link name="camera_rgb_frame"/>

  <joint name="camera_rgb_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.57"/>       <!-- 关键：光学坐标系旋转 -->
    <parent link="camera_rgb_frame"/>
    <child link="camera_rgb_optical_frame"/>
  </joint>
  <link name="camera_rgb_optical_frame"/>
```

### 为什么要三层坐标系

这是相机 URDF 最反直觉的部分：

| 坐标系                       | 约定                                                           | 谁在用         |
| ---------------------------- | -------------------------------------------------------------- | -------------- |
| `camera_link`              | **ROS 约定**：x 向前、y 向左、z 向上                     | TF、机械安装   |
| `camera_rgb_frame`         | 同上，代表 RGB 传感器的物理位置                                | 中间层         |
| `camera_rgb_optical_frame` | **光学约定**：z **向前**、x 向右、y **向下** | 图像处理、点云 |

那句 `rpy="-1.5707 0 -1.57"` 就是两套约定之间的转换（绕 x 转 -90°、绕 z 转 -90°）。

**为什么要两套？** 计算机视觉领域几十年来就是"z 是深度、x 向右、y 向下"（图像原点在左上角）；ROS 机器人学约定是"x 向前"。两边都不改，于是 REP 103 规定：**发布图像和点云用 optical frame，其余用普通 frame。**

**搞错的话**，点云会整体旋转 90°，看起来像躺倒在地上。这是相机集成最常见的 bug。

### 补上深度光学坐标系

RGB-D 的深度传感器和 RGB 传感器物理上不在同一点。在 `</robot>` 之前追加：

```xml
  <joint name="camera_depth_joint" type="fixed">
    <origin xyz="0.003 0.020 0.009" rpy="0 0 0"/>
    <parent link="camera_link"/>
    <child link="camera_depth_frame"/>
  </joint>
  <link name="camera_depth_frame"/>

  <joint name="camera_depth_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.57"/>
    <parent link="camera_depth_frame"/>
    <child link="camera_depth_optical_frame"/>
  </joint>
  <link name="camera_depth_optical_frame"/>
```

## 8.4b 改 SDF：加传感器和插件

**这一步才是真正让相机产出数据的地方。**

编辑 `models/turtlebot3_burger_rgbd/model.sdf`，在 `base_link` 那个 `<link>` 的 `</link>` **之前**插入：

```xml
      <!-- ============ RGB-D 相机 ============ -->
      <sensor name="rgbd_camera" type="depth">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>15.0</update_rate>
        <pose>0.043 0.02 0.119 0 0 0</pose>

        <camera name="rgbd_camera">
          <horizontal_fov>1.02974</horizontal_fov>   <!-- 约 59°，接近 RealSense R200 -->
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>     <!-- 近于此距离看不见 -->
            <far>8.0</far>       <!-- 远于此距离视为无穷远 -->
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>

        <plugin name="rgbd_camera_driver" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/</namespace>
          </ros>
          <camera_name>camera</camera_name>
          <frame_name>camera_depth_optical_frame</frame_name>
          <min_depth>0.1</min_depth>
          <max_depth>8.0</max_depth>
          <hack_baseline>0.07</hack_baseline>
        </plugin>
      </sensor>
```

### 逐项解释

| 元素                        | 作用                                                                                                                                        | 写错会怎样                                        |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------- |
| `type="depth"`            | **必须是 depth 不是 camera**。`libgazebo_ros_camera.so` 支持 `camera`/`depth`/`multicamera` 三种，只有 depth 发布深度图和点云 | 写成 camera 就只有 RGB                            |
| `<pose>`                  | 相机在`base_link` 系里的位置，要和 URDF 里 `camera_joint` 的 origin **对得上**                                                    | 图像视角和 TF 对不上                              |
| `<frame_name>`            | 图像消息里`header.frame_id` 填什么                                                                                                        | **必须填 optical frame**，填错点云旋转 90° |
| `<camera_name>`           | 话题前缀                                                                                                                                    | 决定话题叫`/camera/image_raw` 还是别的          |
| `<clip>`                  | 渲染的远近裁剪面                                                                                                                            | far 设太大浪费性能                                |
| `<min_depth>/<max_depth>` | 有效深度范围，超出置 NaN                                                                                                                    | 影响点云有效范围                                  |
| `<update_rate>`           | 帧率                                                                                                                                        | 30 很吃性能，15 够用                              |

### 会发布哪些话题

`libgazebo_ros_camera.so` 在 depth 模式下发布（源码 `gazebo_ros_camera.cpp` 确认）：

```
/camera/image_raw              sensor_msgs/Image        RGB 图像
/camera/camera_info            sensor_msgs/CameraInfo   RGB 内参
/camera/depth/image_raw        sensor_msgs/Image        深度图（32FC1，单位米）
/camera/depth/camera_info      sensor_msgs/CameraInfo   深度内参
/camera/points                 sensor_msgs/PointCloud2  点云
```

## 8.5 写 launch 文件

在你自己的包里新建 `~/tb3_nav_stage1/src/tb3_stage1/launch/rgbd_world.launch.py`：

```python
#!/usr/bin/env python3
"""启动自定义世界 + 带 RGB-D 相机的 TurtleBot3。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_stage1 = get_package_share_directory('tb3_stage1')

    # URDF 和模型都在自己的包里 —— 别人 clone 仓库即可复现
    world = os.path.join(pkg_stage1, 'worlds', 'stage1_world.world')
    urdf = os.path.join(pkg_stage1, 'urdf', 'turtlebot3_burger_rgbd.urdf')
    sdf = os.path.join(pkg_stage1, 'models', 'turtlebot3_burger_rgbd', 'model.sdf')

    # 让 Gazebo 能解析 model://turtlebot3_common/...（网格来自 apt 装的包）
    # 以及自己包里的模型。这样别人不改 .bashrc 也能跑。
    model_path = ':'.join([
        os.environ.get('GAZEBO_MODEL_PATH', ''),
        os.path.join(pkg_tb3_gazebo, 'models'),
        os.path.join(pkg_stage1, 'models'),
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    # 读 URDF 文本，交给 robot_state_publisher
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),

        # robot_state_publisher：读 URDF 发 TF（包含相机坐标系）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
            }],
        ),

        # spawn：把带相机的 SDF 塞进 Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'burger_rgbd',
                '-file', sdf,
                '-x', '-3.0', '-y', '-2.0', '-z', '0.01',
            ],
            output='screen',
        ),
    ])
```

> 注意 `robot_description` 这里是**直接把 URDF 文本作为参数传进去**，而不是用 `xacro` 命令。因为我们的是纯 URDF 不是 xacro。如果你以后改成 `.xacro`，就要用 `Command(['xacro ', urdf_path])`。

构建并运行 —— **只需要构建你自己的包**：

```bash
cd ~/tb3_nav_stage1 && colcon build --symlink-install
source install/setup.bash

# 确认 URDF 和模型都装进去了
ls install/tb3_stage1/share/tb3_stage1/urdf/
ls install/tb3_stage1/share/tb3_stage1/models/turtlebot3_burger_rgbd/

gzkill
ros2 launch tb3_stage1 rgbd_world.launch.py
```

## 8.6 验证

```bash
# ① 话题都在吗
ros2 topic list | grep camera

# ② 帧率（期望约 15 Hz）
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/points

# ③ 图像编码格式
ros2 topic echo /camera/image_raw --once --field encoding        # 期望 rgb8
ros2 topic echo /camera/depth/image_raw --once --field encoding  # 期望 32FC1

# ④ frame_id 对不对（最容易错的地方）
ros2 topic echo /camera/points --once --field header.frame_id
#    必须是 camera_depth_optical_frame

# ⑤ TF 树里有没有相机坐标系
ros2 run tf2_tools view_frames -o tf_rgbd && xdg-open tf_rgbd.pdf
```

**直观查看图像：**

```bash
sudo apt install -y ros-humble-rqt-image-view
ros2 run rqt_image_view rqt_image_view
# 下拉框选 /camera/image_raw
```

**RViz 里看点云：**

```bash
rviz2 --ros-args -p use_sim_time:=true
```

1. Fixed Frame → `odom`
2. Add → **PointCloud2** → Topic `/camera/points`
3. **Color Transformer** → `RGB8`（能看到彩色点云）
4. Add → **Image** → Topic `/camera/image_raw`

**判断成功的标志**：点云是"竖直站立"的一片扇形，覆盖机器人前方。如果点云**躺在地上**，就是 `frame_name` 写错了（写成了 `camera_link` 而不是 `camera_depth_optical_frame`）。

## ✅ 第 8 章验收标准

- [ ] URDF 和模型都在 `tb3_stage1` 包内，`install/.../urdf/` 和 `install/.../models/` 能看到
- [ ] 五个 camera 话题全部存在且有数据
- [ ] `/camera/image_raw` 编码 `rgb8`，`/camera/depth/image_raw` 编码 `32FC1`
- [ ] 点云 `frame_id` 是 `camera_depth_optical_frame`
- [ ] TF 树里有 `camera_link` → `camera_depth_frame` → `camera_depth_optical_frame`
- [ ] `rqt_image_view` 能看到彩色图像
- [ ] RViz 里点云方向正确（竖直扇形，不是躺平）
- [ ] 机器人仍能正常移动，`/scan` `/odom` 不受影响

## 🔧 第 8 章常见坑

| 现象                                             | 根因                                                          | 解决                                                         |
| ------------------------------------------------ | ------------------------------------------------------------- | ------------------------------------------------------------ |
| 没有任何 camera 话题                             | `type` 写成了 `camera` 而不是 `depth`；或插件文件名拼错 | 检查`<sensor type="depth">` 和 `libgazebo_ros_camera.so` |
| 只有 RGB 没有深度和点云                          | 同上，`type="camera"` 不产生深度                            | 改成`depth`                                                |
| 点云躺在地上 / 旋转 90°                         | `<frame_name>` 填的不是 optical frame                       | 改成`camera_depth_optical_frame`                           |
| 点云位置和机器人对不上                           | SDF 的`<pose>` 和 URDF 的 `camera_joint` origin 不一致    | 两处对齐                                                     |
| RViz 报`frame does not exist`                  | URDF 里没加相机连杆，或 robot_state_publisher 读的还是旧 URDF | 确认 launch 里 urdf 路径改了                                 |
| 改了文件但没生效                                 | 没重新`colcon build`，或 `setup.py` 的 glob 没覆盖        | `ls install/tb3_stage1/share/tb3_stage1/models/` 确认      |
| Gazebo 报找不到`model://turtlebot3_common/...` | `GAZEBO_MODEL_PATH` 没包含 turtlebot3_gazebo 的 models 目录 | launch 里的`SetEnvironmentVariable` 是否生效               |
| 帧率很低、仿真变卡                               | 相机渲染开销大                                                | 降`update_rate` 到 10、降分辨率到 320×240                 |

---

# 第 9 章 附加挑战 2：自定义路径规划器（A* / Dijkstra）

## 9.1 先理解 Nav2 的插件架构

Nav2 的每个功能块都是**可替换的插件**。`planner_server` 本身不含任何算法，它只做三件事：

1. 加载配置里指定的插件
2. 收到规划请求时调用插件的 `createPlan(start, goal)`
3. 把返回的 `nav_msgs/Path` 发布出去

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"   ← 换成你的类名即可
```

**这就是你要做的事**：实现一个符合 `nav2_core::GlobalPlanner` 接口的 C++ 类，编译成动态库，然后在 YAML 里把 `plugin` 换成你的。

> **为什么必须是 C++？** Nav2 用 `pluginlib` 在运行时 `dlopen` 加载 `.so`。Python 写的类没法被 C++ 进程加载。这是硬性约束。

## 9.2 建议的两步走

直接写 C++ 插件对新手跨度太大。建议：

**第一步：先用 Python 独立节点把算法跑通。** 订阅 `/map`，自己发布一条 `nav_msgs/Path` 到 RViz 看。这一步专注于**算法本身**，不涉及插件机制。

**第二步：把算法搬进 C++ 插件。** 这一步专注于**接口对接**，算法已经验证过了。

分开两个关注点，出问题时容易定位。

## 9.3 第一步：Python 版 A*（理解算法）

### 栅格地图的数据结构

`nav_msgs/OccupancyGrid` 长这样：

```
msg.info.resolution   # 0.05，每格 5 cm
msg.info.width        # 列数
msg.info.height       # 行数
msg.info.origin       # 左下角在 map 系的位姿
msg.data              # 一维数组，长度 width*height，行主序
```

`data[i]` 的取值：

| 值      | 含义                     |
| ------- | ------------------------ |
| `0`   | 空闲（可通行）           |
| `100` | 占据（障碍）             |
| `-1`  | 未知                     |
| 1~99    | 概率占据（本项目里少见） |

**索引和坐标的换算**（这两个函数你会反复用到）：

```python
# 栅格 (col, row) → map 系世界坐标 (x, y)
x = origin_x + (col + 0.5) * resolution
y = origin_y + (row + 0.5) * resolution

# map 系世界坐标 → 栅格
col = int((x - origin_x) / resolution)
row = int((y - origin_y) / resolution)

# 二维 → 一维
idx = row * width + col
```

创建 `tb3_stage1/astar_planner_demo.py`：

```python
#!/usr/bin/env python3
"""
A* 路径规划演示节点（独立版，不是 Nav2 插件）。

订阅 /map 和 /goal_pose，用 A* 算出路径发布到 /astar_path，
在 RViz 里可以直接看到。用途是先把算法验证清楚，
再搬进第 9.4 节的 C++ 插件。
"""

import heapq
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener


class AStarPlannerDemo(Node):
    """订阅地图和目标点，用 A* 规划并发布路径。"""

    def __init__(self):
        super().__init__('astar_planner_demo')

        self.declare_parameter('inflation_cells', 3)
        self.declare_parameter('allow_diagonal', True)
        self._inflate = self.get_parameter('inflation_cells').value
        self._diag = self.get_parameter('allow_diagonal').value

        self._map = None
        self._inflated = None

        # /map 是 latched（transient_local），QoS 必须匹配，否则收不到
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self._path_pub = self.create_publisher(Path, '/astar_path', 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            'A* 规划器已启动。在 RViz 里用 "2D Goal Pose" 发目标，'
            '结果发布到 /astar_path')

    # ---------------- 地图处理 ----------------

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg
        self._inflated = self._inflate_obstacles(msg)
        self.get_logger().info(
            f'收到地图 {msg.info.width}x{msg.info.height}, '
            f'分辨率 {msg.info.resolution:.3f} m/格')

    def _inflate_obstacles(self, grid: OccupancyGrid):
        """把障碍物向外膨胀若干格，避免规划出贴墙的路径。

        这是简化版的 costmap 膨胀层：机器人不是质点，
        路径贴着墙走一定会撞。
        """
        w, h, r = grid.info.width, grid.info.height, self._inflate
        blocked = bytearray(w * h)

        for i, v in enumerate(grid.data):
            if v < 0 or v >= 50:          # 未知和占据都视为不可通行
                blocked[i] = 1

        if r <= 0:
            return blocked

        out = bytearray(blocked)
        for row in range(h):
            for col in range(w):
                if not blocked[row * w + col]:
                    continue
                for dr in range(-r, r + 1):
                    for dc in range(-r, r + 1):
                        nr, nc = row + dr, col + dc
                        if 0 <= nr < h and 0 <= nc < w:
                            out[nr * w + nc] = 1
        return out

    # ---------------- 坐标换算 ----------------

    def _world_to_grid(self, x, y):
        info = self._map.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return col, row

    def _grid_to_world(self, col, row):
        info = self._map.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    def _in_bounds(self, col, row):
        return 0 <= col < self._map.info.width and 0 <= row < self._map.info.height

    def _passable(self, col, row):
        return (self._in_bounds(col, row)
                and not self._inflated[row * self._map.info.width + col])

    # ---------------- A* 主体 ----------------

    def _astar(self, start, goal):
        """A* 搜索。start/goal 是 (col, row)，返回栅格路径列表或 None。

        A* = Dijkstra + 启发函数。
        f(n) = g(n) + h(n)
          g(n)：从起点到 n 的实际代价
          h(n)：从 n 到终点的估计代价（启发函数）
        h 恒为 0 时，A* 退化成 Dijkstra。
        """
        if self._diag:
            moves = [(-1, -1, math.sqrt(2)), (0, -1, 1.0), (1, -1, math.sqrt(2)),
                     (-1, 0, 1.0), (1, 0, 1.0),
                     (-1, 1, math.sqrt(2)), (0, 1, 1.0), (1, 1, math.sqrt(2))]
        else:
            moves = [(0, -1, 1.0), (-1, 0, 1.0), (1, 0, 1.0), (0, 1, 1.0)]

        def heuristic(a, b):
            # 对角移动允许时用八方向距离，否则用曼哈顿距离。
            # 关键性质：h 必须"可采纳"（不高估真实代价），否则 A* 不保证最优。
            dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
            if self._diag:
                return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
            return dx + dy

        open_heap = [(heuristic(start, goal), 0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, g, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            closed.add(cur)

            if cur == goal:
                path = [cur]
                while cur in came_from:
                    cur = came_from[cur]
                    path.append(cur)
                path.reverse()
                return path

            for dc, dr, cost in moves:
                nxt = (cur[0] + dc, cur[1] + dr)
                if nxt in closed or not self._passable(*nxt):
                    continue
                # 禁止穿越对角"缝隙"：两个正交邻居都被占时不许斜穿
                if dc != 0 and dr != 0:
                    if not self._passable(cur[0] + dc, cur[1]) or \
                       not self._passable(cur[0], cur[1] + dr):
                        continue
                ng = g + cost
                if ng < g_score.get(nxt, float('inf')):
                    g_score[nxt] = ng
                    came_from[nxt] = cur
                    heapq.heappush(open_heap, (ng + heuristic(nxt, goal), ng, nxt))

        return None

    # ---------------- 目标回调 ----------------

    def _on_goal(self, msg: PoseStamped):
        if self._map is None:
            self.get_logger().warn('还没收到地图，忽略此次目标')
            return

        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception as exc:
            self.get_logger().error(f'查询机器人位姿失败: {exc}')
            return

        start = self._world_to_grid(
            tf.transform.translation.x, tf.transform.translation.y)
        goal = self._world_to_grid(
            msg.pose.position.x, msg.pose.position.y)

        if not self._passable(*start):
            self.get_logger().error('起点落在障碍/膨胀区内')
            return
        if not self._passable(*goal):
            self.get_logger().error('目标点落在障碍/膨胀区内，换个位置')
            return

        self.get_logger().info(f'规划 {start} -> {goal} ...')
        cells = self._astar(start, goal)

        if cells is None:
            self.get_logger().error('未找到可行路径')
            return

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for col, row in cells:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = self._grid_to_world(col, row)
            p.pose.orientation.w = 1.0
            path.poses.append(p)

        self._path_pub.publish(path)
        length = sum(
            math.dist((a.pose.position.x, a.pose.position.y),
                      (b.pose.position.x, b.pose.position.y))
            for a, b in zip(path.poses, path.poses[1:]))
        self.get_logger().info(
            f'✓ 路径已发布：{len(cells)} 个栅格，长度 {length:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

注册入口：`setup.py` 的 `console_scripts` 加一行

```python
'astar_planner_demo = tb3_stage1.astar_planner_demo:main',
```

### 运行和观察

```bash
cd ~/tb3_nav_stage1 && colcon build --symlink-install && source install/setup.bash

# 终端 1：世界 + Nav2（提供 /map 和 TF）
ros2 launch tb3_stage1 navigation.launch.py

# 终端 2：你的规划器
ros2 run tb3_stage1 astar_planner_demo --ros-args -p use_sim_time:=true
```

RViz 里：**Add → Path → Topic `/astar_path`**，颜色调成醒目的绿色。

然后用 "2D Goal Pose" 点目标 —— 你会同时看到两条路径：Nav2 自己的（NavFn 算的）和你的 A*。**对比它们的形状差异，是很好的实验素材。**

### 值得做的对比实验

把 `allow_diagonal` 设成 `false`，再看路径：

```bash
ros2 run tb3_stage1 astar_planner_demo --ros-args \
  -p use_sim_time:=true -p allow_diagonal:=false
```

四邻域的路径会呈现明显的"阶梯状"。**这直观展示了搜索空间的连通性定义如何影响结果。**

再试把启发函数改成恒等于 0（A* 退化为 Dijkstra），观察 `closed` 集合大小的变化 —— Dijkstra 会扩展多得多的节点。这是 A* 价值的直接证明。

## 9.4 第二步：C++ 版 Nav2 插件

算法验证过了，现在做接口对接。

### 建包

```bash
cd ~/tb3_nav_stage1/src
ros2 pkg create nav2_astar_planner \
  --build-type ament_cmake \
  --license Apache-2.0 \
  --description "基于栅格的 A* 全局路径规划器" \
  --maintainer-name "你的名字" \
  --maintainer-email "你的邮箱" \
  --dependencies rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util \
                 nav_msgs geometry_msgs pluginlib tf2_ros builtin_interfaces
```

### 这条命令的三个部分，性质完全不同

```
ros2 pkg create  nav2_astar_planner  --build-type ament_cmake  --dependencies A B C...
                 └── 你自己起的名 ──┘ └── 二选一，不自由 ───┘ └── 由代码决定 ─┘
```

**① 包名：完全自由。** 叫 `my_planner` 也行。约束只有 ROS 2 命名规范：小写字母、数字、下划线，字母开头，不能有横杠或大写。

用 `nav2_` 前缀是社区惯例（一眼看出是给 Nav2 用的），**不是要求**。但定下来后它会出现在 6 个地方，改名成本不低：

| 出现的地方             | 内容                                                         |
| ---------------------- | ------------------------------------------------------------ |
| `CMakeLists.txt`     | `project(...)`、`${PROJECT_NAME}`                        |
| `package.xml`        | `<name>`                                                   |
| `planner_plugin.xml` | `<library path="...">`                                     |
| 头文件宏               | `NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_`                   |
| 目录                   | `include/nav2_astar_planner/`                              |
| C++ 命名空间           | `namespace nav2_astar_planner`（可以不同，但强烈建议一致） |

**② `--build-type`：由语言决定，没有选择余地。**

| 值               | 生成什么           | 何时用                             |
| ---------------- | ------------------ | ---------------------------------- |
| `ament_python` | `setup.py`       | 纯 Python 包（你的`tb3_stage1`） |
| `ament_cmake`  | `CMakeLists.txt` | **有 C++ 代码**              |

这里**必须**是 `ament_cmake`：Nav2 用 `pluginlib` 在运行时 `dlopen` 加载 `.so`，Python 类没法被 C++ 进程加载。要编出 `.so` 就得走 CMake。

**③ `--dependencies`：每一项都对应代码里的一个 `#include`，不是抄来的。**

| 依赖                   | 对应 include                           | 用来干什么                                          | 能省吗                          |
| ---------------------- | -------------------------------------- | --------------------------------------------------- | ------------------------------- |
| `nav2_core`          | `nav2_core/global_planner.hpp`       | **基类**，你的类要继承它                      | ❌ 核心                         |
| `pluginlib`          | `pluginlib/class_list_macros.hpp`    | `PLUGINLIB_EXPORT_CLASS` 宏，把类注册成可动态加载 | ❌ 核心                         |
| `nav2_costmap_2d`    | `nav2_costmap_2d/costmap_2d_ros.hpp` | 读代价地图、`worldToMap()` 换算                   | ❌ 核心                         |
| `rclcpp_lifecycle`   | 间接                                   | `configure()` 首参是 `LifecycleNode::WeakPtr`   | ❌ 接口签名要求                 |
| `tf2_ros`            | `tf2_ros/buffer.h`                   | `configure()` 第三参是 `tf2_ros::Buffer`        | ❌ 接口签名要求                 |
| `nav_msgs`           | `nav_msgs/msg/path.hpp`              | `createPlan()` 返回类型                           | ❌ 接口签名要求                 |
| `geometry_msgs`      | `geometry_msgs/msg/pose_stamped.hpp` | `createPlan()` 参数类型                           | ❌ 接口签名要求                 |
| `nav2_util`          | `nav2_util/node_utils.hpp`           | `declare_parameter_if_not_declared()`             | ⚠️ 可省，但要自己处理重复声明 |
| `rclcpp`             | `rclcpp/rclcpp.hpp`                  | `RCLCPP_INFO` 日志宏                              | ⚠️ 会被传递带入，显式写更清楚 |
| `builtin_interfaces` | 间接                                   | 时间戳类型                                          | ⚠️ 可省                       |

**怎么自己推导这个列表** —— 沿用 2.3 节那套方法，先写代码再反推：

```bash
grep -rhoP '#include\s*[<"]\K[a-z_0-9]+(?=/)' src/ include/ | sort -u
```

输出的每一项基本对应一个依赖包。

> **C++ 漏依赖比 Python 好办得多。** 漏了会在**编译期**明确报错：
>
> ```
> fatal error: nav2_core/global_planner.hpp: No such file or directory
> ```
>
> 而 Python 是**运行时**才 `ModuleNotFoundError`，而且往往只在别人的干净机器上才暴露。这是静态语言的好处之一 —— 所以这里你基本不可能漏。

**`--dependencies` 本身没有魔法**，它只是帮你往两个文件各写几行：

```xml
<!-- package.xml -->
<depend>nav2_core</depend>
```

```cmake
# CMakeLists.txt
find_package(nav2_core REQUIRED)
```

忘了加照样能事后手动补。这个参数纯粹省事，不是必须。

> **⚠️ 别加 `--node-name`。** 那会生成一个带 `main()` 的可执行文件，而**插件是动态库不是程序** —— 它没有入口点，由 `planner_server` 进程加载。加了只会多出个用不上的文件。
>
> 这也是理解插件的好角度：**你写的不是一个"程序"，是一块"被别人的程序加载的代码"**。所以它没有 `main`，只有一组必须实现的虚函数。

### 头文件

`nav2_astar_planner/include/nav2_astar_planner/astar_planner.hpp`：

```cpp
#ifndef NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_astar_planner
{

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  // ---- nav2_core::GlobalPlanner 要求实现的 5 个方法 ----
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct Cell { int x, y; };

  bool worldToMap(double wx, double wy, int & mx, int & my) const;
  void mapToWorld(int mx, int my, double & wx, double & wy) const;
  bool isFree(int mx, int my) const;
  std::vector<Cell> aStarSearch(const Cell & start, const Cell & goal) const;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string global_frame_, name_;

  bool allow_diagonal_{true};
  unsigned char lethal_threshold_{253};
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};
};

}  // namespace nav2_astar_planner

#endif  // NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
```

### 实现

`nav2_astar_planner/src/astar_planner.cpp`：

```cpp
#include "nav2_astar_planner/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node_->get_logger();

  // 声明并读取插件私有参数：YAML 里写在 GridBased: 下面
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_diagonal", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".allow_diagonal", allow_diagonal_);

  int lethal = 253;
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lethal_threshold", rclcpp::ParameterValue(253));
  node_->get_parameter(name_ + ".lethal_threshold", lethal);
  lethal_threshold_ = static_cast<unsigned char>(lethal);

  RCLCPP_INFO(
    logger_, "AStarPlanner [%s] 已配置：allow_diagonal=%s, lethal_threshold=%d",
    name_.c_str(), allow_diagonal_ ? "true" : "false", lethal);
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "清理 AStarPlanner [%s]", name_.c_str());
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "激活 AStarPlanner [%s]", name_.c_str());
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "停用 AStarPlanner [%s]", name_.c_str());
}

bool AStarPlanner::worldToMap(double wx, double wy, int & mx, int & my) const
{
  unsigned int umx, umy;
  if (!costmap_->worldToMap(wx, wy, umx, umy)) {
    return false;
  }
  mx = static_cast<int>(umx);
  my = static_cast<int>(umy);
  return true;
}

void AStarPlanner::mapToWorld(int mx, int my, double & wx, double & wy) const
{
  costmap_->mapToWorld(
    static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
}

bool AStarPlanner::isFree(int mx, int my) const
{
  if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
      my >= static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return false;
  }
  // 代价 >= lethal_threshold 视为不可通行。
  // 注意 NO_INFORMATION(255) 也会被挡掉——保守但安全。
  return costmap_->getCost(mx, my) < lethal_threshold_;
}

std::vector<AStarPlanner::Cell> AStarPlanner::aStarSearch(
  const Cell & start, const Cell & goal) const
{
  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  auto index = [width](int x, int y) { return y * width + x; };

  struct Node { int idx; double f; };
  struct Cmp {
    bool operator()(const Node & a, const Node & b) const { return a.f > b.f; }
  };

  const double kSqrt2 = std::sqrt(2.0);
  std::vector<std::pair<int, int>> moves4 = {{1,0},{-1,0},{0,1},{0,-1}};
  std::vector<std::pair<int, int>> moves8 = {
    {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
  const auto & moves = allow_diagonal_ ? moves8 : moves4;

  auto heuristic = [&](int x, int y) {
    double dx = std::abs(x - goal.x), dy = std::abs(y - goal.y);
    if (allow_diagonal_) {
      return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
    }
    return dx + dy;
  };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;
  std::unordered_map<int, double> g_score;
  std::unordered_map<int, int> came_from;

  const int start_idx = index(start.x, start.y);
  const int goal_idx = index(goal.x, goal.y);
  g_score[start_idx] = 0.0;
  open.push({start_idx, heuristic(start.x, start.y)});

  while (!open.empty()) {
    const int cur = open.top().idx;
    open.pop();

    if (cur == goal_idx) {
      std::vector<Cell> path;
      for (int i = cur; ; ) {
        path.push_back({i % width, i / width});
        auto it = came_from.find(i);
        if (it == came_from.end()) {break;}
        i = it->second;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    const int cx = cur % width, cy = cur / width;
    const double cg = g_score[cur];

    for (const auto & [dx, dy] : moves) {
      const int nx = cx + dx, ny = cy + dy;
      if (!isFree(nx, ny)) {continue;}
      // 禁止斜穿两个障碍之间的缝隙
      if (dx != 0 && dy != 0 && (!isFree(cx + dx, cy) || !isFree(cx, cy + dy))) {
        continue;
      }
      const double step = (dx != 0 && dy != 0) ? kSqrt2 : 1.0;
      const int nidx = index(nx, ny);
      const double ng = cg + step;
      auto it = g_score.find(nidx);
      if (it == g_score.end() || ng < it->second) {
        g_score[nidx] = ng;
        came_from[nidx] = cur;
        open.push({nidx, ng + heuristic(nx, ny)});
      }
    }
  }

  return {};   // 无解
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  if (start.header.frame_id != global_frame_ ||
      goal.header.frame_id != global_frame_)
  {
    RCLCPP_ERROR(
      logger_, "起点/终点的 frame 必须是 %s", global_frame_.c_str());
    return path;
  }

  Cell s{}, g{};
  if (!worldToMap(start.pose.position.x, start.pose.position.y, s.x, s.y)) {
    RCLCPP_ERROR(logger_, "起点在代价地图之外");
    return path;
  }
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, g.x, g.y)) {
    RCLCPP_ERROR(logger_, "目标点在代价地图之外");
    return path;
  }

  const auto cells = aStarSearch(s, g);
  if (cells.empty()) {
    RCLCPP_WARN(logger_, "A* 未找到可行路径");
    return path;
  }

  path.poses.reserve(cells.size());
  for (const auto & c : cells) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    mapToWorld(c.x, c.y, p.pose.position.x, p.pose.position.y);
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  // 终点用请求的精确位姿（含朝向），而不是栅格中心
  path.poses.back() = goal;
  path.poses.back().header = path.header;

  RCLCPP_INFO(logger_, "A* 规划成功：%zu 个点", path.poses.size());
  return path;
}

}  // namespace nav2_astar_planner

PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
```

### 插件描述文件

`nav2_astar_planner/planner_plugin.xml`：

```xml
<library path="nav2_astar_planner">
  <class type="nav2_astar_planner::AStarPlanner" base_class_type="nav2_core::GlobalPlanner">
    <description>基于栅格的 A* 全局路径规划器（教学实现）</description>
  </class>
</library>
```

### CMakeLists.txt

在 `ament_package()` **之前**加：

```cmake
include_directories(include)

set(dependencies
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util
  nav_msgs geometry_msgs pluginlib tf2_ros builtin_interfaces)

add_library(${PROJECT_NAME} SHARED src/astar_planner.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

pluginlib_export_plugin_description_file(nav2_core planner_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
```

`package.xml` 里加：

```xml
  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/planner_plugin.xml" />
  </export>
```

### ⚠️ C++ 插件不需要 entry_points

这一点容易和第 2.4 节混淆。**两者是互不相干的两套发现机制：**

| 产物                             | 类型                       | 怎么被找到                                       | 需要 entry_points  |
| -------------------------------- | -------------------------- | ------------------------------------------------ | ------------------ |
| `waypoint_navigator` 等        | Python 节点                | `ros2 run` 去 `lib/tb3_stage1/` 找可执行文件 | ✅                 |
| **`nav2_astar_planner`** | **C++ 插件 `.so`** | **`pluginlib` 通过 `dlopen` 加载**     | ❌**不需要** |

关键区别在于**它是不是一个"程序"**：

```
Python 节点：  敲命令 → 操作系统启动新进程 → 从 main() 开始
                                            └── entry_points 生成的胶水脚本

C++ 插件：    planner_server 进程已在运行 → 读 YAML 看到你的类名
              → pluginlib 查索引找到 .so → dlopen 加载进自己的地址空间
              → 通过基类指针调用 createPlan()
                                          └── 没有 main()，不是独立进程
```

**插件没有入口点，因为它不是被"启动"的，是被"加载"的。** 这也是前面提醒别加 `--node-name` 的原因。

而且它们属于两个不同的包，各管各的：

```
src/tb3_stage1/          ament_python  → setup.py 的 entry_points
src/nav2_astar_planner/  ament_cmake   → package.xml 的 <nav2_core plugin=.../>
```

### 挂进导航栈

改 `config/nav2_params.yaml`：

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_astar_planner::AStarPlanner"    # ← 换成你的
      allow_diagonal: true
      lethal_threshold: 253
```

> **注意插件名的写法。** `pluginlib` 认的是 `planner_plugin.xml` 里 `class type` 的值，用的是 C++ 命名空间加双冒号。而 nav2 自带的写成 `nav2_navfn_planner/NavfnPlanner`（斜杠形式）—— 那是 pluginlib 的另一种兼容写法。**照着你自己 XML 里写的来。**

### 构建和验证

```bash
cd ~/tb3_nav_stage1
colcon build --symlink-install --packages-select nav2_astar_planner
source install/setup.bash

# 确认插件被 pluginlib 索引到了
ros2 pkg xml nav2_astar_planner | grep nav2_core
ls install/nav2_astar_planner/lib/libnav2_astar_planner.so

colcon build --symlink-install
gzkill && ros2 launch tb3_stage1 navigation.launch.py
```

> **⚠️ "编译成功"不等于"真的被用上"。** 上面这些只证明库编出来了、XML 声明了。**还没证明 `planner_server` 真的加载并调用了它。**

**运行时验证**（这才是真正的判据）：

```bash
# ① 确认 YAML 里换成你的插件了
grep -A4 "^planner_server:" ~/tb3_nav_stage1/src/tb3_stage1/config/nav2_params.yaml
#    plugin: 那行必须是 nav2_astar_planner::AStarPlanner

# ② 启动，看加载日志
gzkill
ros2 launch tb3_stage1 navigation.launch.py 2>&1 | grep -i "astar\|planner_server"
```

**两行日志缺一不可：**

```
[planner_server]: AStarPlanner [GridBased] 已配置：...     ← configure() 被调用了
[planner_server]: A* 规划成功：xxx 个点                     ← createPlan() 被调用了
```

第一行来自 `configure()`，证明插件被加载；第二行在你发出 Nav2 Goal 后出现，证明它真的在规划。**前面所有验证都只说明"东西存在"，这两行才说明"东西在工作"。**

### 对比实验（报告素材）

同一组目标点，分别用 NavFn 和你的 A* 跑，记录：

| 指标         | 怎么测                                               |
| ------------ | ---------------------------------------------------- |
| 规划耗时     | 在`createPlan` 里加计时，或看 `/plan` 的发布延迟 |
| 路径长度     | 累加相邻点距离                                       |
| 路径平滑度   | 相邻段方向变化的总和                                 |
| 实际行驶时间 | 从发目标到`Recoveries` 面板显示完成                |

**预期结果**：你的 A* 路径会比 NavFn 更"贴近栅格"、拐角更硬。因为 NavFn 用的是波前传播（Dijkstra 变体）+ 梯度下降提取路径，天然更平滑。**把这个差异分析清楚，比单纯"我实现了 A*"有价值得多。**

## ✅ 第 9 章验收标准

- [ ] Python 版能在 RViz 里显示 `/astar_path`
- [ ] 四邻域 vs 八邻域的路径差异能观察到并解释
- [ ] C++ 插件编译通过，`.so` 生成
- [ ] `planner_server` 日志显示你的插件被加载和配置
- [ ] "Nav2 Goal" 能用你的规划器成功导航
- [ ] 目标点不可达时能正确返回空路径而非崩溃
- [ ] 有和 NavFn 的定量对比数据

## 🔧 第 9 章常见坑

| 现象                                | 根因                                                                          | 解决                                           |
| ----------------------------------- | ----------------------------------------------------------------------------- | ---------------------------------------------- |
| Python 节点收不到`/map`           | QoS 不匹配，`/map` 是 transient_local                                       | 用代码里那个`map_qos`                        |
| `Failed to create global planner` | 插件名和 XML 里的`class type` 不一致                                        | 两处对齐                                       |
| `Could not find library`          | `pluginlib_export_plugin_description_file` 漏了，或 `<library path>` 写错 | path 填的是库名不带`lib` 前缀和 `.so` 后缀 |
| 规划总是失败                        | `lethal_threshold` 太低，或未知区域被当成障碍                               | 调高阈值；确认地图已加载                       |
| 路径穿墙                            | 没做膨胀，或`isFree` 判断写反                                               | Nav2 的 costmap 已含膨胀层，直接用`getCost`  |
| 规划很慢                            | 地图大 + 八邻域 + 无优化                                                      | 正常现象，可在报告里作为对比项                 |
| 机器人不跟着走                      | 路径点太密或朝向全是默认值                                                    | 通常没问题，控制器只跟位置                     |

---

# 第 10 章 附加挑战 3：视觉感知（目标检测）

> 前置：第 8 章的 RGB-D 相机必须已经跑通，`/camera/image_raw` 有数据。

## 10.1 先明确"简单的目标检测"该做到什么程度

任务书写的是"部署**简单的**目标检测算法，展示结果"。有两条路：

| 方案                      | 原理              | 依赖                | 适合谁             |
| ------------------------- | ----------------- | ------------------- | ------------------ |
| **A. 颜色分割**     | HSV 阈值 + 连通域 | 只要 OpenCV         | **先做这个** |
| **B. 深度学习检测** | YOLOv8 预训练模型 | ultralytics + torch | 有余力再做         |

**强烈建议先做 A。** 原因不是它简单，而是：

1. 你的仿真世界里的障碍物**本来就是纯色的**（红、蓝、绿、黄）—— 颜色分割在这里是**恰当**的方法，不是将就
2. YOLO 的预训练模型是在 COCO 数据集上训的（人、车、猫、狗……），**它认不出 Gazebo 里的彩色方块**。硬套上去只会一个框都检不出来
3. A 方案能让你把整条链路（图像订阅 → 处理 → 结果发布 → 可视化）打通，B 只是换掉中间那一步

做完 A 之后如果想做 B，我在 10.5 给了路径。

## 10.1b 这个节点在系统里的位置

刚写完第 9 章的 C++ 插件，这个对比最能说明两种集成方式的区别：

|                        | 第 9 章 A* 插件                                                                       | 本章检测节点                            |
| ---------------------- | ------------------------------------------------------------------------------------- | --------------------------------------- |
| 产物                   | `libnav2_astar_planner.so`                                                          | 可执行脚本                              |
| **是独立进程吗** | **否** —— 活在 `planner_server` 进程里                                      | **是** —— 自己一个进程          |
| 谁启动它               | `planner_server` 用 `dlopen` 加载                                                 | **你自己** `ros2 run` 或 launch |
| 注册机制               | `PLUGINLIB_EXPORT_CLASS` + `planner_plugin.xml` + `package.xml` 的 `<export>` | `setup.py` 的 `entry_points`        |
| 何时被用到             | **运行期**被 pluginlib 查找并加载                                               | **构建期**生成启动脚本            |
| 谁调用你的函数         | Nav2 调`createPlan()`                                                               | 没人调 —— 你的`main()` 本身就是起点 |

**插件是"被别人的程序加载的代码"，节点是"自己就是一个程序"。** 集成方式不同，注册机制自然不同。

### entry_points 在时间轴上的位置

它是**构建期**的东西，运行时早已退场：

```
构建期（colcon build）
  ├─ 读 setup.py 的 entry_points
  └─ 生成 install/tb3_stage1/lib/tb3_stage1/color_detector 胶水脚本
     ← entry_points 的使命到此结束

运行期（ros2 run）
  ├─ ros2 在 lib/tb3_stage1/ 下找同名可执行文件
  ├─ 执行它 → import 模块 → 调 main()
  └─ rclpy.init() → 节点上线，开始订阅 /camera/image_raw
```

所以它回答的是"**`ros2 run` 怎么找到我的代码**"，不是"Nav2 怎么调用我的代码"。

### 检测节点不在 Nav2 的数据流上

```
                   ┌─► /scan ──► SLAM / AMCL ──► Nav2 ──► /cmd_vel ──┐
                   │                                                  │
  Gazebo ──────────┤                                                  ├──► Gazebo
                   │                                                  │
                   └─► /camera/image_raw ──► color_detector           │
                                                   │                  │
                                                   ▼                  │
                                        /detection/image（给人看）     │
                                        /detection/markers（给 RViz）  │
                                                                      │
                             ↑ 这条支路的输出没有接回控制回路 ─────────┘
```

**Nav2 完全不知道它存在** —— `planner_server`、`controller_server` 没有任何一个订阅 `/detection/*`。检测结果目前只喂给人眼和 RViz。

这符合任务书的要求：附加挑战 3 写的是"**部署**简单的目标检测算法，**展示结果**"，没要求接进控制回路。

**启动顺序里它在哪：**

```
① colcon build              ← entry_points 在这里被消费掉
② ros2 launch 世界           ← Gazebo 起来，相机开始发图
③ ros2 launch nav2          ← 导航栈（和检测节点无关）
④ ros2 run color_detector   ← 节点上线，开始订阅图像
```

**③ 和 ④ 互不依赖，顺序可对调，只做 ④ 不做 ③ 也能跑。** 检测只需要 `/camera/image_raw`，那来自第 ② 步。

验证这一点：

```bash
gzkill
ros2 launch tb3_stage1 rgbd_world.launch.py     # 不启动 Nav2
ros2 run tb3_stage1 color_detector --ros-args -p use_sim_time:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
```

检测照常工作。**这是"松耦合"的一个直接演示**，值得写进报告 —— 话题机制让感知模块可以独立开发、独立测试，不必等导航栈就绪。

## 10.2 关键概念：cv_bridge

ROS 用 `sensor_msgs/Image` 传图像，OpenCV 用 `numpy.ndarray`。**`cv_bridge` 是这两者之间的翻译器。**

```python
from cv_bridge import CvBridge
bridge = CvBridge()

# ROS Image → OpenCV（BGR，OpenCV 的默认顺序）
cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

# OpenCV → ROS Image
out_msg = bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
```

> **注意 `rgb8` 和 `bgr8` 的区别。** Gazebo 发的是 `rgb8`，OpenCV 习惯 `bgr8`。`desired_encoding='bgr8'` 会让 cv_bridge 自动转换。**忘了转的话，红色会变成蓝色** —— 这是个很隐蔽的 bug，因为图像看起来"还行"，只是颜色不对。

安装：

```bash
sudo apt install -y ros-humble-cv-bridge ros-humble-vision-opencv python3-opencv
```

## 10.3 为什么用 HSV 而不是 RGB

做颜色分割**一定要转到 HSV 空间**：

| 空间          | 通道                         | 问题                                                     |
| ------------- | ---------------------------- | -------------------------------------------------------- |
| BGR           | 蓝、绿、红                   | 亮度变化时三个通道**同时**变，阈值极难设           |
| **HSV** | **色调**、饱和度、明度 | 颜色本身只由**H 一个通道**决定，光照变化主要影响 V |

```
BGR 里的"红色"：光照强时 (50,50,250)，光照弱时 (20,20,120) —— 差很远
HSV 里的"红色"：H 都在 0 附近 —— 稳定
```

OpenCV 的 HSV 范围是 **H: 0-179、S: 0-255、V: 0-255**（H 被压缩成 0-179 是为了塞进 uint8）。

各颜色的 H 大致范围：

| 颜色 | H 范围                                             |
| ---- | -------------------------------------------------- |
| 红   | 0-10**或** 170-179（红色跨越 0，要取两段！） |
| 黄   | 20-35                                              |
| 绿   | 35-85                                              |
| 青   | 85-100                                             |
| 蓝   | 100-130                                            |
| 紫   | 130-160                                            |

## 10.4 写检测节点

创建 `tb3_stage1/color_detector.py`：

```python
#!/usr/bin/env python3
"""
基于 HSV 颜色分割的目标检测节点。

订阅 /camera/image_raw，检测世界中的彩色障碍物，
发布标注图像和检测结果，并在 RViz 中显示三维标记。
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# 颜色定义：名称 -> (HSV 下界列表, HSV 上界列表, BGR 画框颜色)
# 红色跨越 H=0，所以需要两段区间
COLOR_TABLE = {
    'red': ([(0, 120, 70), (170, 120, 70)],
            [(10, 255, 255), (179, 255, 255)], (0, 0, 255)),
    'blue': ([(100, 120, 70)], [(130, 255, 255)], (255, 0, 0)),
    'green': ([(40, 100, 70)], [(85, 255, 255)], (0, 255, 0)),
    'yellow': ([(20, 120, 100)], [(35, 255, 255)], (0, 255, 255)),
}


class ColorDetector(Node):
    """用 HSV 颜色分割检测仿真世界中的彩色障碍物。"""

    def __init__(self):
        super().__init__('color_detector')

        self.declare_parameter('min_area', 400)        # 最小连通域面积（像素）
        self.declare_parameter('publish_debug', True)
        self._min_area = self.get_parameter('min_area').value
        self._publish_debug = self.get_parameter('publish_debug').value

        self._bridge = CvBridge()
        self._depth = None       # 最近一帧深度图

        # 相机话题是传感器数据，用 SensorData QoS（BEST_EFFORT）
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._on_depth, qos_profile_sensor_data)

        self._img_pub = self.create_publisher(Image, '/detection/image', 10)
        self._txt_pub = self.create_publisher(String, '/detection/result', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/detection/markers', 10)

        self.get_logger().info(
            f'颜色检测器已启动，检测 {list(COLOR_TABLE)}，'
            f'最小面积 {self._min_area} 像素')

    # ---------------- 回调 ----------------

    def _on_depth(self, msg: Image):
        """缓存深度图，用于估计目标距离。"""
        try:
            self._depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().warn(f'深度图转换失败: {exc}', throttle_duration_sec=5.0)

    def _on_image(self, msg: Image):
        try:
            # Gazebo 发 rgb8，OpenCV 用 bgr8 —— 这里必须显式转换
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'图像转换失败: {exc}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        for name, (lowers, uppers, draw_bgr) in COLOR_TABLE.items():
            mask = self._build_mask(hsv, lowers, uppers)
            for box in self._find_boxes(mask):
                x, y, w, h = box
                dist = self._estimate_distance(x + w // 2, y + h // 2)
                detections.append((name, box, dist))
                self._draw(frame, name, box, dist, draw_bgr)

        self._publish(msg, frame, detections)

    # ---------------- 图像处理 ----------------

    @staticmethod
    def _build_mask(hsv, lowers, uppers):
        """按颜色区间生成二值掩码，并做形态学去噪。"""
        mask = None
        for lo, hi in zip(lowers, uppers):
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        # 开运算去除孤立噪点，闭运算填补目标内部空洞
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _find_boxes(self, mask):
        """从掩码里提取足够大的连通域外接矩形。"""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            if cv2.contourArea(c) < self._min_area:
                continue
            boxes.append(cv2.boundingRect(c))
        return boxes

    def _estimate_distance(self, cx, cy):
        """用深度图估计目标中心的距离（米）。取邻域中位数抗噪。"""
        if self._depth is None:
            return None
        h, w = self._depth.shape[:2]
        if not (0 <= cx < w and 0 <= cy < h):
            return None

        r = 5
        patch = self._depth[max(0, cy - r):cy + r, max(0, cx - r):cx + r]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    @staticmethod
    def _draw(frame, name, box, dist, color):
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        label = name if dist is None else f'{name} {dist:.2f}m'
        cv2.putText(frame, label, (x, max(15, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # ---------------- 发布 ----------------

    def _publish(self, src_msg, frame, detections):
        if self._publish_debug:
            out = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = src_msg.header
            self._img_pub.publish(out)

        if detections:
            parts = []
            for name, (x, y, w, h), dist in detections:
                d = 'unknown' if dist is None else f'{dist:.2f}m'
                parts.append(f'{name}@({x + w // 2},{y + h // 2}) {d}')
            text = ' | '.join(parts)
            self._txt_pub.publish(String(data=text))
            self.get_logger().info(f'检测到 {len(detections)} 个目标: {text}',
                                   throttle_duration_sec=2.0)

        self._publish_markers(src_msg, detections)

    def _publish_markers(self, src_msg, detections):
        """在 RViz 中用球体标记检测到的目标（粗略位置）。"""
        arr = MarkerArray()

        clear = Marker()
        clear.header = src_msg.header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, (name, (x, y, w, h), dist) in enumerate(detections):
            if dist is None:
                continue
            m = Marker()
            m.header = src_msg.header      # frame 是 camera_depth_optical_frame
            m.ns = 'detections'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            # 光学坐标系：z 向前、x 向右、y 向下
            m.pose.position = Point(x=0.0, y=0.0, z=dist)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.a = 0.8
            m.color.r = 1.0 if name in ('red', 'yellow') else 0.0
            m.color.g = 1.0 if name in ('green', 'yellow') else 0.0
            m.color.b = 1.0 if name == 'blue' else 0.0
            arr.markers.append(m)

        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

注册入口点：

```python
'color_detector = tb3_stage1.color_detector:main',
```

`package.xml` 补依赖：

```xml
  <depend>cv_bridge</depend>
  <depend>sensor_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>python3-opencv</depend>
```

### 运行

```bash
cd ~/tb3_nav_stage1 && colcon build --symlink-install && source install/setup.bash

# 终端 1
gzkill && ros2 launch tb3_stage1 rgbd_world.launch.py

# 终端 2
ros2 run tb3_stage1 color_detector --ros-args -p use_sim_time:=true

# 终端 3：看标注结果
ros2 run rqt_image_view rqt_image_view
#   下拉选 /detection/image

# 终端 4：遥控开到障碍物前面
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p use_sim_time:=true -p repeat_rate:=10.0
```

开到红色大方块前面，`/detection/image` 里应该出现红色边框和 `red 1.23m` 的标注。

### 几个值得注意的实现细节

| 细节                          | 为什么                                                              |
| ----------------------------- | ------------------------------------------------------------------- |
| 红色用**两段** H 区间   | 红色在 HSV 色环上跨越 0/179 边界，单区间会漏掉一半                  |
| 用`qos_profile_sensor_data` | 相机话题是 BEST_EFFORT，用默认 RELIABLE QoS**收不到任何数据** |
| 开运算 + 闭运算               | 开运算去孤立噪点，闭运算填内部空洞。顺序不能反                      |
| 深度取邻域**中位数**    | 单点深度容易是 NaN 或跳变值，中位数抗噪                             |
| `throttle_duration_sec`     | 15 Hz 的日志会刷屏，限流后可读                                      |

## 10.5 进阶：用 YOLOv8（可选）

如果想上深度学习，注意前面说的问题：**COCO 预训练模型认不出 Gazebo 里的方块。**

两条可行路线：

**路线一：在世界里放 YOLO 认识的东西**

Gazebo 自带模型库里有 `person_standing`、`bookshelf` 等。在世界文件里 `<include>` 进来，YOLO 就能检出 `person` 等类别。这样能真实展示深度学习检测的效果。

```bash
pip install ultralytics --break-system-packages
```

```python
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self._model = YOLO('yolov8n.pt')      # 首次运行会自动下载
        ...

    def _on_image(self, msg):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self._model(frame, verbose=False)
        annotated = results[0].plot()          # 自带画框
        out = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header
        self._img_pub.publish(out)
```

**路线二：诚实地报告负面结果**

保留 YOLO 节点，但在报告里写清楚：**在合成的几何体场景上，COCO 预训练模型无法检出任何目标；这暴露了预训练模型的域适应（domain adaptation）问题。** 然后说明颜色分割在此场景下是更恰当的选择。

**这个负面结果比强行凑出一个检测框有价值得多。** "选择了合适的方法并解释为什么"是工程能力，"用了最时髦的方法"不是。

## ✅ 第 10 章验收标准

- [ ] `/detection/image` 有标注框输出
- [ ] 至少能稳定检出 4 种颜色障碍物中的 3 种
- [ ] 检测框位置准确（贴合目标边缘）
- [ ] 距离估计合理（和 RViz 里量的距离对得上）
- [ ] `/detection/result` 输出结构化文本
- [ ] RViz 里 MarkerArray 能显示
- [ ] 有对方法选择的说明（为什么用颜色分割而不是 YOLO）

## 🔧 第 10 章常见坑

| 现象                       | 根因                                | 解决                                                                       |
| -------------------------- | ----------------------------------- | -------------------------------------------------------------------------- |
| 节点收不到图像             | QoS 不匹配                          | 用`qos_profile_sensor_data`                                              |
| 红色被检成蓝色             | 忘了 rgb8→bgr8 转换                | `desired_encoding='bgr8'`                                                |
| 红色只能检出一半           | 只用了一段 H 区间                   | 红色必须两段（0-10 和 170-179）                                            |
| 检出一堆碎片               | 没做形态学处理，或`min_area` 太小 | 加开闭运算，调大`min_area`                                               |
| 距离总是`unknown`        | 深度图没订阅到，或全是 NaN          | 检查`/camera/depth/image_raw`；确认目标在 `min_depth`~`max_depth` 内 |
| `ImportError: cv_bridge` | 没装                                | `sudo apt install ros-humble-cv-bridge`                                  |
| Marker 位置乱              | 只用了深度没算横向偏移              | 本例是简化实现，精确定位需要用相机内参反投影                               |

---

# 第 11 章 附加挑战 4：Frontier-based 自主探索

> 这一章把前面所有东西串起来：SLAM 建图 + Nav2 导航 + 你的 ActionClient 节点。**是四个附加题里最有分量的一个。**

## 11.1 什么是 frontier

**Frontier（前沿）= 已知空闲区域和未知区域的边界。**

在 `OccupancyGrid` 里：

```
data[i] == 0      已知空闲
data[i] == 100    已知占据
data[i] == -1     未知
```

**一个栅格是 frontier，当且仅当：它自己是"已知空闲"，且它的邻居里至少有一个是"未知"。**

```
  ░░░░░░░░░░░░        ░ = 未知 (-1)
  ░░░▓▓▓▓▓░░░░        ▓ = 已知空闲 (0)
  ░░▓▓▓▓▓▓▓░░░        █ = 障碍 (100)
  ░░▓▓█▓▓▓▓░░░        ★ = frontier（空闲且挨着未知）
  ░░★▓▓▓▓▓★░░░
  ░░░░░░░░░░░░
```

**为什么这个定义是探索的关键？**

因为 frontier 是"你**能到达**、且**过去就能看到新东西**"的地方。

- 往已知空闲区域走 → 看不到新信息
- 往未知区域走 → 不知道能不能到达（可能是墙里面）
- **往 frontier 走 → 既保证可达，又保证有新信息**

所以整个算法就是：**找 frontier → 挑一个 → 导航过去 → SLAM 更新地图 → 重复，直到没有 frontier。**

没有 frontier 了，说明所有可达区域都探索完了 —— 建图完成。

## 11.2 算法流程

```
┌─────────────────────────────────────────┐
│ 1. 订阅 /map，等待地图更新              │
│ 2. 扫描所有栅格，找出 frontier 点       │
│ 3. 把相邻的 frontier 点聚类成"前沿簇"   │
│ 4. 过滤掉太小的簇（噪声）               │
│ 5. 给每个簇打分，选最优的               │
│ 6. 通过 navigate_to_pose 导航到它的质心 │
│ 7. 到达/失败后回到第 1 步               │
│ 8. 没有 frontier 了 → 探索完成          │
└─────────────────────────────────────────┘
```

**打分函数**是可调的核心。常见策略：

| 策略               | 打分                    | 特点                         |
| ------------------ | ----------------------- | ---------------------------- |
| 最近优先           | `-距离`               | 走得少，但可能反复横跳       |
| 最大簇优先         | `簇大小`              | 信息增益大，但可能来回跑     |
| **加权组合** | `w1*簇大小 - w2*距离` | **实用，本教程用这个** |

## 11.3 写探索节点

创建 `tb3_stage1/frontier_explorer.py`：

```python
#!/usr/bin/env python3
"""
Frontier-based 自主探索节点。

订阅 slam_toolbox 发布的 /map，检测已知空闲区与未知区的边界
（frontier），聚类后选择最优目标，通过 Nav2 的 navigate_to_pose
动作导航过去，循环直到没有可探索的前沿为止。
"""

import math
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

UNKNOWN = -1
FREE_MAX = 25        # 占据概率 <= 25 视为空闲
OCC_MIN = 65         # 占据概率 >= 65 视为障碍


class FrontierExplorer(Node):
    """基于前沿的自主探索。"""

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('min_frontier_size', 8)      # 簇最小格数
        self.declare_parameter('min_goal_distance', 0.4)    # 太近的目标忽略(m)
        self.declare_parameter('size_weight', 1.0)          # 打分：簇大小权重
        self.declare_parameter('distance_weight', 2.0)      # 打分：距离惩罚
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('safety_radius_cells', 4)    # 目标点安全半径

        self._min_size = self.get_parameter('min_frontier_size').value
        self._min_dist = self.get_parameter('min_goal_distance').value
        self._w_size = self.get_parameter('size_weight').value
        self._w_dist = self.get_parameter('distance_weight').value
        self._timeout = self.get_parameter('goal_timeout_sec').value
        self._safety = self.get_parameter('safety_radius_cells').value

        self._map = None
        self._blacklist = []          # 反复失败的目标，记下来别再去
        self._visited_goals = []

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/frontier_markers', 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Frontier 探索节点已启动')

    # ---------------- 基础工具 ----------------

    def _on_map(self, msg):
        self._map = msg

    def _idx(self, col, row):
        return row * self._map.info.width + col

    def _to_world(self, col, row):
        info = self._map.info
        return (info.origin.position.x + (col + 0.5) * info.resolution,
                info.origin.position.y + (row + 0.5) * info.resolution)

    def _robot_xy(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    def _is_free(self, col, row):
        info = self._map.info
        if not (0 <= col < info.width and 0 <= row < info.height):
            return False
        v = self._map.data[self._idx(col, row)]
        return 0 <= v <= FREE_MAX

    def _is_unknown(self, col, row):
        info = self._map.info
        if not (0 <= col < info.width and 0 <= row < info.height):
            return False
        return self._map.data[self._idx(col, row)] == UNKNOWN

    def _is_safe(self, col, row):
        """目标点周围一定半径内不能有障碍，否则 Nav2 会拒绝。"""
        r = self._safety
        info = self._map.info
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                nc, nr = col + dc, row + dr
                if not (0 <= nc < info.width and 0 <= nr < info.height):
                    continue
                if self._map.data[self._idx(nc, nr)] >= OCC_MIN:
                    return False
        return True

    # ---------------- 前沿检测 ----------------

    def _find_frontier_cells(self):
        """扫描全图，找出所有 frontier 栅格。

        定义：自己是已知空闲，且 8 邻域里至少有一个未知。
        """
        info = self._map.info
        cells = []
        for row in range(info.height):
            for col in range(info.width):
                if not self._is_free(col, row):
                    continue
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        if self._is_unknown(col + dc, row + dr):
                            cells.append((col, row))
                            break
                    else:
                        continue
                    break
        return cells

    def _cluster(self, cells):
        """把相邻的 frontier 栅格用 BFS 聚成簇。"""
        cell_set = set(cells)
        clusters = []
        seen = set()

        for cell in cells:
            if cell in seen:
                continue
            queue = deque([cell])
            seen.add(cell)
            group = []
            while queue:
                c = queue.popleft()
                group.append(c)
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        n = (c[0] + dc, c[1] + dr)
                        if n in cell_set and n not in seen:
                            seen.add(n)
                            queue.append(n)
            if len(group) >= self._min_size:
                clusters.append(group)
        return clusters

    def _pick_goal(self, clusters, robot_xy):
        """给每个簇打分，返回最优目标的世界坐标。

        score = w_size * 簇大小 - w_dist * 距离
        簇越大信息增益越大；距离越远代价越高。
        """
        best, best_score = None, -float('inf')
        rx, ry = robot_xy

        for group in clusters:
            # 质心
            cc = sum(c[0] for c in group) / len(group)
            cr = sum(c[1] for c in group) / len(group)
            col, row = int(round(cc)), int(round(cr))

            # 质心可能落在障碍上（例如 U 形前沿），换簇内离质心最近的安全格
            if not (self._is_free(col, row) and self._is_safe(col, row)):
                cand = [c for c in group
                        if self._is_free(*c) and self._is_safe(*c)]
                if not cand:
                    continue
                col, row = min(cand, key=lambda c: (c[0] - cc) ** 2 + (c[1] - cr) ** 2)

            wx, wy = self._to_world(col, row)
            dist = math.hypot(wx - rx, wy - ry)

            if dist < self._min_dist:
                continue
            if any(math.hypot(wx - bx, wy - by) < 0.5
                   for bx, by in self._blacklist):
                continue

            score = self._w_size * len(group) - self._w_dist * dist
            if score > best_score:
                best_score, best = score, (wx, wy, len(group), dist)

        return best

    # ---------------- 可视化 ----------------

    def _publish_markers(self, clusters, goal):
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, group in enumerate(clusters):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = m.scale.y = self._map.info.resolution
            m.color.a, m.color.r, m.color.g = 0.8, 0.0, 1.0
            m.pose.orientation.w = 1.0
            from geometry_msgs.msg import Point
            for col, row in group:
                x, y = self._to_world(col, row)
                m.points.append(Point(x=x, y=y, z=0.0))
            arr.markers.append(m)

        if goal:
            g = Marker()
            g.header.frame_id = 'map'
            g.header.stamp = self.get_clock().now().to_msg()
            g.ns = 'goal'
            g.id = 9999
            g.type = Marker.SPHERE
            g.action = Marker.ADD
            g.pose.position.x, g.pose.position.y = goal[0], goal[1]
            g.pose.position.z = 0.1
            g.pose.orientation.w = 1.0
            g.scale.x = g.scale.y = g.scale.z = 0.3
            g.color.a, g.color.r = 0.9, 1.0
            arr.markers.append(g)

        self._marker_pub.publish(arr)

    # ---------------- 导航 ----------------

    def _spin_until(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def _goto(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        send = self._client.send_goal_async(goal)
        if not self._spin_until(send, 10.0):
            return False
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('  目标被 Nav2 拒绝')
            return False

        result = handle.get_result_async()
        if not self._spin_until(result, self._timeout):
            self.get_logger().warn('  导航超时，取消')
            self._spin_until(handle.cancel_goal_async(), 5.0)
            return False

        return result.result().status == GoalStatus.STATUS_SUCCEEDED

    # ---------------- 主循环 ----------------

    def run(self):
        self.get_logger().info('等待地图和 Nav2 ...')
        while rclpy.ok() and self._map is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not self._client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('等不到 navigate_to_pose 动作服务端')
            return

        self.get_logger().info('开始自主探索')
        round_no, failures = 0, 0

        while rclpy.ok():
            round_no += 1
            # 让地图更新一轮
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)

            robot = self._robot_xy()
            if robot is None:
                self.get_logger().warn('拿不到机器人位姿，等待 ...')
                time.sleep(1.0)
                continue

            cells = self._find_frontier_cells()
            clusters = self._cluster(cells)
            goal = self._pick_goal(clusters, robot)
            self._publish_markers(clusters, goal)

            self.get_logger().info(
                f'[第 {round_no} 轮] frontier 栅格 {len(cells)}，'
                f'有效簇 {len(clusters)}')

            if goal is None:
                self.get_logger().info('=' * 46)
                self.get_logger().info('✓ 没有可探索的前沿了 —— 探索完成！')
                self.get_logger().info('=' * 46)
                return

            x, y, size, dist = goal
            self.get_logger().info(
                f'  前往 ({x:.2f}, {y:.2f})，簇大小 {size}，距离 {dist:.2f} m')

            if self._goto(x, y):
                self.get_logger().info('  ✓ 到达')
                failures = 0
            else:
                failures += 1
                self.get_logger().warn(f'  ✗ 失败（连续 {failures} 次），加入黑名单')
                self._blacklist.append((x, y))
                if failures >= 5:
                    self.get_logger().error('连续失败过多，停止探索')
                    return


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

注册入口点：

```python
'frontier_explorer = tb3_stage1.frontier_explorer:main',
```

## 11.4 SLAM + 导航同时跑的 launch

自主探索需要 **一边建图一边导航** —— 这和第 5 章不同（那时用的是已有地图 + AMCL 定位）。

**关键区别：这次不要 AMCL、不要 map_server。** slam_toolbox 自己会发布 `map→odom`，两者同时跑会打架。

`nav2_bringup` 有个 `slam` 参数正是为此设计的。创建 `launch/exploration.launch.py`：

```python
#!/usr/bin/env python3
"""SLAM + Nav2 + Frontier 探索，一键启动。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_stage1 = get_package_share_directory('tb3_stage1')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    default_params = os.path.join(pkg_stage1, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_stage1, 'config', 'slam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('autostart_explorer', default_value='true'),

        # 世界 + 机器人
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_stage1, 'launch', 'stage1_world.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        ),

        # SLAM（提供 map→odom 和 /map）
        TimerAction(period=6.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'slam_params_file': slam_params,
                }.items(),
            )
        ]),

        # Nav2：slam:=True 表示不启动 map_server 和 amcl
        TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'slam': 'True',
                    'map': '',
                    'params_file': LaunchConfiguration('params_file'),
                    'autostart': 'true',
                }.items(),
            )
        ]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(
                pkg_nav2, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),

        # 探索节点：留 25 秒等 Nav2 全部 active
        TimerAction(period=25.0, actions=[
            Node(
                package='tb3_stage1', executable='frontier_explorer',
                name='frontier_explorer', output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'min_frontier_size': 8,
                    'size_weight': 1.0,
                    'distance_weight': 2.0,
                }],
            )
        ]),
    ])
```

## 11.5 运行和观察

```bash
cd ~/tb3_nav_stage1 && colcon build --symlink-install && source install/setup.bash
gzkill
ros2 launch tb3_stage1 exploration.launch.py
```

**RViz 配置**（这次不用改 Fixed Frame，`nav2_default_view.rviz` 已经是 `map`）：

额外 **Add → MarkerArray** → Topic `/frontier_markers`

你会看到：

- **绿色的点** = 检测到的 frontier 簇
- **红色球** = 当前选中的探索目标
- 地图随着机器人移动**不断向外生长**
- frontier 逐渐减少，最终归零

日志类似：

```
[第 1 轮] frontier 栅格 214，有效簇 3
  前往 (1.05, -0.42)，簇大小 87，距离 2.31 m
  ✓ 到达
[第 2 轮] frontier 栅格 156，有效簇 2
  前往 (2.80, 1.60)，簇大小 61，距离 3.05 m
  ✓ 到达
...
[第 9 轮] frontier 栅格 0，有效簇 0
✓ 没有可探索的前沿了 —— 探索完成！
```

**探索完成后记得存图**：

```bash
cd ~/tb3_nav_stage1/src/tb3_stage1/maps
ros2 run nav2_map_server map_saver_cli -f auto_explored_map \
  --ros-args -p use_sim_time:=true
```

## 11.6 调参与实验设计

`size_weight` 和 `distance_weight` 的比值决定行为：

| 配置               | 行为         | 适合                             |
| ------------------ | ------------ | -------------------------------- |
| `size=1, dist=5` | 强烈偏好近处 | 路径短，但可能在小区域里磨蹭     |
| `size=1, dist=2` | 平衡（默认） | 一般情况                         |
| `size=3, dist=1` | 强烈偏好大簇 | 优先探索大片未知，但会长距离往返 |

**这是很好的实验素材。** 三组参数各跑一次，记录：

| 指标           | 怎么测                               |
| -------------- | ------------------------------------ |
| 总耗时         | 从启动到"探索完成"                   |
| 总路程         | 累加`/odom` 位置增量，或用轨迹长度 |
| 目标点数量     | 日志里的轮数                         |
| 最终地图完整度 | 已知栅格数 / 理论可达栅格数          |
| 失败次数       | 日志里的 ✗ 计数                     |

画一张对比表放进报告，比只跑一次强得多。

## ✅ 第 11 章验收标准

- [ ] 从一张空白地图开始，机器人自主移动
- [ ] RViz 里能看到 frontier 标记和选中的目标
- [ ] 地图持续增长，覆盖两个房间
- [ ] 探索能**自动终止**（打印"探索完成"），不是被你 Ctrl+C
- [ ] 最终地图和第 4 章手动建的图质量相当
- [ ] 保存了自动探索得到的地图
- [ ] 有至少两组参数的对比数据

## 🔧 第 11 章常见坑

| 现象                         | 根因                                     | 解决                                        |
| ---------------------------- | ---------------------------------------- | ------------------------------------------- |
| 收不到`/map`               | QoS 不匹配                               | 用代码里的`map_qos`（TRANSIENT_LOCAL）    |
| 一直报"目标被拒绝"           | 目标落在未知区或障碍膨胀区               | 调大`safety_radius_cells`                 |
| 机器人原地反复横跳           | `distance_weight` 太大，总选最近的     | 调小到 1.0~2.0                              |
| 探索提前结束                 | `min_frontier_size` 太大，小前沿被过滤 | 调小到 4~5                                  |
| 探索永不结束                 | 噪声产生零星假 frontier                  | 调大`min_frontier_size`                   |
| `map→odom` 冲突、位姿乱跳 | AMCL 和 slam_toolbox 同时在跑            | `bringup_launch.py` 必须传 `slam:=True` |
| 检测很慢、卡顿               | 全图逐格扫描是 O(W×H)                   | 正常，地图大时可改为只扫描更新区域          |
| 机器人卡在角落出不来         | 恢复行为没配好                           | 检查`behavior_server` 是否 active         |

---

# 第 12 章 实验报告撰写

> 你现在有一堆跑通的代码和一肚子踩过的坑，但报告还是零。这一章教你怎么把它们变成一份能拿高分的报告。

## 12.1 先明确：报告在考核什么

回看任务书的"考核要点"，它反复出现的词是：

> **"是否理解"**、**"是否正确"**、**"是否养成"**

**没有一处写"是否完成"。** 这决定了报告的写法：

| 写法                                                                                                                                                  | 得分                     |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------ |
| "我按照教程完成了 SLAM 建图，得到了地图。"                                                                                                            | 低 —— 只证明你能执行   |
| "SLAM 建图中出现墙体重影，根因是转弯过快导致扫描匹配失败；通过降低角速度并补跑回环解决。这让我理解到回环检测不是优化项而是修正累积漂移的结构性手段。" | 高 —— 证明你理解了机制 |

**你这几周踩的每一个坑，都是报告里最值钱的内容。** 那些"卡了半天最后发现是环境变量"的经历，恰恰是最能体现工程能力的部分。

## 12.2 报告结构

```markdown
# 科创实践岗 第一阶段考核实验报告

姓名 / 学号 / 日期 / 仓库地址

## 摘要（150 字以内）
一段话说清：做了什么、用了什么技术栈、达成了什么结果、遇到的最主要挑战。
最后写，不要一开始写。

## 1. 引言
### 1.1 任务背景与目标
### 1.2 技术栈选型与依据      ← 重点，见 12.3
### 1.3 报告组织结构

## 2. 系统架构
### 2.1 整体数据流            ← 画图，见 12.4
### 2.2 TF 坐标系树
### 2.3 ROS 2 功能包组织

## 3. 子任务一：自定义仿真环境
### 3.1 世界设计与依据
### 3.2 SDF 关键结构说明
### 3.3 launch 文件设计
### 3.4 传感器数据验证

## 4. 子任务二：SLAM 建图
### 4.1 slam_toolbox 原理简述
### 4.2 关键参数选择与依据
### 4.3 建图策略
### 4.4 地图质量分析
### 4.5 问题与解决：感知混叠   ← 你的亮点

## 5. 子任务三：Nav2 自主导航
### 5.1 Nav2 架构与各组件职责
### 5.2 参数调优过程
### 5.3 导航实验结果
### 5.4 动态避障验证

## 6. 子任务四：多目标点导航节点
### 6.1 Action 通信机制
### 6.2 节点设计与错误处理
### 6.3 实验结果

## 7. 附加挑战（做了哪些写哪些）
### 7.1 RGB-D 相机集成
### 7.2 自定义 A* 规划器
### 7.3 视觉感知
### 7.4 Frontier 自主探索

## 8. 工程实践与问题排查
### 8.1 环境配置中的典型问题     ← 你的另一个亮点，见 12.5
### 8.2 排查方法论总结
### 8.3 可复现性保障措施

## 9. 总结与展望
### 9.1 完成情况自评
### 9.2 当前方案的局限
### 9.3 后续改进方向

## 附录
A. 复现步骤（从 clone 到跑通的完整命令）
B. 实验日志摘录
C. 演示视频链接
D. 参考资料
```

## 12.3 写好"技术选型"这一节

这是最容易被敷衍、也最容易出彩的一节。任务书特别提到要说明选型理由。

**不要写**："本项目使用 ROS 2 Humble 和 Gazebo Classic 11。"

**要写**：

```markdown
### 1.2 技术栈选型与依据

#### 1.2.1 仿真器：为什么是 Gazebo Classic 而非新版 Gazebo

初始环境中安装的是 Gazebo Sim 8（Harmonic）。经调研后改用 Gazebo Classic 11，
依据如下：

**（1）TurtleBot3 仿真模型的插件依赖（决定性因素）**

turtlebot3_gazebo 包中 model.sdf 挂载的插件为：

    libgazebo_ros_diff_drive.so         差速驱动，订阅 /cmd_vel，发布 /odom
    libgazebo_ros_ray_sensor.so         激光雷达，发布 /scan
    libgazebo_ros_imu_sensor.so         IMU
    libgazebo_ros_joint_state_publisher.so

这些是 Gazebo Classic 的插件共享库。新版 Gazebo 的对应实现为
gz-sim-diff-drive-system 等，加载机制与 SDF 参数均不兼容。若在 Harmonic 中
加载该模型，Gazebo 可正常启动、机器人可显示，但 /scan、/odom、/cmd_vel
均不存在——这将导致本次考核的全部四个子任务从第一环即中断。

**（2）ROS 2 Humble 的官方配对版本**

Gazebo 官方为每个 ROS 发行版指定一个配对版本并在其生命周期内提供支持。
Humble 对应 Fortress，而非 Harmonic。Humble + Harmonic 属非官方组合，
需使用 packages.osrfoundation.org 的非 ROS 官方二进制包。

**（3）ROS ↔ Gazebo 桥接架构的差异**

Classic 中 gazebo_ros 插件本身即为 ROS 2 节点，直接发布 ROS 话题，
/clock 亦自动发布。新版 Gazebo 需通过 ros_gz_bridge 逐话题配置桥接，
包含 /clock 与 tf，环节增多即失败点增多。

**（4）局限性说明**

需要指出，Gazebo Classic 11 的官方支持已经结束，该选择是"当前可用性"
而非"技术前瞻性"的权衡。后续迁移路径为 Ubuntu 24.04 + ROS 2 Jazzy +
Gazebo Harmonic。

#### 1.2.2 SLAM 方案：slam_toolbox

（同样的写法：为什么不是 cartographer / gmapping）

#### 1.2.3 机器人型号：burger vs waffle

burger 仅含 2D 激光雷达，waffle 额外带深度相机。相机渲染在仿真中开销显著，
而子任务一至四均不需要视觉输入，故选择 burger。附加挑战一需要相机时，
单独构建了带 RGB-D 的变体模型（见 7.1）。
```

**看出区别了吗**：给出了**具体证据**（插件名、话题名）、**说明了后果**（哪个子任务会断）、**承认了局限**（EOL）。这三样让它从"陈述"变成"论证"。

## 12.4 必须要有的图表

文字描述不清的东西，一定要画图。至少包含：

| 图                       | 内容                                              | 怎么做                                        |
| ------------------------ | ------------------------------------------------- | --------------------------------------------- |
| **系统数据流图**   | Gazebo → 话题 → SLAM/Nav2 → /cmd_vel → Gazebo | 手画或用 draw.io；本教程第 0.8 节有现成的     |
| **TF 树**          | 完整坐标系树                                      | `ros2 run tf2_tools view_frames -o tf_tree` |
| **节点/话题图**    | 运行时的节点连接                                  | `rqt_graph`，截图                           |
| **仿真世界俯视图** | 你设计的两房间布局                                | Gazebo 截图 + 标注尺寸                        |
| **SLAM 地图**      | 最终 .pgm                                         | 直接贴图，标注房间和障碍物                    |
| **导航过程截图**   | 全局路径 + 代价地图 + 粒子云                      | RViz 截图                                     |
| **失败案例对比**   | 好地图 vs 感知混叠破坏的地图                      | **两张并排，很有说服力**                |

安装 rqt_graph：

```bash
sudo apt install -y ros-humble-rqt-graph
ros2 run rqt_graph rqt_graph
```

> **最后一项别省。** "我遇到了问题 + 这是问题的样子 + 这是解决后的样子"，比十段文字描述都有效。

## 12.5 把日志变成报告的第 8 章

你的 `docs/logbook.md` 里应该已经有十几条记录。**不要原样贴进报告** —— 挑 4~6 条最有代表性的，按"问题类型"重新组织：

```markdown
## 8. 工程实践与问题排查

本节记录环境配置与调试过程中的典型问题。这些问题占用了项目约 X% 的时间，
其排查过程本身是本次实践的重要收获。

### 8.1 隐式网络依赖导致的仿真器阻塞

**现象**：Gazebo 启动后界面随机冻结，表现为无法拖动视角，系统提示
"应用程序无响应"。故障呈间歇性，未修改任何配置的情况下时好时坏。

**排查过程**：

依次排除了以下假设——

| 假设 | 排除依据 |
|---|---|
| 软件渲染（llvmpipe） | glxinfo 显示 RTX 3060 + direct rendering: Yes |
| 内存不足 | 31 GB 内存、19 GB 空闲、swap 未使用 |
| 虚拟机 3D 直通不完整 | systemd-detect-virt 返回 none |
| Wayland/XWayland 兼容性 | nvidia-smi 进程列表显示原生 Xorg |
| 显卡驱动故障 | journalctl 无 Xid/NVRM 错误 |

关键转折：nvidia-smi 显示 GPU 处于 P8 睡眠档、gzclient 仅占 27 MiB 显存，
说明进程并非"忙碌"而是"被阻塞"。据此改用 gdb 抓取主线程调用栈：

    QCoreApplication::exec()
     └→ GLWidget::mouseReleaseEvent()
         └→ JointControlWidget::SetModelName()
             └→ gazebo::transport::request()
                 └→ pthread_cond_wait(abstime=0x0)

进一步通过 gzclient --verbose 发现：

    [Msg] Publicized address: 100.79.8.130
    [Wrn] Queue limit reached for topic /gazebo/default/user_camera/pose

**根因**：Gazebo 的 gzserver 与 gzclient 之间是独立进程间的 TCP 通信。
其自动选择本机对外地址的逻辑选中了 Tailscale 虚拟网卡，导致所有内部消息
穿过 tun 设备，延迟不可控。叠加 gzclient 在 GUI 线程上使用无超时的同步
IPC（该设计隐含"本地通信近似瞬时"的前提），延迟一旦超限即永久阻塞。

**解决**：强制 transport 绑定回环接口。

    export GAZEBO_IP=127.0.0.1
    export GAZEBO_HOSTNAME=localhost
    export GAZEBO_MASTER_URI=http://127.0.0.1:11345

**引申**：此类"自动检测本机 IP"的逻辑在存在 VPN 的环境下普遍不可靠，
ROS 2 的 DDS 发现机制存在同类风险（可通过 ROS_LOCALHOST_ONLY=1 规避）。

### 8.2 感知混叠导致的错误回环
（写你那次房间 B 被误匹配成房间 A 的经历）

### 8.3 构建系统的静默失败
（setup.py 的 glob 少匹配一种扩展名，文件未安装但无任何报错）

### 8.4 排查方法论总结

由上述案例可归纳出若干可复用的原则：

1. **区分"忙碌"与"阻塞"是排查的分水岭。** 资源占用低而无响应，
   意味着进程在等待而非计算，此时栈回溯比继续猜测原因高效得多。

2. **将组件从编排中剥离、单独运行其 verbose 模式。** launch 会吞掉子进程
   的原生日志，本项目中的决定性线索正是由 gzclient --verbose 单独运行时给出。

3. **"没有报错"不等于"执行正确"。** 构建系统与仿真器普遍倾向于容错，
   代价是将错误推迟到更难定位的位置。每一环节完成后应立即验证其产物。

4. **让隐式的慢失败转为显式的快失败——但需注意调用方的重试策略。**
   GAZEBO_MODEL_DATABASE_URI="" 有效（使代码不发起请求），而通过 hosts
   阻断域名反而加剧问题（请求照发，快速失败触发密集重试）。

5. **报错信息指向的位置不一定是根因所在。** 例如
   "Was Gazebo started with GazeboRosFactory?" 实际反映的是世界加载未完成。
```

**第 8.4 节是整份报告的最高点。** 它证明你不只是解决了问题，还从中提炼出了可迁移的方法。

## 12.6 实验数据要成表

凡是能量化的，都做成表。例如子任务三：

```markdown
### 5.3 导航实验结果

在建成的地图上选取 5 个目标点进行测试，每个目标点重复 3 次，结果如下：

| # | 目标点 (map 系) | 考察内容 | 成功率 | 平均耗时 | 恢复行为触发 |
|---|---|---|---|---|---|
| 1 | (-1.20, 2.20) | 房间 A 内绕障 | 3/3 | 12.4 s | 0 |
| 2 | (0.00, 1.00) | 门洞通行 | 3/3 | 18.7 s | 0 |
| 3 | (2.50, 0.50) | 跨房间长距离 | 3/3 | 26.1 s | 0 |
| 4 | (3.20, -2.00) | 房间 B 远角 | 2/3 | 31.5 s | 1 |
| 5 | (-3.00, -2.00) | 返回起点 | 3/3 | 29.8 s | 0 |

目标点 4 的一次失败发生在 inflation_radius 调整前，当时该值为默认的 0.55 m，
导致目标点周边被膨胀层完全覆盖，规划器判定不可达。调整为 0.30 m 后未再复现。
```

**注意最后那句话。** 报告失败案例并解释原因，比"全部成功"更可信 —— 因为真实实验里 100% 成功往往意味着测试不充分。

## 12.7 参数调优要写出"为什么"

不要只列最终值，要写清取舍：

```markdown
### 5.2 参数调优过程

#### inflation_radius

该参数控制代价地图中障碍物的膨胀半径，直接决定路径与障碍的最小间距。

| 取值 | 效果 | 判断 |
|---|---|---|
| 0.55（nav2 默认，面向 waffle） | 0.8 m 门洞两侧各膨胀 0.55 m，中间完全封闭，规划器判定不可达 | 不可用 |
| 0.30 | 门洞剩余可通行宽度 0.20 m > burger 直径 0.178 m | **采用** |
| 0.20 | 通行更容易，但路径贴墙，局部控制器易触发碰撞恢复 | 过小 |

最终取 0.30 m。该值的下限由 burger 半径（0.105 m）决定，上限由门洞宽度
（0.80 m）决定，可行区间为 (0.105, 0.311)，取中偏上以保留安全裕度。
```

**这段的价值在于它给出了参数的可行区间及其物理来源**，而不只是"我试出来 0.3 最好"。

## 12.8 写作检查清单

完稿后逐条自查：

- [ ] 摘要能让人在 30 秒内明白你做了什么
- [ ] 每个技术选择都有"为什么"，不只有"是什么"
- [ ] 至少有 6 张图
- [ ] 至少有 3 张数据表
- [ ] 报告了失败案例，不是只有成功
- [ ] 有一节专门讲问题排查（这是加分项）
- [ ] 有对方法论的提炼，不只是流水账
- [ ] 承认了当前方案的局限
- [ ] 附录里的复现步骤，别人照着能跑通
- [ ] 所有截图清晰、有图注、正文里被引用过
- [ ] 代码片段只贴关键部分，长代码放附录或指向仓库
- [ ] 通读一遍，删掉所有"很好地"、"顺利地"、"成功地"这类没信息量的词

## 12.9 演示视频

3~5 分钟，录屏用 `sudo apt install simplescreenrecorder` 或 OBS。

建议脚本：

| 时间      | 内容                                                           |
| --------- | -------------------------------------------------------------- |
| 0:00-0:30 | 启动自定义世界，俯视展示两房间布局和障碍物                     |
| 0:30-1:00 | RViz 中激光点云（Decay Time 开大），说明"这是机器人看到的世界" |
| 1:00-1:45 | SLAM 建图过程（快进），展示地图生长与回环发生时的位姿跳变      |
| 1:45-2:30 | Nav2 导航：RViz 发目标，展示全局路径、代价地图、机器人执行     |
| 2:30-2:50 | 动态避障：用`gz model` 移动箱子，展示重新规划                |
| 2:50-3:30 | 运行 Python 节点，连续走完多个航点，终端日志特写               |
| 3:30-结束 | 附加挑战成果（如有）                                           |

**建议加简短字幕或旁白**说明每一步在做什么 —— 无声的录屏观感很差。

## 12.10 立刻可以做的三件事

现在就动手，不要等：

**① 把 logbook 整理成表**

```bash
cd ~/tb3_nav_stage1/docs
grep "^## " logbook.md
```

看看有几条、覆盖哪些类型，挑出最有价值的 4~6 条。

**② 补齐截图**

对照 12.4 那张表，缺哪张补哪张。趁环境还能跑，一次性截完。

**③ 先写第 8 章（问题排查）**

反直觉但有效：**从你记忆最鲜活、材料最充足的一章开始写**，而不是从引言开始。引言最后写 —— 那时你才真正知道自己做了什么。

# 附录 A 排错速查

## A.1 一分钟体检

```bash
# 环境
echo "ROS_DISTRO=$ROS_DISTRO  TB3_MODEL=$TURTLEBOT3_MODEL"
env | grep '^GAZEBO_'    # 应有 MODEL_PATH / RESOURCE_PATH / PLUGIN_PATH / MASTER_URI
ros2 pkg list | grep -E 'tb3_stage1|turtlebot3_gazebo|slam_toolbox|nav2_bringup'

# 进程
pgrep -af 'gzserver|gzclient'   # pgrep 只接受一个模式，必须用 -f + 正则
ros2 node list

# 数据流
ros2 topic hz /clock
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /cmd_vel

# TF
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_tools view_frames -o tf_tree

# Nav2 生命周期
for n in map_server amcl planner_server controller_server bt_navigator; do
  echo -n "$n: "; ros2 lifecycle get /$n
done
```

## A.2 按症状定位

```
Gazebo 里看不见机器人
  └─ ros2 topic list 有 /robot_description 吗？
     ├─ 没有 → robot_state_publisher 没起来
     └─ 有   → spawn_entity 失败，看终端 [Err]；或 gzserver 有残留

/scan 没有数据
  └─ Gazebo 里有机器人吗？
     ├─ 没有 → 先解决上一条
     └─ 有   → 插件没加载 → 检查 model.sdf（见 1.3 的 grep 验证）

/scan 有数据但 ranges 全是 inf
  └─ 世界里的物体只写了 <visual> 没写 <collision>

SLAM 地图不更新
  └─ ros2 node list 有 /slam_toolbox 吗？
     ├─ 没有 → launch 失败，看报错
     └─ 有   → ros2 param get /slam_toolbox use_sim_time 是 true 吗？
               └─ 是 → tf2_echo odom base_footprint 有输出吗？

Nav2 起来了但不动
  └─ tf2_echo map base_footprint 有输出吗？
     ├─ 没有 → AMCL 没定位 → RViz 里设 2D Pose Estimate
     └─ 有   → ros2 topic echo /plan --once 有路径吗？
               ├─ 没有 → 全局规划失败 → 调 inflation_radius / 换目标点
               └─ 有   → ros2 topic hz /cmd_vel 有吗？
                         └─ 没有 → 控制器问题 → 查 controller_server 日志

一切正常但报 "Timed out waiting for transform"
  └─ 一定是某个节点的 use_sim_time 是 false。逐个查。
```

## A.3 高频命令

```bash
# 环境
source /opt/ros/humble/setup.bash
source ~/tb3_nav_stage1/install/setup.bash
export TURTLEBOT3_MODEL=burger

# 查装了哪些相关包 / 版本
dpkg -l | grep -E 'turtlebot3|gazebo' | awk '{print $2, $3}'
apt-mark showhold

# 构建
cd ~/tb3_nav_stage1 && colcon build --symlink-install
colcon build --packages-select tb3_stage1        # 只编一个包
colcon build --symlink-install --parallel-workers 2   # 限制并行度

# 清理
rm -rf build install log && colcon build --symlink-install
pkill -9 gzserver; pkill -9 gzclient

# 检查安装结果
ls -R install/tb3_stage1/share/tb3_stage1/

# 遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true

# 保存地图
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p use_sim_time:=true

# 清代价地图
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"

# 手动发一个导航目标（不写代码测试用）
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

## A.4 TurtleBot3 burger 参数速查

| 项目                         | 值                   |
| ---------------------------- | -------------------- |
| 尺寸 (L×W×H)               | 138 × 178 × 192 mm |
| 半径（用于`robot_radius`） | 0.105 m              |
| 最大线速度                   | 0.22 m/s             |
| 最大角速度                   | 2.84 rad/s           |
| 激光雷达                     | LDS-01/LDS-02        |
| 激光量程                     | 0.12 ~ 3.5 m         |
| 激光扫描频率                 | 约 5 Hz              |
| 激光角分辨率                 | 1°（360 点）        |
| 雷达安装高度                 | 约 0.18 m            |
| base frame                   | `base_footprint`   |
| 激光 frame                   | `base_scan`        |

---

# 附录 B 验收自检表

对照任务书的"考核要点"逐条打勾。

## 子任务 1：仿真环境

- [ ] 功能包目录结构正确（package.xml / setup.py / launch/ / config/ / worlds/）
- [ ] `package.xml` 依赖声明完整，`rosdep install` 能一键装齐
- [ ] 世界含 ≥2 个由墙分隔的房间
- [ ] 世界含 ≥3 个不同尺寸障碍物（立方体 + 圆柱）
- [ ] 世界完全封闭，机器人出不去
- [ ] TurtleBot3 在 Gazebo 中正确显示且有物理属性
- [ ] `/scan` `/odom` `/imu` 正常发布，频率符合预期
- [ ] TF 树完整无断裂

## 子任务 2：SLAM 建图

- [ ] SLAM 节点正常启动，`use_sim_time` 全局一致
- [ ] 走廊边界清晰、房间轮廓完整
- [ ] 无明显漂移，无鬼影墙
- [ ] `.pgm` + `.yaml` 已保存
- [ ] 地图能被 map_server 正常加载

## 子任务 3：自主导航

- [ ] Nav2 各节点均为 active，协同工作
- [ ] 全局路径平滑、不穿墙
- [ ] 局部避障有效，遇障能重新规划
- [ ] ≥3 个不同位置目标点导航成功并记录

## 子任务 4：Python 节点

- [ ] 使用 `rclpy` 编写
- [ ] 使用 `navigate_to_pose` 动作客户端
- [ ] ≥3 个目标点按顺序依次到达
- [ ] 日志输出当前目标和到达状态
- [ ] 目标点从 YAML 读取
- [ ] 有错误处理（拒绝 / 超时 / 失败）
- [ ] 代码风格规范

## 附加挑战（选做，做了才勾）

**挑战 1：RGB-D 相机**

- [ ] URDF 中加入相机连杆与三层坐标系（含 optical frame）
- [ ] SDF 中 `<sensor type="depth">` + `libgazebo_ros_camera.so`
- [ ] 五个 camera 话题有数据，点云 frame_id 为 `camera_depth_optical_frame`
- [ ] RViz 中点云方向正确

**挑战 2：自定义规划器**

- [ ] A* 算法实现正确（Python 验证版）
- [ ] C++ 插件编译通过并被 `planner_server` 加载
- [ ] 能用自己的规划器完成导航
- [ ] 有与 NavFn 的定量对比

**挑战 3：视觉感知**

- [ ] 检测节点能稳定输出标注图像
- [ ] 至少检出 3 种目标
- [ ] 有方法选择的论证（为何颜色分割优于 COCO 预训练模型）

**挑战 4：自主探索**

- [ ] frontier 检测与聚类正确
- [ ] 探索能自动终止
- [ ] 自动建图质量与手动建图相当
- [ ] 有参数对比实验

## 交付物

- [ ] GitHub private repository
- [ ] 完整代码 + 配置 + 地图
- [ ] `docs/logbook.md` 实验日志
- [ ] 实验报告
- [ ] 演示视频

---

## 建议时间安排（3-4 周）

| 周   | 内容                           | 章节       |
| ---- | ------------------------------ | ---------- |
| W1   | 环境搭建 + 概念学习 + 仓库骨架 | 第 0~2 章  |
| W2   | 自定义世界 + SLAM 建图         | 第 3~4 章  |
| W3   | Nav2 导航 + 参数调优           | 第 5 章    |
| W4   | Python 节点 + 日志整理         | 第 6~7 章  |
| W4+  | 附加挑战（按兴趣选做）         | 第 8~11 章 |
| 收尾 | 实验报告 + 演示视频            | 第 12 章   |

**附加挑战的推荐顺序**：8（相机）→ 10（视觉，依赖 8）→ 11（探索，独立且分量最重）→ 9（规划器，需要 C++）。时间有限就优先做 **11**，它把 SLAM、Nav2、Action 全串起来，最能体现系统能力。

**每周 check-in 前**：把这周的日志整理一下，准备好一个能演示的成果和一个想不通的问题。带着问题去 check-in 比带着"都做完了"更有收获。
