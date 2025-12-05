# Dogwalker — 开发者指南（面向小白）

此文档按 PRD 要求，将指导你如何在本地/Jetson+ROS 环境中运行项目的各个模块，并提供初学者的调试流程和常见排查方法。

目录
- 项目概览
- 目录结构
- 运行环境说明（Ubuntu/ROS 与 本地测试）
- 快速开始（不接真实机器人）
- 在 Jetson/Ubuntu 上运行 ROS 导航演示（推荐）
- 常见调试命令与故障排查
- 下一步建议

---

## 项目概览

该仓库为 PRD 的 MVP 骨架：
- `dogwalker/backend`：已包含 FastAPI 示例（用于快速后端接口测试）
- `dogwalker/ui`：Streamlit 示例界面（可触发后端接口）
- `dogwalker/ros/dogwalker_core`：ROS 包骨架，包含控制节点与一个 Flask-ROS 桥接示例

目标是把导航（gmapping/amcl/move_base）、交互逻辑与语音反馈串联起来，演示“领跑/掉队/语音鼓励”的闭环。

## 目录结构（关键文件）
- `dogwalker/backend/api.py` — FastAPI 后端（本地调试用）
- `dogwalker/ui/app.py` — Streamlit 前端示例
- `dogwalker/ros/dogwalker_core/` — ROS 包骨架
  - `scripts/controller.py` — 模拟的机器人控制节点（发布 `/cmd_vel` 和 `/dog_state`）
  - `scripts/web_server.py` — Flask API，桥接到 ROS（发布 `/dog_mode`）
  - `launch/navigation_demo.launch` — 演示用的 launch（占位，需在 ROS 环境中调整）
- `requirements.txt` — Python 依赖（本地测试用）
- `DEVELOPER_GUIDE.md` — 本文档

## 运行环境说明

两类场景：
A) 本地快速测试（Windows/任何能跑 Python 的机器）—— 用于前端和后端联调（不启动 ROS）
B) 目标部署（Jetson Nano / Ubuntu 18.04 + ROS Melodic）—— 完整功能与导航

Python 依赖（本地）：在项目根目录运行：

```powershell
# Windows / PowerShell（在项目根目录）
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r .\dogwalker\requirements.txt
```

在 Jetson/Ubuntu 上，需先安装 ROS Melodic、catkin 工作空间与相关包（gmapping, amcl, move_base, teb_local_planner 等）。这一步超出本文件的自动化范围，但我在下面给出常用命令参考。

## 快速本地运行（不接真实机器人）

1. 启动后端 FastAPI（项目中包含示例）

```powershell
# 在 Windows PowerShell 中
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r .\dogwalker\requirements.txt
# 运行 FastAPI（uvicorn）
uvicorn dogwalker.backend.api:app --reload --port 8000
```

2. 在另一个终端运行前端（Streamlit）

```powershell
.\.venv\Scripts\Activate.ps1
streamlit run dogwalker/ui/app.py
```

3. 在 Streamlit 页面点击“刷新状态 / 模拟遛狗 / 设置模式”来测试后端接口。

注意：本地 FastAPI 示例不会连接 ROS，它只使用了 `backend/robot_controller.py` 中的 `FakeDog` 作为模拟器。

## 在 Jetson/Ubuntu 上运行 ROS 演示（推荐用于导航功能）

先决条件：Jetson 或 Ubuntu 18.04，ROS Melodic 已安装并配置。

示例步骤（在目标机器上执行，bash）：

```bash
# 假设你已在 ~/catkin_ws
cd ~/catkin_ws/src
# 将仓库 clone 到 catkin_ws 或把 ros/dogwalker_core 移入 src
# 然后回到工作空间并编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 启动导航演示（该 launch 为占位示例，真实环境需配置TF、雷达topic等）
roslaunch dogwalker_core navigation_demo.launch
```

若用物理 Go1 机器人与雷达，请保证以下几点：
- LiDAR 的驱动节点能把扫描发布到 `/scan` 或者你在 launch 中指定的 topic
- 里程计（odom）与 base_link 的 TF 连通
- move_base 的参数（全局/局部代价地图、速度上限）已按 PRD 中的安全建议配置（max_vel_x <= 0.5）

### 运行 Flask ROS 桥接

在编译并 source 工作空间后（ROS 环境已激活）：

```bash
# 启动 web server（会初始化 ROS node 并能发布 /dog_mode）
rosrun dogwalker_core web_server.py
```

然后网页或任何 HTTP 客户端可以调用：
POST http://<jetson-ip>:5000/api/mode  JSON body: {"mode": "boss"}

## 常见调试命令（ROS）

- `rostopic list`：查看当前发布/订阅的 topic
- `rostopic echo /dog_state`：查看狗的当前状态消息
- `rostopic pub /dog_mode std_msgs/String "data: 'boss'" -1`：手动发布模式切换
- `rosnode list`：列出所有 ROS 节点
- `rosrun rqt_graph rqt_graph`：可视化节点与 topic 关系
- `rviz`：可视化地图、激光扫描与路径（在有 GUI 的机器或通过 X forwarding）

日志与排查：
- `roslaunch` 输出会显示失败的节点与报错堆栈；常见问题是 topic 名不匹配、TF 未发布或参数路径错误。
- 若机器人不动：检查 `/cmd_vel` 是否被正确发布以及是否被底层控制器（Go1）或安全层覆盖。
- 若定位不稳：检查激光数据是否干净（遮挡、反射）、里程计是否噪声太大，尝试调节 AMCL 参数（粒子数、更新频率）。

## 非 ROS 部分（FastAPI + Streamlit）调试技巧

- 若 Streamlit 无法连接后端：在浏览器的开发者工具查看请求与响应；也可以在 PowerShell 使用 `curl` 或 `http` 工具调用接口测试。
- FastAPI 出错：查看 `uvicorn` 控制台日志，或在代码中添加 `print` / `logging`。

## 初学者常见问题与建议

- ROS 环境变量：务必在每个新终端执行 `source ~/catkin_ws/devel/setup.bash`（或把它放进 `~/.bashrc`）。很多“找不到包/节点”的问题都源于没有 source。
- 权限问题：连接硬件设备（如 LiDAR）时，可能需要修改 udev 规则 或 使用 `sudo`（谨慎）。
- 调参要有耐心：导航算法依赖多个参数（costmap、inflation、tf 等），先用仿真环境（如 Gazebo 或静态激光与地图）做小步尝试。

## 下一步建议（展示路径）

1. 用真实 LiDAR 采集地图（gmapping），获得一张可靠的 2D map。
2. 在 map 上标记关键点（工位、茶水间、导师办公室），作为导航目标。
3. 把 `controller.py` 中的掉队检测逻辑增强为基于后方激光或简单的超声传感器，触发语音播报。
4. 用 `sound_play` 或系统 TTS 做语音反馈样例（健康/老板模式两种语音包）。

---

如果你愿意，我可以：
- 把 `README.md` 更新成包含这些运行命令的快速启动节（并生成一份适用于 Jetson 的步骤清单）；
- 或者在本机（Windows 环境）帮你实际运行 FastAPI+Streamlit 来验证前端后端交互（我可以生成要执行的 PowerShell 命令）。

你希望我下一步做哪件事？（建议：我可以先把 `README.md` 更新为快速启动指南）

