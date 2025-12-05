# Repository Guidelines

## 项目概述
DogWalker 是一个“狗溜人”MVP：本地 FastAPI + Streamlit 用于演示状态/模式切换，ROS 包提供控制与 Web 桥接，目标在 Unitree Go1 + Jetson + 2D LiDAR 上完成领跑、掉队检测和语音提示。

## 目录结构
- `backend/` FastAPI 小型接口：`api.py` 入口，`robot_controller.py` 模拟动作，`state.py` 维护运行态。
- `ui/` Streamlit 控制面板（单页）：调用后端触发模拟遛狗与模式切换。
- `ros/dogwalker_core/` Catkin 包：`scripts/controller.py` 领跑/停走逻辑，`scripts/web_server.py` Flask-ROS 桥，`launch/navigation_demo.launch` 占位导航启动文件。
- 根目录文档与依赖：`README.md`、`DEVELOPER_GUIDE.md`、`requirements.txt`。

## 安装与运行（本地 Windows 示例）
- 创建与激活虚拟环境：`python -m venv .venv; .\.venv\Scripts\Activate.ps1`。
- 安装依赖：`pip install -r .\dogwalker\requirements.txt`。
- 启动后端：`uvicorn dogwalker.backend.api:app --reload --port 8000`。
- 启动前端：`streamlit run dogwalker/ui/app.py`（假设后端在 `http://localhost:8000`）。

## 安装与运行（Jetson/Ubuntu + ROS）
- 将 `ros/dogwalker_core` 放入 `~/catkin_ws/src`，执行 `cd ~/catkin_ws && catkin_make && source devel/setup.bash`（可写入 `~/.bashrc` 以自动加载环境变量）。
- 启动占位导航：`roslaunch dogwalker_core navigation_demo.launch`；启动桥接：`rosrun dogwalker_core web_server.py`。

## 页面与 API
- UI：单页控制台（按钮：刷新状态、simulate_walk、设置模式）。
- 后端 API：`GET /status` 返回运行态；`POST /mode` JSON `{"mode":"walking"}` 切换模式；`POST /simulate_walk` 触发模拟遛狗。
- ROS 话题：`/dog_mode`（订阅/发布模式），`/dog_state`（发布当前模式），`/cmd_vel`（安全速度上限 0.5 m/s 内）。

## 技术栈与依赖
- Python 3.x、FastAPI、Streamlit、httpx、Pydantic、Uvicorn；ROS Melodic（gmapping、amcl、move_base、teb_local_planner 需按环境安装）。
- 未使用 Node 构建链；若后续引入前端打包，请使用 pnpm 而非 npm。

## 风格与提交流程
- 代码风格：PEP 8，四空格缩进；ROS 节点使用 `rospy.loginfo` 记录日志。
- 提交信息建议使用祈使句，必要时带作用域（如 `backend: add walk simulation status`），并在 PR 描述中说明运行环境（本地/ROS）、执行过的命令及截图。
