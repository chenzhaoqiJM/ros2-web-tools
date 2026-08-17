# ROS 2 TF 局域网可视化

`remote_tf_web.py` 在远程 ROS 2 主机上订阅 TF 和抓取任务话题，并通过内置的
Python HTTP 服务器提供浏览器页面。局域网内的其他电脑无需安装 ROS 2，只要使用
浏览器访问远程主机的端口即可。

浏览器页面可以显示：

- TF 树、父子坐标系之间的连线和各坐标系的 XYZ 轴；
- `/grasp_task/target_pose` 的目标位姿及高亮坐标轴；
- MoveIt 的紫色末端规划轨迹和轨迹终点坐标轴；
- 白色实际末端位置；
- `/grasp_task/status` 任务状态、轨迹点数和估算的执行进度。

## 运行前提

在运行程序的远程主机上准备：

- ROS 2 Humble；
- Python 3；
- ROS 2 Python 包：`rclpy`、`geometry_msgs`、`nav_msgs`、`tf2_msgs` 和
  `std_msgs`。

如果使用 MoveIt 轨迹显示，远程工作区中的 `grasp_task` 还需要发布：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态 TF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静态 TF |
| `/grasp_task/target_pose` | `geometry_msgs/msg/PoseStamped` | 目标位姿 |
| `/grasp_task/planned_path` | `nav_msgs/msg/Path` | 末端规划路径 |
| `/grasp_task/status` | `std_msgs/msg/String` | 任务状态 |
| `/grasp_task/current_pose` | `geometry_msgs/msg/PoseStamped` | 实际末端位姿 |

## 启动服务

以下命令均为直接启动 Python 程序，不需要 `sh` 或其他启动脚本。

### 1. 进入目录并加载 ROS 2 环境

在远程 ROS 2 主机上执行：

```bash
cd ros2-web-tools/remote_tf_see
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=25
```

### 2. 直接启动 Web 服务

```bash
python3 remote_tf_web.py
```

默认监听所有网卡的 `8765` 端口。保持该终端运行，然后在同一局域网的电脑上访问：

```text
http://<远程主机IP>:8765
```

### 自定义端口或监听地址

端口被占用时，直接通过命令行参数指定端口：

```bash
python3 remote_tf_web.py --port 9000
```

只允许本机访问时可以绑定到回环地址：

```bash
python3 remote_tf_web.py --host 127.0.0.1
```

如果需要局域网访问，使用默认的 `--host 0.0.0.0`。目标位姿话题也可以通过参数修改：

```bash
python3 remote_tf_web.py --target-topic /my_target_pose
```

程序启动后会在终端打印可访问的局域网地址。若远程主机启用了防火墙，需要允许对应
TCP 端口，例如：

```bash
sudo ufw allow 8765/tcp
```

## 测试目标位姿

服务启动后，在远程 ROS 2 主机的另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=25
ros2 topic pub -r 1 /grasp_task/target_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'Body_Link5'}, pose: {position: {x: 0.3699144707, y: -0.1029056584, z: 0.1153295323}, orientation: {x: 0.6442097806, y: -0.127646211, z: 0.0158094389, w: 0.7539564079}}}"
```

如果页面没有数据，可以先确认 ROS 2 话题和域 ID：

```bash
export ROS_DOMAIN_ID=25
ros2 topic list
ros2 topic echo /tf
ros2 topic echo /grasp_task/target_pose
```

## 页面操作

- 鼠标左键拖动：旋转视角；
- 滚轮：缩放；
- 双击：复位视角；
- 点击“适配视图”：根据当前 TF 范围重新调整缩放比例。

按 `Ctrl+C` 可停止 Web 服务。
