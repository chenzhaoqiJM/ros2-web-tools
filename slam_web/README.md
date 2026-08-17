# ROS 2 SLAM 浏览器实时地图

本工具对应 `slam.rviz` 中的主要显示项，在远程 ROS 2 主机订阅地图、激光、TF 和建图轨迹，并通过局域网网页提供二维俯视视图。浏览器所在电脑不需要安装 ROS 2。

## 显示内容

| RViz 显示 | ROS 2 话题 | 网页效果 |
| --- | --- | --- |
| Map | `/map` | 占用栅格地图，支持地图持续更新 |
| LaserScan | `/scan` | 红色实时激光点云 |
| TF / RobotModel | `/tf`、`/tf_static` | 机器人位置和朝向 |
| MarkerArray | `/trajectory_node_list` | 紫色建图轨迹 |
| Twist | `/cmd_vel` | 网页速度控制 |

固定坐标系与 RViz 配置一致，使用地图消息的 frame（通常为 `map`）。

## 在远程 ROS 2 主机运行

```bash
cd ros2-web-tools/slam_web
source /opt/ros/humble/setup.bash
source <你的工作空间>/install/setup.bash
export ROS_DOMAIN_ID=<远程主机使用的域ID>
python3 slam_web.py
```

默认监听 `0.0.0.0:8766`。在同一局域网电脑打开：

```text
http://<远程主机IP>:8766
```

可选参数：

```bash
python3 slam_web.py --port 9000 \
  --map-topic /map \
  --scan-topic /scan \
  --trajectory-topic /trajectory_node_list \
  --cmd-vel-topic /cmd_vel
```

默认速度限制为线速度 `0.5 m/s`、角速度 `1.5 rad/s`，可以通过启动参数调整：

```bash
python3 slam_web.py --max-linear-speed 0.3 --max-angular-speed 1.0
```

网页右侧“速度控制”面板可以调整速度并按住方向按钮控制机器人，也支持键盘 `W/A/S/D`。
按钮松开、键盘释放、浏览器标签页隐藏或连接失效后，服务端看门狗会在最多约 `0.5` 秒内发布零速度。
网页控速只适用于确认 `/cmd_vel` 已连接到机器人底盘控制器的环境，测试前应先抬起驱动轮或准备物理急停。

如果网页始终显示“等待”，先在运行服务的同一终端环境检查：

```bash
ros2 topic list
ros2 topic echo --once /map
ros2 topic echo --once /scan
ros2 run tf2_ros tf2_echo map base_footprint
```

若机器人使用 `base_link` 且没有 `base_footprint`，网页会自动回退到 `base_link`。防火墙开启时需放行对应 TCP 端口。

## 页面操作

- 鼠标拖动：平移地图；
- 滚轮或左上角 `+`/`−`：缩放；
- `⌖`：适配整张地图；
- `◎`：切换是否跟随机器人；
- 右侧复选框：控制地图、激光、轨迹和米制网格图层。

按 `Ctrl+C` 停止服务。
