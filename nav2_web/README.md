# Nav2 Web

在运行 ROS 2 Nav2 的机器人主机上启动：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 nav2_web.py
```

默认监听 `0.0.0.0:8767`，在远程浏览器打开程序打印的局域网地址。

页面功能：

- 显示 `/map`、全局/局部 costmap、机器人 TF 和规划路径
- 在地图上拖动鼠标发布 Nav2 `NavigateToPose` 目标
- 右键或点击“取消目标”取消当前导航
- 可通过参数覆盖地图、路径和 action 名称

例如：

```bash
python3 nav2_web.py --port 8767 --map-topic /map --action-name /navigate_to_pose
```

页面发布的是 `nav2_msgs/action/NavigateToPose`，目标坐标默认使用 `map` frame。确保当前终端已经 source 了包含 `nav2_msgs` 的 ROS 2 环境，并且 Nav2 action server 已启动。
