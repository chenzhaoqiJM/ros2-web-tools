# Nav2 Web

在运行 ROS 2 Nav2 的机器人主机上启动：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 nav2_web.py
```

默认监听 `0.0.0.0:8767`，在远程浏览器打开程序打印的局域网地址。

页面功能：

- 显示 `/map`、全局/局部 costmap、机器人 TF 和规划路径
- 显示 AMCL 粒子云与二维定位协方差椭圆
- 分层显示全局路径、局部轨迹、膨胀代价和局部致命障碍物
- 显示全局/局部 footprint
- 显示 Behavior Tree 当前阶段、正在执行的恢复行为
- 显示导航剩余距离、预计时间、执行时间、恢复次数和重规划次数
- 在地图上拖动鼠标发布 Nav2 `NavigateToPose` 目标
- 右键或点击“取消目标”取消当前导航
- 可通过参数覆盖地图、路径和 action 名称

例如：

```bash
python3 nav2_web.py --port 8767 --map-topic /map --action-name /navigate_to_pose
```

默认使用以下附加话题：

| 数据 | 默认话题 |
| --- | --- |
| AMCL 位姿 | `/amcl_pose` |
| AMCL 粒子云 | `/particle_cloud` |
| 全局 footprint | `/global_costmap/published_footprint` |
| 局部 footprint | `/local_costmap/published_footprint` |
| Behavior Tree 日志 | `/behavior_tree_log` |

话题名称不同时可以覆盖：

```bash
python3 nav2_web.py \
  --amcl-pose-topic /amcl_pose \
  --particle-topic /particle_cloud \
  --global-footprint-topic /global_costmap/published_footprint \
  --local-footprint-topic /local_costmap/published_footprint \
  --bt-log-topic /behavior_tree_log
```

“重规划次数”按导航期间全局路径几何发生变化的次数统计。局部障碍图层来自局部
costmap 的致命代价值，因此包含动态障碍，也可能包含进入局部窗口的静态障碍。
Behavior Tree 面板需要 Nav2 发布 `/behavior_tree_log`；未启用该日志时其他导航反馈
仍可正常显示。

页面发布的是 `nav2_msgs/action/NavigateToPose`，目标坐标默认使用 `map` frame。确保当前终端已经 source 了包含 `nav2_msgs` 的 ROS 2 环境，并且 Nav2 action server 已启动。
