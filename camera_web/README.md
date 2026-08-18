# ROS 2 多传感器相机 Web

自动发现 ROS 2 图像话题，并通过 WebRTC 在浏览器中低延迟显示 RGB、深度和红外相机。

## 功能

- 自动发现 `sensor_msgs/msg/Image` 和 `sensor_msgs/msg/CompressedImage`；话题上线或下线无需重启
- 根据话题名和图像编码自动分类 RGB、深度、红外，并可在页面快速筛选、切换
- 支持常见 RGB/BGR、RGBA/BGRA、Mono8、Mono16、16UC1、32FC1 和 Bayer 编码
- 深度图使用 Turbo 伪彩，红外/灰度图自动拉伸动态范围
- WebRTC 点对点低延迟视频，自带拥塞控制
- 自动策略根据浏览器丢包和实际码率在 1080p/30、720p/24、360p/12 档位间调整
- 页面显示帧率、码率、RTT、丢包率，可保存当前帧和全屏观看

## 安装与运行

先进入已安装 ROS 2 图像消息的环境，再安装 Python 依赖：

```bash
cd ros2-web-tools/camera_web
source /opt/ros/$ROS_DISTRO/setup.bash
source <你的工作空间>/install/setup.bash  # 如果需要
python3 -m pip install -r requirements.txt
python3 camera_web.py
```

默认监听 `0.0.0.0:8768`。在同一局域网设备打开启动日志打印的地址：

```text
http://<ROS主机IP>:8768
```

可修改监听地址和端口：

```bash
python3 camera_web.py --host 0.0.0.0 --port 9000
```

服务使用 best-effort、depth 1 的 ROS QoS 来降低图像延迟。页面每 2 秒刷新话题列表，新相机发布后会自动出现。

## 深度/红外识别规则

服务会结合话题名和消息编码分类：

- 包含 `depth`，或编码为 `16UC1` / `32FC1`：深度
- 包含 `infra`、`/ir`、`thermal`，或编码为 `mono16`：红外
- 其他图像：RGB

若一个 `mono16` 话题实际是深度图，建议在话题名中包含 `depth`；话题名优先用于表达传感器语义。

## 网络说明

当前实现使用 host ICE candidate，适合机器人与浏览器位于同一局域网的场景。跨 NAT 或公网访问时，需要在代码的 `RTCPeerConnection` 配置中加入可用的 STUN/TURN 服务，并建议通过 HTTPS 提供页面。防火墙除网页 TCP 端口外，还需允许 WebRTC 使用的 UDP 流量。

按 `Ctrl+C` 停止服务。
