# Camera Socket MJPEG Server

把 `/dev/video5` 相机画面通过 HTTP MJPEG 推流到同一局域网浏览器，可以调整index。

## 启动

在连接相机的设备上运行：

```
python3 camera_socket/camera_mjpeg_server.py --camera 5 --width 640 --height 400 --fps 30
```

- 相机：`/dev/video5`
- 分辨率：`640x400`
- 帧率：`30`
- 格式：`YUYV`
- 端口：`8080`

## 浏览器查看

脚本启动后会打印类似：

```text
LAN:   http://192.168.x.x:8080/
```

在同一个局域网 PC 的浏览器打开这个地址即可查看图像。

如果浏览器打不开，检查设备防火墙是否放行对应端口。

## ROS2 图像话题转 MJPEG

`ros_image_mjpeg_server.py` 用于订阅 ROS2 图像话题，并通过 HTTP MJPEG 推流到浏览器。

默认订阅话题：`/face_tracker_demo/debug_image`

默认端口：`8081`

启动：

```bash
python3 camera_socket/ros_image_mjpeg_server.py
```

指定话题和端口：

```bash
python3 camera_socket/ros_image_mjpeg_server.py \
  --topic /face_tracker_demo/debug_image \
  --port 8081
```

脚本启动后，在同一局域网浏览器打开打印出的 `LAN` 地址即可查看调试画面。
