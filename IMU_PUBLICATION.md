# 可选 ROS IMU 数据面（原 TODO M3）

Provider 支持内部 `sensor_msgs/Imu` 发布，不新增 Atlas capability，也不修改控制命令。
默认关闭。在使用当前源码的 Primitive config 中显式配置：

```yaml
publish_imu: true
imu_topic: /imu/data
imu_frame: measured_imu_frame
```

`measured_imu_frame` 是占位名称，必须替换为已核对的传感器坐标系。程序不自动添加
base_link 到 IMU 的 TF，不把 IMU 原始数据当作已变换到 base_link 的数据。
默认 dev.2 远端 tag 不包含此改动；不能只给旧发布包添加新参数。

消息使用 best-effort QoS，ROS stamp 为本机观察时刻；厂商 imu_ms 只用于去重，
不解释为 ROS epoch。发布检查频率与 odom_rate_hz 相同，但实际 IMU 发布频率取决于
厂商 IMU 时间戳变化；父状态过期/断连时不发布。没有 IMU 字段时不发布空消息。
方向四元数归一化，缺失分量的 covariance[0]=-1；已提供分量的全零 covariance
按 sensor_msgs/Imu 规范表示未知，不代表零误差。实测 covariance 及坐标/单位确认仍待完成。

验证：165 项离线单测通过；隔离网络的 ROS Humble 容器使用真实 Imu 消息运行新增
4 项测试通过，覆盖重复样本、缺失/非法数据、归一化及显式启用门禁。
未连接真机、未重启现有 SDK 容器。后续 verify_m3_ros.py 已通过真实 ROS 发布订阅、
去重、异常状态抑制和恢复验证；总单测更新为 167 项。物理外参和实测协方差仍待验收。
