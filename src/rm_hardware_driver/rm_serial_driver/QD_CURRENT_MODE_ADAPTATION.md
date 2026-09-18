# QYG `current_mode` 适配 QD 视觉模式说明

## 改动目的

QYG 接收帧中的 `current_mode` 原用于导航模式。当前系统不再需要该导航语义，因此将这个
字节直接用于传输 QD 原生 `VisionMode`。与此同时，`sentry_state` 恢复为独立的 16 位原始
状态字段，不再借用其中的任何比特传输视觉模式。

## 模式约定

下位机应按下表填写接收帧偏移 2 处的 `current_mode`：

| 数值 | QD `VisionMode` | 功能 |
| ---: | --- | --- |
| 0 | `AUTO_AIM_RED` | 自瞄红方目标 |
| 1 | `AUTO_AIM_BLUE` | 自瞄蓝方目标 |
| 2 | `SMALL_RUNE_RED` | 小符红方目标 |
| 3 | `SMALL_RUNE_BLUE` | 小符蓝方目标 |
| 4 | `BIG_RUNE_RED` | 大符红方目标 |
| 5 | `BIG_RUNE_BLUE` | 大符蓝方目标 |

上位机收到合法值后，将其同时写入：

- `SerialReceiveData.current_mode`：保留串口原始字段；
- `SerialReceiveData.mode`：供 `VisionStateMachine` 和 detector/solver 模式服务直接使用。

不再通过 `qyg_enemy_color` 参数拼接敌方颜色，也不再进行 QYG 四状态到 QD 六状态的二次
映射。

## 非法模式处理

QD 当前仅定义模式 `0～5`。收到其他值时：

1. `SerialReceiveData.current_mode` 仍保留收到的原始值，便于排查下位机协议；
2. `SerialReceiveData.mode` 保持上一次合法 QD 模式，避免非法枚举进入状态机；
3. 相同非法值连续出现时只记录一次警告，恢复合法值后再出现异常会重新告警；
4. 上电后尚未收到合法值时，回退模式为 `AUTO_AIM_RED`（编号 0）。

## `sentry_state` 保留规则

`sentry_state` 继续位于接收帧偏移 15，占 2 字节，按小端序传输。上位机不再对它执行
移位、掩码或视觉模式解析，而是完整复制到 ROS 消息：

```text
QYG frame.sentry_state[15:0]
              │ 原值复制
              ▼
SerialReceiveData.sentry_state[15:0]
```

例如线路值为 `0xD234` 时，ROS 消息中仍为 `0xD234`。各状态位的业务含义由状态字段的
生产者和消费者约定，串口驱动不改变其位置。

## 云台控制行为

QD `VisionMode` 不包含 `IDLE`。发送帧的控制使能不再依赖旧 QYG 模式，而只依赖
`GimbalCmd.distance`：

- `distance >= 0`：存在有效目标，允许云台控制；
- `distance < 0`：无有效目标，发送 `mode=0`、`distance=-1`、`target_id=0` 和
  `target_v_yaw=0`。

## 线协议兼容性

本次修改只改变字段语义，不改变二进制布局：

| 项目 | 修改后 |
| --- | --- |
| 接收帧长度 | 39 字节 |
| `current_mode` | 偏移 2，`uint8` |
| `sentry_state` | 偏移 15，`uint16` |
| 接收 CRC | 偏移 37，校验前 37 字节 |
| 发送帧长度 | 34 字节 |

因此帧头、字段偏移、粘包/分包处理和 CRC 算法均保持不变，但下位机必须同步将
`current_mode` 改为 QD 的 `0～5` 编码。

## 代码改动位置

- `include/rm_serial_driver/protocol/qyg_protocol.hpp`
- `src/protocol/qyg_protocol.cpp`
- `include/rm_serial_driver/protocol/qyg_sentry_protocol.hpp`
- `src/protocol/qyg_sentry_protocol.cpp`
- `test/test_qyg_protocol.cpp`
- `../../rm_bringup/config/node_params/serial_driver_params.yaml`

## 验证结果

在仓库 Docker 开发容器 `rv_devel_` 中执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm_serial_driver rm_bringup --parallel-workers 4
./build/rm_serial_driver/test_qyg_protocol
```

两个包均构建成功，QYG 协议的 11 项测试全部通过。测试覆盖以下行为：

- `current_mode=0～5` 原值解码为 QD 模式；
- `current_mode=6` 和 `0xFF` 被判定为非法；
- `sentry_state=0xD234` 完整通过序列化和解析；
- 接收帧长度、字段偏移和 CRC 校验保持不变；
- 原有发送帧、噪声、分片和粘包测试继续通过。
