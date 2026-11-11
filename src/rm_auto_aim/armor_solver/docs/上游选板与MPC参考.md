# 上游选板与 MPC 参考

运行路径为 `Solver::solve()` → `build_aim_reference()` → `plan_mpc()`。

- 跟踪器继续通过可见板更新整车状态，不要求相机先看到下一板。
- `build_aim_reference()` 在上游为整个窗口生成追板/等板决定和角度序列。
- `plan_mpc()` 仅求解给定序列，不读取整车几何，不重新选板，不决定开火。
- 火控使用同一参考中点的板、击打窗口以及实际反馈。等待位置不可开火。

## 决策规则

整车模式每个采样点先按图像年龄、发送延迟及飞行时间预测所有板。
在 `max_orientation_angle` 朝向窗口内优先保持当前板，否则选择摆动代价较小的直接板。
没有直接板时，旋转目标按角速度符号计算各板下一次进入朝向边界的时间，选择最先进入者并参考其到达位置。
旧直接选板的额外 15° 和间接选板的 0.5 rad 粘滞评分不参与新运行路径。

当前板依然可见时，未来采样中就可以出现下一板的等待/跟踪参考。
等待位置是板未来到达的位置，不是两个板之间人为插值的击打点。
窗口中点决定本轮当前目标；未来采样不会修改跨帧保持状态。
成功规划后才保存本轮目标，下一轮按相位和时间重新关联，因此数组索引仅在本帧内有意义。
已经决定等待下一板时，只要仍同向接近进入边界，就保持该决定，避免旧板在边界抖动时重新抢占；反转时重新选择。
当前版本不根据单次求解失败擅自换目标，不做多候选优化结果的反复比较。

`top0` 且 `use_armor_top=true` 保持单板平移模型，调试索引为 -1。
`top1/top2` 使用各自 `max_orientation_angle`，配置单位 degree。
`top2.max_orientation_angle=0` 是零宽直接朝向窗口：绝大多数时刻会处于等板，火控也受此限制。

## 时间与求解

保留原有 100 点、10 ms 步长、中点输出，未调整权重、加速度约束和迭代上限。
样本 k 相对中点的时间为 `(k-output_index)*dt`，额外预测前移只应用一次。
中点之前样本通过运动模型负时间外推，不能截成同一个状态。
当前 MPC 仍使用参考起点初始化，不代表基于实际反馈计算了转移可达性。
完整未来参考不保证每帧发送角连续，也不保证所有工况收敛；需要用回放/实测检查。

## 查看已有调试话题

`/armor_solver/mpc_debug` 继续承载两轴诊断，用 `axis` 区分，先发 yaw 再发 pitch。
现在即使另一轴未执行，也发 `stage=not_run`，避免误读旧结果。

新增字段：

- `reference_valid/reference_status`：上游参考有效性，不能等同于两轴求解成功。
- `reference_dt/reference_output_index`：采样间隔和当前规划点。
- `current_armor_index`：当前决定的板（可能正在等它），-1 为单板平移。
- `next_armor_index/time_to_switch_s`：中点之后首次改选另一板及其相对时间；没有则为 -1。
- `reference_angles_deg`：本轴实际交给 MPC 的完整参考。
- `reference_armor_indices`：每个点对应的本帧板索引。
- `reference_fire_allowed`：每个点是否在直接击打窗口，1/0；不是最终开火命令。
- `reference_wait_durations_s`：各点等到板进入窗口所需时间，直接板为 0。

用 `ros2 topic echo /armor_solver/mpc_debug --filter "m.axis == 'yaw'"` 单独查看 yaw。
应同时记录 `/armor_solver/cmd_gimbal` 与 `/serial/receive`，区分参考、规划、实际反馈和无效帧。

## 构建与验证

本次扩展了 `rm_interfaces/msg/MpcDebug.msg`，需要在项目容器内重建消息及其依赖：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to armor_solver --parallel-workers 4
source install/setup.bash
colcon test --packages-select armor_solver --event-handlers console_direct+
colcon test-result --verbose
```

节点必须加载同一套新消息与配置后重启。启动仍使用项目的 `bringup_SingleProcess.launch.py`。

## 已知验证边界

独立替身环境中，中心 x=5.2 m、半径 0.2 m、角速度 3 rad/s、30° 朝向窗口的正转用例，
当前相位 0.2 rad 时，未来换板参考能在现有 100 次上限内求解；
当前相位 0.45 rad、接近切板时，100/300/1000/3000 次上限探针仍返回未收敛。
这说明决策接通不等于收敛问题已经解决，也不能仅靠继续增加迭代次数。
保留求解失败的无效输出，未放宽收敛检查；真实弹道、ROS 定时和实车跟随仍需验证。
