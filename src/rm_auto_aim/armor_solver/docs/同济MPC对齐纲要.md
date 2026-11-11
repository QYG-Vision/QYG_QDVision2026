# 同济 MPC 对齐纲要

## 基线和范围

- 奇点源码基线：1159aa3；当前 c9b5697 的文件树与该版本一致。
- 对照：本机 sp_vision_25 的 bd9f5e7，tasks/auto_aim/planner/planner.cpp。
- 按用户要求，保留奇点时间补偿与弹道实现；对齐截图中的选板、参考窗口、负时间采样、火控、输出条件及 MPC 求解调用。
- 不改检测器、跟踪器、ROS 消息和串口协议，也不声称它们与济瞄完全一致。

## 改动

1. Solver::solve 不再调用 get_aim/get_aim_positon。所有模式统一使用整车候选板；旧朝向窗口、15° 放宽、间接选板粘滞和等待时间不参与 MPC 路径。
2. 每个参考采样点按水平距离最近选择装甲板，与济瞄 Planner::aim 一致。窗口可包含未来换板，不固定本轮的一块板。
3. 时间补偿先对候选状态调用奇点 solve_impact_state，保留图像年龄、predict2send_delay、additional_prediction_time 和原飞行时间求解。之后按济瞄顺序回退 -(50+1)*0.01 s，再逐点推进；不把负采样时间送入会截断时间的预测函数，也不叠加旧等待补偿。
4. 角度参考和角速度按济瞄 get_trajectory 构造：yaw 相对当前 yaw0 归一化，角速度使用前后采样点中心差分，包括窗口端点。窗口 100 点、10 ms，输出第 50 点。
5. TinyMPC 内部改用 rad，公共接口保留 degree 以兼容奇点串口。保留 9e6/0/1 权重、10 次迭代及等价的 50/100 rad/s² 限制。沿用济瞄工作状态，不额外清空对偶变量，不对输出加速度再钳位；不因 tiny_solve 非零返回值丢弃有限结果。
6. tiny_api.cpp 直接采用上述济瞄版本，恢复其权重初始化、预计算迭代上限和停止条件。其他库文件保留现有 Eigen 兼容和消除未使用变量的构建适配。
7. 火控直接比较第 52 点参考与两轴 MPC 预测角误差的 hypot，阈值 solver.mpc.fire_thresh=0.003 rad（采用济瞄 standard4.yaml/demo.yaml 的数值）。不再附加原 isSuggestFire、弹速门槛或实际反馈对准条件；弹道本身无效仍不输出有效规划。
8. 不以实际反馈是否存在、反馈年龄、反馈模式、反馈坐标系或目标年龄阈值阻断规划。tracking、输入有效性和有限输出检查保留。TF 姿态用于可视化，缺失不阻断规划；实际反馈只用于误差诊断。ROS 状态机、串口编码的接口检查仍属现有适配层。

## 保留内容

- predict2send_delay=0.17 s、controller_delay、additional_prediction_time 配置值不变；controller_delay 不再用于已替换的旧火控链。
- Ceres 线阻力弹道、重力/阻力参数、弹速估计、手动角度补偿保持原实现。没有引入济瞄的默认 22 m/s 回退，以免改变奇点弹道输入。
- 奇点跟踪器的整车状态、候选板几何、ROS 调度和串口接收端未替换。因此这是规划路径对齐，不是整套自瞄移植。
- 旧 use_armor_top、feedback_timeout、target_timeout 参数保留兼容，当前规划路径不使用对应切换/门槛。

## 验证

- MPC 核心使用本机 Eigen/gtest 独立编译，6/6 测试通过；包含静态、匀速、约束、非法输入和连续三轮换板对照测试。
- axis_mpc.cpp 通过 -Wall -Wextra -Werror 语法检查；git diff --check 通过；tiny_api.cpp 与济瞄原文件逐字一致。
- 对照测试直接复现济瞄 tiny_setup/tiny_set_x0/Xref/tiny_solve 调用，比较第 50 点角度/速度/加速度及第 52 点角度。
- ROS 测试已更新：无反馈输出、整车参考、反馈模式/坐标系/年龄不阻断、无 TF 不阻断、弹道保留、窗口逐点最近板参考。
- 当前无 ROS 安装及可用 Docker 镜像，未完成 ROS 包构建、集成测试或实车验证。应在项目容器重建 armor_solver 后运行 colcon test --packages-select armor_solver，再查看 colcon test-result --verbose。
