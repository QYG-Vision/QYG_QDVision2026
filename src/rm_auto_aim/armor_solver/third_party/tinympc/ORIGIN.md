# 来源与本地修改

来源：本地 `sp_vision_25/tasks/auto_aim/planner/tinympc`。
引入 TinyMPC ADMM 实现；保留源文件，随附来源仓库 LICENSE.sp_vision_25。

本地兼容修改：admm.cpp 使用向量末元素索引替代弃用的 Eigen placeholder，
删除 adaptive-rho 分支中未使用的残差局部变量。

数值修正：tiny_api.cpp 的 work->Q/R 保留原始代价，避免预计算再次加入 rho 时重复叠加，
同时避免参考线性项错误使用增广权重。Riccati 预计算收敛阈值由1e-5收紧至1e-12，
迭代上限由1000提升至10000；仅构造时执行。运行时仍为 TinyMPC ADMM，没有活动集求解器。

构建使用 admm.cpp、tiny_api.cpp、rho_benchmark.cpp；codegen.cpp 保留来源但不参与运行库构建。
奇点适配器位于 src/axis_mpc.cpp，负责单位缩放、约束、状态重置、收敛检查和 p/v/a 输出。
