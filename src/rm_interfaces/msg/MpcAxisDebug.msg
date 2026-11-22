# 单轴求解诊断；attempted=false 时数值无效，accepted 与 converged 含义独立。
bool attempted
bool accepted
bool converged
string status
int32 iterations
float64 solve_time_ms
float64 max_acceleration
float64 acceleration_tolerance
float64 position
float64 velocity
float64 acceleration
float64 acceleration_excess
float64 primal_residual_state
float64 primal_residual_input
float64 dual_residual_state
float64 dual_residual_input
# 与参考采样点对齐；角度 rad，角速度 rad/s，角加速度 rad/s²（少一个末端点）。
float64[] positions
float64[] velocities
float64[] accelerations
