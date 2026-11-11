std_msgs/Header header
string target_id
string axis
# status = -1 and iterations = 0 if tiny_solve was skipped.
string stage
int32 status
int32 iterations
bool x_finite
bool u_finite
int32 max_iterations
float64 max_acceleration_deg_s2
float64 max_abs_reference_second_diff_deg
int32 second_diff_peak_index
float64 primal_residual_state
float64 primal_residual_input
float64 dual_residual_state
float64 dual_residual_input

# 上游选板安排；数组顺序与本轴 reference_angles_deg 对应。
# 索引仅对本帧有效，-1 为平移单板或没有下一板。
bool reference_valid
string reference_status
float64 reference_dt
int32 reference_output_index
int32 current_armor_index
int32 next_armor_index
float64 time_to_switch_s
float64[] reference_angles_deg
int32[] reference_armor_indices
uint8[] reference_fire_allowed
float64[] reference_wait_durations_s
