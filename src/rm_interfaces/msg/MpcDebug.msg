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
