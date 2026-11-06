# 执行器实际状态；0 表示没有对应轨迹。撤销编号不会在同一 session 中复用。
std_msgs/Header header
uint64 session
int32 current_traj_id
int32 pending_traj_id
int32 retired_traj_id
string reason
