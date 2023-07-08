#  Task message is given by
# 
# - task pose R^6 [x,y,z,roll,pitch,yaw] {meters,radiants}
# - task wrench R^6 [fx,fy,fz,mx,my,mz]
# - task precision R^6 [x,y,z,roll,pitch,yaw] {meters,radiants}
#
#  These quantities should come from a task planner.
#  > If the task wrench is not known, you can insert zero, 
#  > If the task precision is not known, you CAN NOT insert zero 
#	 Please specify a value for your application even if it is a rough approssimation.

bool cartesian_space
float64[] joints_position
float64[] joints_position_reference
float64[] task_pose_reference
float64[] task_wrench
float64[] task_precision
