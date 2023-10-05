#  Robot message is given by
# 
# - robot_stiffness R^nj - k
# - robot_damping R^nj - d
# - robot_feedforward_torque R^nj - tauff
#
#
#  where nj represents the number of joints considered in the chain.
#  These joints are considered as actuated joints.
#  The elements above constitute the following robot control law
#
#  K = diag{k};
#  D = diag{d};
#  tau = K * ( qdes - q ) - D * qdot + tauff

float64[] joints_position_reference
float64[] robot_stiffness
float64[] robot_damping
float64[] robot_feedforward_torque