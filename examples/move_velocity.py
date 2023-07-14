import rospy
import arc_ros
import numpy as np


# before you start:
# $catkin build
# $roslaunch arc_gazebo robolab.launch
# $roslaunch arc_ros arc_ros_node.launch

# start example with
# $cd arc/ros/examples
# $python move_robolab.py


rospy.init_node('example_node', anonymous=True) 

iiwa = arc_ros.Iiwa.Iiwa()
rospy.sleep(1)

T_traj = 3

velarr = [0,0,0,0.1,0,0]

print("start moving")

q_init = np.ones((7,1))
t0 = iiwa.get_time()
iiwa.move_jointspace(q_init, t0, T_traj,False)
rospy.sleep(T_traj)

while not rospy.is_shutdown():

  t0 = iiwa.get_time()
  print("start time: " + str(t0))
  iiwa.move_cartesian_velocity(t0, T_traj, velarr)
  print("waiting for " + str(T_traj) + " seconds")
  rospy.sleep(T_traj)

