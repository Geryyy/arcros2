import rclpy
import arcros2
import numpy as np


# before you start:
# $catkin build
# $roslaunch arc_gazebo robolab.launch
# $roslaunch arc_ros arc_ros_node.launch

# start example with
# $cd arc/ros/examples
# $python move_robolab.py


rclpy.init_node('example_node', anonymous=True) 

# linear_axis = arc_ros.LinearAxis.LinearAxis()
iiwa = arcros2.Iiwa.Iiwa()

T_traj = 3

while not rospy.is_shutdown():

  t0 = iiwa.get_time()
  q_la = [0.8, 1.2]
  q_iiwa = 1*np.ones((7,1))
      
  # linear_axis.move_jointspace(q_la, t0, T_traj,False)
  iiwa.move_jointspace(q_iiwa, t0, T_traj,False)
  rospy.sleep(T_traj)


  t0 = iiwa.get_time()
  q_la = [0.3,0.5]
  q_iiwa = 0*np.ones((7,1))
      
  linear_axis.move_jointspace(q_la, t0, T_traj,False)
  iiwa.move_jointspace(q_iiwa, t0, T_traj,False)
  rospy.sleep(T_traj)
