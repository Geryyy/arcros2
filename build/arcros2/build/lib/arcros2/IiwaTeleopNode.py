#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Time
from arc_ros.msg import TaskSpaceTraj
from arc_ros.msg import JointSpaceTraj
from arc_ros.msg import CartVelTraj
import numpy as np
import arc_ros
from geometry_msgs.msg import Twist


class IiwaTeleopNode:

    def __init__(self):

        # get ip address of network interface at which the client is listening for arc communication
        rate = float(rospy.get_param('~rate', '20.0'))
        self.robot = arc_ros.Iiwa.Iiwa()
        self.T_traj = 1.0

        rospy.sleep(1.0)
        q1 = np.ones((7,1))
        t0 = self.robot.get_time()
        T_traj = self.T_traj
        self.robot.move_jointspace(q1,t0,T_traj,cubic_curve=True)

        # subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.callback_twist) 
        
        # run node
        while not rospy.is_shutdown():

            # run publishers

            if rate:
                rospy.sleep(1/rate)
            else:
                rospy.sleep(1.0)


    def __del__(self):
        self.joint_state_publisher.unregister()


    ### subscriber callbacks

    def callback_twist(self,data):
        rospy.loginfo("Send Cartesian Velocity Trajectory!")

        t0 = self.robot.get_time()
        T_traj = self.T_traj
        velarr = np.array([data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z])
        print("linear: \n", data.linear)
        print("angular: \n", data.angular)
        self.robot.move_cartesian_velocity(t0, T_traj, velarr)


    
if __name__ == '__main__':
    rospy.init_node('iiwa_teleop_node', anonymous=True) 

    try:
        node = IiwaTeleopNode()
    except rospy.ROSInterruptException: 
        pass
