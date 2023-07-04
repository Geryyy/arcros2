#!/usr/bin/env python3

import rclpy
import arcpy
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from arcros2.msg import TaskSpaceTraj
from arcros2.msg import JointSpaceTraj
from arcros2.msg import ToolParam
from arcros2.msg import PoseState
from arcros2.msg import CartVelTraj
from arcros2.msg import RobotState
from geometry_msgs.msg import Pose, Twist

import numpy as np
from rclpy.node import Node


class IiwaArcNode(Node):

    def __init__(self):
        super().__init__('IiwaArcNode')
        self.declare_parameter('rate', 1000.0)
        self.declare_parameter('client_ip_addr', '127.0.0.1')
        self.declare_parameter('topic_prefix', '/arc/Iiwa/')

        rate = self.get_parameter('rate').value
        ip_addr = self.get_parameter('client_ip_addr').value
        topic_prefix = self.get_parameter('topic_prefix').value
        self.timer = self.create_timer(1, self.timer_callback)

        # create arc robot instance
        self.robot = arcpy.Robots.Iiwa(ip_addr)

        # create name list for the joints
        self.joint_names = list("Joint" + str(i) for i in range(7))

        # subscribers
        self.create_subscription(JointSpaceTraj, topic_prefix + 'set_jointspace_trajectory',
                                 self.callback_jointspace, 1) 
        self.create_subscription(TaskSpaceTraj, topic_prefix + 'set_taskspace_trajectory',
                                 self.callback_taskspace, 1) 
        self.create_subscription(JointState, topic_prefix + 'set_nullspace',
                                 self.callback_nullspace, 1) 
        self.create_subscription(JointSpaceTraj, topic_prefix +
                                 'set_jointspace_jerk_trajectory',
                                 self.callback_jointspace_jerk,
                                 1) 
        self.create_subscription(JointSpaceTraj, topic_prefix + 'set_taskspace_jerk_trajectory',
                                 self.callback_taskspace_jerk, 1) 
        self.create_subscription(CartVelTraj, topic_prefix +
                                 'set_taskspace_velocity_trajectory',
                                 self.callback_taskspace_velocity,
                                 1)
        # js param
        # ts param
        self.create_subscription(ToolParam, topic_prefix + 'set_tool_parameter',
                                 self.callback_toolparam, 1) 

        #publishers
        self.joint_state_publisher = self.create_publisher(JointState, topic_prefix + "joint_state", 0)
        self.joint_set_state_publisher = self.create_publisher(JointState, topic_prefix + "joint_set_state", 0)
        self.cartesian_set_state_publisher = self.create_publisher(PoseState, topic_prefix + "cartesian_set_state", 0)
        self.joint_error_publisher = self.create_publisher(JointState, topic_prefix + "joint_error_state", 0)
        self.time_publisher = self.create_publisher(Time, topic_prefix + "time", 0)
        self.pub_timer = self.create_timer(1/rate, self.pub_timer_callback)
        self.robot_state_publisher = self.create_publisher(RobotState, topic_prefix + "robot_state", 0)

        self.get_logger().info("All Publishers spawned!")

    def pub_timer_callback(self):
        # Run publishers
        self.pub_robot_state()
        self.pub_joint_state()
        self.pub_set_joint_state()
        self.pub_set_cartesian_state()
        self.pub_error_joint_state()
        self.pub_time()

    def timer_callback(self):
        rate = self.get_parameter('rate')
        ip = self.get_parameter('client_ip_addr')
        prefix = self.get_parameter('topic_prefix')
        # self.get_logger().info('Rate: %s' % rate.value)
        new_parameters = [rate, ip, prefix]
        self.set_parameters(new_parameters)

    ### subscriber callbacks

    def callback_jointspace(self,data):
        self.get_logger().info("Send Jointspace Trajectory!")

        joint_vec = list(n.Joints for n in data.JointsArray)
        time_vec = list(n.to_sec() for n in data.t_k)

        self.robot.send_jointspace_trajectory(time_vec, joint_vec)


    def callback_taskspace(self,data):
        self.get_logger().info("Send Taskspace Trajectory!")

        pose_vec = []
        for n in data.PoseArray:
            temp = list(n.position) + list(n.orientation)
            pose_vec.append(temp)

        time_vec = list(n.to_sec() for n in data.t_k)

        self.robot.send_taskspace_trajectory(time_vec, pose_vec)

    
    def callback_taskspace_velocity(self,data):
        self.get_logger().info("Send Taskspace Velocity Trajectory!")
        
        N = len(data.t_k)
        vel = np.zeros((N,6))
        for i in range(N):
            vel[i,:] = [data.linear[i].x, data.linear[i].y, data.linear[i].z, data.angular[i].x, data.angular[i].y, data.angular[i].z]
        
        time_vec = list(n.to_sec() for n in data.t_k)

        self.robot.send_cartesian_velocity_trajectory(time_vec, vel)


    def callback_nullspace(self,data):
        self.get_logger().info("Send Nullspace Configuration!")

        q_ns = data.position
        time = self.robot.get_time() # send now!

        self.robot.send_nullspace_trajectory(time, q_ns)


    def callback_toolparam(self,data):
        self.get_logger().info("Send Tool Parameter!")

        mass = data.m
        cog = [data.spx, data.spy, data.spz]
        time = self.robot.get_time() # send now!

        self.robot.send_toolparam_trajectory(time, mass, cog)

    def callback_jointspace_jerk(self, data):
        self.get_logger().info("Send Jointspace Jerk Trajectory!")

        jerk_points = list(n.Joints for n in data.JointsArray)
        time_vec = list(n.to_sec() for n in data.t_k)

        self.robot.send_jointspace_jerk_trajectory(time_vec, jerk_points)

    def callback_taskspace_jerk(self, data):
        self.get_logger().info("Send Taskspace Jerk Trajectory!")

        jerk_points = list(n.Joints for n in data.JointsArray)
        time_vec = list(n.to_sec() for n in data.t_k)

        self.robot.send_taskspace_jerk_trajectory(time_vec, jerk_points)


    ### publisher methods

    def pub_robot_state(self):
        robot_state = self.robot.state

        # exit()
        msg = RobotState()

        # Populate timestamp
        msg.stamp = self.get_clock().now().to_msg()

        # Populate joint states
        msg.q_act = robot_state.get_q_act().flatten().tolist()
        msg.q_dot_act = robot_state.get_q_dot_act().flatten().tolist()
        msg.q_dotdot_act = robot_state.get_q_dotdot_act().flatten().tolist()

        # Populate joint effort
        msg.tau = robot_state.get_tau().flatten().tolist()

        # Populate joint setpoints
        msg.q_set = robot_state.get_q_set().flatten().tolist()
        msg.q_dot_set = robot_state.get_q_dot_set().flatten().tolist()
        msg.q_dotdot_set = robot_state.get_q_dotdot_set().flatten().tolist()

        # Populate cartesian space setpoints
        # Handling the different order for quaternion
        x_set_pose = Pose()
        x_set = robot_state.get_x_set().flatten().tolist()
        x_set_pose.position.x = x_set[0]
        x_set_pose.position.y = x_set[1]
        x_set_pose.position.z = x_set[2]
        x_set_pose.orientation.w = x_set[3]
        x_set_pose.orientation.x = x_set[4]
        x_set_pose.orientation.y = x_set[5]
        x_set_pose.orientation.z = x_set[6]
        msg.x_set = x_set_pose

        x_dot_set = robot_state.get_x_dot_set().flatten().tolist()
        msg.x_dot_set = Twist()
        msg.x_dot_set.linear.x = x_dot_set[0]
        msg.x_dot_set.linear.y = x_dot_set[1]
        msg.x_dot_set.linear.z = x_dot_set[2]
        msg.x_dot_set.angular.x = x_dot_set[3]
        msg.x_dot_set.angular.y = x_dot_set[4]
        msg.x_dot_set.angular.z = x_dot_set[5]

        x_dotdot_set = robot_state.get_x_dotdot_set().flatten().tolist()
        msg.x_dotdot_set = Twist()
        msg.x_dotdot_set.linear.x = x_dotdot_set[0]
        msg.x_dotdot_set.linear.y = x_dotdot_set[1]
        msg.x_dotdot_set.linear.z = x_dotdot_set[2]
        msg.x_dotdot_set.angular.x = x_dotdot_set[3]
        msg.x_dotdot_set.angular.y = x_dotdot_set[4]
        msg.x_dotdot_set.angular.z = x_dotdot_set[5]

        # Populate other setpoints
        msg.q_ns_set = robot_state.get_q_d_NS().flatten().tolist()

        self.robot_state_publisher.publish(msg)


    def pub_joint_state(self):
        robot_state = self.robot.state

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position = robot_state.get_q_act().flatten().tolist()
        msg.velocity = robot_state.get_q_dot_act().flatten().tolist()
        msg.effort = robot_state.get_tau().flatten().tolist()
        msg.name = self.joint_names

        self.joint_state_publisher.publish(msg)

    def pub_set_joint_state(self):
        robot_state = self.robot.state

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position = robot_state.get_q_set().flatten().tolist()
        msg.velocity = robot_state.get_q_dot_set().flatten().tolist()
        # The effort is not given as a setpoint, instead the joint acceleration
        # is given. There is no message type for this directly so for now I will
        # abuse the JointState message. Might be better to define a custom
        # message. 
        msg.effort = robot_state.get_q_dotdot_set().flatten().tolist()
        msg.name = self.joint_names

        self.joint_set_state_publisher.publish(msg)

    def pub_set_cartesian_state(self):
        robot_state = self.robot.state

        msg = PoseState()
        msg.header.stamp = self.get_clock().now().to_msg()

        pose_set = robot_state.get_x_set().flatten().tolist()
        x_dot_set = robot_state.get_x_dot_set().flatten().tolist()
        x_dotdot_set = robot_state.get_x_dotdot_set().flatten().tolist()
        msg.pose.position = pose_set[:3]
        msg.pose.orientation = pose_set[3:]
        msg.p_vel = x_dot_set[:3]
        msg.omega = x_dot_set[3:]
        msg.p_accl = x_dotdot_set[:3]
        msg.omega_dot = x_dotdot_set[3:]

        self.cartesian_set_state_publisher.publish(msg)

    def pub_error_joint_state(self):
        robot_state = self.robot.state

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position = (robot_state.get_q_act() - robot_state.get_q_set()).flatten().tolist()
        msg.velocity = (robot_state.get_q_dot_act() - robot_state.get_q_dot_set()).flatten().tolist()
        # The effort is not given as a setpoint because it is the input,
        # therefore there is no error
        msg.effort = np.zeros((7,)).tolist()
        msg.name = self.joint_names

        self.joint_error_publisher.publish(msg)

    def pub_time(self):
        msg = self.get_clock().now().to_msg()
        self.time_publisher.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    iiwa_node = IiwaArcNode()
    rclpy.spin(iiwa_node)
    iiwa_node.destroy_node()
    rclpy.shutdown()
