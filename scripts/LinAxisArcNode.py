#!/usr/bin/env python
import rclpy
import arcpy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from arc_ros2.msg import TaskSpaceTraj
from arc_ros2.msg import JointSpaceTraj
from arc_ros2.msg import ToolParam
from arc_ros2.msg import PoseState
import numpy as np
from rclpy.node import Node


class LinAxisArcNode(Node):

    def __init__(self):
        super().__init__('LinAxisArcNode')
        self.declare_parameter('rate', 1000.0)
        self.declare_parameter('client_ip_addr', '127.0.0.1')
        self.declare_parameter('topic_prefix', '/arc/LinearAxis/')

        rate = self.get_parameter('rate').value
        ip_addr = self.get_parameter('client_ip_addr').value
        topic_prefix = self.get_parameter('topic_prefix').value
        self.timer = self.create_timer(1, self.timer_callback)

        # create arc robot instance
        self.robot = arcpy.Robots.LinearAxis(ip_addr)

        # create name list for the joints
        self.joint_names = list("Joint" + str(i) for i in range(7))

        # subscribers
        self.create_subscription(JointSpaceTraj, topic_prefix +
                                 'set_jointspace_trajectory',
                                 self.callback_jointspace, 0) 
        self.create_subscription(JointSpaceTraj, topic_prefix +
                                 'set_jointspace_jerk_trajectory',
                                 self.callback_jointspace_jerk, 0) 
        self.create_subscription(JointSpaceTraj, topic_prefix +
                                 'set_taskspace_jerk_trajectory',
                                 self.callback_taskspace_jerk, 0) 

        #publishers
        self.joint_state_publisher = self.create_publisher(JointState, topic_prefix + "joint_state", 1)
        self.joint_set_state_publisher = self.create_publisher(JointState, topic_prefix + "joint_set_state", 1)
        self.cartesian_set_state_publisher = self.create_publisher(PoseState, topic_prefix + "cartesian_set_state", 1)
        self.joint_error_publisher = self.create_publisher(JointState, topic_prefix + "joint_error_state", 1)
        self.time_publisher = self.create_publisher(Time, topic_prefix + "time", 0)

    def pub_timer_callback(self):
        # Run publishers
        self.pub_joint_state()
        self.pub_set_joint_state()
        self.pub_set_cartesian_state()
        self.pub_error_joint_state()
        self.pub_time()

    def timer_callback(self):
        rate = self.get_parameter('rate')
        ip = self.get_parameter('client_ip_addr')
        prefix = self.get_parameter('topic_prefix')
        # self.get_logger().info('Hello %s!' % rate)
        new_parameters = [rate, ip, prefix]
        self.set_parameters(new_parameters)


    ### subscriber callbacks

    def callback_jointspace(self,data):
        self.get_logger().info("Send Jointspace Trajectory!")

        joint_vec = list(n.Joints for n in data.JointsArray)
        time_vec = list(n.to_sec() for n in data.t_k)

        self.robot.send_jointspace_trajectory(time_vec, joint_vec)

    def callback_jointspace_jerk(self, data):
        self.get_logger().info("Send Jointspace Trajectory!")

        jerk_points = list(n.Joints for n in data.JointsArray)
        time_vec = list(n.to_sec() for n in data.t_k)

        self.robot.send_jointspace_jerk_trajectory(time_vec, jerk_points)

    def callback_taskspace_jerk(self, data):
        self.get_logger().info("Send Taskspace Jerk Trajectory!")

        jerk_points = list(n.Joints for n in data.JointsArray)
        time_vec = list(n.to_sec() for n in data.t_k)

        self.robot.send_taskspace_jerk_trajectory(time_vec, jerk_points)

    ### publisher methods

    def pub_joint_state(self):
        robot_state = self.robot.state

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position = robot_state.get_q_act().flatten().tolist()
        msg.velocity = robot_state.get_q_dot_act().flatten().tolist()
        msg.effort = robot_state.get_tau().flatten().tolist()
        msg.name = list("Joint" + str(i) for i in range(len(msg.position)))

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
        msg.name = list("Joint" + str(i) for i in range(len(msg.position)))

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

        msg.position = robot_state.get_q_act() - robot_state.get_q_set()
        msg.velocity = robot_state.get_q_dot_act() - robot_state.get_q_dot_set()
        # The effort is not given as a setpoint because it is the input,
        # therefore there is no error
        msg.effort = np.zeros((len(msg.position), 1))
        msg.name = list("Joint" + str(i) for i in range(len(msg.position)))

        self.joint_error_publisher.publish(msg)

    def pub_time(self):
        msg = self.get_clock().now().to_msg()
        self.time_publisher.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    linaxis_node = LinAxisArcNode()
    rclpy.spin(linaxis_node)
    linaxis_node.destroy_node()
    rclpy.shutdown()


    
