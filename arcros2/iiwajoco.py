import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import time
import mujoco
import mujoco.viewer
import numpy as np 
from scipy.spatial.transform import Rotation
import arc_core
import arcpy
import os
import ament_index_python



class Hello(Node):

    def __init__(self):
        super().__init__('Hello')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # initialize mujoco
        package_path = ament_index_python.get_package_share_directory('arcros2')
        print('package_path: ', package_path)
        assets_path = os.path.join(package_path,"assets")
        xml_filepath = os.path.join(assets_path,"mj_shelf.xml")
        with open(xml_filepath, 'r') as f:
          xml = f.read()

        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)

        self.Ts = self.model.opt.timestep
        self.__init_arc()

        # prime controller with init traj
        q0 = self.data.qpos[0:7]
        T_traj = self.Ts
        self.arc_contr.start(self.data.time, q0, q0, T_traj)

        timer_period = self.Ts
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i_step = 0

        self.time_begin = time.time()

    
    def __init_arc(self):
        Ts = self.Ts
        sim_flag = True
        ctr_param = arc_core.Iiwa.IiwaContrParam(sim_flag)
        f_c_slow_norm = ctr_param.f_c_slow * (2*Ts)
        robot_model = arc_core.Iiwa.RobotModel(f_c_slow_norm,Ts)

        js_param = arc_core.Iiwa.JointCTParameter(ctr_param)
        ts_param = arc_core.Iiwa.CartesianCTParameter(ctr_param)
        f_c_fast_norm = ctr_param.f_c_fast * (2*Ts)
        sp_param = arc_core.Iiwa.SingularPerturbationParameter(ctr_param.K_sp, ctr_param.D_sp, f_c_fast_norm)
        B_fc = robot_model.get_B()
        fc_param = arc_core.Iiwa.FrictionCompensationParameter(ctr_param.L_fc, B_fc, f_c_fast_norm)
        gc_param = arc_core.Iiwa.GravityCompParameter(ctr_param.D_gc)

        hanging = False
        arc_contr = arc_core.Iiwa.LBRIiwa(robot_model, Ts, js_param, ts_param, sp_param, fc_param, gc_param, hanging)
        arc_contr.set_singular_perturbation_state(False)
        arc_contr.set_friction_compensation_state(False)

        gripper_mass = 0.925
        gripper_spx = 0
        gripper_spy = 0
        gripper_spz = 0.058
        arc_contr.set_toolparam(gripper_mass, gripper_spx, gripper_spy, gripper_spz)

        com_server = arc_core.Iiwa.TrajectoryServer(arc_contr)
        
        self.com_server = com_server
        self.robot_model = robot_model
        self.arc_contr = arc_contr


    def timer_callback(self):
        time = self.i_step*self.Ts
        time_now = time.time() - self.time_begin
        # print time vs time_now with annotation and fixed format 2.3 decimal places
        print('time: %2.3f\ttime_now: %2.3f' % (time, time_now))

        if self.i_step < 20:
            exit()


        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i_step += 1


def main(args=None):
    rclpy.init(args=args)

    hello_world = Hello()

    rclpy.spin(hello_world)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hello_world.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
