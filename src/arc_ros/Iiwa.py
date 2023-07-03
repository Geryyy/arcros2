import rospy
import arcpy
import numpy as np
import tf.transformations as tftrans
from std_msgs.msg import Header
from std_msgs.msg import Time
from sensor_msgs.msg import JointState
from arc_ros.msg import TaskSpaceTraj
from arc_ros.msg import JointSpaceTraj
from arc_ros.msg import Joints
from arc_ros.msg import Pose
from arc_ros.msg import ToolParam
from arc_ros.msg import CartVelTraj
from geometry_msgs.msg import Vector3

import matplotlib.pyplot as plt
import scipy.spatial.transform as sst



#OK
class Iiwa:

  def __init__(self):
    self.robot = arcpy.Iiwa.RobotModel()
    self.time = 0
    topic_prefix = rospy.get_param('~topic_prefix', '/arc/Iiwa/')

    # subscriber
    self.joint_state_sub = rospy.Subscriber(topic_prefix + "joint_state", JointState, self.stateCallback, queue_size=1)
    self.time_sub = rospy.Subscriber(topic_prefix + "time", Time, self.timeCallback, queue_size=1)
  
    self.q = np.zeros((7,1))
    self.q_dot = np.zeros((7,1))
    self.q_dotdot = np.zeros((7,1))
    self.H_0_e = np.eye(4)
    self.tau = np.zeros((7,1))

    ## publisher
    self.jointspace_pub = rospy.Publisher(topic_prefix + 'set_jointspace_trajectory',JointSpaceTraj,queue_size=1)
    self.jointspace_jerk_pub = rospy.Publisher(topic_prefix + 'set_jointspace_jerk_trajectory',JointSpaceTraj,queue_size=1)
    self.taskspace_pub = rospy.Publisher(topic_prefix + 'set_taskspace_trajectory',TaskSpaceTraj,queue_size=1)
    self.taskspace_vel_pub = rospy.Publisher(topic_prefix + 'set_taskspace_velocity_trajectory',CartVelTraj,queue_size=1)
    self.taskspace_jerk_pub = rospy.Publisher(topic_prefix + 'set_taskspace_jerk_trajectory',JointSpaceTraj,queue_size=1)
    self.toolpar_pub = rospy.Publisher(topic_prefix + 'set_tool_parameter',ToolParam,queue_size=1)
    self.nullspace_pub = rospy.Publisher(topic_prefix + 'set_nullspace',JointState,queue_size=1)


    self.N_trajectory_points = 10 

    # stored timing of old trajectory
    self.t0_old = 0
    self.t1_old = 0

  def __del__(self):
    self.jointspace_pub.unregister()
    self.taskspace_pub.unregister()
    self.toolpar_pub.unregister()

#region Subscriber
  ## subscribe 
  def stateCallback(self, msg):
    stamp = msg.header.stamp
    t = stamp.secs + stamp.nsecs * 10**(-9)
    self.q = np.array(msg.position).reshape(7, 1)
    self.q_dot = np.array(msg.velocity).reshape(7, 1)
    self.q_dotdot = 0 * self.q_dot
    self.tau = np.array(msg.effort).reshape(7,1)

    self.robot.update(self.q, self.q_dot, self.q_dotdot)

    self.H_0_e = self.robot.get_H_0_e()

  def timeCallback(self, msg):
    self.time = msg.data.to_sec()

  def get_time(self):
      return self.time

  def get_ros_time(self):
      return rospy.Time.from_sec(self.time)

#endregion

#region publisher
  def set_toolparameter(self,m7,sp7x,sp7y,sp7z):
    header_obj = Header()
    header_obj.stamp = rospy.get_rostime()
    header_obj.frame_id = "Tool Parameter"
    header_obj.seq = header_obj.seq + 1
    msg = ToolParam()
    msg.header = header_obj

    msg.m = m7 
    msg.spx = sp7x
    msg.spy = sp7y
    msg.spz = sp7z

    self.toolpar_pub.publish(msg)


  def set_nullspace(self,q_d_NS_):
    msg = JointState()
    msg.position = q_d_NS_
    msg.header = Header()
    #print(msg)
    self.nullspace_pub.publish(msg)


  def _publish_taskspace_trajectory(self,t_k,posarr):
    header_obj = Header()
    # header_obj.seq = header_obj.seq + 1
    header_obj.frame_id = "TaskspaceTrajectory"
    header_obj.stamp = rospy.get_rostime()
    header_obj.seq = header_obj.seq + 1
    msg = TaskSpaceTraj()
    msg.header = header_obj
    msg.PoseArray = posarr
    msg.t_k = t_k
    msg.OrientationAsQaut = True
    self.taskspace_pub.publish(msg)

  
  def _publish_taskspace_velocity_trajectory(self,t_k,velarr):
    header_obj = Header()
    # header_obj.seq = header_obj.seq + 1
    header_obj.frame_id = "TaskspaceVelocityTrajectory"
    header_obj.stamp = rospy.get_rostime()
    header_obj.seq = header_obj.seq + 1
    msg = CartVelTraj()

    # assign velarr to msg.linear and msg.angular
    msg.header = header_obj

    for i in range(velarr.shape[0]):
      msg.linear.append(Vector3(velarr[i,0],velarr[i,1],velarr[i,2]))
      msg.angular.append(Vector3(velarr[i,3],velarr[i,4],velarr[i,5]))
  
    msg.t_k = t_k
    self.taskspace_vel_pub.publish(msg)


  def _publish_jointspace_trajectory(self,t_k,JointsList):
    JointsArray = []
    for elem in JointsList:
      jmsg = Joints()
      jmsg.Joints = elem
      JointsArray.append(jmsg)

    header_obj = Header()
    header_obj.frame_id = "JointspaceTrajectory"
    header_obj.stamp = rospy.get_rostime()
    header_obj.seq = header_obj.seq + 1
    msg = JointSpaceTraj()
    msg.header = header_obj
    msg.JointsArray = JointsArray
    msg.t_k = t_k
    #print(msg)
    self.jointspace_pub.publish(msg)

  def _publish_jointspace_jerk_trajectory(self, t, jerk_points):
    jerk_array = []
    for elem in jerk_points:
      jmsg = Joints()
      jmsg.Joints = elem
      jerk_array.append(jmsg)

    header_obj = Header()
    header_obj.frame_id = "JointspaceJerkTrajectory"
    header_obj.stamp = rospy.get_rostime()
    header_obj.seq = header_obj.seq + 1
    msg = JointSpaceTraj()
    msg.header = header_obj
    msg.JointsArray = jerk_array
    msg.t_k = t
    self.jointspace_jerk_pub.publish(msg)

  def _publish_taskspace_jerk_trajectory(self, t, jerk_points):
    jerk_array = []
    for elem in jerk_points:
      jmsg = Joints()
      jmsg.Joints = elem
      jerk_array.append(jmsg)

    header_obj = Header()
    header_obj.frame_id = "TaskspaceJerkTrajectory"
    header_obj.stamp = rospy.get_rostime()
    header_obj.seq = header_obj.seq + 1
    msg = JointSpaceTraj()
    msg.header = header_obj
    msg.JointsArray = jerk_array
    msg.t_k = t
    self.taskspace_jerk_pub.publish(msg)

#endregion 


  def move_jointspace(self,q1,t0,T_traj,cubic_curve=False):
    q0 = self.q
    # JointSpaceTrajectory1 .. Joint space trajectory calculation using cubic
    # polynomial. Start and end velocity is set to zero.
    # input:
    # q0 .. start position
    # q1 .. end position
    # t0 .. start time
    # T_traj .. trajectory move duration
    # output:
    # q .. calculated joint positions. m x n matrix with n joints and m points
    # q_p .. joint velocities, m x n matrix
    # t .. trajectory time points
  

    N = self.N_trajectory_points # nr of waypoints
    n = len(q0)
    # polynomial coefficients
    #t0 = 0.0
    t1 = t0 + T_traj

    a0 = q0
    # a1 = 0
    a2 = np.zeros((n,1))
    a3 = np.zeros((n,1))

    A = np.matrix([[1,1],[3,2]],dtype=np.double)

    for i in np.arange(0,n):
      b = np.matrix([[q1[i]-q0[i]],[0]],dtype=np.double)
      x = np.linalg.solve(A,b)
      a3[i]= x[0]
      a2[i] = x[1]

    t = np.linspace(t0,t1, int(N),endpoint=True)
    m = len(t)

    joint_vec = []
    joint_dot_vec = []
    joint_dotdot_vec = []
    t_k = []
    for i in range(m):
      ti = (t[i] - t0)/(t1-t0)
      # print("ti: " + str(ti))

      if cubic_curve:
        # cubic interpolation
        joint_vec.append(np.squeeze(a3*ti**3 + a2*ti**2 + a0))
        joint_dot_vec.append(np.squeeze(3*a3*ti**2 + 2*a2*ti))
        joint_dotdot_vec.append(np.squeeze(6*a3*ti + 2*a2))
      else:

        hard_lin = False # False..constant acceleration and deacceleration phase, True..velocity jump at start and stop 
        if hard_lin:
          ri = ti # linear, velocity jump at start and stop!
          vi = 1.0
        else:
          # smooth start and stop
          # for more details see: to 2021_03_14_Trajektorienplanung.mw
          x = 0.1 # x in (0, 0.5), acceleration and deacceleration fraction of trajectory, in between constant velocity
          A = -1 / x / (-1 + x)
          # path parameter
          if ti <= 0:
            ri = 0
          else:
            if ti <= x:
              ri = A/2 * ti ** 2 
            else:
              if ti <= 1 - x:
                ri = A/2 * x * (2 * ti - x)  
              else:
                if ti <= 1:
                  ri = A/2 * (-ti ** 2 - 2 * x ** 2 + 2 * ti + 2 * x - 1)
                else:  
                  if 1 < ti:
                    ri = A/2 * -2 * x * (-1 + x)
                  else:
                    ri = 0

          # first derivative of path parameter
          if ti <= 0:
            vi = 0
          else:
            if ti <= x:
              vi = A * ti
            else:
              if ti <= 1 - x:
                vi = A * x
              else:
                if ti <= 1:
                  vi = A * (1-ti)
                else:  
                  if 1 < ti:
                    vi = 0
                  else:
                    vi = 0

        # linear interpolation
        q1_ = np.array(q1).reshape((7,1))
        q0_ = np.array(q0).reshape((7,1))
        q_ = (q1_- q0_)*ri + q0_
        q_p_ = (q1_ - q0_)/(t1-t0) * vi
        joint_vec.append(np.squeeze( q_ ))
        joint_dot_vec.append(np.squeeze(  q_p_ ))
        joint_dotdot_vec.append(np.squeeze( np.zeros((n,1)) ))

      # print("t[i]-t0: " + str(t[i]-t0))
      t_k.append(rospy.Time.from_sec(t[i]))

    # save trajectory timing only if new trajectory
    if t0 >= self.t1_old:
      print("Move jointspace new trajectory!")
      self.t1_old = t1
      self.t0_old = t0
    else:
      print("Move jointspace trajectory update!")

    # send trajectory
    self._publish_jointspace_trajectory(t_k, joint_vec)

    # return traj. if you want to take a look
    return t_k, joint_vec, joint_dot_vec, joint_dotdot_vec
    

  def move_taskspace(self,H1,t0,T_traj,debug=False,cubic_curve=False):
    H0 = self.H_0_e
    quat_start = tftrans.quaternion_from_matrix(H0)
    quat_R1 = tftrans.quaternion_from_matrix(H1)

    # flip end quaternion?
    diff = quat_start - quat_R1
    sum = quat_start + quat_R1
    if diff.dot(diff) < sum.dot(sum):
      quat_end = quat_R1
    else:
      quat_end = -quat_R1

    N = self.N_trajectory_points
    t1 = t0 + T_traj
    t = np.linspace(t0,t1, int(N),endpoint=True)
    p_start = H0[0:3,3]
    p_end = H1[0:3,3] 

    posarr = []
    t_k = []

    r = np.zeros((N,1))
    v = np.zeros((N,1))

    for i in np.arange(0,N):

      if t0 < self.t1_old:
        # print("t0: " + str(t0))
        # print("t1_old: " + str(self.t1_old))
        ti = (t[i] - self.t0_old)/(t1-self.t0_old)
      else:
        ti = (t[i] - t0)/(t1-t0)
      
      # ti = i /float(N-1) # i = 0 .. (N-1)

      ### normalized cubic polynomial as pathparameter
      ### provides a trajectory that can be continuously differentiated twice

      if cubic_curve:
        r[i] = -2*ti**3 + 3*ti**2 # cubic
      else:
        hard_lin = False
        if hard_lin:
          r[i] = ti # linear, velocity jump at start and stop!
        else:
          # smooth start and stop
          # for more details see: to 2021_03_14_Trajektorienplanung.mw
          x = 0.2 # x in (0, 0.5), acceleration and deacceleration fraction of trajectory, in between constant velocity
          A = -1 / x / (-1 + x)
          # path parameter
          if ti <= 0:
            r[i] = 0
          else:
            if ti <= x:
              r[i] = A/2 * ti ** 2 
            else:
              if ti <= 1 - x:
                r[i] = A/2 * x * (2 * ti - x)  
              else:
                if ti <= 1:
                  r[i] = A/2 * (-ti ** 2 - 2 * x ** 2 + 2 * ti + 2 * x - 1)
                else:  
                  if 1 < ti:
                    r[i] = A/2 * -2 * x * (-1 + x)
                  else:
                    r[i] = 0

          # first derivative of path parameter
          if ti <= 0:
            v[i] = 0
          else:
            if ti <= x:
              v[i] = A * ti
            else:
              if ti <= 1 - x:
                v[i] = A * x
              else:
                if ti <= 1:
                  v[i] = A * (1-ti)
                else:  
                  if 1 < ti:
                    v[i] = 0
                  else:
                    v[i] = 0

    
    for i in np.arange(0,N):
      # rescale r[i] to 0..1
      ri =  (r[i]- r[0]) / (r[-1] - r[0])

      quat = tftrans.quaternion_slerp(quat_start,quat_end,ri)
      p = Pose()
      x = np.interp(ri,[0,1.0],[p_start[0],p_end[0]])
      y = np.interp(ri,[0,1.0],[p_start[1],p_end[1]])
      z = np.interp(ri,[0,1.0],[p_start[2],p_end[2]])
      p.position = [x,y,z]
      p.orientation = [quat[3], quat[0], quat[1], quat[2]] # w x y z
      posarr.append(p)
      t_k.append(rospy.Time.from_sec(t[i]))

    # save trajectory timing only if new trajectory
    if t0 >= self.t1_old:
      print("Move taskspace new trajectory!")
      self.t1_old = t1
      self.t0_old = t0
    else:
      print("Move taskspace trajectory update!")

    

    ## 3D plot of trajectory
    if debug:
      # plot trajectory
      fig = plt.figure()
      ax = fig.gca(projection='3d')
      ax.set_aspect("equal")

      x = []
      y = []
      z = []
      u = []
      v = []
      w = []

      for pos in posarr:
        # scipy spatial quaternion: scalar last notation
        tf_quat = pos.orientation
        sst_quat = [tf_quat[1],tf_quat[2],tf_quat[3],tf_quat[0]]
        rot = sst.Rotation.from_quat(sst_quat)
        RotM = rot.as_dcm()

        ## draw graticule (position and orientation) with quiver
        # x axis
        x.append(pos.position[0])
        y.append(pos.position[1])
        z.append(pos.position[2])

        u.append(RotM[0,0])
        v.append(RotM[1,0])
        w.append(RotM[2,0])

        # y axis
        x.append(pos.position[0])
        y.append(pos.position[1])
        z.append(pos.position[2])

        u.append(RotM[0,1])
        v.append(RotM[1,1])
        w.append(RotM[2,1])

        # z axis
        x.append(pos.position[0])
        y.append(pos.position[1])
        z.append(pos.position[2])

        u.append(RotM[0,2])
        v.append(RotM[1,2])
        w.append(RotM[2,2])
    
      #draw the arrow
      ax.quiver(x,y,z,u,v,w,length=0.05)

      plt.show()
    
    # send trajectory 
    self._publish_taskspace_trajectory(t_k, posarr)

    # return traj. just in case
    return t_k, posarr
  

  def move_cartesian_velocity(self, t0, T_traj, velarr):
    """Move the robot based on the velocity input at the specified times. 
    
    Parameters
    ----------
    t0 : float
        Time at which the trajectory starts
    T_traj : float
        Total time of the trajectory
    velarr : array 1 x 6
        Velocity vector for linear and angular velocity in x, y, z, roll, pitch, yaw
    """
    # t_k starts at t0 and ends at t0 + T_traj with self.N points
    t_ = np.linspace(t0, t0 + T_traj, self.N_trajectory_points)
    # convert t_k to rostime
    t_k = [rospy.Time.from_sec(t) for t in t_]

    # v_k is the velocity at each time step with last value being 0
    v_k = np.zeros((self.N_trajectory_points, 6))
    v_k[0:-1,:] = velarr
    
    # send trajectory
    self._publish_taskspace_velocity_trajectory(t_k, v_k)



  def move_jointspace_jerk(self, time_points, jerk_matrix):
    """Move the robot based on the jerk input at the specified times. The
    trajectory is calculated by integrating linear jerk functions. 
    
    Parameters
    ----------
    time_points : array t x 1
        Time points at which the jerk values are specified
    jerk_matrix : matrix j x t
        Jerk points for every time step and every axis 
    """
    if time_points[0] >= self.t1_old:
      print("Move jointspace jerk new trajectory!")
      self.t1_old = time_points[-1]
      self.t0_old = time_points[0]
    else:
      print("Move jointspace jerk trajectory update!")

    # Convert to ROS time array
    ros_times = []
    for i in range(time_points.shape[0]):
        ros_times.append(rospy.Time.from_sec(time_points[i]))

    self._publish_jointspace_jerk_trajectory(ros_times, jerk_matrix.T.tolist())

  def move_taskspace_jerk(self, time_points, jerk_matrix):
    """Move the robot based on the jerk input at the specified times. The
    trajectory is calculated by integrating linear jerk functions. 
    
    Parameters
    ----------
    time_points : array t x 1
        Time points at which the jerk values are specified
    jerk_matrix : matrix 6 x t
        Jerk points for every time step and every spacial dimension
    """
    if time_points[0] >= self.t1_old:
      print("Move taskspace jerk new trajectory!")
      self.t1_old = time_points[-1]
      self.t0_old = time_points[0]
    else:
      print("Move taskspace jerk trajectory update!")

    # Convert to ROS time array
    ros_times = []
    for i in range(time_points.shape[0]):
        ros_times.append(rospy.Time.from_sec(time_points[i]))

    self._publish_taskspace_jerk_trajectory(ros_times, jerk_matrix.T.tolist())

  def homogeneous_transformation_endeffector (self):
    return self.robot.get_H_0_e()

  #unittest
  def jacobian_endeffector_p (self):
    return self.robot.get_J_dot()

  #unittest
  def jacobian_endeffector_pp (self):
    return self.robot.get_J_dotdot()

  #unittest
  def jacobian_endeffector (self):
    return self.robot.get_J()


  #unittest
  def coriolis_matrix (self):
    return self.robot.get_C()

  #unittest
  def inertia_matrix (self):
    return self.robot.get_M()

  #unittest
  def gravitational_force (self):
    return self.robot.get_G()


  #tested
  def inverse_kinematics_analytic(self,H_e,find_minimum_norm=False):
    #return self.robot.inverse_kinematics_analytic(H_e,find_minimum_norm)
    # not implemented
    pass



if __name__ == '__main__':
  rospy.init_node('iiwa_node', anonymous=True) 

  iiwa = Iiwa()
  
  T_traj = 3

  while not rospy.is_shutdown():

    q1 = 1*np.ones((7,1))
    t0 = rospy.Time.now().to_sec()
        
    iiwa.move_jointspace(q1, t0, T_traj,False)
    rospy.sleep(T_traj)
    
    
    t0 = rospy.Time.now().to_sec()
    H0 = iiwa.homogeneous_transformation_endeffector()
    dH = np.eye(4)
    dH[0:3,3] = [0.1,0.1,0.1]
    H1 = np.matmul(H0,dH)
        
    iiwa.move_taskspace(H1, t0, T_traj)
    rospy.sleep(T_traj)
       
