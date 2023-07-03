
import rospy
import arcpy
import numpy as np
from std_msgs.msg import Header
from std_msgs.msg import Time
from sensor_msgs.msg import JointState
from arc_ros.msg import JointSpaceTraj
from arc_ros.msg import Joints



#OK
class LinearAxis:

  def __init__(self):
    self.robot = arcpy.LinearAxis.RobotModel()
    self.time = 0
    topic_prefix = rospy.get_param('~topic_prefix', '/arc/LinearAxis/')

    # subscriber
    self.joint_state_sub = rospy.Subscriber(topic_prefix + "joint_state", JointState, self.stateCallback, queue_size=1)
    self.time_sub = rospy.Subscriber(topic_prefix + "time", Time, self.timeCallback, queue_size=1)
  
    self.q = np.zeros((2,1))
    self.q_dot = np.zeros((2,1))
    self.q_dotdot = np.zeros((2,1))
    self.H_0_e = np.eye(4)
    self.tau = np.zeros((2,1))

    ## publisher
    self.jointspace_pub = rospy.Publisher(topic_prefix + 'set_jointspace_trajectory',JointSpaceTraj,queue_size=1)

    self.N_trajectory_points = 10 

    # stored timing of old trajectory
    self.t0_old = 0
    self.t1_old = 0

  def __del__(self):
    self.jointspace_pub.unregister()


  ## subscribe 
  def stateCallback(self, msg):
    stamp = msg.header.stamp
    t = stamp.secs + stamp.nsecs * 10**(-9)
    self.q = np.array(msg.position).reshape(2,1)
    self.q_dot = np.array(msg.velocity).reshape(2,1)
    self.q_dotdot = 0 * self.q_dot
    self.tau = np.array(msg.effort).reshape(2,1)

    self.model.update(state.get_q_act(), state.get_q_dot_act(),
                      state.get_q_dotdot_act(), state.get_q_dotdotdot_act())

    self.H_0_e = self.robot.get_H_0_e()

  def timeCallback(self, msg):
    self.time = msg.data

  def _publish_jointspace_trajectory(self,t_k,JointsList):
    JointsArray = []
    for elem in JointsList:
      jmsg = Joints()
      jmsg.Joints = elem
      JointsArray.append(jmsg)

    header_obj = Header()
    # header_obj.seq = header_obj.seq + 1
    header_obj.frame_id = "JointspaceTrajectory"
    header_obj.stamp = rospy.get_rostime()
    header_obj.seq = header_obj.seq + 1
    msg = JointSpaceTraj()
    msg.header = header_obj
    msg.JointsArray = JointsArray
    msg.t_k = t_k
    #print(msg)
    self.jointspace_pub.publish(msg)



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
                  vi = A * (1-t)
                else:  
                  if 1 < ti:
                    vi = 0
                  else:
                    vi = 0

        # linear interpolation
        q1_ = np.array(q1).reshape((2,1))
        q0_ = np.array(q0).reshape((2,1))
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
    


  def homogeneous_transformation_endeffector(self):
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




if __name__ == '__main__':
  rospy.init_node('linear_axis_node', anonymous=True) 

  linear_axis = LinearAxis()
  
  T_traj = 3

  while not rospy.is_shutdown():

    q1 = 1*np.ones((2,1))
    t0 = rospy.Time.now().to_sec()
        
    linear_axis.move_jointspace(q1, t0, T_traj,False)
    rospy.sleep(T_traj)

    q1 = 0*np.ones((2,1))
    t0 = rospy.Time.now().to_sec()
        
    linear_axis.move_jointspace(q1, t0, T_traj,False)
    rospy.sleep(T_traj)
    
    

       
