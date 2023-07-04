import numpy as np
import pdb
import tf.transformations as tftrans
from std_msgs.msg import Header
from std_msgs.msg import Time
from geometry_msgs.msg import PoseArray
from arc_ros.msg import TaskSpaceTraj
from arc_ros.msg import Pose
import rospy

def skew(v):
  S = np.zeros((3,3))
  S[0,1] = -v[2]
  S[0,2] =  v[1]
  S[1,2] = -v[0]

  S[1,0] =  v[2]
  S[2,0] = -v[1]
  S[2,1] =  v[0]
  return(S)


def unskew(S):
  tmp=S+np.transpose(S)
  tmp_norm=np.linalg.norm(tmp[:])
  if(len(S)!=3):
    print('error in unskew(): Invalid matrix size S') # error
  else:
    if(tmp_norm>1e-8):
      if(tmp_norm>1):
        # pdb.set_trace()
        print('error in unskew(): Non-skew-symmetric matrix S, norm(S+S'')={}'.format(tmp_norm))
      else:
        print('warning in unskew(): Non-skew-symmetric matrix S, norm(S+S'')={}'.format(tmp_norm))
    v = np.zeros((3,1))
    v[0]=S[2,1]
    v[1]=S[0,2]
    v[2]=S[1,0]
  return(v)

def wrap(alpha):
  #wrapToPi Wrap angle in radians to [-pi pi]
  #
  #   lambdaWrapped = wrapToPi(LAMBDA) wraps angles in LAMBDA, in radians,
  #   to the interval [-pi pi] such that pi maps to pi and -pi maps to
  #   -pi.  (In general, odd, positive multiples of pi map to pi and odd,
  #   negative multiples of pi map to -pi.)

  # for i, element in enumerate(alpha):
  #   if (element < -np.pi) | (np.pi < element):
  #     alpha[i] = wrap2pi(alpha[i] + np.pi) - np.pi
  # for x, y in np.ndindex(alpha.shape):
  #   element = alpha[x,y]
  #   if (element < -np.pi) | (np.pi < element):
  #     alpha[x,y] = wrap2pi(element + np.pi) - np.pi
  for x in np.nditer(alpha, op_flags = ['readwrite']):
    if (x < -np.pi) | (np.pi < x):
      x[...] = wrap2pi(x + np.pi) - np.pi
  return alpha

def wrap2pi(alpha):
  #wrapTo2Pi Wrap angle in radians to [0 2*pi]
  #
  #   lambdaWrapped = wrapTo2Pi(LAMBDA) wraps angles in LAMBDA, in radians,
  #   to the interval [0 2*pi] such that zero maps to zero and 2*pi maps
  #   to 2*pi. (In general, positive multiples of 2*pi map to 2*pi and
  #   negative multiples of 2*pi map to zero.)

  # positiveInput = (alpha > 0)
  # alpha = np.mod(alpha, 2*np.pi)
  
  # for i, element in enumerate(alpha):
  #   if (element == 0) & positiveInput[i]:
  #     alpha[i] = 2*np.pi
  posInput = alpha > 0
  alpha = np.mod(alpha,2*np.pi)
  if (alpha==0) & posInput:
    alpha = 2*np.pi
  return alpha

# omega = T * angle_p
def T_Matrix(phi,theta,psi):
  T = np.matrix([[0,-np.sin(phi),np.cos(phi)*np.sin(theta)],\
    [0,np.cos(phi),np.sin(phi)*np.sin(theta)],[1,0,np.cos(theta)]],dtype=np.float)
  return T

