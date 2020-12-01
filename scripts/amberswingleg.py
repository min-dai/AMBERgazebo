#!/usr/bin/env python3
#
#   amberswingleg.py
#
#   Create a motion by continually sending joint values.  Also listen
#   to the point input.
#
#   Publish:   /joint_states      sensor_msgs/JointState
#   Subscribe: /point             geometry_msgs/PointStamped
#
import rospy
import numpy as np
import tf as transforms
import math
import yaml
from gazebodemos.kinematics2 import Kinematics
from sensor_msgs.msg         import JointState
from geometry_msgs.msg       import PointStamped
from std_msgs.msg            import Float64


currentJointState = JointState()
def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg
#
#  Joint Command Publisher
#
#  Publish the commands on /joint_states (so RVIZ can see) as well as
#  on the /sevenbot/jointX_position/command topics (for Gazebo).
#
class JointCommandPublisher:
    def __init__(self, urdfnames, controlnames):
        # Make sure the name lists have equal length.
        assert len(urdfnames) == len(controlnames), "Unequal lengths"

        # Save the dofs = number of names/channels.
        self.n = len(urdfnames)

        # Create a publisher to /joint_states and pre-populate the message.
        self.pubjs = rospy.Publisher("/joint_states",
                                     JointState, queue_size=100)
        self.msgjs = JointState()

        for name in urdfnames:
            self.msgjs.name.append(name)
            self.msgjs.position.append(0.0)

        # Prepare a list of publishers for each joint commands.
        self.pubX  = []
        for name in controlnames:
            topic = "/amber3m/" + name + "/command"
            self.pubX.append(rospy.Publisher(topic, Float64, queue_size=100))

        # Wait until connected.  You don't have to wait, but the first
        # messages might go out before the connection and hence be lost.
        # rospy.sleep(0.5)

        # Report.
        rospy.loginfo("Ready to publish command for %d DOFs", self.n)

    def dofs(self):
        # Return the number of DOFs.
        return self.n

    def send(self, q):
        # Send each individual command and populate the joint_states.
        for i in range(self.n):
            self.pubX[i].publish(Float64(q[i]))
            self.msgjs.position[i] = q[i]

        # Send the command (with specified time).
        self.msgjs.header.stamp = rospy.Time.now()
        self.pubjs.publish(self.msgjs)



#
#  Basic Rotation Matrices
#
#  Note the angle is specified in radians.
#
def Rx(phi):
    return np.array([[ 1, 0          , 0          ],
                     [ 0, np.cos(phi),-np.sin(phi)],
                     [ 0, np.sin(phi), np.cos(phi)]])

def Ry(phi):
    return np.array([[ np.cos(phi), 0, np.sin(phi)],
                     [ 0          , 1, 0          ],
                     [-np.sin(phi), 0, np.cos(phi)]])

def Rz(phi):
    return np.array([[ np.cos(phi),-np.sin(phi), 0],
                     [ np.sin(phi), np.cos(phi), 0],
                     [ 0          , 0          , 1]])

#
#  Simple Vector Utilities
#
#  Just collect a 3x1 column vector, perform a dot product, or a cross product.
#
def vec(x,y,z):
    return np.array([[x], [y], [z]])

def dot(a,b):
    return a.T @ b

def cross(a,b):
    return np.cross(a, b, axis=0)
def bezier(s,alpha):
#    alpha is a row array []
    bi = 0
    k = 0
    M = len(alpha)-1
    while k<=M:
        bi = bi + alpha[k]*math.factorial(M)/math.factorial(k)/math.factorial(M-k)*(s ** k)*((1-s) ** (M-k))
        k = k+1
    return bi
def desiredoutput(tau,alphas):
    # alphas is a m-by-n array, m is number of outputs, n is degree of bezier + 1
    if tau<0:
        tau = 0
    elif tau>1:
        tau = 1
    yd = np.zeros([alphas.shape[0],1])
    for i in range(alphas.shape[0]):
        yd[i] = bezier(tau,alphas[i,:])
    return yd
#
#  6x1 Error Computation
#
#  Note the 3x1 translation is on TOP of the 3x1 rotation error!
#
#  Also note, the cross product does not return a column vector but
#  just a one dimensional array.  So we need to convert into a 2
#  dimensional matrix, and transpose into the column form.  And then
#  we use vstack to stack vertically...
#
def etip(p, pd, R, Rd):
    ep = pd - p
    eR = 0.5 * (cross(R[:,[0]], Rd[:,[0]]) +
                cross(R[:,[1]], Rd[:,[1]]) +
                cross(R[:,[2]], Rd[:,[2]]))
    return np.vstack((ep,eR))


#
#  Calculate the Desired
#
#  This computes the desired position and orientation, as well as the
#  desired translational and angular velocities for a given time.
#
def desired(t):
    # The point is simply taken from the subscriber.
    pd = point.position()
    vd = np.zeros((3,1))

    # The orientation is constant (at the zero orientation).
    Rd = np.array([[ -1,  0,  0],
                   [  0,  0,  1],
                   [  0,  1,  0]])
    wd = np.zeros((3,1))

    # Return the data.
    return (pd, Rd, vd, wd)



#
#  Main Code
#
if __name__ == "__main__":
    print(bezier(.5,np.array([.2,.4,.6,.4,.8])))
    # load yaml file containing optimized gait info
    with open("/home/user/demo_ws/src/gazebodemos/config/params_2016-09-07T21-36-04-00.yaml", 'r') as stream:
        try:
            yamllist = yaml.safe_load(stream)
            alphas = np.array(yamllist["domain"]["a"])
            thetamp = np.array(yamllist["domain"]["p"])
        except yaml.YAMLError as exc:
            print(exc)


    lf = .4064
    lT = .4999
    lt = 1.373-lT-lf
    c = np.array([-lt-lf,-lf,0,0,0])
    # hip_pos = c*q  #q from joint data  remember to convert q[0] from qTor to qsf
    # tau = .5
    # yd = desiredoutput(tau, alphas)

    #
    #  LOGISTICAL SETUP
    #
    # Prepare the node.
    rospy.init_node('amberswingleg')
    rospy.loginfo("Starting the demo code for the amber to swing leg...")
    broadcaster = transforms.TransformBroadcaster()
    # Initialize Torso position
    p_pw = np.array([[0],
                   [0],
                   [3]])    # Choose the pelvis w.r.t. world position
    R_pw = np.array([[1,0,0],
                   [0,1,0],
                   [0,0,1]])   # Choose the pelviw w.r.t. world orientation

    # Determine the quaternions for the orientation, using a T matrix:
    T_pw = np.vstack((np.hstack((R_pw, p_pw)),
                      np.array([[0, 0, 0, 1]])))
    quat_pw = transforms.transformations.quaternion_from_matrix(T_pw)

    # Place the pelvis w.r.t. world.
    broadcaster.sendTransform(p_pw, quat_pw, rospy.Time.now(), 'Torso', 'world')
    rospy.loginfo("place torso in world")
    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running with a loop dt of %f seconds (%fHz)" %
                  (dt, rate))

    # # Set up the kinematics, from world to tip.
    # urdf = rospy.get_param('/robot_description')
    # kin  = Kinematics(urdf, 'LeftShin', 'RightShin')
    # N    = kin.dofs()
    # rospy.loginfo("Loaded URDF for %d joints" % N)

    # Set up the publisher, naming the joints!
    pub = JointCommandPublisher(('LeftKnee', 'LeftHip', 'RightHip','RightKnee'),
                                ('joint1_position_controller',
                                 'joint2_position_controller',
                                 'joint3_position_controller',
                                 'joint4_position_controller'))
    # # Make sure the URDF and publisher agree in dimensions.
    # if not pub.dofs() == kin.dofs():
    #     rospy.logerr("FIX Publisher to agree with URDF!")



    # Set the numpy printing options (as not to be overly confusing).
    # This is entirely to make it look pretty (in my opinion).
    np.set_printoptions(suppress = True, precision = 6)


    #
    #  PICK AN INITIAL GUESS and INITIAL DESIRED
    #
    # Pick an initial joint position (pretty bad initial guess, but
    # away from the worst singularities).
    theta = np.array([[0.0], [0.0], [0.0], [0.0]])

    #
    #  TIME LOOP
    #
    # I play one "trick": I start at (t=-1) and use the first second
    # to allow the poor initial guess to move to the starting point.
    #
    t   = -1.0
    tf  =  9.0

    while not rospy.is_shutdown():
        # Advance to the new time step.
        t += dt
        tau = t
        if tau%2 <1:
            yd = desiredoutput(tau%2, alphas)
        else:
            yd = desiredoutput(tau%2-1, alphas)
            yd = np.array([[yd[3]],[yd[2]],[yd[1]],[yd[0]]])

        # Setup subscriber to amber3m states
        # ERROR: size of q is 1-by-4!!!!!!
        # rospy.Subscriber("/amber3m/joint_states", JointState, jointStatesCallback)


        # q = np.array(currentJointState.position)
        # if q.size > 0:
        #     print(c.shape,q.shape)
        #     tau = c @ q.T
        # else:
        #     tau = 0;
        # print(tau)


        # Publish and sleep for the rest of the time.  You can choose
        # whether to show the initial "negative time convergence"....
        # if not t<0:
        pub.send(yd)
        servo.sleep()

        # Break the loop when we are finished.
        if (t >= tf):
            break
