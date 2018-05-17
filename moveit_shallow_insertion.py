#!/usr/bin/env python

#Author: Olivia
#Date: 2018/5/15


# Included headers
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf import transformations as tf
import numpy as np
from math import *
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

global gripper_command_publisher


def calc_R_h_from_ksi(w_init, v_init, theta):
  w_cross_v_m = np.matrix('0.0; 0.0; 0.0')
  R_ksi_1st = np.matrix('0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0')
  R_w_1st = np.matrix('0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0')
  R_w_hat = np.matrix('0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0')
  R_I = np.matrix('1.0 0.0 0.0; 0.0 1.0 0.0; 0 0 1')
  w_init_v = np.array([w_init[0], w_init[1], w_init[2]])
  v_init_v = np.array([v_init[0], v_init[1], v_init[2]])
  w_cross_v = np.cross(w_init_v, v_init_v)  
  w_cross_v_m[0, 0] = w_cross_v[0]
  w_cross_v_m[1, 0] = w_cross_v[1]
  w_cross_v_m[2, 0] = w_cross_v[2]
  
  theta_1st = theta
# R_w_hat = np.matrix([[0,             -w_init[2],  w_init[1]], [w_init[2],  0,              -w_init[0]], [-w_init[1], w_init[0],   0] ])
  # print R_w_hat
# R_w_hat << 0,             -w_init(2, 0),  w_init(1, 0), \
#            w_init(2, 0),  0,              -w_init(0, 0), \
#            -w_init(1, 0), w_init(0, 0),   0;

# //calculate R_w_hat_theta
  R_w_1st = R_I + R_w_hat * sin(theta_1st) + R_w_hat * R_w_hat * (1 - cos(theta_1st));
  tt = R_w_1st[0, 0]
  R_ksi_1st[0, 0] =  tt
  # print 'R_ksi_1st[0, 0]', R_ksi_1st[0, 0]
  R_ksi_1st[0, 1] =  R_w_1st[0, 1]
  R_ksi_1st[0, 2] =  R_w_1st[0, 2]

  R_ksi_1st[1, 0] =  R_w_1st[1, 0]
  R_ksi_1st[1, 1] =  R_w_1st[1, 1]
  R_ksi_1st[1, 2] =  R_w_1st[1, 2]

  R_ksi_1st[2, 0] =  R_w_1st[2, 0]
  R_ksi_1st[2, 1] =  R_w_1st[2, 1]
  R_ksi_1st[2, 2] =  R_w_1st[2, 2]
  
  temp = (R_I - R_w_1st) * w_cross_v_m;
  R_ksi_1st[0, 3] = temp[0, 0]
  R_ksi_1st[1, 3] = temp[1, 0]
  R_ksi_1st[2, 3] = temp[2, 0]
  R_ksi_1st[3, 3] = 1
  return R_ksi_1st

def get_pose_from_R_h(R_h):
  new_q = tf.quaternion_from_matrix(R_h)
  #set robot target
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = new_q[0]
  pose_target.orientation.y = new_q[1]
  pose_target.orientation.z = new_q[2]
  pose_target.orientation.w = new_q[3]
  pose_target.position.x = R_h[0, 3]
  pose_target.position.y = R_h[1, 3]
  pose_target.position.z = R_h[2, 3]
  return pose_target

def get_R_h_from_pose(pose):
  #get homogenious matrix of initial state
  init_q = []
  init_q.append(pose.orientation.x)
  init_q.append(pose.orientation.y)
  init_q.append(pose.orientation.z)
  init_q.append(pose.orientation.w)
  init_R_h = tf.quaternion_matrix(init_q)
  init_R_h[0, 3] = pose.position.x
  init_R_h[1, 3] = pose.position.y 
  init_R_h[2, 3] = pose.position.z
  return init_R_h

def calc_waypoints_ARC(init_pose, center_point, axis, total_angle):
  way_points = []
  way_points.append(copy.deepcopy(init_pose));

  
  #calculate rotate matrix
  step_angle = 5.0 / 180 * 3.1415926
  current_angle = step_angle
  total_angle = total_angle / 180.0 * 3.1415926

  v_init = np.cross(center_point, axis)
  w_init = axis
  R_h_transform = calc_R_h_from_ksi(w_init, v_init, step_angle);

  current_R_h = get_R_h_from_pose(init_pose);
  print 'current angle ', current_angle
  print 'total angle ', total_angle
  
  while (current_angle <= total_angle):
    current_R_h = R_h_transform * current_R_h 
    new_target = get_pose_from_R_h(current_R_h)
    way_points.append(new_target)
    print 'add target'
    current_angle += step_angle

  return way_points


# Gripper command functions
def gripper_init():
  global gripper_command_publisher
  gripper_command_publisher.publish('a')
  rospy.sleep(1)

  gripper_command_publisher.publish('o')
  rospy.sleep(1)

  gripper_command_publisher.publish('c')
  rospy.sleep(1)

  gripper_command_publisher.publish('o') 
  rospy.sleep(1)

def gripper_open():
  global gripper_command_publisher
  gripper_command_publisher.publish('o')
  rospy.sleep(1)

def gripper_200():
  global gripper_command_publisher
  gripper_command_publisher.publish('200')
  rospy.sleep(1)

def gripper_open_degree(open_degree):
  global gripper_command_publisher
  gripper_command_publisher.publish(open_degree)
  rospy.sleep(1)


def gripper_close():
  global gripper_command_publisher
  gripper_command_publisher.publish('c')
  rospy.sleep(1)

#gripper command function end

def scale_trajectory_speed(traj, scale):
  # Create a new trajectory object
  new_traj = RobotTrajectory()

  # Initialize the new trajectory to be the same as the planned trajectory
  new_traj.joint_trajectory = traj.joint_trajectory

  # Get the number of joints involved
  n_joints = len(traj.joint_trajectory.joint_names)

  # Get the number of points on the trajectory
  n_points = len(traj.joint_trajectory.points)

  # Store the trajectory points
  points = list(traj.joint_trajectory.points)

  # Cycle through all points and scale the time from start, speed and acceleration
  for i in range(n_points):
     point = JointTrajectoryPoint()
     point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
     point.velocities = list(traj.joint_trajectory.points[i].velocities)
     point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
     point.positions = traj.joint_trajectory.points[i].positions
                  
     for j in range(n_joints):
         point.velocities[j] = point.velocities[j] * scale
         point.accelerations[j] = point.accelerations[j] * scale * scale
     
     points[i] = point

  # Assign the modified points to the new trajectory
  new_traj.joint_trajectory.points = points

  # Return the new trajecotry
  return new_traj


def shallow_insertion_start():
  
  global gripper_command_publisher

  ## 1. Initialize robot environment 
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  ##group = moveit_commander.MoveGroupCommander("left_arm")
  group = moveit_commander.MoveGroupCommander("manipulator")

  #set max velocity and acceleration scaler
  group.set_max_velocity_scaling_factor(0.1);
  group.set_max_acceleration_scaling_factor(0.1);


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)

  #initialize the gripper publisher variable
  gripper_command_publisher = rospy.Publisher(
                                      'CModelRobotInputRob',
                                      String,
                                      queue_size=20)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(2)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ End effector: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing group state"
  print group.get_current_pose()
  


  # ## Then, we will get the current set of joint values for the group
  # group_variable_values = group.get_current_joint_values()
  # print "============ Joint values: ", group_variable_values

  ## Now, let's modify one of the joints, plan to the new joint
  ## space goal and visualize the plan
  group_variable_values = [-0.02749187151064092, -1.598161522542135, -1.8247435728656214, -3.7606776396380823, -1.3931792418109339, 1.796911358833313]
  group.set_joint_value_target(group_variable_values)
  plan2 = group.plan()

  print "============ Waiting while RVIZ displays plan2..."
  group.execute(plan2)

  rospy.sleep(2)


  # moveit_commander.roscpp_shutdown()

  # return
  # Initialize robot environment END
  

  #2. Check if the gripper works well, then open it 
  gripper_init()
  waypoints = []  


  #3. Go to the beginning position before closing the gripper and pick up the card 
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = 0.5
  pose_target.orientation.y = 0.5
  pose_target.orientation.z = -0.5
  pose_target.orientation.w = 0.5
  pose_target.position.x = 0.02 -0.805
  pose_target.position.y =  0.235 - 0.0125
  pose_target.position.z = -0.12 + 0.46 - 0.03
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  rospy.sleep(2)

  #4. Close the gripper
  gripper_close()


  #5. Pick up the card, and move to the top of the beginning point
  pose_target.position.z = -0.12 + 0.46 + 0.1
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  rospy.sleep(2)


  #6. Move to the top of the insertion beginning point
  pose_target.position.x = 0.02 -0.805
  pose_target.position.y =  0.235 - 0.0125 - 0.225 - 0.002
  pose_target.position.z = -0.12 + 0.1 + 0.46  
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  rospy.sleep(2)
 

  ##7. Go down and start the insertion process
  pose_target.orientation.x = 0.48868426425
  pose_target.orientation.y = 0.504593823773
  pose_target.orientation.z = -0.51414136297
  pose_target.orientation.w = 0.492170114665
  pose_target.position.x = -0.77998840649
  pose_target.position.y = 0.00084252595
  pose_target.position.z = 0.369628519
  group.set_pose_target(pose_target)
  plan1 = group.plan()  
  group.execute(plan1)
  print 'point 1'
  rospy.sleep(2)

  #calculate ARC waypoints
  center_point = [-0.77998840649, 0.00084252595, 0.369628519 - 0.315]
  rotate_axis = [0, 1, 0]
  waypoints_new = calc_waypoints_ARC(pose_target, center_point, rotate_axis, 45)

  #execute path with waypoints
  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.

  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints_new,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
                               
  group.execute(plan3)
  rospy.sleep(2)

  #13. Widen the gripper
  print 'gripper 200'
  gripper_200()


  #14. Move to the 8th way point in the insertion process
  pose_target.orientation.x = 0.300275507697
  pose_target.orientation.y = 0.622241259232
  pose_target.orientation.z = -0.638282291773
  pose_target.orientation.w = 0.339479234704
  pose_target.position.x = -0.586538559597
  pose_target.position.y = 0.010062837864
  pose_target.position.z = 0.301186931067
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 8'
  rospy.sleep(2)


  #15. Widen the gripper
  gripper_open_degree('180')

  
  #16. Move to the 9th way point in the insertion process
  pose_target.orientation.x = 0.347952330633
  pose_target.orientation.y = 0.63268202582
  pose_target.orientation.z = -0.596866944095
  pose_target.orientation.w = 0.349846367507
  pose_target.position.x = -0.597923336817
  pose_target.position.y = -0.00229020222052
  pose_target.position.z = 0.305400357222
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 9'
  rospy.sleep(2)


  #17. Move to the 10th way point in the insertion process
  pose_target.orientation.x = 0.359182117133
  pose_target.orientation.y = 0.630010602994
  pose_target.orientation.z = -0.585982817717
  pose_target.orientation.w = 0.361523144746
  pose_target.position.x = -0.594702188085
  pose_target.position.y = -0.00658854236333
  pose_target.position.z = 0.31898880102
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 10'
  rospy.sleep(2)


   #18. Move to the 11th way point in the insertion process
  pose_target.orientation.x = 0.364964183344
  pose_target.orientation.y = 0.628658362491
  pose_target.orientation.z = -0.580710320412
  pose_target.orientation.w = 0.366558770068
  pose_target.position.x = -0.592416763802
  pose_target.position.y = -0.00599166509442
  pose_target.position.z = 0.320244273086
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 11'
  rospy.sleep(2)


  #19. Move to the 12th way point in the insertion process
  pose_target.orientation.x = 0.396925730678
  pose_target.orientation.y = 0.583354341151
  pose_target.orientation.z = -0.680227755847
  pose_target.orientation.w = 0.198589720686
  pose_target.position.x = -0.527264025781
  pose_target.position.y = 0.194388091811
  pose_target.position.z = 0.437013772921
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 12'
  rospy.sleep(2)


  # The card should have fallen into the shallow hole
  print "============ STOPPING"

  # Shutdown moveit 
  moveit_commander.roscpp_shutdown()

  return



if __name__=='__main__':
  try:
    shallow_insertion_start()
  except rospy.ROSInterruptException:
    pass

