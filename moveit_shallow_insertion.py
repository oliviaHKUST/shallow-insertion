#!/usr/bin/env python

#Author: Olivia
#Date: 2018/5/3


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

global gripper_command_publisher


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
  rospy.sleep(5)
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

  # Initialize robot environment END
  

  #2. Check if the gripper works well, then open it 
  gripper_init()


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


  #8. Move to the 2nd way point in the insertion process
  pose_target.orientation.x = 0.48757600855
  pose_target.orientation.y = 0.504651397542
  pose_target.orientation.z = -0.516403279155
  pose_target.orientation.w = 0.490840357064
  pose_target.position.x = -0.77884294211
  pose_target.position.y = 0.0011804883180
  pose_target.position.z = 0.37023884140
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 2'
  rospy.sleep(2)


  #9. Move to the 3rd way point in the insertion process
  pose_target.orientation.x = 0.482312613696
  pose_target.orientation.y = 0.501096936651
  pose_target.orientation.z = -0.524521148339
  pose_target.orientation.w = 0.491074299565
  pose_target.position.x = -0.775233949849
  pose_target.position.y = 0.00547868308962
  pose_target.position.z = 0.369664566685
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 3'
  rospy.sleep(2)


  #9. Move to the 4th way point in the insertion process
  pose_target.orientation.x = 0.44689637914
  pose_target.orientation.y = 0.524912356908
  pose_target.orientation.z = -0.56100246517
  pose_target.orientation.w = 0.458286894805
  pose_target.position.x = -0.73393384646
  pose_target.position.y = 0.0108484437617
  pose_target.position.z = 0.365896888399
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 4'
  rospy.sleep(2)


  #10. Move to the 5th way point in the insertion process
  pose_target.orientation.x = 0.411150860585
  pose_target.orientation.y = 0.541654120717
  pose_target.orientation.z = -0.595856425356
  pose_target.orientation.w = 0.42722465251
  pose_target.position.x = -0.695318810462
  pose_target.position.y = 0.0177422577652
  pose_target.position.z = 0.356604255418
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 5'
  rospy.sleep(2)


  #11. Move to the 6th way point in the insertion process
  pose_target.orientation.x = 0.371364384225
  pose_target.orientation.y = 0.579907287009
  pose_target.orientation.z = -0.603816843902
  pose_target.orientation.w = 0.40149875669
  pose_target.position.x = -0.662692143568
  pose_target.position.y = 0.0118211753503
  pose_target.position.z = 0.343935175687
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 6'
  rospy.sleep(2)


  #12. Move to the 7th way point in the insertion process
  pose_target.orientation.x = 0.291773722137
  pose_target.orientation.y = 0.629675553548
  pose_target.orientation.z = -0.644637617184
  pose_target.orientation.w = 0.32065422942
  pose_target.position.x = -0.592417814392
  pose_target.position.y = 0.010615234106
  pose_target.position.z = 0.299961240327
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  group.execute(plan1)
  print 'point 7'
  rospy.sleep(2)


  #13. Widen the gripper
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


   #18. Move to the 10th way point in the insertion process
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
  print 'point 10'
  rospy.sleep(2)


  #19. Move to the 11th way point in the insertion process
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
  print 'point 11'
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


