#!/usr/bin/env python
# -*- coding: utf-8 -*-



"""
�ȳ�ʼ��moveit�������ⶩ�Ľڵ��Լ�CAN��
����Ҫ��ɵĶ�������ĩ��ִ����(end_effector_link)�ڿռ��е����꣬Ȼ����moveit!ִ��(���˶�ѧ����)IK��
���Ľڵ㶩��rostopic:/move_group/display_planned_path����ȡrosmsg:moveit_msgs/DisplayTrajectory��
����Ϣ�н�����PVT���ݣ���CAN���߷��͵���λ��6���ڵ��ϣ��Ƚڵ�����PVT���ݺ��ٴη��Ϳ�ʼ��������
�ڵ㣬���һ����̬�����С�
"""



import rospy, sys
import moveit_commander
import time, threading
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class MyRobotDemo:
    def __init__(self):
	# Initialize the move_group API
	moveit_commander.roscpp_initialize(sys.argv)

	rospy.init_node('myrobot_control')
		
	# Initialize the move group for the right arm
	self.myrobot_arm = moveit_commander.MoveGroupCommander('Arm')
		
	# Get the name of the end-effector link
	self.end_effector_link = self.myrobot_arm.get_end_effector_link()
		        
	# Set the reference frame for pose targets
	self.reference_frame = 'base'

	# Set the right arm reference frame accordingly
	self.myrobot_arm.set_pose_reference_frame(self.reference_frame)
		
	# Allow replanning to increase the odds of a solution
	self.myrobot_arm.allow_replanning(True)

	# Allow some leeway in position (meters) and orientation (radians)
	self.myrobot_arm.set_goal_position_tolerance(0.01)
	self.myrobot_arm.set_goal_orientation_tolerance(0.05)

	# Finish Initialize
        print 'Finish Initialize'
	

    def controlrobot(self):	
	# Start the arm in the "Start" pose stored in the SRDF file
        print 'Start controlrobot'
	self.myrobot_arm.set_named_target('Start')
        #print 'Go Start Position', time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime(time.time()))
	self.myrobot_arm.go()
	rospy.sleep(2)

	self.myrobot_arm.set_named_target('Reset')
        #print 'Go Reset Position', time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime(time.time()))
	self.myrobot_arm.go()
	#saved_target_pose = self.myrobot_arm.get_current_pose(self.end_effector_link)
        #print saved_target_pose
	rospy.sleep(2)

        '''joint_positions = [-0.0867, -1.274, 0.02832, 0.0820, -1.273, -0.003]
 
        # Set the arm's goal configuration to the be the joint positions
        right_arm.set_joint_value_target(joint_positions)
                 
        # Plan and execute the motion
        right_arm.go()
        rospy.sleep(1)'''
	       

	# Set the target pose.  This particular pose has the gripper oriented horizontally
	# 0.85 meters above the ground, 0.10 meters to the right and 0.20 meters ahead of 
	# the center of the robot base.
	'''target_pose = PoseStamped()
	target_pose.header.frame_id = self.reference_frame
	target_pose.header.stamp = rospy.Time.now()     
	target_pose.pose.position.x = -0.5
	target_pose.pose.position.y = 0.005
	target_pose.pose.position.z = 0.1
	target_pose.pose.orientation.x = -0.072
	target_pose.pose.orientation.y = 0.02
	target_pose.pose.orientation.z = 0.71
	target_pose.pose.orientation.w = 0.006

	# Set the start state to the current state
	#myrobot_arm.set_start_state_to_current_state()

	# Set the goal pose of the end effector to the stored pose
	self.myrobot_arm.set_pose_target(target_pose, self.end_effector_link)

	# Plan the trajectory to the goal
	traj = self.myrobot_arm.plan()

	# Execute the planned trajectory
	self.myrobot_arm.execute(traj)

	# Pause for a second
	rospy.sleep(1)'''

	# Shift the end-effector to the right 5cm
	for i in range(5):
	    self.myrobot_arm.shift_pose_target(1, -0.01, self.end_effector_link)
            #print i, time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime(time.time()))
	    self.myrobot_arm.go()
	    rospy.sleep(0.4)

	for i in range(5):
	    self.myrobot_arm.shift_pose_target(1, 0.01, self.end_effector_link)
            #print i, time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime(time.time()))
	    self.myrobot_arm.go()
	    rospy.sleep(0.4)
	   
	# Finish up in the resting position  
	self.myrobot_arm.set_named_target('Start')
        #print 'Go Start Position', time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime(time.time()))
	self.myrobot_arm.go()

        print 'End controlrobot'

	# Shut down MoveIt cleanly
	moveit_commander.roscpp_shutdown()

	# Exit MoveIt
	moveit_commander.os._exit(0)



if __name__ == "__main__":
    myrobot = MyRobotDemo()
    myrobot.controlrobot()
    #MyRobotDemo()
