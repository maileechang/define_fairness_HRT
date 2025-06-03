from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
import rospy
import sys
import time
import moveit_commander
from hlpr_manipulation_utils.manipulator import Gripper
from math import pi
import ast

from geometry_msgs.msg import Pose

START_POSE = {0 : {'j2s7s300_joint_4': 0.9404750989371984, 'j2s7s300_joint_5': -0.012584501415060068, 'j2s7s300_joint_6': 4.591517153253699, 'j2s7s300_joint_7': 1.6913602925038738, 'j2s7s300_joint_1': 1.5751904094354727, 'j2s7s300_joint_2': 5.2509541886770155, 'j2s7s300_joint_3': -3.131875577720421},
1 : {'j2s7s300_joint_4': 0.9404750989371984, 'j2s7s300_joint_5': -0.012584501415060068, 'j2s7s300_joint_6': 4.591517153253699, 'j2s7s300_joint_7': 1.6913602925038738, 'j2s7s300_joint_1': 1.5751904094354727, 'j2s7s300_joint_2': 5.2509541886770155, 'j2s7s300_joint_3': -3.131875577720421}}

LEFT_BIN = {0 : {'j2s7s300_joint_4': 2.744462071837177, 'j2s7s300_joint_5': 0.4229419390376634, 'j2s7s300_joint_6': 4.452002665859936, 'j2s7s300_joint_7': 1.6210116938869399, 'j2s7s300_joint_1': 0.9892881119782334, 'j2s7s300_joint_2': 4.525115492530317, 'j2s7s300_joint_3': -2.592108751144251},
1 : {'j2s7s300_joint_4': 2.744462071837177, 'j2s7s300_joint_5': 0.4229419390376634, 'j2s7s300_joint_6': 4.452002665859936, 'j2s7s300_joint_7': 1.6210142238899758, 'j2s7s300_joint_1': 0.9892881119782334, 'j2s7s300_joint_2': 4.525115492530317, 'j2s7s300_joint_3': -2.592108751144251},
2 : {'j2s7s300_joint_4': 2.6724723001984825, 'j2s7s300_joint_5': 0.42296697275191075, 'j2s7s300_joint_6': 4.4519728384557276, 'j2s7s300_joint_7': 1.6210142238899758, 'j2s7s300_joint_1': 0.7308839216737988, 'j2s7s300_joint_2': 4.525147983095616, 'j2s7s300_joint_3': -2.592107153247597},
3 : "open_gripper",
4 : {'j2s7s300_joint_4': 2.9459658946602785, 'j2s7s300_joint_5': 0.42382717378400603, 'j2s7s300_joint_6': 4.908862091908631, 'j2s7s300_joint_7': 1.6210339312820423, 'j2s7s300_joint_1': 1.051854021783574, 'j2s7s300_joint_2': 4.530774177214519, 'j2s7s300_joint_3': -2.5922621492230395}}

RIGHT_BIN = {0 : {'j2s7s300_joint_4': 3.02788899084886, 'j2s7s300_joint_5': -0.5196423834828767, 'j2s7s300_joint_6': 4.806049160129468, 'j2s7s300_joint_7': 1.6238503572928558, 'j2s7s300_joint_1': 2.2853349641202794, 'j2s7s300_joint_2': 4.434256159303976, 'j2s7s300_joint_3': 2.3200676447409836},
1 : {'j2s7s300_joint_4': 3.0536393617449455, 'j2s7s300_joint_5': -0.1203061717154581, 'j2s7s300_joint_6': 4.805419588847774, 'j2s7s300_joint_7': 1.6238552841408724, 'j2s7s300_joint_1': 2.3212096082164098, 'j2s7s300_joint_2': 4.433649491207655, 'j2s7s300_joint_3': 1.8984086835584657},
2 : {'j2s7s300_joint_4': 3.0454802351132697, 'j2s7s300_joint_5': -0.1014179680001801, 'j2s7s300_joint_6': 4.682527487713849, 'j2s7s300_joint_7': 1.6238602109888887, 'j2s7s300_joint_1': 2.550640137178409, 'j2s7s300_joint_2': 4.3507103990632086, 'j2s7s300_joint_3': 1.9028725410196021},
3 : "open_gripper",
4 : {'j2s7s300_joint_4': 2.6529089851468695, 'j2s7s300_joint_5': -0.09919635901883606, 'j2s7s300_joint_6': 4.746732573170134, 'j2s7s300_joint_7': 1.6238849783870266, 'j2s7s300_joint_1': 1.8552372443916272, 'j2s7s300_joint_2': 4.543271326945824, 'j2s7s300_joint_3': 2.0293951965095767}}

poses = {}

def openGripper(grip):
	grip.open()
	time.sleep(1)

def closeGripper(grip):
	grip.close()
	time.sleep(1)


def execute(arm, filename, bin_pos):
	if ".txt" not in filename:
		filename = filename + ".txt"

	with open('./kt_files/'+filename) as f:
		for line in f.readlines():
			contents = line.split("|")
			if "open_gripper" in contents[1] or "close_gripper" in contents[1]:
				poses.update({int(contents[0]) : contents[1]})
			else:
				poses.update({int(contents[0]) : ast.literal_eval(contents[1])})


	if arm == None:
		print "Initializing arm..."
		moveit_commander.roscpp_initialize(sys.argv)

		arm =  ArmMoveIt()

	currentPose = arm.get_current_pose()
	print "Current pose: ", currentPose

	gripper = Gripper()
	print "Opening gripper if not open..."
	openGripper(gripper)

	for i in range(len(START_POSE)-1):
		result = arm.move_to_joint_pose(START_POSE[i])
		if not result:
			print "Failed to move to start pose:", START_POSE[i]

	for i in range(1,len(poses)-1):
		print "Moving to pose", i
		if "open_gripper" in poses[i]:
			continue
		elif "close_gripper" in poses[i]:
			closeGripper(gripper)
		else:
			result = arm.move_to_joint_pose(poses[i])
			if not result:
				print "Failed to move to pose:", poses[i]

	if bin_pos == "right":
		bin_poses = RIGHT_BIN
	else:
		bin_poses = LEFT_BIN

	for i in range(len(bin_poses)-1):
		if "open_gripper" in bin_poses[i]:
			openGripper(gripper)
		elif "close_gripper" in bin_poses[i]:
			closeGripper(gripper)
		else:
			result = arm.move_to_joint_pose(bin_poses[i])
			if not result:
				print "Failed to move to bin pose:", bin_poses[i]

	for i in range(len(START_POSE)-1):
		result = arm.move_to_joint_pose(START_POSE[i])
		if not result:
			print "Failed to move to start pose:", START_POSE[i]
	return True


if __name__ == "__main__":
	rospy.init_node('test_cup1', anonymous=True)
	## First initialize moveit_commander and rospy.
	print "Initializing arm..."
	moveit_commander.roscpp_initialize(sys.argv)

	arm =  ArmMoveIt()
	execute(arm, "t", "left")
