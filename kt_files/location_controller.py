# Study: Defining Fairness in Human-Robot Teams

# Location controller for robotic arm pick-and-place operations
# This module executes complete motion sequences: pickup → transport → place → return

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
import rospy
import sys
import time
import moveit_commander
from hlpr_manipulation_utils.manipulator import Gripper
from math import pi
import ast

from geometry_msgs.msg import Pose

# Pre-programmed joint configurations for the 7-DOF Kinova arm (j2s7s300)
# START_POSE: Home/neutral position that the arm returns to between tasks
# Contains multiple waypoints (0,1) for safe approach to home position

START_POSE = {0 : {'j2s7s300_joint_4': 0.9404750989371984, 'j2s7s300_joint_5': -0.012584501415060068, 'j2s7s300_joint_6': 4.591517153253699, 'j2s7s300_joint_7': 1.6913602925038738, 'j2s7s300_joint_1': 1.5751904094354727, 'j2s7s300_joint_2': 5.2509541886770155, 'j2s7s300_joint_3': -3.131875577720421},
1 : {'j2s7s300_joint_4': 0.9404750989371984, 'j2s7s300_joint_5': -0.012584501415060068, 'j2s7s300_joint_6': 4.591517153253699, 'j2s7s300_joint_7': 1.6913602925038738, 'j2s7s300_joint_1': 1.5751904094354727, 'j2s7s300_joint_2': 5.2509541886770155, 'j2s7s300_joint_3': -3.131875577720421}}

# LEFT_BIN: Sequence of joint poses for placing objects in the left bin
# Includes approach poses (0,1,2), gripper open command (3), and final placement (4)
LEFT_BIN = {0 : {'j2s7s300_joint_4': 2.744462071837177, 'j2s7s300_joint_5': 0.4229419390376634, 'j2s7s300_joint_6': 4.452002665859936, 'j2s7s300_joint_7': 1.6210116938869399, 'j2s7s300_joint_1': 0.9892881119782334, 'j2s7s300_joint_2': 4.525115492530317, 'j2s7s300_joint_3': -2.592108751144251},
1 : {'j2s7s300_joint_4': 2.744462071837177, 'j2s7s300_joint_5': 0.4229419390376634, 'j2s7s300_joint_6': 4.452002665859936, 'j2s7s300_joint_7': 1.6210142238899758, 'j2s7s300_joint_1': 0.9892881119782334, 'j2s7s300_joint_2': 4.525115492530317, 'j2s7s300_joint_3': -2.592108751144251},
2 : {'j2s7s300_joint_4': 2.6724723001984825, 'j2s7s300_joint_5': 0.42296697275191075, 'j2s7s300_joint_6': 4.4519728384557276, 'j2s7s300_joint_7': 1.6210142238899758, 'j2s7s300_joint_1': 0.7308839216737988, 'j2s7s300_joint_2': 4.525147983095616, 'j2s7s300_joint_3': -2.592107153247597},
3 : "open_gripper",
4 : {'j2s7s300_joint_4': 2.9459658946602785, 'j2s7s300_joint_5': 0.42382717378400603, 'j2s7s300_joint_6': 4.908862091908631, 'j2s7s300_joint_7': 1.6210339312820423, 'j2s7s300_joint_1': 1.051854021783574, 'j2s7s300_joint_2': 4.530774177214519, 'j2s7s300_joint_3': -2.5922621492230395}}

# RIGHT_BIN: Sequence of joint poses for placing objects in the right bin
# Similar structure to LEFT_BIN but with different joint configurations
RIGHT_BIN = {0 : {'j2s7s300_joint_4': 3.02788899084886, 'j2s7s300_joint_5': -0.5196423834828767, 'j2s7s300_joint_6': 4.806049160129468, 'j2s7s300_joint_7': 1.6238503572928558, 'j2s7s300_joint_1': 2.2853349641202794, 'j2s7s300_joint_2': 4.434256159303976, 'j2s7s300_joint_3': 2.3200676447409836},
1 : {'j2s7s300_joint_4': 3.0536393617449455, 'j2s7s300_joint_5': -0.1203061717154581, 'j2s7s300_joint_6': 4.805419588847774, 'j2s7s300_joint_7': 1.6238552841408724, 'j2s7s300_joint_1': 2.3212096082164098, 'j2s7s300_joint_2': 4.433649491207655, 'j2s7s300_joint_3': 1.8984086835584657},
2 : {'j2s7s300_joint_4': 3.0454802351132697, 'j2s7s300_joint_5': -0.1014179680001801, 'j2s7s300_joint_6': 4.682527487713849, 'j2s7s300_joint_7': 1.6238602109888887, 'j2s7s300_joint_1': 2.550640137178409, 'j2s7s300_joint_2': 4.3507103990632086, 'j2s7s300_joint_3': 1.9028725410196021},
3 : "open_gripper",
4 : {'j2s7s300_joint_4': 2.6529089851468695, 'j2s7s300_joint_5': -0.09919635901883606, 'j2s7s300_joint_6': 4.746732573170134, 'j2s7s300_joint_7': 1.6238849783870266, 'j2s7s300_joint_1': 1.8552372443916272, 'j2s7s300_joint_2': 4.543271326945824, 'j2s7s300_joint_3': 2.0293951965095767}}

# Dictionary to store loaded pose sequences from files
poses = {}

# Gripper control functions with timing delays for mechanical settling
def openGripper(grip):
	grip.open()
	time.sleep(1)

def closeGripper(grip):
	grip.close()
	time.sleep(1)


def execute(arm, filename, bin_pos):
"""
Execute complete pick-and-place operation

Args:
	arm: ArmMoveIt object for controlling the robotic arm
	filename: Letter corresponding to pickup location (e.g., 'a', 'b', 'c')
	bin_pos: Target bin position ('left' or 'right')

Returns:
	bool: True if execution completed (note: doesn't indicate success of all moves)
"""
	# Ensure filename has .txt extension for file reading
	if ".txt" not in filename:
		filename = filename + ".txt"

	# Load pickup sequence from corresponding location file
	# File format: step_number|joint_configuration or step_number|gripper_command
	with open('./kt_files/'+filename) as f:
		for line in f.readlines():
			contents = line.split("|")
			# Check if this step is a gripper command or joint pose
			if "open_gripper" in contents[1] or "close_gripper" in contents[1]:
				poses.update({int(contents[0]) : contents[1]})
			else:
				# Parse joint configuration dictionary from string
				poses.update({int(contents[0]) : ast.literal_eval(contents[1])})

	# Initialize arm if not already provided (for standalone testing)
	if arm == None:
		print "Initializing arm..."
		moveit_commander.roscpp_initialize(sys.argv)

		arm =  ArmMoveIt()

	# Display current arm position for debugging
	currentPose = arm.get_current_pose()
	print "Current pose: ", currentPose

	# Initialize gripper and ensure it starts in open position
	gripper = Gripper()
	print "Opening gripper if not open..."
	openGripper(gripper)

	# PHASE 1: Move to start/home position
	# Execute all waypoints in START_POSE sequence for safe approach
	for i in range(len(START_POSE)-1):
		result = arm.move_to_joint_pose(START_POSE[i])
		if not result:
			print "Failed to move to start pose:", START_POSE[i]

	# PHASE 2: Execute pickup sequence from loaded file
	# Skip first and last poses (usually duplicates or special cases)
	for i in range(1,len(poses)-1):
		print "Moving to pose", i
		if "open_gripper" in poses[i]:
			continue # Skip explicit open commands (gripper already open)
		elif "close_gripper" in poses[i]:
			closeGripper(gripper)
		else:
			# Execute joint movement
			result = arm.move_to_joint_pose(poses[i])
			if not result:
				print "Failed to move to pose:", poses[i]
	
	# PHASE 3: Transport and place object in designated bin
	# Select appropriate bin sequence based on target location
	if bin_pos == "right":
		bin_poses = RIGHT_BIN
	else:
		bin_poses = LEFT_BIN

	# Execute bin placement sequence
	for i in range(len(bin_poses)-1):
		if "open_gripper" in bin_poses[i]:
			openGripper(gripper) # Release the object
		elif "close_gripper" in bin_poses[i]:
			closeGripper(gripper) # This shouldn't happen in bin sequence
		else:
			# Execute joint movement for bin approach/placement
			result = arm.move_to_joint_pose(bin_poses[i])
			if not result:
				print "Failed to move to bin pose:", bin_poses[i]
	
	# PHASE 4: Return to home position
	# Execute return sequence to prepare for next task
	for i in range(len(START_POSE)-1):
		result = arm.move_to_joint_pose(START_POSE[i])
		if not result:
			print "Failed to move to start pose:", START_POSE[i]
	return True # Always returns True regardless of individual move success

# Standalone testing section
if __name__ == "__main__":
	# Initialize ROS node for testing
	rospy.init_node('test_cup1', anonymous=True)
	
	# Initialize MoveIt commander and arm controller
	print "Initializing arm..."
	moveit_commander.roscpp_initialize(sys.argv)

	# Test execution: pickup from location 't' and place in left bin
	arm =  ArmMoveIt()
	execute(arm, "t", "left")
