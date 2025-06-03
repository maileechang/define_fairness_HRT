from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
import rospy
import sys
import time
import moveit_commander
from hlpr_manipulation_utils.manipulator import Gripper
from math import pi

from geometry_msgs.msg import Pose

print "You're file will be saved in ./kt_files/"
response = raw_input("What would you like to name your file?\n")

if '.' in response:
	fileName = response.split('.')[0]
else:
	fileName = response
file = open("./kt_files/" + fileName + ".txt", "w")

rospy.init_node('kinesthetic_writing', anonymous=True)

print "Initializing arm..."
moveit_commander.roscpp_initialize(sys.argv)

arm =  ArmMoveIt()

gripper = Gripper()

states = {0 : arm.get_current_pose()} #Initial state
quit = False
while not quit:
	print "-"*50
	print "Please use the following controls:"
	print "- Enter to save the current state"
	print "- O to open the gripper"
	print "- C to close the gripper"
	print "- Q to save and quit"
	next_step = raw_input("")

	if next_step == "":
		states.update({len(states) : arm.get_current_pose()})
	elif next_step.upper() == "C":
		states.update({len(states) : "close_gripper"})
		print "Closing gripper..."
		gripper.close()
	elif next_step.upper() == "O":
		states.update({len(states) : "open_gripper"})
		print "Opening gripper..."
		gripper.open()
	elif next_step.upper() == "Q":
		quit = True

for key in states.keys():
	file.write(str(key) + "|" + str(states[key]) + "\n")

file.close()
