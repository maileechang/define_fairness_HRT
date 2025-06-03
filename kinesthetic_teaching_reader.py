from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
import rospy
import sys
import time
import moveit_commander
from kinova_msgs.srv import Start, Stop
from actionlib_msgs.msg import GoalStatusArray
from hlpr_manipulation_utils.manipulator import Gripper
from math import pi

from geometry_msgs.msg import Pose

print "What file would you like to open? The file must be in kt_files."
response = raw_input("")

if '.txt' not in response:
	fileName = response.split('.')[0] + '.txt'
else:
	fileName = response
file = open('./kt_files/' + fileName, 'r')

states = {}
for line in file:
	key = line.split('|')[0]
	value = line.split('|')[1][:-1]
	if 'gripper' in value:
		states.update({int(key) : value})
	else:
		states.update({int(key) : [float(x) for x in value[1:-1].split(',')]})


rospy.init_node('kinesthetic_writing', anonymous=True)

startForceControl = rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', Start)
stopForceControl = rospy.ServiceProxy('/j2s7s300_driver/in/stop_force_control', Stop)

print "Initializing arm..."
moveit_commander.roscpp_initialize(sys.argv)

arm =  ArmMoveIt()

gripper = Gripper()

#Returns True if action succeeded
def arm_action(action):
	if action == "close_gripper":
		gripper.close()
		rospy.sleep(1)
		return True
	elif action == "open_gripper":
		gripper.open()
		rospy.sleep(1)
		return True
	else:
		result = arm.move_to_joint_pose(action)
		return result

def run_through(start):
	stopForceControl()
	for i in xrange(start, len(states)-1):
		print "Going to pose #" + str(i)
		arm_action(states[i])
	startForceControl()


state_index = 0
while True:
	print "="*50
	for key in states.keys():
		if state_index == key:
			print ">>> " + str(key) + ": " + str(states[key])
		else:
			print str(key) + ": " + str(states[key])
	print '-'*50
	print "Please use the following controls:"
	print "- N to go to the next state"
	print "- P to go to the previous state"
	print "- E to go to the end"
	print "- S to go to the start"
	print "- R to run through the states"
	print "- Q to exit"
	next_step = raw_input("").upper()

	if next_step == "N":
		if state_index == len(states)-1:
			print "Already at the end!"
			continue
		else:
			state_index += 1
	elif next_step == "P":
		if state_index > 1:
			state_index -= 1
		else:
			print "Already at the start!"
			continue
	elif next_step == "E":
		state_index = len(states)-1
	elif next_step == "S":
		state_index = 0
	elif next_step == "R":
		run_through(state_index)
		state_index = len(states)-1
	elif next_step == "Q":
		break
	else:
		continue

	stopForceControl()
	result = arm_action(states[state_index])
	if not result:
		print "Failed to move to state #" + str(state_index)
		break
	startForceControl()

file.close()
