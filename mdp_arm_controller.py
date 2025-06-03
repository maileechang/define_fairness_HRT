# Study: Defining Fairness in Human-Robot Teams
# ROS node for controlling Kinova arm in an MDP cleaning/sorting task
# This script listens for arm movement commands and executes pick-and-place operations

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from std_msgs.msg import String
from kt_files import location_controller
import rospy
import sys
import time
import moveit_commander
import signal

# Signal handler for shutdown when Ctrl+C is pressed
def signal_handler(sig, frame):
	print "\nCleaning up..."
	rospy.signal_shutdown("Ctrl+C was pressed")
	sys.exit(0)

# Register the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

# Create mapping from button numbers (0-19) to location letters (a-t)
# This allows the system to reference physical locations by button number
kt_dict = {}
for i in range(20):
	kt_dict.update({str(i) : chr(97+i)}) #97 is the ASCII value of "a", this just maps buttons to their locations

# Map bin colors to their physical directions
# "red" bin is on the right side, "blue" bin is on the left side
bin_dict = {"red" : "right", "blue" : "left"}

# Publisher for sending arm status updates to other nodes
pub = rospy.Publisher('/mdp_cleaning/arm_status', String, queue_size=10)

if __name__ == "__main__":
	# Initialize this ROS node
	rospy.init_node('mdp_arm_controller', anonymous=True)
	
	# Initialize MoveIt commander for motion planning
	print "Initializing arm..."
	moveit_commander.roscpp_initialize(sys.argv)

	# Create arm controller object using the ArmMoveIt wrapper
	arm =  ArmMoveIt()

	# Main control loop - runs until ROS shutdown is requested
	while not rospy.is_shutdown():
		print "Waiting for a message..."
		
		# Block and wait for incoming arm command message
		# Expected format: "button_number,bin_number" (e.g., "0,0")
		msg = rospy.wait_for_message('/mdp_cleaning/arm_commands', String)
		
		# Wait until we have subscribers before publishing status
		while pub.get_num_connections() == 0:
			rospy.sleep(0.1)
		
		# Notify other nodes that we're processing the command
		pub.publish("Working")
		
		# Parse the incoming message - split on comma to get button and bin numbers
		msg_content = msg.data.split(",") #msgs will be in the format button#,bin# e.g. "0,0"
		
		# Validate that the button number is within acceptable range (0-19)
		if msg_content[0] not in [str(i) for i in range(20)]:
			print "Bad message data:", msg.data
			continue # Skip this iteration and wait for next message
		else:
			# Ensure we still have subscribers before publishing
			while pub.get_num_connections() == 0:
				rospy.sleep(0.1)

			# Publish working status again (redundant but ensures status is sent)
			pub.publish("Working")

			# Initialize result flag
			result = False

			# Execute the arm movement:
			# - Convert button number to location letter using kt_dict
			# - Convert bin number to bin direction using bin_dict
			# - Call location_controller to perform the actual pick-and-place
			result = location_controller.execute(arm, kt_dict[msg_content[0]], bin_dict[msg_content[1]])
		
		# If the arm movement was successful, publish completion status
		if result:
			print "Publishing \"Done\""
			
			# Wait for subscribers before publishing final status
			while pub.get_num_connections() == 0:
				rospy.sleep(0.1)

			# Notify other nodes that the task is complete
			pub.publish("Done")
