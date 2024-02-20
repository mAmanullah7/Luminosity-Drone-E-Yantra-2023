#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Swift():
    def __init__(self):
        rospy.init_node('drone_control')  # Initialize the ROS node

        # Initialize the drone's current position
        self.drone_position = [0.0, 0.0, 0.0]

        # Setpoint for position (x, y, z)
        self.setpoint = [2.0, 2.0, 20.0]

        # Initialize the PID gains for roll, pitch, and throttle
        self.Kp = [0.0, 0.0, 0.0]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0]

        # Initialize previous error, error, and sum of error for each axis
        self.prev_error = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]

        # Initialize max and min values for roll, pitch, and throttle
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        # Initialize the cmd message for controlling the drone
        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # Create ROS publishers
        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
        self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)

        # Subscribe to whycon/poses for receiving position feedback
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)

        # Subscribe to PID tuning topics for throttle, pitch, and roll
        rospy.Subscriber('/pid_tuning_throttle', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)

    #------------------------------------------------------------------------------------------------------------
        self.arm() # ARMING THE DRONE

#------------------------------------------------------------------------------------------------
	# Disarming condition of the drone
    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

#------------------------------------------------------------------------------------------------

	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
    def arm(self):
        self.disarm()
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)	# Publishing /drone_command
        rospy.sleep(1)


    def whycon_callback(self, msg):
        # Update the drone's current position from whycon feedback
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_set_pid(self, alt):
        # Update the PID gains for throttle from the topic
        self.Kp[2] = alt.Kp 
        self.Ki[2] = alt.Ki
        self.Kd[2] = alt.Kd

    def roll_set_pid(self, roll):
        # Update the PID gains for roll from the topic
        self.Kp[0] = roll.Kp
        self.Ki[0] = roll.Ki
        self.Kd[0] = roll.Kd

    def pitch_set_pid(self, pitch):
        # Update the PID gains for pitch from the topic
        self.Kp[1] = pitch.Kp
        self.Ki[1] = pitch.Ki
        self.Kd[1] = pitch.Kd

    def pid(self):
        # Calculate errors for each axis
        self.error[0] = self.setpoint[0] - self.drone_position[0]
        self.error[1] = self.setpoint[1] - self.drone_position[1]
        self.error[2] = self.setpoint[2] - self.drone_position[2]

        # Calculate PID control output for each axis
        for axis in range(3):
            self.cmd.values[axis] = (
                self.Kp[axis] * self.error[axis] +
                self.Ki[axis] * self.sum_error[axis] +
                self.Kd[axis] * (self.error[axis] - self.prev_error[axis])
            )

            # Clamp the control output within the specified range
            if self.cmd.values[axis] > self.max_values[axis]:
                self.cmd.values[axis] = self.max_values[axis]
            elif self.cmd.values[axis] < self.min_values[axis]:
                self.cmd.values[axis] = self.min_values[axis]

            # Update the sum of error and previous error for the current axis
            self.sum_error[axis] += self.error[axis]
            self.prev_error[axis] = self.error[axis]

        # Publish the control command
        self.command_pub.publish(self.cmd)

        # Publish error values for monitoring
        self.alt_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[1])
        self.roll_error_pub.publish(self.error[0])

if __name__ == '__main__':
    controller = Swift()
    rate = rospy.Rate(30)  # Control rate (adjust as needed)

    while not rospy.is_shutdown():
        controller.pid()
        rate.sleep()
