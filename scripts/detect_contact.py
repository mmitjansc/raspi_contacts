#!/usr/bin/env python

"""This script publishes foot contact detections. Open switch is HIGH (True), closed switched is LOW (False)"""

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

class FootContact:

    def __init__(self,input_channel):
        
        self.in_channel = input_channel
        self.foot_pub = rospy.Publisher("right_foot",Bool,queue_size=1)
        self.rate = rospy.Rate(10)
    
    def run(self):
        # global in_channel

        while not rospy.is_shutdown():

            read_val = GPIO.input(self.in_channel)
            print("Read:",read_val)

            contact_msg = Bool(int(read_val))

            self.foot_pub.publish(contact_msg)
            
            self.rate.sleep()

            
def main():

    # What is the input channel
    in_channel = 12

    # Pin 12 is the signal input -- we need BOARD and not BCM (in BCM, it'd be 18)
    GPIO.setmode(GPIO.BOARD)

    # Set channel 12 as input
    GPIO.setup(in_channel, GPIO.IN)

    foot_contact = FootContact(in_channel)
    foot_contact.run()

if __name__ == "__main__":

    print("Initializing ROS...")
    rospy.init_node("foot_contact_node",anonymous=True)
    print("ROS initialized.")

    try:
        main()

    finally:
        GPIO.cleanup()
        print("GPIOs cleaned up. Exiting node.")
