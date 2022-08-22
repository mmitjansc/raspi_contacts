#!/usr/bin/env python

"""This script publishes foot contact detections. Open switch is HIGH (True), closed switched is LOW (False)"""

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool, Int16
import os 

# First, set the ROS_MASTER_URI to 192.168.1.162
os.environ["ROS_MASTER_URI"] = "http://192.168.1.162:11311"

class FootContact:

    def __init__(self,heel_channel,toe_channel,foot="right"):
        
        self.heel_channel = heel_channel
        self.toe_channel = toe_channel

        self.foot_pub = rospy.Publisher(f"{foot}_foot",Int16,queue_size=1)
        rate = rospy.get_param("pub_rate",20)
        self.rate = rospy.Rate(rate)

        # Store what type of contact we're having.
        # 0: ground
        # 1: air
        self.current_contact = 0 # Let's assume we start with both feet on the ground

        # # GPIO callbacks
        # GPIO.add_event_detect(self.heel_channel,GPIO.FALLING,callback=self.heel_callback)
        # GPIO.add_event_detect(self.toe_channel,GPIO.RISING,callback=self.toe_callback)

    def heel_callback(self,channel):
        '''Called when FALLING edge detected on heel channel'''
        # Heel strike happened!
        self.current_contact = 0
        print("FOOT ON THE GROUND!")

    def toe_callback(self,channel):
        '''Called when RISING edge detected on heel channel'''
        # Toe-off happened!
        self.current_contact = 1
        print("FOOT ON THE AIR!")

    def parse_contact(self):
        """Based on heel value and toe value (0/1), what type of contact am I having?
        INPUTS:
        - 0: Contact (closed switch)
        - 1: No contact (open switch)
        OUTPUTS:
        - 0: whole foot contact -- UNUSED
        - 1: heel-strike
        - 2: toe-off
        - 3: air
        """

        heel_val = GPIO.input(self.heel_channel)
        toe_val = GPIO.input(self.toe_channel)

        if not heel_val:
            # Heel on the ground
            print("HEEL ON GROUND")
            return 1
        if not toe_val:
            print("TOE-OFF")
            # Toe-off
            return 2
        else:
            print("SWING PHASE")
            return 3


        # # print("Heel val:",heel_val)
        # # print("Toe val:",toe_val)

        # if not heel_val and toe_val:
        #     # Heel-strike
        #     return 1

        # if heel_val and not toe_val:
        #     # Toe-off
        #     return 2

        # # Now, the two remaining cases:
        # if self.current_contact == 1:
        #     # Air
        #     return 3
        
        # if self.current_contact == 0:
        #     # Ground
        #     return 0

    
    def run(self):
        # global in_channel

        while not rospy.is_shutdown():

            contact_val = self.parse_contact()

            contact_msg = Int16(int(contact_val))

            self.foot_pub.publish(contact_msg)
            
            self.rate.sleep()

            
def main():

    # What are the input channels
    heel_channel = 12
    toe_channel = 16

    # Pin 12 is the signal input -- we need BOARD and not BCM (in BCM, it'd be 18)
    GPIO.setmode(GPIO.BOARD)

    # Set channel 12 as input
    GPIO.setup(heel_channel, GPIO.IN)
    GPIO.setup(toe_channel, GPIO.IN)

    foot_contact = FootContact(heel_channel,toe_channel,foot='left')
    foot_contact.run()

if __name__ == "__main__":


    try:
        print("Initializing ROS...")
        rospy.init_node("foot_contact_node",anonymous=True,disable_signals=True)
        print("Running node...")
        main()

    except KeyboardInterrupt:
        print("Shutting down ROS...")
        rospy.signal_shutdown("Clean shutdown")

    finally:
        GPIO.cleanup()
        print("GPIOs cleaned up. Exiting node.")
