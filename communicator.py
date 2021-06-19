#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point



class Communicator:
    def __init__(self, master):
        self.data = master.data
        self.pub = rospy.Publisher('/drone', String, queue_size = 1)
        self.msg = String() 
        rospy.init_node('Drone', anonymous=False)

    
    def mission_write(self, msg):
        self.data['mission'] = msg.data
        print("=====New Mission Received: ", msg, '=====')
        
    def publish_info(self):
        rate = rospy.Rate(1) #1hz
        while not rospy.is_shutdown():
            self.msg = self.data['mission']
            self.pub.publish(self.msg)
            rate.sleep()
            
            
    def angle_write(self, msg):
        self.data['des_angle'] = msg.y
        print("msg.y:", msg.y)
    


    
    def run(self):
        #rate = rospy.Rate(1) #1hz
        
        rospy.Subscriber('/gc_order', String, self.mission_write)
        rospy.Subscriber('/mission_mode/mission', Point, self.angle_write)

        self.publish_info()
        rospy.spin()


