import rospy
from std_msgs.msg import String
from sig_int_handler import SigIntHandler

class GroundControl:
    
    def __init__(self):
        self.mission = "NOPE"
        
    def run(self):
        self.pub = rospy.Publisher('gc_order', String, queue_size=10)
        rospy.init_node('GroundControl', anonymous=False)
        rate = rospy.Rate(1) #1hz
        
        while not rospy.is_shutdown():
            self.mission = input('MISSION?: ')
            
            if self.mission == 'exit':
                print("==EXIT==")
                break
            
            self.pub.publish(self.mission)
            print('Mission Sent: ', self.mission)
            rate.sleep()
        
        

if __name__ == '__main__':
    SI = SigIntHandler()
    SI.run()
    gc = GroundControl()        

    try:
        gc.run()
    except rospy.ROSInterruptException:
        exit(0)