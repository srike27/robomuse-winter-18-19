#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
class robomuse_mover():
    def __init__(self):
        self.yaw_publisher =  rospy.Publisher('/robomuse/yaw_joint_position_controller/command', Float64, queue_size=1)
        rospy.Subscriber('/cmd_vel',  Twist, self.callback_robomuse_vel)
        
    def turn(self, twist_msg):
        corrent_pos = rospy.wait_for_message("/joint_states", JointState)
        current_yaw = corrent_pos.position[2]
        new_pos = Float64()
        if twist_msg.angular.z > 0:
            new_pos.data = current_yaw + 0.1
            
        elif  twist_msg.angular.z < 0:
            new_pos.data = current_yaw - 0.1
            
        else:
            new_pos.data = current_yaw
        self.yaw_publisher.publish(new_pos)

            
    def callback_robomuse_vel(self,data):
        self.turn(data)

def loop():
        rospy.init_node("controller")
        mover = robomuse_mover()
        rospy.spin()
        
if __name__ == "__main__":
    loop()
