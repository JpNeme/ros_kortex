import sys
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64
from math import pi
from functools import partial
from control_msgs.msg import GripperCommandActionGoal
#from pynput import keyboard



def callback_posicion(data):
    print(data)

def main():
    rospy.Subscriber('/arm/measured_js', JointState, callback_posicion, queue_size=1, buff_size=1)
    while not rospy.is_shutdown():
        try:


            pass

        except ROSInterruptException:
            pass