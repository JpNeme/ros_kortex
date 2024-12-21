#!/usr/bin/env python3
import sys
import time
import rospy
from sensor_msgs.msg import JointState, Joy

def main():
    rospy.init_node('prueba')

    pub = rospy.Publisher('/arm/measured_js', JointState, queue_size=1)
    sentido = 1
    joe = 0
    rate = rospy.Rate(10)
    #rate_descanso = rospy.Rate()
    while not rospy.is_shutdown():
        try:
            if joe > 10:
                joe = 0
                sentido *= -1

                rospy.sleep(4)
            
            mensaje = JointState()
            mensaje.header.stamp = rospy.Time.now()
            mensaje.position = [0.0] * 6
            mensaje.velocity = [0.0] * 6
            #Posiciones
            mensaje.position[0] = 0.0
            mensaje.position[1] = 0.0
            mensaje.position[2] = 0.0
            mensaje.position[3] = 0.0
            mensaje.position[4] = joe*sentido
            mensaje.position[5] = joe*sentido

            pub.publish(mensaje)

            #data = rospy.wait_for_message('my_gen3/joint_states', JointState)
            joe +=0.1
            #print(data)
            rate.sleep()
        except rospy.ROSInterruptException:
            print("He acabao")
            pass

if __name__ == '__main__':
    main()
