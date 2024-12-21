#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_move_it_trajectories.py __ns:=my_gen3

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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from pynput import keyboard

# Variables globales
prev_pomni = [-0.0, 0.26888954639434814, -0.6397024095058441, 0.0013847780646756291, 0.794719398021698, 0.4079062342643738]
#6.396767147087701e-06, 0.0008715093686593178, 0.2502484952612356, -3.1489630588008204, -2.263139731031508, -0.006562771028641556, 0.9511477701026623, 7.846856068729541
pomni = [0.001, 0.250, -3.149, -2.263, -0.006, 0.951]


def callback_boton(pub_Gripper, data):
    print("boton")
    if(data.buttons[0] == 1):
        rospy.loginfo("Closing the gripper 50%...")
        mensaje = GripperCommandActionGoal()
        mensaje.header.stamp = rospy.Time.now()
        mensaje.header.frame_id = ''
        mensaje.goal_id.stamp = rospy.Time.now()
        mensaje.goal_id.id = ''
        mensaje.goal.command.position = 1.0
        mensaje.goal.command.max_effort = 10.0
        pub_Gripper.publish(mensaje)


    elif(data.buttons[0] == 0):
        rospy.loginfo("Opening the gripper...")
        mensaje = GripperCommandActionGoal()
        mensaje.header.stamp = rospy.Time.now()
        mensaje.header.frame_id = ''
        mensaje.goal_id.stamp = rospy.Time.now()
        mensaje.goal_id.id = ''
        mensaje.goal.command.position = 0.0
        mensaje.goal.command.max_effort = 10.0
        pub_Gripper.publish(mensaje)




def callback_posicion(msg):
    global pomni
    # Extraer solo la posición (x, y, z) desde PoseStamped
    pomni = [msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5]]

def calcular_peso(masa, g=9.81):

    # Calcular la magnitud de la fuerza
    fuerza_magnitud = masa * g

    # El vector de fuerza está dirigido hacia abajo (eje Z negativo)
    fuerza = [0.0, -fuerza_magnitud, 0.0]

    return fuerza

def calcular_pared(fuerza_pared):

    # El vector de fuerza está dirigido hacia abajo (eje Z negativo)
    fuerza = [-fuerza_pared, 0.0, 0.0]

    return fuerza

def main():
    global pub_fuerza, pub_marker, prev_pomni

    #Kinova = ExampleMoveItTrajectories()
    rospy.init_node('holabuenas')

    masa = 20

    joint_names = [
    "finger_joint", "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6", "joint_7"
    ]

    pub_Gripper = rospy.Publisher('/my_gen3/robotiq_2f_140_gripper_controller/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)
    sub = rospy.Subscriber('/arm/button2', Joy, partial(callback_boton, pub_Gripper))

    # Suscribirse al topic que publica la posición del omni usando PoseStamped
    #rospy.Subscriber('/arm/measured_cp', PoseStamped, callback_posicion)

    # Publicador para aplicar la fuerza en WrenchStamped
    pub_fuerza = rospy.Publisher('/arm/servo_cf', WrenchStamped, queue_size=10)

    # Publicador para RVIZ
    pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    rospy.Subscriber('/arm/measured_js', JointState, callback_posicion, queue_size=10)
    #rospy.wait_for_message('/arm/measured_js', JointState, callback_posicion)

    KinovaPub = rospy.Publisher('/my_gen3/gen3_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.Subscriber('/my_gen3/joint_states', JointState, queue_size=10)

    #IR TOCANDO EL RATE PARA PROBAR COMO VA CON EL PHANTOM
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Posición actual del OMNI") 
            print(pomni)
            """
            # Get actual pose
            actual_pose = Kinova.get_cartesian_pose()
            actual_pose.position.x = pomni[2] + 0.659
            actual_pose.position.y = pomni[0]
            actual_pose.position.z = pomni[1] + 0.434
            actual_pose.orientation.x = 0.5
            actual_pose.orientation.y = 0.5
            actual_pose.orientation.z = 0.5
            actual_pose.orientation.w = 0.5

            
            
            # Orientation constraint (we want the end effector to stay the same orientation)
            constraints = moveit_msgs.msg.Constraints()
            orientation_constraint = moveit_msgs.msg.OrientationConstraint()
            orientation_constraint.orientation = actual_pose.orientation
            constraints.orientation_constraints.append(orientation_constraint)
            

            # Send the goal
            Kinova_armgroup = Kinova.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
            """

            #Kinova.reach_joint_angles(pomni, 0.001)

            msg = JointTrajectory()
            msg.header.stamp = rospy.Time.now()
            msg.joint_names = [
                'joint_1', 'joint_2', 'joint_3', 
                'joint_4', 'joint_5', 'joint_6', 'joint_7'
            ]
            point = JointTrajectoryPoint()
            point.positions = [-pomni[0], -(pomni[1]-1), 0.0, -(pomni[2] - 1.5), pomni[3], -(pomni[4] - 2), -pomni[5]]  # Posición deseada
            point.velocities = [0.0] * 7  # Velocidades (0 para detenerse)
            point.time_from_start = rospy.Duration(0.5)  # Tiempo para alcanzar la posición (5 segundos)
    
            msg.points.append(point)
            KinovaPub.publish(msg)

            dataEffort = rospy.wait_for_message('/my_gen3/joint_states', JointState)
            if (dataEffort.effort[0] >= 0.2):
                fuerza = calcular_peso(masa)

                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = rospy.Time.now()
                wrench_msg.header.frame_id = "base"  # Asegúrate de usar el frame correcto  MIRAR FRAME EN RVIZ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                wrench_msg.wrench.force.x = fuerza[0]
                wrench_msg.wrench.force.y = fuerza[1]
                wrench_msg.wrench.force.z = fuerza[2]

                pub_fuerza.publish(wrench_msg)

            elif (dataEffort.effort[1] <= -30.0):
                fuerza = calcular_pared(500)

                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = rospy.Time.now()
                wrench_msg.header.frame_id = "base"  # Asegúrate de usar el frame correcto  MIRAR FRAME EN RVIZ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                wrench_msg.wrench.force.x = fuerza[0]
                wrench_msg.wrench.force.y = fuerza[1]
                wrench_msg.wrench.force.z = fuerza[2]

                pub_fuerza.publish(wrench_msg)


            else: 
                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = rospy.Time.now()
                wrench_msg.header.frame_id = "base"  # Asegúrate de usar el frame correcto  MIRAR FRAME EN RVIZ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                wrench_msg.wrench.force.x = 0.0
                wrench_msg.wrench.force.y = 0.0
                wrench_msg.wrench.force.z = 0.0

                pub_fuerza.publish(wrench_msg)

            rate.sleep()


            #masa = 20
            # Actualizar la posición anterior
            prev_pomni = pomni

        except rospy.ROSInterruptException:
            #Kinova.reach_named_position("home")
            wrench_msg.header.stamp = rospy.Time.now()
            wrench_msg.header.frame_id = "base"  # Asegúrate de usar el frame correcto  MIRAR FRAME EN RVIZ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            wrench_msg.wrench.force.x = 0.0
            wrench_msg.wrench.force.y = 0.0
            wrench_msg.wrench.force.z = 0.0

            pub_fuerza.publish(wrench_msg)

            pass

if __name__ == '__main__':
    main()