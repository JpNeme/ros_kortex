import rospy
import numpy as np
from kinova_gen3.kinematics.jacobian import jacobian
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
from visualization_msgs.msg import Marker
from kinova_gen3 import *
#from kinova_gen3.performance_criteria.manipulability import manipulability_gradient
#from kinova_gen3.kinematics import inverse_kinematics   #pyKinovaGen3
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import JointState, Joy
import sys
import time
import moveit_commander
import moveit_msgs.msg
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from math import pi

# Variables globales
prev_pomni_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
pomni_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
kin_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories"""
    def __init__(self):

        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True


    def reach_named_position(self, target):
        arm_group = self.arm_group
        
        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)

    def reach_joint_angles(self, tolerance):
        arm_group = self.arm_group
        success = True

        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in joint_positions: rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = pi/2
            joint_positions[1] = 0
            joint_positions[2] = pi/4
            joint_positions[3] = -pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2
            joint_positions[6] = 0.2
        elif self.degrees_of_freedom == 6:
            joint_positions[0] = 0
            joint_positions[1] = 0
            joint_positions[2] = pi/2
            joint_positions[3] = pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2

        arm_group.set_joint_value_target(joint_positions)
        
        # Plan and execute in one command
        #success &= arm_group.go(wait=True)
        rm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions: rospy.loginfo(p)
        return success

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        #rospy.loginfo("Actual cartesian pose is : ")
        #rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group
        
        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        #rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)

    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False



# Callback para actualizar la velocidad del omni desde TwistStamped
#LOS EJES ESTAN RAROS Y LAS VELOCIDADES ANGULARES DAN 0 MIRAR SI EL TOPIC DA LAS VELOCIDADES EN QUATERNIONES
def callback_velocidad(msg):
    global pomni_vel
    # Extraer solo la velocidad linel y angular (x, y, z) desde TwistStamped
    pomni_vel = [-msg.twist.linear.y , msg.twist.linear.x, msg.twist.linear.z, msg.twist.linear.x , msg.twist.linear.y, msg.twist.linear.z]

def inverse_kinematics(joint_position, end_effector_vel):

    

    return np.linalg.pinv(jacobian(joint_position)) @ end_effector_vel

def callback_boton(data):
    print(data.buttons)


def positions_callback(data):
    global kin_positions
    kin_positions = data.position[1],data.position[2],data.position[3],data.position[4],data.position[5],data.position[6],data.position[7]

def calcular_peso(masa, g=9.81):

    # Calcular la magnitud de la fuerza
    fuerza_magnitud = masa * g

    # El vector de fuerza está dirigido hacia abajo (eje Z negativo)
    fuerza = [0.0, 0.0, -fuerza_magnitud]

    return fuerza

def publicar_fuerza():

    fuerza = calcular_peso(masa)

    # Crear un mensaje WrenchStamped para la fuerza
    wrench_msg = WrenchStamped()
    wrench_msg.header.stamp = rospy.Time.now()
    wrench_msg.header.frame_id = "base"  # Asegúrate de usar el frame correcto  MIRAR FRAME EN RVIZ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    wrench_msg.wrench.force.x = fuerza[0]
    wrench_msg.wrench.force.y = fuerza[1]
    wrench_msg.wrench.force.z = fuerza[2]

    pub_fuerza.publish(wrench_msg)

def main():
    global pub_fuerza, pub_marker, prev_pomni_vel

    Kinova = ExampleMoveItTrajectories()

    # Suscribirse al topic que publica la posición del omni usando TwistStamped
    sub1 = rospy.Subscriber('/arm/measured_cv', TwistStamped, callback_velocidad)

    # Publicador para aplicar la velocidad en Kinova
    sub2 = rospy.Subscriber('/my_gen3/joint_states', JointState, queue_size=10)

    # Publicador para aplicar la velocidad en Kinova Base_JointSpeeds
    pub_vel_Kinova = rospy.Publisher('/my_gen3/in/joint_velocity', Base_JointSpeeds, positions_callback)

    # Publicador para aplicar la fuerza en WrenchStamped
    pub_fuerza = rospy.Publisher('/arm/servo_cf', WrenchStamped, queue_size=10)

    sub3 = rospy.Subscriber('/arm/button1', Joy, callback_boton)


    rate = rospy.Rate(100)  # 100 Hz
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        try:
            """
            # Calcular el tiempo entre iteraciones
            current_time = rospy.get_time()
            dt = current_time - start_time
            print(f"dt:{dt}")
            start_time = current_time
            """

            #rospy.loginfo("Velocidad actual del OMNI") 
            #print(pomni_vel)

            velocities_array = inverse_kinematics(kin_positions, pomni_vel)
            
            joints_speeds = Base_JointSpeeds()
            #rospy.loginfo("Velocidad actual del OMNI") 
            #print(velocities_array)
            for i in range(len(velocities_array)):
                speed = JointSpeed()
                speed.joint_identifier = i
                speed.value = velocities_array[i]
                speed.duration = 0
                joints_speeds.joint_speeds.append(speed)

            pub_vel_Kinova.publish(joints_speeds)

            prev_pomni_vel = pomni_vel

            rospy.sleep(0.01)

        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo("Estoy en la excepcion")
            sub1.unregister()
            sub2.unregister()

            Kinova.reach_joint_angles(tolerance=0.01)

            """
            velocities_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            for i in range(len(velocities_array)):
                speed = JointSpeed()
                speed.joint_identifier = i
                speed.value = velocities_array[i]
                speed.duration = 0
                joints_speeds.joint_speeds.append(speed)

            pub_vel_Kinova.publish(joints_speeds)
            """
            
            return


if __name__ == '__main__':
    main()