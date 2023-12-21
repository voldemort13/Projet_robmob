import math
import rospy
from open_manipulator_msgs.srv import *
from open_manipulator_msgs.msg import *

class Open_manipulator_x:

    def __init__(self):
            # Service Initialization
            rospy.init_node('service_set_joint_position_client')
            self.service_forward_kinematics = "/goal_joint_space_path"

    def Inverse_kinematics(self,xe,ye,ze,alpha):

        a2 = 0.13
        a3 = 0.124
        a4 = 0.130
        d1 = 0.077
        d2 = 0.024

        q0 = math.atan2(ye, xe)
        r4 = math.sqrt(xe**2 + ye**2)
        r3 = r4 - a4 * math.cos(alpha)
        z3 = ze + a4 * math.sin(alpha)
        D = (a3 ** 2 + d2 ** 2 + a2 ** 2 - r3 ** 2 - (z3 - d1) ** 2) / (2 * math.sqrt(a2 ** 2 + d2 ** 2) * a3)
        q2 = math.pi / 2 - math.acos(D)
        phi1 = math.acos((a3 ** 2 + r3 ** 2 + (z3 - d1) ** 2 - a2 ** 2 - d2 ** 2) / (2 * a3 * math.sqrt(r3 ** 2 + (z3 - d1) ** 2)))
        phi2 = math.acos((a4 ** 2 + r3 ** 2 + (z3 - d1) ** 2 - r4 ** 2 - (ze - d1) ** 2) / (2 * a4 * math.sqrt(r3 ** 2 + (z3 - d1) ** 2)))
        q3 = math.pi - phi1 - phi2
        q1 = alpha - q2 - q3

        if abs(q0)<math.pi/2 and abs(q1)<math.pi/2 and abs(q2)<1.35 and abs(q3)<1.86 and isinstance(q0+q1+q2+q3, (int, float)):
            print('Achievable position')
            return q0 , q1 , q2 , q3
        else:
            print('Unreachable position')
            return 0 , 0 , 0 , 0
    
    def perform_MGD(self, joint_angles, path_time):
        # Function utilizing "/goal_joint_space_path" service for robot motion planning based on MGD (Modified Geometric Description).
        # Inputs: joint_angles (list of radians), path_time (time in seconds).
        # Output: is_planned (boolean) indicating motor reachability.
        # Wait for service response.
        q0 , q1 , q2 , q3 = joint_angles
        rospy.wait_for_service(self.service_forward_kinematics)
        try:
            # Create a function to call the service
            # Create a message to send to the service
            joints_service = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
            request_message = SetJointPositionRequest()
            
            request_message.planning_group = 'arm'

            # Specify which joint angle corresponds to which motor
            request_message.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']

            # Specify the joint angles to reach
            request_message.joint_position.position = [q0 , q1 , q2 , q3]
            
            request_message.joint_position.max_accelerations_scaling_factor = 1.0
            request_message.joint_position.max_velocity_scaling_factor = 1.0

            # Specify the time to reach the joint angles
            request_message.path_time = path_time
            # Call the service
            response = joints_service(request_message)
            return response
            
        # In case of an error, return "False" to indicate that the command generation has failed
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
 