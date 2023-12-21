from Open_manipulator_x import Open_manipulator_x
from open_manipulator_msgs.srv import *
from open_manipulator_msgs.msg import *
import math
import rospy
from projet_robotx import process_image

Qx, Qy , levels = process_image("./koro.jpg")
robot =  Open_manipulator_x()


# process_image()
rospy.init_node('service_set_joint_position_client')
rospy.wait_for_service('/goal_joint_space_path')

goal_joint_space_path_service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
goal_joint_space_path_request_object = SetJointPositionRequest()
# 1st Movement
goal_joint_space_path_request_object.planning_group = 'arm'
goal_joint_space_path_request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
goal_joint_space_path_request_object.joint_position.position = [0, 0, 0, 0]
goal_joint_space_path_request_object.joint_position.max_accelerations_scaling_factor = 1.0
goal_joint_space_path_request_object.joint_position.max_velocity_scaling_factor = 1.0
goal_joint_space_path_request_object.path_time = 1
rospy.loginfo("Doing Service Call 1...")
result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
print( result)
rospy.sleep(5)


# joints = robot.Inverse_kinematics(0.19, 0 , 0.13 , 0)
# robot.perform_MGD(joints, 2)

#                                       Opening the gripper
rospy.init_node('service_set_joint_position_client')
rospy.wait_for_service('/goal_tool_control')
goal_joint_space_path_service_client = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
goal_joint_space_path_request_object = SetJointPositionRequest()

goal_joint_space_path_request_object.planning_group = 'gripper'
goal_joint_space_path_request_object.joint_position.joint_name = ['gripper']
goal_joint_space_path_request_object.joint_position.position = [-0.01]
goal_joint_space_path_request_object.joint_position.max_accelerations_scaling_factor = 1.0
goal_joint_space_path_request_object.joint_position.max_velocity_scaling_factor = 1.0
goal_joint_space_path_request_object.path_time = 2.0

rospy.loginfo("Moving Gripper...")

result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
print(result)

# # joints = robot.Inverse_kinematics(0.18 , 0 , 0.056 , math.pi/2)
# # robot.perform_MGD(joints, 2)

# #
# #                                     Petit Code pour tracer un cercle
# #



# # x = [0.03 * math.cos(i * (2 * math.pi) / (50 - 1)) + 0.15 for i in range(50)]
# # y = [0.03 * math.sin(i * (2 * math.pi) / (50 - 1)) for i in range(50)]

# # joints = robot.Inverse_kinematics(0.18 , 0 , 0.06 , math.pi/2)
# # robot.perform_MGD(joints, 4)

# # for i in range(50):
# #     joints = robot.Inverse_kinematics(x[i] , y[i] , 0.04 , math.pi/2)
# #     robot.perform_MGD(joints, 2)

robot.perform_MGD(robot.Inverse_kinematics(Qx[0], Qy[0] , 0.07 , math.pi/2), 2)
rospy.sleep(1)

for i in range(Qx.shape[0]):
    if Qx[i] < 0.1:
        if i not in levels[:6]:
            robot.perform_MGD(robot.Inverse_kinematics(Qx[i], Qy[i] , 0.03 , math.pi/2), 1)
            rospy.sleep(2)
        else:
            robot.perform_MGD(robot.Inverse_kinematics(Qx[i-1], Qy[i-1] , 0.06 , math.pi/2), 2)
            rospy.sleep(2)

            robot.perform_MGD(robot.Inverse_kinematics(Qx[i+1], Qy[i+1] , 0.06 , math.pi/2), 2)
            rospy.sleep(2)

            joints = robot.Inverse_kinematics(Qx[i+1], Qy[i+1] , 0.03 , math.pi/2)
            robot.perform_MGD(joints, 2)
            rospy.sleep(2)
    if 0.1< Qx[i] < 0.14:
        if i not in levels[:6]:
            robot.perform_MGD(robot.Inverse_kinematics(Qx[i], Qy[i] , 0.039 , math.pi/2), 1)
            rospy.sleep(2)
        else:
            robot.perform_MGD(robot.Inverse_kinematics(Qx[i-1], Qy[i-1] , 0.06 , math.pi/2), 2)
            rospy.sleep(2)

            robot.perform_MGD(robot.Inverse_kinematics(Qx[i+1], Qy[i+1] , 0.06 , math.pi/2), 2)
            rospy.sleep(2)

            joints = robot.Inverse_kinematics(Qx[i+1], Qy[i+1] , 0.039 , math.pi/2)
            robot.perform_MGD(joints, 2)
            rospy.sleep(2)
    else:
        if i not in levels[:6]:
            robot.perform_MGD(robot.Inverse_kinematics(Qx[i], Qy[i] , 0.04 , math.pi/2), 1)
            rospy.sleep(2)
        else:
            robot.perform_MGD(robot.Inverse_kinematics(Qx[i-1], Qy[i-1] , 0.06 , math.pi/2), 2)
            rospy.sleep(2)

            robot.perform_MGD(robot.Inverse_kinematics(Qx[i+1], Qy[i+1] , 0.06 , math.pi/2), 2)
            rospy.sleep(2)

            joints = robot.Inverse_kinematics(Qx[i+1], Qy[i+1] , 0.04 , math.pi/2)
            robot.perform_MGD(joints, 2)
            rospy.sleep(2)
        
rospy.loginfo("Finished drawing")
rospy.sleep(1)
# 2nd Movement
goal_joint_space_path_request_object.planning_group = 'arm'
goal_joint_space_path_request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
goal_joint_space_path_request_object.joint_position.position = [0, 0, 0, 0]
goal_joint_space_path_request_object.joint_position.max_accelerations_scaling_factor = 1.0
goal_joint_space_path_request_object.joint_position.max_velocity_scaling_factor = 1.0
goal_joint_space_path_request_object.path_time = 1
rospy.loginfo("Doing Service Call 1...")
result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
print( result)
rospy.sleep(5)