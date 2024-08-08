#!/usr/bin/env python3

import sys
import rospy
import math
import moveit_commander
import tf.transformations as tf
from geometry_msgs.msg import Pose
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

class RobotControlMoveit():
    
    def __init__(self, group_name, robot_model_name, eef_link, tcp_link, home_deg):
        rospy.loginfo("Robot Control Moveit initializing...")
    
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.robot_model_name = robot_model_name
        self.eef_link = eef_link
        self.tcp_link = tcp_link
        self.group_name = group_name
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_end_effector_link(self.tcp_link)
        
        self.home_deg = home_deg
        
        self.init_gazebo_services()
        
        rospy.sleep(1)
        rospy.loginfo("Robot is ready now.")
    
    def set_vel_and_acc(self, velocity: int, acc: int):
        rospy.loginfo("Velocity and acceleration are setting to {} and {} ...".format(velocity, acc))
        self.move_group.set_max_velocity_scaling_factor(velocity)
        self.move_group.set_max_acceleration_scaling_factor(acc)
    
    def init_gazebo_services(self):
        rospy.loginfo("Waiting for gazebo services...")
        """ self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.attach_srv.wait_for_service()
        self.detach_srv.wait_for_service() """
    
    def get_current_joint_positions(self) -> list:
        joint_rad_pos = self.move_group.get_current_joint_values()
        joint_deg_pos = self.get_deg_from_rad(joint_rad_pos)
        return joint_deg_pos
    
    def get_current_cart_position(self) -> list:
        eef_pose = self.move_group.get_current_pose().pose
        cart_pos = [eef_pose.position.x, eef_pose.position.y, eef_pose.position.z]
        return cart_pos
    
    def get_current_rpy(self) -> list:
        rpy = self.move_group.get_current_rpy()
        return rpy
    
    def go_to_joint_position(self, joint_angles: list, angle_unit: str) -> bool:
        success = False
        
        if angle_unit == "deg":
            rad_angles = self.get_rad_from_deg(joint_angles)
            deg_angles = joint_angles
        elif angle_unit == "rad":
            rad_angles = joint_angles
            deg_angles = self.get_deg_from_rad(joint_angles)
        else:
            rospy.logerr("Angle unit should be 'deg' or 'rad' !!")
        
        string = [str(i) for i in deg_angles]
        rospy.loginfo("Going joint target --> [%s]", " ".join(string))
        
        plan_exec = self.move_group.go(rad_angles, wait=True)
        if plan_exec != True:
            rospy.logwarn("Plan couldn't be executed")
        else:
            rospy.loginfo("Target positions executed.")
            success = True
        self.move_group.stop()
        
        return success
    
    def go_to_cart_position(self, x: float, y: float, z: float, euler_rpy: list) -> bool:
        rospy.loginfo("Going cart target --> %f %f %f", x, y, z)
        pose_goal = Pose()
        
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        quaternion = tf.quaternion_from_euler(euler_rpy[0], euler_rpy[1], euler_rpy[2])
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        self.move_group.set_pose_target(pose_goal)
        
        success = self.move_group.go(wait=True)
        if success != True:
            rospy.logwarn("Plan couldn't be executed")
        else:
            rospy.loginfo("Target positions executed.")
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        return success
    
    def go_with_relative_linear_cart(self, x: float, y: float, z: float):
        eef_pose = self.move_group.get_current_pose().pose
        
        eef_pose.position.x = eef_pose.position.x + x
        eef_pose.position.y = eef_pose.position.y + y
        eef_pose.position.z = eef_pose.position.z + z
        
        rospy.loginfo("Going relative cart target --> %f %f %f", eef_pose.position.x, eef_pose.position.y, eef_pose.position.z)
        (plan, fraction) = self.move_group.compute_cartesian_path([eef_pose], 0.01, 0.0)
        fraction_thresh = 0.6
        if fraction < fraction_thresh:
            rospy.logwarn("Cartesian fraction is lower than %f ", fraction_thresh)
        success = self.move_group.execute(plan, wait=True)
        if success != True:
            rospy.logwarn("Plan couldn't be executed")
        else:
            rospy.loginfo("Target positions executed.")
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success
    
    def go_home(self):
        rospy.loginfo("Going home ...")
        success = self.go_to_joint_position(self.home_deg, "deg")
        return success

    def get_rad_from_deg(self, deg_angles):
        radian_angles = [math.radians(i) for i in deg_angles]
        return radian_angles

    def get_deg_from_rad(self, rad_angles):
        degree_angles = [math.degrees(i) for i in rad_angles]
        return degree_angles
        
    def open_vacuum_gripper(self, obj_model, obj_link):
        rospy.loginfo("Vacuum gripper activating for %s model object", obj_model)
        req = AttachRequest()
        req.model_name_1 = self.robot_model_name
        req.link_name_1 = self.eef_link
        req.model_name_2 = obj_model
        req.link_name_2 = obj_link
        self.attach_srv.call(req)
        rospy.sleep(0.5)
    
    def close_vacuum_gripper(self, obj_model, obj_link):
        rospy.loginfo("Vacuum gripper deactivating for %s model object", obj_model)
        req = AttachRequest()
        req.model_name_1 = self.robot_model_name
        req.link_name_1 = self.eef_link
        req.model_name_2 = obj_model
        req.link_name_2 = obj_link
        self.detach_srv.call(req)
        rospy.sleep(0.5)
