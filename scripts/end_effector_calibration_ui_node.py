#!/usr/bin/python3
# from typing import Any
import traceback
import sys, os

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray as rosmsg_Float64MultiArray

import rosilo_conversions as rc
from rosilo_robot_driver import RobotDriverInterface
import pathology_robot as PathologyRobot


from end_effector_calibration_ui import EndEffectorCalibrationUI
from PyQt5.QtWidgets import QApplication

import dqrobotics as dql


class AbrivatedKinematicsInterface():
    def __init__(self, topic_prefix):
        self.tool_pose_ = None
        self.subscriber_tool_pose_ = rospy.Subscriber(topic_prefix + "get/pose",
                                                        PoseStamped,
                                                        self._callback_tool_pose)

    def is_enabled(self):
        if self.tool_pose_ is None:
            return False
        return True

    def _callback_tool_pose(self, msg):
        self.tool_pose_ = rc.geometry_msgs_pose_to_dq(msg.pose)

    def get_tool_pose(self):
        return self.tool_pose_

def calibration_ui_main():
    rosrate = rospy.Rate(200)
    if calibrationConfig['use_tool_pose']:
        robot = None
    else:
        robot_DH = PathologyRobot.VS050Robot(calibrationConfig['robot_parameter_file_path'])
        robot = robot_DH.kinematics()
        # Define Joint Limits ([rad])
        robot_q_minus = robot.get_lower_q_limit()
        robot_q_plus = robot.get_upper_q_limit()
        robot_dim = robot.get_dim_configuration_space()

        dq_change = dql.DQ([1, 0, 0, 0, 0, 0, 0, 0])
        robot.set_effector(dq_change)
        robot.set_reference_frame(dq_change)
    try:

        rospy.loginfo("[" + name + "]::Waiting to connect to RobotDriverInterface...")
        # Robot driver interface
        robot_interface = RobotDriverInterface(rosConfig['arm_driver_ns']+'joints/')
        while not robot_interface.is_enabled():
            rosrate.sleep()
        rospy.loginfo("[" + name + "]:: robot_interface enabled.")

        if calibrationConfig['use_tool_pose']:
            robot_kinematics_interface = AbrivatedKinematicsInterface(rosConfig['arm_driver_ns']+'pose/')
            while not robot_kinematics_interface.is_enabled():
                rosrate.sleep()
        else:
            robot_kinematics_interface = None

        rospy.loginfo("[" + name + "]:: robot_kinematics_interface enabled.")

        # robot.set_reference_frame(robot_interface.get_reference_frame())

    except Exception as exp:
        # traceback.print_exc()
        rospy.logerr("[" + name + "]::"+traceback.format_exc())
        rospy.signal_shutdown(name + ": ERROR on ros Init")

    robot={
        "robot": robot,
        "robot_interface": robot_interface,
        "robot_kinematics_interface": robot_kinematics_interface,
    }
    rospy.loginfo("[" + name + "]::ROS setup complete.")

    app = QApplication([])
    widget = EndEffectorCalibrationUI(robot, rosConfig, calibrationConfig)
    widget.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    rospy.init_node('end_effector_calibration_ui_node',
                    anonymous=False,
                    disable_signals=True)

    name = rospy.get_name()
    params = rospy.get_param(name)

    calibrationConfig = {}
    rosConfig = {}

    try:
        ## still run through everythong to ensure all exist
        calibrationConfig['use_tool_pose'] = params.get('use_tool_pose', False)
        if calibrationConfig['use_tool_pose']:
            rospy.logwarn("[" + name + "]::calibrator is specified to use pose info from driver directly")
        else:
            rospy.logwarn("[" + name + "]::robot pose will be infered from robot parameter and joints")
            calibrationConfig['robot_parameter_file_path'] = params['robot_parameter_file_path']

        calibrationConfig['end_effector_info_save_path'] = params['end_effector_info_save_path']
        calibrationConfig['calibration_data_namespace'] = params['calibration_data_namespace']

        rospy.loginfo("[" + name + "]::calibrationConfig Parameter load OK.")

        rosConfig['arm_driver_ns'] = params['arm_driver_namespace']

        rospy.loginfo("[" + name + "]::rosConfig Parameter load OK.")
    except KeyError as e:

        rospy.logerr(name + ": initialization ERROR on parameters")
        rospy.logerr(name + ": " + str(e))
        rospy.signal_shutdown(name + ": initialization ERROR on parameters")
        exit()

    ns = rospy.get_namespace()
    rosConfig.update({
        'name': name,
        'ns': ns,
    })

    calibration_ui_main()
