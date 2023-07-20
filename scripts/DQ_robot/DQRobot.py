import numpy as np
import math
import os
#############################
# Setup logger
import logging

try:
    import rospy


    # Custom logging handler that forwards log messages to rospy logger
    class RospyLoggingHandler(logging.Handler):
        def emit(self, record):
            msg = self.format(record)
            if record.levelno >= logging.ERROR:
                rospy.logerr(msg)
            elif record.levelno >= logging.WARNING:
                rospy.logwarn(msg)
            elif record.levelno >= logging.INFO:
                rospy.loginfo(msg)
            else:
                rospy.logdebug(msg)


    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(RospyLoggingHandler())
    UNDER_ROS = True

except ModuleNotFoundError:
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    UNDER_ROS = False

import dqrobotics as dql
from dqrobotics.interfaces.json11 import DQ_JsonReader

import json


# Class definition of VS050 DENSO robot for pathology

class DQRobot:
    def __init__(self, json_path=None):
        # Standard of VS050

        if json_path is None or not os.path.isfile(json_path):
            raise ValueError("robot.json not specified")

        try:
            with open(json_path) as j_f:
                jdata = json.load(j_f)

        except Exception as e:
            raise ValueError("DH loading file read error")

        reader = DQ_JsonReader()

        if jdata['robot_type'] == "DQ_SerialManipulatorDH":
            self.robot = reader.get_serial_manipulator_dh_from_json(json_path)
        elif jdata['robot_type'] == "DQ_SerialManipulatorDenso":
            self.robot = reader.get_serial_manipulator_denso_from_json(json_path)
        else:
            raise ValueError("json parameter type definition error: " + str(type))

    # @staticmethod
    def get_kinematics(self):
        return self.robot

    def fkm(self, qs):
        return self.robot.fkm(qs)

    def get_reference_frame(self):
        return self.robot.get_reference_frame()

    def set_base_frame(self, x):
        return self.robot.set_base_frame(x)

    def set_effector(self, ee_x):
        self.robot.set_effector(ee_x)


############################# for testing only ################################
if __name__ == '__main__':
    jpath = "./denso_vs050_official_arm.json"
    # jpath="./denso_vs050_denso_11U483.json"
    robot = DQRobot(jpath).kinematics()
    logger.info(str(robot))
    logger.info(str(robot.fkm([0, 0, 0, 0, 0, 0])))
############################# for testing only ################################
