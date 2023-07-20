from enum import Enum
import time

import dqrobotics as dql
from dqrobotics.interfaces.vrep import DQ_VrepInterface

try:
    from .DQRobot import DQRobot
except ImportError:
    from DQRobot import DQRobot


class DQVrepRobot(DQRobot):
    class MODE(Enum):
        SYNCHRONOUS = 0
        ASYNCHRONOUS = 1

    def __init__(self, json_path=None, vrep=None, vrep_joint_names=None, mode=MODE.ASYNCHRONOUS):
        super().__init__(json_path=json_path)

        assert isinstance(vrep, DQ_VrepInterface), "expect vrep interface"
        assert isinstance(vrep_joint_names, list), "expect input with list of joint name"
        assert len(vrep_joint_names) == self.robot.get_dim_configuration_space(), "joint list length mismatch"

        self.joint_names = vrep_joint_names
        self.vrep = vrep
        if mode == self.MODE.SYNCHRONOUS:
            self.vrep.set_synchronous(True)
            self.vrep.start_simulation()

    def get_joint_positions(self):
        return self.vrep.get_joint_positions(self.joint_names)

    def get_object_pose(self, name):
        return self.vrep.get_object_pose(name)

    def get_joint_pose_from_vrep(self, jointn):
        return self.vrep.get_object_pose(self.joint_names[jointn])

    def set_joint_target_positions(self, qs):
        return self.vrep.set_joint_target_positions(self.joint_names, qs)

    def set_joint_positions(self, qs):
        return self.vrep.set_joint_positions(self.joint_names, qs)


if __name__ == '__main__':
    from dqrobotics.interfaces.vrep import DQ_VrepInterface

    vrep = DQ_VrepInterface()
    if vrep.connect("10.198.113.186", 20001, 10, 100):
        print("connection successful")

    cobotta1_vrep = DQVrepRobot(
        json_path="/Moonshot/catkin_ws/src/aisp_ros_robot/robots/aisp_cobotta_robot_1_old.json",
        vrep=vrep,
        vrep_joint_names=[
            "Cobotta2_rjointb", "Cobotta2_pjointb", "Cobotta2_rjoint1", "Cobotta2_rjoint2", "Cobotta2_rjoint3",
            "Cobotta2_rjoint4", "Cobotta2_rjoint5", "Cobotta2_rjoint6"
        ],
        mode=DQVrepRobot.MODE.SYNCHRONOUS
    )
    time.sleep(1)
    print("getting joint position")

    print(cobotta1_vrep.get_object_pose("xd2"))
    print(cobotta1_vrep.get_joint_positions())
