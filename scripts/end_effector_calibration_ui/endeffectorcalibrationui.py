# This Python file uses the following encoding: utf-8
import os
from pathlib import Path
import sys
import yaml

from PyQt5 import uic, QtGui
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtCore import QFile

import dqrobotics as dql
# from dqrobotics.robot_modeling import DQ_Kinematics
import numpy as np

import rospy


def combn2(keys):
    index1 = []
    index2 = []
    for ind, key in enumerate(keys):
        for j in range(ind + 1, len(keys)):
            index1.append(keys[ind])
            index2.append(keys[j])

    return index1, index2


class EndEffectorCalibrationUI(QWidget):
    def __init__(self, robot, rosConfig, calibrationConfig):
        super(EndEffectorCalibrationUI, self).__init__()

        # save robot handle
        self._robot = robot['robot']
        self._robot_donfig_dim = robot['robot'].get_dim_configuration_space()
        self._robot_interface = robot['robot_interface']
        self._robot_kinematics_interface = robot['robot_kinematics_interface']
        self._rosConfig = rosConfig
        self._calibrationConfig = calibrationConfig

        # list of data points
        self._calibration_joint_list = {}
        self._calibration_pose_list = {}
        self._calibration_point_count = 0

        self.load_ui()

    def load_ui(self):
        path = os.path.join(os.path.dirname(__file__), "form.ui")
        uic.loadUi(path, self)  # Load the .ui file

        self.addButton.clicked.connect(self.add_btn_pressed_)
        self.removeButton.clicked.connect(self.remove_btn_pressed_)
        self.calculateButton.clicked.connect(self.calculate_btn_pressed_)
        self.poseListWidget.currentItemChanged.connect(self.list_select_changed_)

        calibration_joint_list = rospy.get_param(
            self._calibrationConfig['calibration_data_namespace'] + "calibration_joint", {})
        calibration_pose_list = rospy.get_param(
            self._calibrationConfig['calibration_data_namespace'] + "calibration_pose", {})

        # generate joint display
        for i in range(1, self._robot_donfig_dim + 1):
            setattr(self, "jointDisplay_" + str(i), QLabel(
                parent=self, text="N/A"
            ))
            setattr(self, "jointLabel_" + str(i), QLabel(
                parent=self, text="joint " + str(i)
            ))
            self.gridLayout.addWidget(getattr(self, "jointLabel_" + str(i)), i, 0)
            self.gridLayout.addWidget(getattr(self, "jointDisplay_" + str(i)), i, 1)

        for ind, (key, val) in enumerate(calibration_joint_list.items()):
            try:
                self._calibration_point_count = int(key) + 1
            except ValueError:
                continue
            self._calibration_joint_list[key] = val
            self._calibration_pose_list[key] = dql.DQ(calibration_joint_list[key]).normalize()
            rospy.loginfo(
                "[" + self._rosConfig['name'] + "]::got historical data " + str(self._calibration_pose_list[key]))
            self.poseListWidget.insertItem(ind, key)

    def update_ros_param_(self):
        for key, joint_q in self._calibration_joint_list.items():
            if not isinstance(joint_q, list):
                joint_q = joint_q.tolist()
            rospy.set_param(os.path.join(self._calibrationConfig['calibration_data_namespace'],
                                         "calibration_joint", key),
                            joint_q)
        for key, pose in self._calibration_pose_list.items():
            rospy.set_param(os.path.join(self._calibrationConfig['calibration_data_namespace'],
                                         "calibration_pose", key),
                            pose.vec8().tolist())

    def set_max_entry_count_(self):
        calibration_point_count = 0
        for key in self._calibration_joint_list:
            if int(key) >= calibration_point_count:
                calibration_point_count = int(key) + 1

        self._calibration_point_count = calibration_point_count

    def add_btn_pressed_(self):
        joints_entry = self._robot_interface.get_joint_positions()
        if self._calibrationConfig['use_tool_pose']:
            pose_entry = self._robot_kinematics_interface.get_tool_pose()
            rospy.loginfo("[" + self._rosConfig['name'] + "]::logged with tool pose")
        else:
            pose_entry = self._robot.fkm(joints_entry)

        self._calibration_joint_list[str(self._calibration_point_count)] = joints_entry
        self._calibration_pose_list[str(self._calibration_point_count)] = pose_entry
        self._calibration_point_count += 1

        self.poseListWidget.insertItem(0, str(self._calibration_point_count - 1))

        rospy.loginfo("[" + self._rosConfig['name'] + "]::Added datapoint: (joint) "
                      + str(joints_entry)
                      + " (pose) " + str(pose_entry))
        self.update_ros_param_()

    def remove_btn_pressed_(self):

        selected = self.poseListWidget.selectedItems()
        if not selected:
            return
        for item in selected:
            self._calibration_joint_list.pop(item.text(), None)
            self._calibration_pose_list.pop(item.text(), None)
            itemIndex = self.poseListWidget.row(item)
            self.poseListWidget.takeItem(itemIndex)

        self.set_max_entry_count_()
        self.update_ros_param_()

    def list_select_changed_(self, current, previous):
        if current is None:
            for ind in range(6):
                label_h = getattr(self, "jointDisplay_" + str(ind + 1))
                label_h.setText("N/A")
            return
        key = current.text()
        for ind, joint_val in enumerate(self._calibration_joint_list[key]):
            label_h = getattr(self, "jointDisplay_" + str(ind + 1))
            label_h.setText("{:.5f}".format(joint_val))

    def calculate_btn_pressed_(self):

        calibration_pose_list = self._calibration_pose_list

        n = len(calibration_pose_list.keys())

        list_of_rotations = {}
        list_of_translations = {}
        for key, pose in calibration_pose_list.items():
            list_of_rotations[key] = dql.rotation(pose)
            list_of_translations[key] = dql.translation(pose)

        index1, index2 = combn2(list(calibration_pose_list))
        ncomb = len(index1)

        rospy.loginfo("[" + self._rosConfig['name'] + "]::Number of possible 2x2 combinations = " + str(ncomb))

        A_ext = np.zeros((ncomb * 4, 4))
        B_ext = np.zeros((ncomb * 4))

        for i in range(ncomb):
            a = index1[i]
            b = index2[i]

            td = list_of_translations[a] - list_of_translations[b];
            B = np.array([0, td.q[1], td.q[2], td.q[3]])
            B_ext[i * 4:i * 4 + 4] = B

            ra = list_of_rotations[a]
            rb = list_of_rotations[b]
            A = dql.hamiplus4(ra) @ dql.haminus4(dql.conj(ra)) - dql.hamiplus4(rb) @ dql.haminus4(dql.conj(rb))
            A_ext[i * 4:i * 4 + 4, :] = A

        d = -np.linalg.pinv(A_ext) @ B_ext
        rospy.loginfo("[" + self._rosConfig['name'] + "]::Distance vector found as = " + str(d))

        calibration_resultdq = dql.DQ([1, 0, 0, 0, 0, d[1], d[2], d[3]])
        rospy.set_param(os.path.join(self._calibrationConfig['calibration_data_namespace'], "calibration", "result"),
                        calibration_resultdq.q.tolist())
        data = {
            "translation": d[1:].tolist(),
            "dq_change": calibration_resultdq.q.tolist(),
        }
        with open(self._calibrationConfig['end_effector_info_save_path'], 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style=None)


if __name__ == "__main__":
    app = QApplication([])
    widget = EndEffectorCalibrationUI()
    widget.show()
    sys.exit(app.exec_())
