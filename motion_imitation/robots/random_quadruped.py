# coding=utf-8
# Copyright 2020 The Google Research Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pybullet simulation of a Laikago robot."""

import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import math
import re
import numpy as np
import pybullet as pyb  # pytype: disable=import-error

from motion_imitation.robots import laikago_constants, base_robot
from motion_imitation.robots import laikago_motor
from motion_imitation.robots import robot_config
from motion_imitation.envs import locomotion_gym_config
from motion_imitation.utilities.quadruped_generator import create_robot

NUM_MOTORS = 12
NUM_LEGS = 4

MOTOR_NAMES = [
    "joint2",
    "joint4",
    "joint5",
    "joint7",
    "joint9",
    "joint10",
    "joint12",
    "joint14",
    "joint15",
    "joint17",
    "joint19",
    "joint20"
]

INIT_RACK_POSITION = [0, 0, 1]
INIT_POSITION = [0, 0, 0.32]
JOINT_DIRECTIONS = np.ones(12)
HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = 0.0
KNEE_JOINT_OFFSET = 0.0
DOFS_PER_LEG = 3
JOINT_OFFSETS = np.array(
    [HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] * 4)
PI = math.pi

MAX_MOTOR_ANGLE_CHANGE_PER_STEP = 0.2
_DEFAULT_HIP_POSITIONS = (
    (0.17, -0.135, 0),
    (0.17, 0.13, 0),
    (-0.195, -0.135, 0),
    (-0.195, 0.13, 0),
)

COM_OFFSET = -np.array([0.012731, 0.002186, 0.000515])
HIP_OFFSETS = np.array([[0.183, -0.047, 0.], [0.183, 0.047, 0.],
                        [-0.183, -0.047, 0.], [-0.183, 0.047, 0.]
                        ]) + COM_OFFSET

ABDUCTION_P_GAIN = 100.0
ABDUCTION_D_GAIN = 1.
HIP_P_GAIN = 100.0
HIP_D_GAIN = 2.0
KNEE_P_GAIN = 100.0
KNEE_D_GAIN = 2.0

# Bases on the readings from Laikago's default pose.
INIT_MOTOR_ANGLES = np.array([0, 0.9, -1.8] * NUM_LEGS)

HIP_NAME_PATTERN = re.compile(r"\w+_hip_\w+")
UPPER_NAME_PATTERN = re.compile(r"\w+_upper_\w+")
LOWER_NAME_PATTERN = re.compile(r"\w+_lower_\w+")
TOE_NAME_PATTERN = re.compile(r"\w+_toe\d*")
IMU_NAME_PATTERN = re.compile(r"imu\d*")

HIPS = {'joint2', 'joint3', 'joint7', 'joint8', 'joint12', 'joint13', 'joint17', 'joint18'}
UPPER = {'joint4', 'joint9', 'joint14', 'joint19'}
LOWER = {'joint5', 'joint10', 'joint15', 'joint20'}
TOES = {'joint6', 'joint11', 'joint16', 'joint21'}
IMU = {'joint1'}

URDF_FILENAME = "ERROR - Random Quadruped has no urdf."

_BODY_B_FIELD_NUMBER = 2
_LINK_A_FIELD_NUMBER = 3

DEFAULT_MASS = 12.458

class RandomQuadruped(base_robot.Base_robot):

    # At high replanning frequency, inaccurate values of BODY_MASS/INERTIA
    # doesn't seem to matter much. However, these values should be better tuned
    # when the replan frequency is low (e.g. using a less beefy CPU).
    MPC_BODY_MASS = 108 / 9.8
    MPC_BODY_INERTIA = np.array((0.017, 0, 0, 0, 0.057, 0, 0, 0, 0.064)) * 4.
    MPC_BODY_HEIGHT = 0.24
    MPC_VELOCITY_MULTIPLIER = 0.5
    ACTION_CONFIG = [
        locomotion_gym_config.ScalarField(name="joint2",
                                          upper_bound=0.802851455917,
                                          lower_bound=-0.802851455917),
        locomotion_gym_config.ScalarField(name="joint4",
                                          upper_bound=4.18879020479,
                                          lower_bound=-1.0471975512),
        locomotion_gym_config.ScalarField(name="joint5",
                                          upper_bound=-0.916297857297,
                                          lower_bound=-2.69653369433),
        locomotion_gym_config.ScalarField(name="joint7",
                                          upper_bound=0.802851455917,
                                          lower_bound=-0.802851455917),
        locomotion_gym_config.ScalarField(name="joint9",
                                          upper_bound=4.18879020479,
                                          lower_bound=-1.0471975512),
        locomotion_gym_config.ScalarField(name="joint10",
                                          upper_bound=-0.916297857297,
                                          lower_bound=-2.69653369433),
        locomotion_gym_config.ScalarField(name="joint12",
                                          upper_bound=0.802851455917,
                                          lower_bound=-0.802851455917),
        locomotion_gym_config.ScalarField(name="joint14",
                                          upper_bound=4.18879020479,
                                          lower_bound=-1.0471975512),
        locomotion_gym_config.ScalarField(name="joint15",
                                          upper_bound=-0.916297857297,
                                          lower_bound=-2.69653369433),
        locomotion_gym_config.ScalarField(name="joint17",
                                          upper_bound=0.802851455917,
                                          lower_bound=-0.802851455917),
        locomotion_gym_config.ScalarField(name="joint19",
                                          upper_bound=4.18879020479,
                                          lower_bound=-1.0471975512),
        locomotion_gym_config.ScalarField(name="joint20",
                                          upper_bound=-0.916297857297,
                                          lower_bound=-2.69653369433),
    ]

    def __init__(
            self,
            pybullet_client,
            urdf_filename=URDF_FILENAME,
            enable_clip_motor_commands=False,
            time_step=0.001,
            action_repeat=10,
            sensors=None,
            control_latency=0.002,
            on_rack=False,
            enable_action_interpolation=True,
            enable_action_filter=False,
            motor_control_mode=None,
            reset_time=1,
            allow_knee_contact=False,
            delta_scale=0,
    ):
        self._urdf_filename = urdf_filename
        self._allow_knee_contact = allow_knee_contact
        self._enable_clip_motor_commands = enable_clip_motor_commands
        self._delta_scale = delta_scale
        self.init_position = INIT_POSITION

        motor_kp = [
            ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN,
            HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN,
            ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN
        ]
        motor_kd = [
            ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN,
            HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN,
            ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN
        ]

        super(RandomQuadruped, self).__init__(
            pybullet_client=pybullet_client,
            time_step=time_step,
            action_repeat=action_repeat,
            num_motors=NUM_MOTORS,
            dofs_per_leg=DOFS_PER_LEG,
            motor_direction=JOINT_DIRECTIONS,
            motor_offset=JOINT_OFFSETS,
            motor_overheat_protection=False,
            motor_control_mode=motor_control_mode,
            motor_model_class=laikago_motor.LaikagoMotorModel,
            sensors=sensors,
            motor_kp=motor_kp,
            motor_kd=motor_kd,
            control_latency=control_latency,
            on_rack=on_rack,
            enable_action_interpolation=enable_action_interpolation,
            enable_action_filter=enable_action_filter,
            reset_time=reset_time)
        totalmass = self._CalcTotalMass()
        self.SetMotorGains(
            np.asarray(motor_kp) * totalmass / DEFAULT_MASS,
            np.asarray(motor_kd) * totalmass / DEFAULT_MASS
        )
        self._CalcInitPos()

    def _LoadRobotURDF(self):
        error = 999
        while error:
            n = error - 1
            error = 0
            try:
                self.quadruped = create_robot(self._pybullet_client, delta_scale=self._delta_scale)
            except Exception as e:
                with open("loadRobotURDF_zzz_293.txt", "a") as f:
                    f.write("Issue creating robot in RandomQuadruped._LoadRobotURDF(): {}".format(e))
                    f.write("Retrying..., n = {}".format(n))
                error = n

    def _CalcTotalMass(self):
        totalmass = 0
        for i in range(-1, self.pybullet_client.getNumJoints(self.quadruped)): #include base
            totalmass += self.pybullet_client.getDynamicsInfo(self.quadruped, i)[0]
        return totalmass

    def _CalcInitPos(self):
        t = self._pybullet_client.getAABB(self.quadruped)[0][2]
        base_height = self._pybullet_client.getBasePositionAndOrientation(self.quadruped)[0][2]
        for i in range(self._pybullet_client.getNumJoints(self.quadruped)):
            t_i = self._pybullet_client.getAABB(self.quadruped, i)
            t = min(t, self._pybullet_client.getAABB(self.quadruped, i)[0][2])
            test = self._pybullet_client.getLinkState(self.quadruped, i)
        self.init_position = [0, 0, base_height - t] #toes right on the ground

    def _SettleDownForReset(self, default_motor_angles, reset_time):
        self.ReceiveObservation()
        if reset_time <= 0:
            return

        for _ in range(500):
            self._StepInternal(
                INIT_MOTOR_ANGLES,
                motor_control_mode=robot_config.MotorControlMode.POSITION)

        if default_motor_angles is not None:
            num_steps_to_reset = int(reset_time / self.time_step)
            for _ in range(num_steps_to_reset):
                self._StepInternal(
                    default_motor_angles,
                    motor_control_mode=robot_config.MotorControlMode.POSITION)

    def GetHipPositionsInBaseFrame(self):
        return _DEFAULT_HIP_POSITIONS

    def GetFootContacts(self):
        all_contacts = self._pybullet_client.getContactPoints(bodyA=self.quadruped)

        contacts = [False, False, False, False]
        for contact in all_contacts:
            # Ignore self contacts
            if contact[_BODY_B_FIELD_NUMBER] == self.quadruped:
                continue
            try:
                toe_link_index = self._foot_link_ids.index(
                    contact[_LINK_A_FIELD_NUMBER])
                contacts[toe_link_index] = True
            except ValueError:
                continue

        return contacts

    def ResetPose(self, add_constraint):
        del add_constraint
        for name in self._joint_name_to_id:
            joint_id = self._joint_name_to_id[name]
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=(joint_id),
                controlMode=self._pybullet_client.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0)
        angles = self.GetDefaultInitJointPose()
        for name, i in zip(MOTOR_NAMES, range(len(MOTOR_NAMES))):
            angle = angles[i]
            self._pybullet_client.resetJointState(self.quadruped,
                                                  self._joint_name_to_id[name],
                                                  angle,
                                                  targetVelocity=0)

    def GetURDFFile(self):
        return self._urdf_filename

    def _BuildUrdfIds(self):
        """Build the link Ids from its name in the URDF file.

        Raises:
          ValueError: Unknown category of the joint name.
        """
        num_joints = self.pybullet_client.getNumJoints(self.quadruped)
        self._hip_link_ids = [-1]
        self._leg_link_ids = []
        self._motor_link_ids = []
        self._lower_link_ids = []
        self._foot_link_ids = []
        self._imu_link_ids = []

        for i in range(num_joints):
            joint_info = self.pybullet_client.getJointInfo(self.quadruped, i)
            joint_name = joint_info[1].decode("UTF-8")

            joint_id = self._joint_name_to_id[joint_name]

            if joint_name in HIPS:
                self._hip_link_ids.append(joint_id)
            elif joint_name in UPPER:
                self._motor_link_ids.append(joint_id)
            # We either treat the lower leg or the toe as the foot link, depending on
            # the urdf version used.
            elif joint_name in LOWER:
                self._lower_link_ids.append(joint_id)
            elif joint_name in TOES:
                # assert self._urdf_filename == URDF_WITH_TOES
                self._foot_link_ids.append(joint_id)
            elif joint_name in IMU:
                self._imu_link_ids.append(joint_id)
            else:
                raise ValueError("Unknown category of joint %s" % joint_name)

        self._leg_link_ids.extend(self._lower_link_ids)
        self._leg_link_ids.extend(self._foot_link_ids)

        # assert len(self._foot_link_ids) == NUM_LEGS
        self._hip_link_ids.sort()
        self._motor_link_ids.sort()
        self._lower_link_ids.sort()
        self._foot_link_ids.sort()
        self._leg_link_ids.sort()

    def _GetMotorNames(self):
        return MOTOR_NAMES

    def _GetDefaultInitPosition(self):
        if self._on_rack:
            return INIT_RACK_POSITION
        else:
            return self.init_position

    def _GetDefaultInitOrientation(self):
        # The Laikago URDF assumes the initial pose of heading towards z axis,
        # and belly towards y axis. The following transformation is to transform
        # the Laikago initial orientation to our commonly used orientation: heading
        # towards -x direction, and z axis is the up direction.
        init_orientation = pyb.getQuaternionFromEuler([0., 0., 0.])
        return init_orientation

    def GetDefaultInitJointPose(self):
        """Get default initial joint pose."""
        joint_pose = (INIT_MOTOR_ANGLES + JOINT_OFFSETS) * JOINT_DIRECTIONS
        return joint_pose

    def GetInitMotorAngles(self):
        return INIT_MOTOR_ANGLES

    @classmethod
    def GetConstants(cls):
        del cls
        return laikago_constants

