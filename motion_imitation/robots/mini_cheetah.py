import math
import os
# import gin
import numpy as np
import re
from motion_imitation.robots import base_robot
from motion_imitation.robots import robot_config
from motion_imitation.envs import locomotion_gym_config

_BODY_B_FIELD_NUMBER = 2
_LINK_A_FIELD_NUMBER = 3
MAX_MOTOR_ANGLE_CHANGE_PER_STEP = 0.2
URDF_FILENAME = "robot_descriptions/mini_cheetah_description/mini_cheetah.urdf"

NUM_MOTORS = 12
NUM_LEGS = 4

MOTOR_NAMES = [
	'torso_to_abduct_fr_j',
	'abduct_fr_to_thigh_fr_j',
	'thigh_fr_to_knee_fr_j',
	'torso_to_abduct_fl_j',
	'abduct_fl_to_thigh_fl_j',
	'thigh_fl_to_knee_fl_j',
	'torso_to_abduct_hr_j',
	'abduct_hr_to_thigh_hr_j',
	'thigh_hr_to_knee_hr_j',
	'torso_to_abduct_hl_j',
	'abduct_hl_to_thigh_hl_j',
	'thigh_hl_to_knee_hl_j'
]
HIP_NAME_PATTERN = re.compile(r"torso_to_abduct_\w+")
HIP_MOTOR_NAME_PATTERN = re.compile(r"torso_to_abduct_\w+")
UPPER_NAME_PATTERN = re.compile(r"abduct_\w+")
LOWER_NAME_PATTERN = re.compile(r"thigh_\w+")
TOE_NAME_PATTERN = re.compile(r"toe_\w*")
IMU_NAME_PATTERN = re.compile(r"imu\d*")

_DEFAULT_TORQUE_LIMITS = [30, 30, 30] * 4
INIT_RACK_POSITION = [0, 0, 1.4]
INIT_POSITION = [0, 0, 0.35]
JOINT_DIRECTIONS = np.array([1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1])
HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = 0.0
KNEE_JOINT_OFFSET = 0.0
DOFS_PER_LEG = 3
JOINT_OFFSETS = np.array(
	[HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] * 4)
PI = math.pi
DEFAULT_ABDUCTION_ANGLE = 0.0
DEFAULT_HIP_ANGLE = 0.9
DEFAULT_KNEE_ANGLE = -1.8
# Bases on the readings from 's default pose.
INIT_MOTOR_ANGLES = np.array([
	                    DEFAULT_ABDUCTION_ANGLE, DEFAULT_HIP_ANGLE, DEFAULT_KNEE_ANGLE
                    ] * NUM_LEGS)
DEFAULT_LOCAL_TOE_POSITIONS = [[0.17, -0.11, -0.16], [0.17, 0.11, -0.16],
                               [-0.20, -0.11, -0.16], [-0.20, 0.11, -0.16]]

SIM_TOE_JOINT_IDS = [
    7,  # left hand
    15,  # left foot
    3,  # right hand
    11,  # right foot
]

ABDUCTION_P_GAIN = 100.0
ABDUCTION_D_GAIN = 1.
HIP_P_GAIN = 100.0
HIP_D_GAIN = 2.0
KNEE_P_GAIN = 100.0
KNEE_D_GAIN = 2.0

# @gin.configurable
class Mini_cheetah(base_robot.Base_robot):
	"""A simulation for the sirius robot."""
	ACTION_CONFIG = [
		locomotion_gym_config.ScalarField(name="torso_to_abduct_fr_j",
                                        upper_bound=0.802851455917,
                                        lower_bound=-0.802851455917),
      	locomotion_gym_config.ScalarField(name="abduct_fr_to_thigh_fr_j",
                                        upper_bound=4.18879020479,
                                        lower_bound=-1.0471975512),
      	locomotion_gym_config.ScalarField(name="thigh_fr_to_knee_fr_j",
                                        upper_bound=-0.916297857297,
                                        lower_bound=-2.69653369433),
      	locomotion_gym_config.ScalarField(name="torso_to_abduct_fl_j",
                                        upper_bound=0.802851455917,
                                        lower_bound=-0.802851455917),
      	locomotion_gym_config.ScalarField(name="abduct_fl_to_thigh_fl_j",
                                        upper_bound=4.18879020479,
                                        lower_bound=-1.0471975512),
      	locomotion_gym_config.ScalarField(name="thigh_fl_to_knee_fl_j",
                                        upper_bound=-0.916297857297,
                                        lower_bound=-2.69653369433),
      	locomotion_gym_config.ScalarField(name="torso_to_abduct_hr_j",
                                        upper_bound=0.802851455917,
                                        lower_bound=-0.802851455917),
      	locomotion_gym_config.ScalarField(name="abduct_hr_to_thigh_hr_j",
                                        upper_bound=4.18879020479,
                                        lower_bound=-1.0471975512),
      	locomotion_gym_config.ScalarField(name="thigh_hr_to_knee_hr_j",
                                        upper_bound=-0.916297857297,
                                        lower_bound=-2.69653369433),
      	locomotion_gym_config.ScalarField(name="torso_to_abduct_hl_j",
                                        upper_bound=0.802851455917,
                                        lower_bound=-0.802851455917),
      	locomotion_gym_config.ScalarField(name="abduct_hl_to_thigh_hl_j",
                                        upper_bound=4.18879020479,
                                        lower_bound=-1.0471975512),
      locomotion_gym_config.ScalarField(name="thigh_hl_to_knee_hl_j",
                                        upper_bound=-0.916297857297,
                                        lower_bound=-2.69653369433),
	]
	def __init__(self,
	             pybullet_client,
	             motor_control_mode,
	             urdf_filename=URDF_FILENAME,
	             enable_clip_motor_commands=False,
	             time_step=0.001,
	             action_repeat=33,
	             sensors=None,
	             control_latency=0.002,
	             on_rack=False,
	             enable_action_interpolation=True,
	             enable_action_filter=False,
	             reset_time=-1,
	             allow_knee_contact=True,
	             ):

		self._urdf_filename = urdf_filename
		self._allow_knee_contact = allow_knee_contact
		self._enable_clip_motor_commands = enable_clip_motor_commands
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

		super(Mini_cheetah, self).__init__(
			pybullet_client=pybullet_client,
			time_step=time_step,
			action_repeat=action_repeat,
			motor_direction=JOINT_DIRECTIONS,
			motor_torque_limits=_DEFAULT_TORQUE_LIMITS,
			motor_offset=JOINT_OFFSETS,
			motor_overheat_protection=False,
			motor_control_mode=motor_control_mode,
			sensors=sensors,
			motor_kp=motor_kp,
			motor_kd=motor_kd,
			control_latency=control_latency,
			on_rack=on_rack,
			enable_action_interpolation=enable_action_interpolation,
			enable_action_filter=enable_action_filter,
			reset_time=reset_time,
			allow_knee_contact=allow_knee_contact,
			enable_clip_motor_commands=enable_clip_motor_commands)

	def GetURDFFile(self):
		return self._urdf_filename

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
			self._pybullet_client.resetJointState(
				self.quadruped, self._joint_name_to_id[name], angle, targetVelocity=0)

	def _BuildUrdfIds(self):
		num_joints = self.pybullet_client.getNumJoints(self.quadruped)
		self._hip_link_ids = [-1]
		self._hip_motor_link_ids = []
		self._leg_link_ids = []
		self._motor_link_ids = []
		self._lower_link_ids = []
		self._foot_link_ids = []
		self._imu_link_ids = []

		for i in range(num_joints):
			joint_info = self.pybullet_client.getJointInfo(self.quadruped, i)
			joint_name = joint_info[1].decode("UTF-8")
			joint_id = self._joint_name_to_id[joint_name]
			if HIP_NAME_PATTERN.match(joint_name):
				self._hip_link_ids.append(joint_id)
			elif UPPER_NAME_PATTERN.match(joint_name):
				self._motor_link_ids.append(joint_id)
			# We either treat the lower leg or the toe as the foot link, depending on
			# the urdf version used.
			elif LOWER_NAME_PATTERN.match(joint_name):
				self._lower_link_ids.append(joint_id)
			elif TOE_NAME_PATTERN.match(joint_name):
				#assert self._urdf_filename == URDF_WITH_TOES
				self._foot_link_ids.append(joint_id)
			elif IMU_NAME_PATTERN.match(joint_name):
				self._imu_link_ids.append(joint_id)
				raise ValueError("Unknown category of joint %s" % joint_name)
			# build motor_id separately.
			if HIP_MOTOR_NAME_PATTERN.match(joint_name):  # use for hip reward calculation
				self._hip_motor_link_ids.append(joint_id)
		self._leg_link_ids.extend(self._lower_link_ids)
		if self._allow_knee_contact:
			self._foot_link_ids.extend(self._lower_link_ids)
			
		self._hip_link_ids.sort()
		self._hip_motor_link_ids.sort()
		self._motor_link_ids.sort()
		self._lower_link_ids.sort()
		self._foot_link_ids.sort()
		self._leg_link_ids.sort()

	def _ClipMotorCommands(self, motor_commands):
		"""Clips motor commands.

		Args:
		motor_commands: np.array. Can be motor angles, torques, hybrid commands,
			or motor pwms (for Minitaur only).

		Returns:
		Clipped motor commands.
		"""

		# clamp the motor command by the joint limit, in case weired things happens
		max_angle_change = MAX_MOTOR_ANGLE_CHANGE_PER_STEP
		current_motor_angles = self.GetMotorAngles()
		motor_commands = np.clip(motor_commands,
								current_motor_angles - max_angle_change,
								current_motor_angles + max_angle_change)
		return motor_commands

	def _GetMotorNames(self):
		return MOTOR_NAMES

	def _GetDefaultInitPosition(self):
		if self._on_rack:
			return INIT_RACK_POSITION
		else:
			return INIT_POSITION

	def _GetDefaultInitOrientation(self):
		init_orientation = self._pybullet_client.getQuaternionFromEuler(
			[0,0,0])
		return init_orientation

	def GetDefaultInitJointPose(self):
		"""Get default initial joint pose."""
		joint_pose = (INIT_MOTOR_ANGLES + JOINT_OFFSETS) * JOINT_DIRECTIONS
		return joint_pose

	def GetFootContacts(self): # check the id
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

	def GetInitMotorAngles(self):
		return INIT_MOTOR_ANGLES