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
URDF_FILENAME = "robot_descriptions/siriusmid_belt_description/urdf/siriusmid_belt.urdf"

NUM_MOTORS = 12
NUM_LEGS = 4

MOTOR_NAMES = [
	'rf_abad_joint',
	'rf_hip_joint',
	'rf_knee_joint',
	'lf_abad_joint',
	'lf_hip_joint',
	'lf_knee_joint',
	'rr_abad_joint',
	'rr_hip_joint',
	'rr_knee_joint',
	'lr_abad_joint',
	'lr_hip_joint',
	'lr_knee_joint'
]
HIP_NAME_PATTERN = re.compile(r"\w+_hip_\w+")
HIP_MOTOR_NAME_PATTERN = re.compile(r"\w+_hip_\w+")
UPPER_NAME_PATTERN = re.compile(r"\w+_abad_\w+")
LOWER_NAME_PATTERN = re.compile(r"\w+_knee_\w+")
TOE_NAME_PATTERN = re.compile(r"\w+_foot_\d*")
IMU_NAME_PATTERN = re.compile(r"imu\d*")

_DEFAULT_TORQUE_LIMITS = [30, 30, 30] * 4
INIT_RACK_POSITION = [0, 0, 1.4]
INIT_POSITION = [0, 0, 0.45]
JOINT_DIRECTIONS = np.array([-1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1])
HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = 0.0
KNEE_JOINT_OFFSET = 0.0
DOFS_PER_LEG = 3
JOINT_OFFSETS = np.array(
	[HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] * 4)
PI = math.pi
DEFAULT_ABDUCTION_ANGLE = 0
DEFAULT_HIP_ANGLE = 0.7
DEFAULT_KNEE_ANGLE = -1.4
# Bases on the readings from 's default pose.
INIT_MOTOR_ANGLES = np.array([
	                    DEFAULT_ABDUCTION_ANGLE, DEFAULT_HIP_ANGLE, DEFAULT_KNEE_ANGLE
                    ] * NUM_LEGS)
DEFAULT_LOCAL_TOE_POSITIONS = [[0.17, -0.11, -0.16], [0.17, 0.11, -0.16],
                               [-0.20, -0.11, -0.16], [-0.20, 0.11, -0.16]]

SIM_TOE_JOINT_IDS = [
    5,  #LF
    13,  #LR
    9, #RF
    17, #RR
]

ABDUCTION_P_GAIN = 250
ABDUCTION_D_GAIN = 2.3
HIP_P_GAIN = 250
HIP_D_GAIN = 2.3
KNEE_P_GAIN = 250
KNEE_D_GAIN = 2.3

# @gin.configurable
class Siriusmid_belt(base_robot.Base_robot):
	"""A simulation for the sirius robot."""
	ACTION_CONFIG = [
		locomotion_gym_config.ScalarField(name="rf_abad_joint",
		                                  upper_bound=1.5,
		                                  lower_bound=-1.5),
		locomotion_gym_config.ScalarField(name="rf_hip_joint",
		                                  upper_bound=2.88,
		                                  lower_bound=-2.88),
		locomotion_gym_config.ScalarField(name="rf_knee_joint",
		                                  upper_bound=2.88,
		                                  lower_bound=-2.88),
		locomotion_gym_config.ScalarField(name="lf_abad_joint",
		                                  upper_bound=1.5,
		                                  lower_bound=-1.5),
		locomotion_gym_config.ScalarField(name="lf_hip_joint",
		                                 upper_bound=2.88,
		                                  lower_bound=-2.88),
		locomotion_gym_config.ScalarField(name="lf_knee_joint",
		                                  upper_bound=2.88,
		                                  lower_bound=-2.88),
		locomotion_gym_config.ScalarField(name="rr_abad_joint",
		                                  upper_bound=1.5,
		                                  lower_bound=-1.5),
		locomotion_gym_config.ScalarField(name="rr_hip_joint",
		                                  upper_bound=2.88,
		                                  lower_bound=-2.88),
		locomotion_gym_config.ScalarField(name="rr_knee_joint",
		                                  upper_bound=2.88,
		                                  lower_bound=-2.88),
		locomotion_gym_config.ScalarField(name="lr_abad_joint",
		                                  upper_bound=1.5,
		                                  lower_bound=-1.5),
		locomotion_gym_config.ScalarField(name="lr_hip_joint",
		                                  upper_bound=2.88,
		                                  lower_bound=-2.88),
		locomotion_gym_config.ScalarField(name="lr_knee_joint",
		                                  upper_bound=2.88,
		                                  lower_bound=-2.88),
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
	             allow_knee_contact=False
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

		super(Siriusmid_belt, self).__init__(
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
			# 	#assert self._urdf_filename == URDF_WITH_TOES
				self._foot_link_ids.append(joint_id)
			elif IMU_NAME_PATTERN.match(joint_name):
				self._imu_link_ids.append(joint_id)
				raise ValueError("Unknown category of joint %s" % joint_name)
			# build motor_id separately.
			if HIP_MOTOR_NAME_PATTERN.match(joint_name):  # use for hip reward calculation
				self._hip_motor_link_ids.append(joint_id)
		# self._foot_link_ids = self._lower_link_ids.copy()  # copy from lower_link_ids
		self._leg_link_ids.extend(self._lower_link_ids)
		self._leg_link_ids.extend(self._foot_link_ids)
		assert len(self._foot_link_ids) == NUM_LEGS
		self._hip_link_ids.sort()
		self._hip_motor_link_ids.sort()
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
