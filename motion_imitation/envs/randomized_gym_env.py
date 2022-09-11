
import random

import numpy as np

from motion_imitation.envs.locomotion_gym_env import LocomotionGymEnv
from motion_imitation.robots.random_quadruped import RandomQuadruped


ROBOT_CLASSES = [RandomQuadruped]#[A1, Base_robot, Sirius, Mini_cheetah, RandomQuadruped]
ROBOT_CLASS_IDX = {robot : i for i, robot in enumerate(ROBOT_CLASSES)}


class RandomizedGymEnv(LocomotionGymEnv):

  def __init__(self,
               gym_config,
               robot_class=None,
               env_sensors=None,
               robot_sensors=None,
               task=None,
               env_randomizers=None):
    if not robot_class:
      robot_class = random.choice(ROBOT_CLASSES)

    self.robot_sample_counters = np.array([1 for _ in ROBOT_CLASSES], dtype=np.int64)
    self._curriculum_scale = 1
    super().__init__(gym_config, robot_class, env_sensors, robot_sensors, task, env_randomizers)

  def _robot_reload(self):
    return True

  def load_robot(self):
    w = 1.0 / self.robot_sample_counters
    prob = w / np.sum(w)
    self._robot_class = np.random.choice(ROBOT_CLASSES, p=prob)
    super().load_robot()

  def build_robot(self):
    # Rebuild the robot
    self._robot = self._robot_class(
      pybullet_client=self._pybullet_client,
      sensors=self._robot_sensors,
      on_rack=self._on_rack,
      action_repeat=self._gym_config.simulation_parameters.
        num_action_repeat,
      motor_control_mode=self._gym_config.simulation_parameters.
        motor_control_mode,
      reset_time=self._gym_config.simulation_parameters.reset_time,
      enable_clip_motor_commands=self._gym_config.simulation_parameters.
        enable_clip_motor_commands,
      enable_action_filter=self._gym_config.simulation_parameters.
        enable_action_filter,
      enable_action_interpolation=self._gym_config.simulation_parameters.
        enable_action_interpolation,
      allow_knee_contact=self._gym_config.simulation_parameters.
        allow_knee_contact,
      delta_scale=self._curriculum_scale)

  def _update_robot_sample_counters(self):
    self.robot_sample_counters[ROBOT_CLASS_IDX[self._robot_class]] += self.env_current_cumulative_rewards

  def reset(self,
            initial_motor_angles=None,
            reset_duration=0.0,
            reset_visualization_camera=True):
    self._update_robot_sample_counters()
    return super().reset(initial_motor_angles=initial_motor_angles,
                  reset_duration=reset_duration,
                  reset_visualization_camera=reset_visualization_camera)

  def set_curriculum_scale(self, scale):
    self._curriculum_scale = scale