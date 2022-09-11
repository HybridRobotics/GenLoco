from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import os
import inspect

from motion_imitation.envs.env_wrappers.imitation_task import ImitationTask
from motion_imitation.envs.randomized_gym_env import ROBOT_CLASSES

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import os
import numpy as np

from motion_imitation.envs.env_wrappers import imitation_terminal_conditions
from motion_imitation.robots.base_robot import Base_robot
from motion_imitation.robots.a1 import A1
from motion_imitation.robots.anymal_b_simple import Anymal_b
from motion_imitation.robots.mini_cheetah import Mini_cheetah
from motion_imitation.robots.random_quadruped import RandomQuadruped

A1_LENGTH = 0.269
A1_HEIGHT = 0.2691589873082658

MOTION_FILES = {
  A1 : "motion_imitation/data/motions/a1_pace.txt",
  Base_robot : "motion_imitation/data/motions/laikago_pace.txt",
  Anymal_b : "motion_imitation/data/motions/anymal_b_pace.txt",
  Mini_cheetah : "motion_imitation/data/motions/mini_cheetah_pace.txt",
  RandomQuadruped: "motion_imitation/data/motions/a1_pace.txt"
}

S_0 = 0.5848105871191225

class RandomizedImitationTask(ImitationTask):

  def __init__(self,
               visualize,
               weight=1.0,
               terminal_condition=imitation_terminal_conditions.imitation_terminal_condition,
               ref_motion_filenames=None,
               enable_cycle_sync=True,
               enable_sync_root_rotation=False,
               clip_velocity=None,
               tar_frame_steps=None,
               clip_time_min=np.inf,
               clip_time_max=np.inf,
               ref_state_init_prob=1.0,
               enable_rand_init_time=True,
               enable_phase_only=False,
               warmup_time=0.0,
               pose_weight=0.6,
               velocity_weight=0.1,
               end_effector_weight=0.2,
               root_pose_weight=0.15,
               root_velocity_weight=0.15,
               pose_err_scale=5.0,
               velocity_err_scale=0.01, #altered from original paper
               end_effector_err_scale=40,
               end_effector_height_err_scale=3.0,
               root_pose_err_scale=20,
               root_velocity_err_scale=2,
               perturb_init_state_prob=0.0,
               tar_obs_noise=None,
               draw_ref_model_alpha=0.5):
    self._ref_motion_dict = {}
    self._robot_reward_scale = {}
    super().__init__(
               visualize,
               weight=weight,
               terminal_condition=terminal_condition,
               ref_motion_filenames=ref_motion_filenames,
               enable_cycle_sync=enable_cycle_sync,
               enable_sync_root_rotation=enable_sync_root_rotation,
               clip_velocity=clip_velocity,
               tar_frame_steps=tar_frame_steps,
               clip_time_min=clip_time_min,
               clip_time_max=clip_time_max,
               ref_state_init_prob=ref_state_init_prob,
               enable_rand_init_time=enable_rand_init_time,
               enable_phase_only=enable_phase_only,
               warmup_time=warmup_time,
               pose_weight=pose_weight,
               velocity_weight=velocity_weight,
               end_effector_weight=end_effector_weight,
               root_pose_weight=root_pose_weight,
               root_velocity_weight=root_velocity_weight,
               pose_err_scale=pose_err_scale,
               velocity_err_scale=velocity_err_scale,
               end_effector_err_scale=end_effector_err_scale,
               end_effector_height_err_scale=end_effector_height_err_scale,
               root_pose_err_scale=root_pose_err_scale,
               root_velocity_err_scale=root_velocity_err_scale,
               perturb_init_state_prob=perturb_init_state_prob,
               tar_obs_noise=tar_obs_noise,
               draw_ref_model_alpha=draw_ref_model_alpha)

  def reset(self, env):
    """Resets the internal state of the task."""
    self._env = env
    self._last_base_position = self._get_sim_base_position()
    self._episode_start_time_offset = 0.0

    if not self._ref_motion_dict:
      self.load_all_motions()
    self._ref_motions = self._ref_motion_dict[type(self._env.robot)]  ## load ref motions for current robot
    self._active_motion_id = self._sample_ref_motion()
    self._generate_new_motion()

    if self._visualize:
      self._ref_model = self._build_ref_model()

    self._build_joint_data()
    self._default_pose = self._record_default_pose()
    self._update_robot_reward_scale()

    rand_val = self._rand_uniform(0.0, 1.0)
    ref_state_init = rand_val < self._ref_state_init_prob

    self._curr_episode_warmup = False
    if not ref_state_init and self._enable_warmup():
      self._curr_episode_warmup = True

    self._reset_ref_motion()

    perturb_state = False
    if self._enable_perturb_init_state():
      rand_val = self._rand_uniform(0.0, 1.0)
      perturb_state = rand_val < self._perturb_init_state_prob

    self._sync_sim_model(perturb_state)

    return

  # def _reset_ref_motion(self):
  #   super()._reset_ref_motion()

  def _generate_new_motion(self):
    cur_ref_motion = self._ref_motions[self._sample_ref_motion()]
    new_ref_motion = copy.deepcopy(cur_ref_motion)
    offset = self._env.robot.GetDefaultInitPosition()[2] - A1_HEIGHT
    new_ref_motion._frames[...,2] += offset
    p = self._get_pybullet_client()
    length = p.getAABB(self._env.robot.quadruped)[1][0] - p.getAABB(self._env.robot.quadruped)[0][0]
    scale = length / A1_LENGTH
    new_ref_motion._frames[..., :2] *= scale
    new_ref_motion.refresh_motion()
    self._active_motion = new_ref_motion

  def get_active_motion(self):
    return self._active_motion

  def load_all_motions(self):
    for robot in ROBOT_CLASSES:
      self._ref_motion_dict[robot] = self._load_ref_motions([MOTION_FILES[robot]])

  def _robot_scale(self):
    return S_0 / self._robot_reward_scale[type(self._env.robot)]

  def _update_robot_reward_scale(self):
    robot = type(self._env.robot)
    if robot not in self._robot_reward_scale or robot == RandomQuadruped:
      p = self._get_pybullet_client()
      aabb = p.getAABB(self._env.robot.quadruped)
      self._robot_reward_scale[robot] = aabb[1][0] - aabb[0][0]
