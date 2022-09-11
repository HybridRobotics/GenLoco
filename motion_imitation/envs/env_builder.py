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

import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from motion_imitation.envs import locomotion_gym_env
from motion_imitation.envs import locomotion_gym_config
from motion_imitation.envs import randomized_gym_env
from motion_imitation.envs.env_wrappers import imitation_wrapper_env
from motion_imitation.envs.env_wrappers import observation_dictionary_to_array_wrapper
from motion_imitation.envs.env_wrappers import imitation_task
from motion_imitation.envs.env_wrappers import randomized_imitation_task
from motion_imitation.envs.env_wrappers import default_task

from motion_imitation.envs.sensors import environment_sensors
from motion_imitation.envs.sensors import sensor_wrappers
from motion_imitation.envs.sensors import robot_sensors
from motion_imitation.envs.utilities import controllable_env_randomizer_from_config
from motion_imitation.robots import base_robot
from motion_imitation.robots import a1
from motion_imitation.robots import robot_config





def build_laikago_env( motor_control_mode, enable_rendering):

  sim_params = locomotion_gym_config.SimulationParameters()
  sim_params.enable_rendering = enable_rendering
  sim_params.motor_control_mode = motor_control_mode
  sim_params.reset_time = 2
  sim_params.num_action_repeat = 10
  sim_params.enable_action_interpolation = False
  sim_params.enable_action_filter = False
  sim_params.enable_clip_motor_commands = False
  
  gym_config = locomotion_gym_config.LocomotionGymConfig(simulation_parameters=sim_params)

  robot_class = base_robot.Base_robot

  sensors = [
      robot_sensors.MotorAngleSensor(num_motors=base_robot.NUM_MOTORS),
      robot_sensors.IMUSensor(),
      environment_sensors.LastActionSensor(num_actions=base_robot.NUM_MOTORS)
  ]

  task = default_task.DefaultTask()

  env = locomotion_gym_env.LocomotionGymEnv(gym_config=gym_config, robot_class=robot_class,
                                            robot_sensors=sensors, task=task)

  #env = observation_dictionary_to_array_wrapper.ObservationDictionaryToArrayWrapper(env)
  #env = trajectory_generator_wrapper_env.TrajectoryGeneratorWrapperEnv(env)

  return env


def build_imitation_env(motion_files, num_parallel_envs, mode,
                        enable_randomizer, enable_rendering,
                        enable_sync_root_rotation,
                        enable_phase_only, enable_randomized_robot,
                        robot_class=base_robot.Base_robot, visualize=False):
  ### MAKESHIFT PART FOR VERIFYING TRAINING WORKS
  visualize = (not enable_randomized_robot) and enable_rendering
  assert len(motion_files) > 0

  curriculum_episode_length_start = 20
  curriculum_episode_length_end = 300
  
  sim_params = locomotion_gym_config.SimulationParameters()
  sim_params.enable_rendering = enable_rendering
  sim_params.allow_knee_contact = True
  sim_params.motor_control_mode = robot_config.MotorControlMode.POSITION

  gym_config = locomotion_gym_config.LocomotionGymConfig(simulation_parameters=sim_params)

  if enable_phase_only:
    sensors = [
      sensor_wrappers.HistoricSensorWrapper(wrapped_sensor=robot_sensors.MotorAngleSensor(num_motors=base_robot.NUM_MOTORS), num_history=15),
      sensor_wrappers.HistoricSensorWrapper(wrapped_sensor=robot_sensors.IMUSensor(channels=['R', 'P', 'Y', 'dR', 'dP', 'dY']), num_history=15),
      sensor_wrappers.HistoricSensorWrapper(wrapped_sensor=environment_sensors.LastActionSensor(num_actions=base_robot.NUM_MOTORS), num_history=15)
  ]
  else:
    sensors = [
      sensor_wrappers.HistoricSensorWrapper(wrapped_sensor=robot_sensors.MotorAngleSensor(num_motors=base_robot.NUM_MOTORS), num_history=15),
      sensor_wrappers.HistoricSensorWrapper(wrapped_sensor=robot_sensors.IMUSensor(), num_history=15),
      sensor_wrappers.HistoricSensorWrapper(wrapped_sensor=environment_sensors.LastActionSensor(num_actions=base_robot.NUM_MOTORS), num_history=15)
  ]
  # prevent initializing from reference motion when testing, 
  # as we do not provide robot-specific reference motion.
  if mode == "test":
    ref_state_init_prob=0.0  
  else:
    ref_state_init_prob=0.9  # training

  if enable_randomized_robot:
    task = randomized_imitation_task.RandomizedImitationTask(ref_motion_filenames=motion_files,
                                        enable_cycle_sync=True,
                                        enable_sync_root_rotation=enable_sync_root_rotation,
                                        enable_phase_only=enable_phase_only,
                                        tar_frame_steps=[1, 2, 10, 30],
                                        ref_state_init_prob=ref_state_init_prob,
                                        warmup_time=0.25,
                                        visualize=False)
  else:
    task = imitation_task.ImitationTask(ref_motion_filenames=motion_files,
                                        mode=mode,
                                        enable_cycle_sync=True,
                                        enable_sync_root_rotation=enable_sync_root_rotation,
                                        enable_phase_only=enable_phase_only,
                                        tar_frame_steps=[1, 2, 10, 30],
                                        ref_state_init_prob=ref_state_init_prob,
                                        warmup_time=0.25,
                                        visualize=False)

  randomizers = []
  if enable_randomizer:
    randomizer = controllable_env_randomizer_from_config.ControllableEnvRandomizerFromConfig(verbose=False)
    randomizers.append(randomizer)

  if enable_randomized_robot:
    env = randomized_gym_env.RandomizedGymEnv(gym_config=gym_config, robot_class=robot_class,
                                              env_randomizers=randomizers, robot_sensors=sensors, task=task)
  else:
    env = locomotion_gym_env.LocomotionGymEnv(gym_config=gym_config, robot_class=robot_class,
                                              env_randomizers=randomizers, robot_sensors=sensors, task=task)

  env = observation_dictionary_to_array_wrapper.ObservationDictionaryToArrayWrapper(env)
  # env = trajectory_generator_wrapper_env.TrajectoryGeneratorWrapperEnv(env)

  if mode == "test":
      curriculum_episode_length_start = curriculum_episode_length_end

  env = imitation_wrapper_env.ImitationWrapperEnv(env,
                                                  episode_length_start=curriculum_episode_length_start,
                                                  episode_length_end=curriculum_episode_length_end,
                                                  curriculum_steps=30000000,
                                                  num_parallel_envs=num_parallel_envs)
  return env



