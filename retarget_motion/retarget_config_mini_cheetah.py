import numpy as np

URDF_FILENAME = "../robot_descriptions/quadruped_ctrl/urdf/mini_cheetah/mini_cheetah.urdf.xacro"

REF_POS_SCALE = 0.825
INIT_POS = np.array([0, 0, 0])
INIT_ROT = np.array([0, 0, 0, 1.0])

SIM_TOE_JOINT_IDS = [
    7,  # left hand
    15,  # left foot
    3,  # right hand
    11,  # right foot
]

SIM_HIP_JOINT_IDS = [4, 12, 0, 8]

SIM_ROOT_OFFSET = np.array([0, 0, -0.06])
SIM_TOE_OFFSET_LOCAL = [
    np.array([0.0, 0.03, 0.0]),
    np.array([0.0, 0.03, 0.01]),
    np.array([0.0, -0.03, 0.0]),
    np.array([0.0, -0.03, 0.01])
]

DEFAULT_JOINT_POSE = np.array([0, -0.9, 1.8, 0, -0.9, 1.8, 0, -0.9, 1.8, 0, -0.9, 1.8])
JOINT_DAMPING = [0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01]


FORWARD_DIR_OFFSET = np.array([0, 0, 0.025])
