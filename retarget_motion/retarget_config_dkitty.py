import numpy as np

URDF_FILENAME = "../robot_descriptions/dkitty_description/urdf/dkitty.urdf"

REF_POS_SCALE = 0.6
INIT_POS = np.array([0, 0, 0.4])
INIT_ROT = np.array([0, 0, 0, 1.0])

SIM_TOE_JOINT_IDS = [
    20,  # left hand
    10,  # left foot
    40,  # right hand
    30,  # right foot
]

SIM_HIP_JOINT_IDS = [12, 2, 32, 22]

SIM_ROOT_OFFSET = np.array([0, 0, 0])

SIM_TOE_OFFSET_LOCAL = [
    np.array([0, -0.02, 0]),
    np.array([0, -0.02, 0]),
    np.array([0, 0.02, 0]),
    np.array([0, 0.02, 0])
]

DEFAULT_JOINT_POSE = np.array([0, -1.1, 2.3, 0, -1.1, 2.3, 0, -1.1, 2.3, 0, -1.1, 2.3])
JOINT_DAMPING = [0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01]

FORWARD_DIR_OFFSET = np.array([0, 0, 0])
