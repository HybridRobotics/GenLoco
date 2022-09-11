import numpy as np

URDF_FILENAME = "../robot_descriptions/anymal_c_simple_description/urdf/anymal.urdf"

REF_POS_SCALE = 1.2
INIT_POS = np.array([0, 0, 0.4])
INIT_ROT = np.array([0, 0, 0, 1.0])

SIM_TOE_JOINT_IDS = [
    45,  # left hand
    65,  # left foot
    55,  # right hand
    75,  # right foot
]

SIM_HIP_JOINT_IDS = [40, 60, 50, 70] # using HFE flexors instead of HAA abductors

SIM_ROOT_OFFSET = np.array([0, 0, -0.06])

SIM_TOE_OFFSET_LOCAL = [
    np.array([-0.1, 0, 0]),
    np.array([0, 0, 0]),
    np.array([-0.1, 0, 0]),
    np.array([0, 0, 0])
]

DEFAULT_JOINT_POSE = np.array([0, 0.67, -1.25, 0, 0.67, -1.25, 0, -0.67, 1.25, 0, -0.67, 1.25])

JOINT_DAMPING = [0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01,
                 0.1, 0.05, 0.01]

FORWARD_DIR_OFFSET = np.array([0, 0, 0])
