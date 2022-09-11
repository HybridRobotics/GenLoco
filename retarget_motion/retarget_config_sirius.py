import numpy as np
from motion_imitation.utilities import pose3d
from pybullet_utils  import transformations

URDF_FILENAME = "../robot_descriptions/sirius_dog_v6/urdf/sirius_dog_v6_toes.urdf"

REF_POS_SCALE = 1
INIT_POS = np.array([0, 0, 0])
INIT_ROT = np.array([0, 0, 0, 1.0])

SIM_TOE_JOINT_IDS = [
    7,  #LR
    15, #RR
    3,  #LF
    11, #RF
]
SIM_HIP_JOINT_IDS = [5, 13, 1, 9]
SIM_ROOT_OFFSET = np.array([0, 0, 0])
SIM_TOE_OFFSET_LOCAL = [
    np.array([-0.05, -0.05, 0.0]),
    np.array([0, -0.05, 0.0]),
    np.array([-0.05, 0.05, 0.0]),
    np.array([0, 0.05, 0.0])
]

DEFAULT_JOINT_POSE = np.array([0, 1.1, -2.3, 0, -1.1, 2.3, 0, 1.1, -2.3, 0, -1.1, 2.3])
JOINT_DAMPING = [0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01,
                 0.5, 0.05, 0.01]


FORWARD_DIR_OFFSET = np.array([0, 0, 0.025])
