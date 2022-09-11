
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)

import pybullet
import pybullet_data as pd
import retarget_config_anymal_c as config

p = pybullet
p.connect(p.GUI, options="--width=1920 --height=1080 --mp4=\"test.mp4\" --mp4fps=60")
p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)

pybullet.setAdditionalSearchPath(pd.getDataPath())
robot = pybullet.loadURDF(config.URDF_FILENAME, config.INIT_POS, config.INIT_ROT)
num_joints = pybullet.getNumJoints(robot)

for i in range(num_joints):
    joint_info = pybullet.getJointInfo(robot, i)
    joint_type = joint_info[2]
    print(joint_info[0], joint_info[1])
