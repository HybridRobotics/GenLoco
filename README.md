# GenLoco: Generalized Locomotion Controllers for Quadrupedal Robots

Official codebase for [GenLoco: Generalized Locomotion Controllers for Quadrupedal Robots](http://arxiv.org/abs/2209.05309), containing code for training on randomized robotic morphologies to imitate reference motions as well as pre-trained policies and code to deploy these on simulated or real-world robots.

<p align="center">
<img src="https://github.com/HybridRobotics/GenLoco/blob/main/motion_imitation/data/genloco.gif" width="80%" height="80%"/>
</p>

We demonstrate zero-shot transfer for locomotion control of the pacing and spinning gaits for the following 10 robots, using one single policy for each motion ([video](https://youtu.be/5QUs32MjNu4)). However, you can also test the GenLoco models on your own robots (see the `Adding New Robots` section)!
- A1
- Go1
- Laikago
- Aliengo
- Sirius
- Mini Cheetah
- ANYmal-B
- ANYmal-C
- Spot
- SpotMicro

## Getting Started

We use this repository with Python 3.7 or Python 3.8 on Ubuntu, MacOS and Windows.

- Install MPC extension (Optional) `python3 setup.py install --user`

Install dependencies:

- Install MPI: `sudo apt install libopenmpi-dev`
- Install requirements: `pip3 install -r requirements.txt`
## Training Models in Simulation

To train a policy using randomized robotic morphologies, run the following command:
```bash
python3 motion_imitation/run.py --mode train --randomized_robot --phase_only --int_save_freq 10000000 --timesteps_per_actorbatch 8192 --optim_batchsize 512 --visualize
```
- `--int_save_freq` specifies the frequency for saving intermediate policies every n policy steps.
- `--visualize` enables visualization, and rendering can be disabled by removing the flag.

- the trained model and logs will be written to `output/`.

For parallel training with MPI run:
```bash
mpiexec -n 16 --use-hwthread-cpus python3 motion_imitation/run.py --mode train --randomized_robot --phase_only --int_save_freq 10000000 --timesteps_per_actorbatch 8192 --optim_batchsize 512
```
- `-n` is the number of parallel.

## Testing Models in Simulation

To test a trained model, run the following command
```bash
python3 motion_imitation/run.py --mode test --model_file motion_imitation/data/policies/morphology_generator_pace_model.zip --robot ${robot_name} --phase_only --visualize
```
- `--model_file` specifies the `.zip` file that contains the trained model. Pretrained GenLoco models using morphology generation are available in `motion_imitation/data/policies/`.
- `--robot` specifies the robot to deploy the model on. In this codebase we include support for: `laikago`, `a1`, `anymal_b`, `anymal_c`, `siriusmid_belt`, `mini_cheetah`, `go1`, `aliengo`, `spot`, `spotmicro`. See the following section for how to add new robots.


## Adding New Robots
You can also add your own robot into this pipeline for zero-shot pacing and spinning using our provided RL policies.
### Step 1: Add urdf files and meshes
Add urdf and meshes into `robot_descriptions`
### Step 2: Add robot config file
- Build a file in `motion_imitation/robots` using `a1.py` as a template.
- Be sure to define the constant robot-specific parameters such as 
    - `MOTOR_NAMES`: joint names marked in your urdf file in a fixed order: [RF,LF,RR,LR]
    - `INIT_RACK_POSITION`: [x,y,z] position that the robot is initialized at in rack mode
    - `INIT_POSITION`: [x,y,z] position that the robot is initialized at
    - `JOINT_DIRECTIONS`: directions of the motors
    - `PD_GAINS`: suitable PD gains for the robot
    - `INIT_MOTOR_ANGLES`: an array of size 12 to regulate motor angles while initializing. 
    - `NAME_PATTERN`: mark the links/joints by regular expression
    - `URDF_FILENAME`: your urdf file path
    - `ACTION_CONFIG`: set bounds for different joints (note: should not be used in position mode)
    - Specify a new class name.

### Step 3: Add robot into run.py
Add your new class into `motion_imitation/run.py`, line 56:
```bash
    "spotmicro":spotmicro.SpotMicro,
    "real_a1":a1_robot_real.A1Robot,
    # add new robot class here
}
```
Replace `# add new robot class here` with 
`"your_new_robot": your_robot_class`

Then, run the following code to test deployment:
```bash
python3 motion_imitation/run.py --mode test --model_file motion_imitation/data/policies/morphology_generator_pace_model.zip --robot ${your_new_robot} --phase_only --visualize
```
## An Exploration on a Different Morphological Structure
We also explore randomizing the joint direction. This model can be be found at `motion_imitation/data/policies/morphology_generator_inverse.zip`

<p align="center">
<img src="https://github.com/HybridRobotics/GenLoco/blob/main/motion_imitation/data/inversed.gif" width="30%" height="30%"/>
</p>

## Deploying the GenLoco policy on Real Robots
We demonstrate how to use our codebase for deployment of the GenLoco policy on real robots. The following instructions show how to deploy on the real A1.

The code in the folder `unitree_ros` needs to be placed in your ROS workspace.

The `unitree_ros/reinforce_controller` ROS package provides a reinforcement learning controller interface for locomotion of the A1.

### Dependencies

- ROS Melodic or ROS Kinetic
- [unitree_legged_sdk 3.2](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.2)
### Build
For ROS Melodic:
```bash
sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```
For ROS Kinetic:
```bash
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-position-controllers ros-kinetic-robot-controllers ros-kinetic-robot-state-publisher ros-kinetic-gazebo8-ros ros-kinetic-gazebo8-ros-control ros-kinetic-gazebo8-ros-pkgs ros-kinetic-gazebo8-ros-dev
```
Add configurations:
```bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=~/catkin_ws/devel/lib:${LD_LIBRARY_PATH}
export UNITREE_LEGGED_SDK_PATH=~/catkin_ws/src/unitree_ros/unitree_legged_sdk
#amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"
``` 
Build `unitree_legged_sdk` follow unitree_legged_sdk  v3.2 [README](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.2). 
- You can also export your own unitree_legged_sdk path.

Then use catkin_make to build:
```bash
cd ~/catkin_ws
catkin_make
```

### Deploying on the Real A1
1. Setup the ip for udp on both policy side and ros side.
    -    ROS side: in file `reinforce_controller/yaml/`

    -    Policy side: in file `motion_imitation/real_a1/a1_robot_real.py`

2. Start the rl controller: 

```bash 
roslaunch reinforce_controller rl_control_real.launch config_file:=${yaml_config_file_name}
```

3. Run the position_lcm node:
```bash 
rosrun reinforce_controller position_lcm
```

4. Start running on the policy side after the controller is ready:

```bash
 python motion_imitation/run.py --robot real_a1 --motion_file motion_imitation/data/motions/a1_pace.txt --mode test --model motion_imitation/data/policies/morphology_generator_pace_model.zip --phase_only
```

5. Choose to walk/stand/damp by typing on the keyboard.
    - mode `stand`: press `[s + /Enter]`preload → stand_wait → stand_on.
    - mode `walk`:  press `[w + /Enter]` stand_on → walking_on
    - mode `damp`:  press `[d + /Enter]` any → damp_on.

### Deploying on Gazebo
Delete the `CATKIN_IGNORE` file in `unitree_gazebo` package and rebuild.
