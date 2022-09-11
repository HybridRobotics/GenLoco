import numpy as np
from numpy.random import uniform, normal

a1_base = np.array([0.1335, 0.097, 0.057])
base_delta = np.array([0.045, 0.03, 0.02])
base_delta = a1_base / 2

a1_base_mass = 4.713
a1_density = a1_base_mass / np.prod(a1_base)
density_delta = a1_density / 2

link_masses = np.array([0.001,
              0.696, 0.001, 1.013, 0.166, 0.06,
              0.696, 0.001, 1.013, 0.166, 0.06,
              0.696, 0.001, 1.013, 0.166, 0.06,
              0.696, 0.001, 1.013, 0.166, 0.06])
link_masses_delta = link_masses / 2

a1_cylinder = np.array([0.02, 0.22]) #radius, length
cyl_delta = np.array([0.0025, 0.025])
cyl_delta = a1_cylinder / 2

# base_delta *= 0
# density_delta *= 0
# link_masses_delta *= 0
# cyl_delta *= 0

def create_robot(p,
                 basePosition=[0, 0, 0],
                 baseOrientation=[0, 0, 0, 1],
                 delta_scale=1):

    base_delta_cur = base_delta * delta_scale
    density_delta_cur = density_delta * delta_scale
    link_masses_delta_cur = link_masses_delta * delta_scale
    cyl_delta_cur = cyl_delta * delta_scale

    #RANDOMIZATIONS

    randomized_scale = 1
    randomized_scale = uniform(0.8, 1.2)

    x_base, y_base, z_base = uniform(a1_base - base_delta_cur, a1_base + base_delta_cur) * randomized_scale

    lower_cylinder_radius, lower_cylinder_length = uniform(a1_cylinder - cyl_delta_cur, a1_cylinder + cyl_delta_cur) * randomized_scale
    # upper_cylinder_radius, upper_cylinder_length = uniform(a1_cylinder - cyl_delta_cur, a1_cylinder + cyl_delta_cur)
    upper_cylinder_radius = lower_cylinder_radius * uniform(0.75, 1.25)
    upper_cylinder_length = lower_cylinder_length * uniform(0.75, 1.25)

    linkMasses = uniform(link_masses - link_masses_delta_cur, link_masses + link_masses_delta_cur) * (randomized_scale**3)

    baseDensity = uniform(a1_density - density_delta_cur, a1_density + density_delta_cur)
    baseMass = baseDensity * x_base * y_base * z_base

    baseCollisionShapeIndex = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        radius=1,
        halfExtents=[x_base, y_base, z_base],  # 0.1335, 0.097, 0.057
        height=2,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, 0.0],
        collisionFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    baseVisualShapeIndex = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[x_base, y_base, z_base],  # 0.1335, 0.097, 0.057
        radius=1,
        length=10,
        fileName='robot_descriptions/a1_description/meshes/trunk.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # imu_joint
    imu_joint_collision = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        radius=1,
        halfExtents=[0.0005000000000000004, 0.0005000000000000004, 0.0005000000000000004],
        height=2,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, 0.0],
        collisionFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    imu_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[0.0005, 0.0005, 0.0005],
        radius=1,
        length=10,
        fileName='meshfile',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[0.800000011920929, 0.0, 0.0, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # FR_hip_joint
    FR_hip_joint_collision = -1

    FR_hip_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        halfExtents=[3.5, 4.0, 4.5],
        radius=1,
        length=10,
        fileName='robot_descriptions/a1_description/meshes/hip.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[-1.0, -0.0, 0.0, 1.0353401823502196e-13])

    # FR_hip_fixed
    FR_hip_fixed_collision = -1

    FR_hip_fixed_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[3.5, 4.0, 4.5],
        radius=0.041,
        length=0.032,
        fileName='meshfile',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 1.0, 1.0, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.7071067811848163, 0.0, 0.0, 0.7071067811882787])

    # FR_upper_joint
    FR_upper_joint_collision = p.createCollisionShape(
        shapeType=p.GEOM_CYLINDER,
        radius=upper_cylinder_radius,
        halfExtents=[0.1, 0.01225, 0.017],
        height=upper_cylinder_length/5,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, -upper_cylinder_length/2],
        collisionFrameOrientation=[0.0, 0.7071067811865475, -0.0, 0.7071067811865476])

    FR_upper_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[0.1, 0.01225, 0.017],
        radius=upper_cylinder_radius,
        length=upper_cylinder_length,
        fileName='robot_descriptions/a1_description/meshes/thigh_mirror.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, -upper_cylinder_length/2],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # FR_lower_joint
    FR_lower_joint_collision = p.createCollisionShape(
        shapeType=p.GEOM_CYLINDER,
        radius=lower_cylinder_radius,
        halfExtents=[0.1, 0.008, 0.008],
        height=lower_cylinder_length/5,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, -lower_cylinder_length/2],
        collisionFrameOrientation=[0.0, 0.7071067811865475, -0.0, 0.7071067811865476])

    FR_lower_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[0.1, 0.008, 0.008],
        radius=lower_cylinder_radius,
        length=lower_cylinder_length,
        fileName='robot_descriptions/a1_description/meshes/calf.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, -lower_cylinder_length/2],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # FR_toe_fixed
    FR_toe_fixed_collision = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=1.5*lower_cylinder_radius,
        halfExtents=[3.5, 4.0, 4.5],
        height=2,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, 0.0],
        collisionFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    FR_toe_fixed_visual = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        halfExtents=[3.5, 4.0, 4.5],
        radius=1.5*lower_cylinder_radius,
        length=10,
        fileName='meshfile',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # FL_hip_joint
    FL_hip_joint_collision = -1

    FL_hip_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        halfExtents=[3.5, 4.0, 4.5],
        radius=1,
        length=10,
        fileName='robot_descriptions/a1_description/meshes/hip.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # FL_hip_fixed
    FL_hip_fixed_collision = -1

    FL_hip_fixed_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[3.5, 4.0, 4.5],
        radius=0.041,
        length=0.032,
        fileName='meshfile',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 1.0, 1.0, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.7071067811848163, 0.0, 0.0, 0.7071067811882787])

    # FL_upper_joint
    FL_upper_joint_collision = p.createCollisionShape(
        shapeType=p.GEOM_CYLINDER,
        radius=upper_cylinder_radius,
        halfExtents=[0.1, 0.01225, 0.017],
        height=upper_cylinder_length/5,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, -upper_cylinder_length/2],
        collisionFrameOrientation=[0.0, 0.7071067811865475, -0.0, 0.7071067811865476])

    FL_upper_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[0.1, 0.01225, 0.017],
        radius=upper_cylinder_radius,
        length=upper_cylinder_length,
        fileName='robot_descriptions/a1_description/meshes/thigh.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, -upper_cylinder_length/2],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # FL_lower_joint
    FL_lower_joint_collision = p.createCollisionShape(
        shapeType=p.GEOM_CYLINDER,
        radius=lower_cylinder_radius,
        halfExtents=[0.1, 0.008, 0.008],
        height=lower_cylinder_length/5,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, -lower_cylinder_length/2],
        collisionFrameOrientation=[0.0, 0.7071067811865475, -0.0, 0.7071067811865476])

    FL_lower_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[0.1, 0.008, 0.008],
        radius=lower_cylinder_radius,
        length=lower_cylinder_length,
        fileName='robot_descriptions/a1_description/meshes/calf.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, -lower_cylinder_length/2],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # FL_toe_fixed
    FL_toe_fixed_collision = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=1.5*lower_cylinder_radius,
        halfExtents=[3.5, 4.0, 4.5],
        height=2,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, 0.0],
        collisionFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    FL_toe_fixed_visual = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        halfExtents=[3.5, 4.0, 4.5],
        radius=1.5*lower_cylinder_radius,
        length=10,
        fileName='meshfile',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # RR_hip_joint
    RR_hip_joint_collision = -1

    RR_hip_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        halfExtents=[3.5, 4.0, 4.5],
        radius=1,
        length=10,
        fileName='robot_descriptions/a1_description/meshes/hip.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[1.0341155355510723e-13, 1.0341155355510721e-13, 1.0, 6.123233994667371e-17])

    # RR_hip_fixed
    RR_hip_fixed_collision = -1

    RR_hip_fixed_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[3.5, 4.0, 4.5],
        radius=0.041,
        length=0.032,
        fileName='meshfile',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 1.0, 1.0, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.7071067811848163, 0.0, 0.0, 0.7071067811882787])

    # RR_upper_joint
    RR_upper_joint_collision = p.createCollisionShape(
        shapeType=p.GEOM_CYLINDER,
        radius=upper_cylinder_radius,
        halfExtents=[0.1, 0.01225, 0.017],
        height=upper_cylinder_length/5,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, -upper_cylinder_length/2],
        collisionFrameOrientation=[0.0, 0.7071067811865475, -0.0, 0.7071067811865476])

    RR_upper_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[0.1, 0.01225, 0.017],
        radius=upper_cylinder_radius,
        length=upper_cylinder_length,
        fileName='robot_descriptions/a1_description/meshes/thigh_mirror.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, -upper_cylinder_length/2],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # RR_lower_joint
    RR_lower_joint_collision = p.createCollisionShape(
        shapeType=p.GEOM_CYLINDER,
        radius=lower_cylinder_radius,
        halfExtents=[0.1, 0.008, 0.008],
        height=lower_cylinder_length/5,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, -lower_cylinder_length/2],
        collisionFrameOrientation=[0.0, 0.7071067811865475, -0.0, 0.7071067811865476])

    RR_lower_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[0.1, 0.008, 0.008],
        radius=lower_cylinder_radius,
        length=lower_cylinder_length,
        fileName='robot_descriptions/a1_description/meshes/calf.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, -lower_cylinder_length/2],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # RR_toe_fixed
    RR_toe_fixed_collision = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=1.5*lower_cylinder_radius,
        halfExtents=[3.5, 4.0, 4.5],
        height=2,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, 0.0],
        collisionFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    RR_toe_fixed_visual = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        halfExtents=[3.5, 4.0, 4.5],
        radius=1.5*lower_cylinder_radius,
        length=10,
        fileName='meshfile',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # RL_hip_joint
    RL_hip_joint_collision = -1

    RL_hip_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        halfExtents=[3.5, 4.0, 4.5],
        radius=1,
        length=10,
        fileName='robot_descriptions/a1_description/meshes/hip.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[6.1232339957374e-17, 1.0, 6.1232339957374e-17, -1.0341155355510722e-13])

    # RL_hip_fixed
    RL_hip_fixed_collision = -1

    RL_hip_fixed_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[3.5, 4.0, 4.5],
        radius=0.041,
        length=0.032,
        fileName='meshfile',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 1.0, 1.0, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.7071067811848163, 0.0, 0.0, 0.7071067811882787])

    # RL_upper_joint
    RL_upper_joint_collision = p.createCollisionShape(
        shapeType=p.GEOM_CYLINDER,
        radius=upper_cylinder_radius,
        halfExtents=[0.1, 0.01225, 0.017],
        height=upper_cylinder_length/5,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, -upper_cylinder_length/2],
        collisionFrameOrientation=[0.0, 0.7071067811865475, -0.0, 0.7071067811865476])

    RL_upper_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[0.1, 0.01225, 0.017],
        radius=upper_cylinder_radius,
        length=upper_cylinder_length,
        fileName='robot_descriptions/a1_description/meshes/thigh.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, -upper_cylinder_length/2],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # RL_lower_joint
    RL_lower_joint_collision = p.createCollisionShape(
        shapeType=p.GEOM_CYLINDER,
        radius=lower_cylinder_radius,
        halfExtents=[0.1, 0.008, 0.008],
        height=lower_cylinder_length/5,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, -lower_cylinder_length/2],
        collisionFrameOrientation=[0.0, 0.7071067811865475, -0.0, 0.7071067811865476])

    RL_lower_joint_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        halfExtents=[0.1, 0.008, 0.008],
        radius=lower_cylinder_radius,
        length=lower_cylinder_length,
        fileName='robot_descriptions/a1_description/meshes/calf.obj',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, -lower_cylinder_length/2],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # RL_toe_fixed
    RL_toe_fixed_collision = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=1.5*lower_cylinder_radius,
        halfExtents=[3.5, 4.0, 4.5],
        height=2,
        fileName='meshfile',
        meshScale=[1, 1, 1],
        collisionFramePosition=[0.0, 0.0, 0.0],
        collisionFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    RL_toe_fixed_visual = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        halfExtents=[3.5, 4.0, 4.5],
        radius=1.5*lower_cylinder_radius,
        length=10,
        fileName='meshfile',
        meshScale=[1.0, 1.0, 1.0],
        rgbaColor=[1.0, 0.42352941632270813, 0.03921568766236305, 1.0],
        visualFramePosition=[0.0, 0.0, 0.0],
        visualFrameOrientation=[0.0, 0.0, 0.0, 1.0])

    # Format of value below:
    # imu_joint
    # FR_hip_joint, FR_hip_fixed, FR_upper_joint, FR_lower_joint, FR_toe_fixed
    # FL_hip_joint, FL_hip_fixed, FL_upper_joint, FL_lower_joint, FL_toe_fixed
    # RR_hip_joint, RR_hip_fixed, RR_upper_joint, RR_lower_joint, RR_toe_fixed
    # RL_hip_joint, RL_hip_fixed, RL_upper_joint, RL_lower_joint, RL_toe_fixed

    linkCollisionShapeIndices = [imu_joint_collision,
                                 FR_hip_joint_collision, FR_hip_fixed_collision, FR_upper_joint_collision,
                                 FR_lower_joint_collision, FR_toe_fixed_collision,
                                 FL_hip_joint_collision, FL_hip_fixed_collision, FL_upper_joint_collision,
                                 FL_lower_joint_collision, FL_toe_fixed_collision,
                                 RR_hip_joint_collision, RR_hip_fixed_collision, RR_upper_joint_collision,
                                 RR_lower_joint_collision, RR_toe_fixed_collision,
                                 RL_hip_joint_collision, RL_hip_fixed_collision, RL_upper_joint_collision,
                                 RL_lower_joint_collision, RL_toe_fixed_collision]

    linkVisualShapeIndices = [imu_joint_visual,
                              FR_hip_joint_visual, FR_hip_fixed_visual, FR_upper_joint_visual,
                              FR_lower_joint_visual, FR_toe_fixed_visual,
                              FL_hip_joint_visual, FL_hip_fixed_visual, FL_upper_joint_visual,
                              FL_lower_joint_visual, FL_toe_fixed_visual,
                              RR_hip_joint_visual, RR_hip_fixed_visual, RR_upper_joint_visual,
                              RR_lower_joint_visual, RR_toe_fixed_visual,
                              RL_hip_joint_visual, RL_hip_fixed_visual, RL_upper_joint_visual,
                              RL_lower_joint_visual, RL_toe_fixed_visual]

    # 0.18299999833106995, 0.04699999839067459 - randomize x and y
    # uniform(0.05, 0.25, size=1)
    hip_joint_link_pos_1, hip_joint_link_pos_2 = 0.18299999833106995, 0.04699999839067459
    # hip_joint_link_pos_1 += (x_base - a1_base[0]) / 2
    # hip_joint_link_pos_2 += (y_base - a1_base[1]) / 2
    hip_joint_link_pos_1 *= x_base / a1_base[0] * uniform(0.9, 1.1)
    hip_joint_link_pos_2 *= y_base / a1_base[1] * uniform(0.9, 1.1)

    # 0.08100000023841858 - randomize y
    hip_fixed_link_pos = 0.08100000023841858 * uniform(0.9, 1.1)
    # hip_fixed_link_pos += (y_base - a1_base[1]) / 2
    # hip_fixed_link_pos *= y_base / a1_base[1] * y_r

    # 0.08505000174045563- randomize y
    # upper_joint_link_pos = upper_cylinder_length / 2
    # 0.20000000298023224 - randomize z
    # lower_joint_link_pos = lower_cylinder_length / 2
    # lower_joint_link_pos *= upper_cylinder_length / a1_cylinder[1]
    # lower_joint_link_pos += (upper_cylinder_length - a1_cylinder[1]) / 2
    # 0.19999998807907104 - randomize z
    # toe_fixed_link_pos = 0.19999998807907104 / 2
    # toe_fixed_link_pos *= lower_cylinder_length / a1_cylinder[1]
    # toe_fixed_link_pos += (lower_cylinder_length - a1_cylinder[1]) / 2

    linkPositions = [(0.0, 0.0, 0.0),
                     (hip_joint_link_pos_1, -hip_joint_link_pos_2, 0.0),
                     (0.0, -hip_fixed_link_pos, 0.0),
                     (0.0, -hip_fixed_link_pos, 0.0),
                     (0.0, 0.0, -upper_cylinder_length),
                     (0.0, 0.0, -lower_cylinder_length),
                     (hip_joint_link_pos_1, hip_joint_link_pos_2, 0.0),
                     (0.0, hip_fixed_link_pos, 0.0),
                     (0.0, hip_fixed_link_pos, 0.0),
                     (0.0, 0.0, -upper_cylinder_length),
                     (0.0, 0.0, -lower_cylinder_length),
                     (-hip_joint_link_pos_1, -hip_joint_link_pos_2, 0.0),
                     (0.0, -hip_fixed_link_pos, 0.0),
                     (0.0, -hip_fixed_link_pos, 0.0),
                     (0.0, 0.0, -upper_cylinder_length),
                     (0.0, 0.0, -lower_cylinder_length),
                     (-hip_joint_link_pos_1, hip_joint_link_pos_2, 0.0),
                     (0.0, hip_fixed_link_pos, 0.0),
                     (0.0, hip_fixed_link_pos, 0.0),
                     (0.0, 0.0, -upper_cylinder_length),
                     (0.0, 0.0, -lower_cylinder_length)]


    # linkPositions = [(0.0, 0.0, 0.0),
    #                     (0.18299999833106995, -0.04699999839067459, 0.0),
    #                     (0, -0.08100000023841858, 0.0),
    #                     (-0.08505000174045563, -0.08505000174045563, 0.0), #
    #                     (0.1, 0.0, -0.1),
    #                     (0.0, 0.0, -0.11),
    #                     (0.18299999833106995, 0.04700000211596489, 0.0),
    #                     (0.0, 0.08100000023841858, 0.0),
    #                     (-0.08505000174045563, 0.08505000174045563, 0.0),
    #                     (0.1, 0.0, -0.1),
    #                     (0.0, 0.0, -0.11),
    #                     (-0.18299999833106995, -0.04699999839067459, 0.0),
    #                     (0.0, -0.08100000023841858, 0.0),
    #                     (-0.08505000174045563, -0.08505000174045563, 0.0),
    #                     (0.1, 0.0, -0.1),
    #                     (0.0, 0.0, -0.11),
    #                     (-0.18299999833106995, 0.04700000211596489, 0.0),
    #                     (0.0, 0.08100000023841858, 0.0),
    #                     (-0.08505000174045563, 0.08505000174045563, 0.0),
    #                     (0.1, 0.0, -0.1),
    #                     (0.0, 0.0, -0.11)]

    # linkPositions = [(0.0, 0.0, 0.0),
    #                  (hip_joint_link_pos_1, -hip_joint_link_pos_2, 0.0),
    #                  (0.0, -hip_fixed_link_pos, 0.0),
    #                  (-upper_joint_link_pos, -upper_joint_link_pos, -0.0),
    #                  (lower_joint_link_pos - lower_cylinder_radius / 2, 0.0, -lower_joint_link_pos - lower_joint_z_adj),
    #                  (0.0, 0.0, -toe_fixed_link_pos),
    #                  (hip_joint_link_pos_1, hip_joint_link_pos_2, 0.0), (0.0, hip_fixed_link_pos, 0.0),
    #                  (-upper_joint_link_pos, upper_joint_link_pos, 0.0),
    #                  (lower_joint_link_pos - lower_cylinder_radius / 2, 0.0, -lower_joint_link_pos - lower_joint_z_adj),
    #                  (0.0, 0.0, -toe_fixed_link_pos),
    #                  (-hip_joint_link_pos_1, -hip_joint_link_pos_2, 0.0), (0.0, -hip_fixed_link_pos, 0.0),
    #                  (-upper_joint_link_pos, -upper_joint_link_pos, 0.0),
    #                  (lower_joint_link_pos - lower_cylinder_radius / 2, 0.0, -lower_joint_link_pos - lower_joint_z_adj),
    #                  (0.0, 0.0, -toe_fixed_link_pos),
    #                  (-hip_joint_link_pos_1, hip_joint_link_pos_2, 0.0), (0.0, hip_fixed_link_pos, 0.0),
    #                  (-upper_joint_link_pos, upper_joint_link_pos, 0.0),
    #                  (lower_joint_link_pos - lower_cylinder_radius / 2, 0.0, -lower_joint_link_pos - lower_joint_z_adj),
    #                  (0.0, 0.0, -toe_fixed_link_pos)]

    # linkPositions = [(0.0, 0.0, 0.0),
    #                  (0.18299999833106995, -0.04699999839067459, 0.0), (0.0, -0.08100000023841858, 0.0),
    #                  (0.0, -0.08505000174045563, 0.0), (0.0, 0.0, -0.20000000298023224),
    #                  (0.0, 0.0, -0.19999998807907104),
    #                  (0.18299999833106995, 0.04700000211596489, 0.0), (0.0, 0.08100000023841858, 0.0),
    #                  (0.0, 0.08505000174045563, 0.0), (0.0, 0.0, -0.20000000298023224),
    #                  (0.0, 0.0, -0.19999998807907104),
    #                  (-0.18299999833106995, -0.04699999839067459, 0.0), (0.0, -0.08100000023841858, 0.0),
    #                  (0.0, -0.08505000174045563, 0.0), (0.0, 0.0, -0.20000000298023224),
    #                  (0.0, 0.0, -0.19999998807907104),
    #                  (-0.18299999833106995, 0.04700000211596489, 0.0), (0.0, 0.08100000023841858, 0.0),
    #                  (0.0, 0.08505000174045563, 0.0), (0.0, 0.0, -0.20000000298023224),
    #                  (0.0, 0.0, -0.19999998807907104)]

    linkOrientations = [(0.0, 0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 1.0)]

    linkInertialFramePositions = [(0.0, 0.0, 0.0),
                                  (-0.003311, -0.000635, 3.1e-05), (0.0, 0.0, 0.0),
                                  (-0.003237, 0.022327, -0.027326), (0.006435, 0.0, -0.107388), (0.0, 0.0, 0.0),
                                  (-0.003311, 0.000635, 3.1e-05), (0.0, 0.0, 0.0),
                                  (-0.003237, -0.022327, -0.027326), (0.006435, 0.0, -0.107388), (0.0, 0.0, 0.0),
                                  (0.003311, -0.000635, 3.1e-05), (0.0, 0.0, 0.0), (-0.003237, 0.022327, -0.027326),
                                  (0.006435, 0.0, -0.107388), (0.0, 0.0, 0.0),
                                  (0.003311, 0.000635, 3.1e-05), (0.0, 0.0, 0.0), (-0.003237, -0.022327, -0.027326),
                                  (0.006435, 0.0, -0.107388), (0.0, 0.0, 0.0)]

    linkInertialFrameOrientations = [(0.0, 0.0, 0.0, 1.0),
                                     (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                                     (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                                     (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                                     (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                                     (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                                     (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                                     (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
                                     (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)]

    linkParentIndices = [0,
                         0, 2, 2, 4, 5,
                         0, 7, 7, 9, 10,
                         0, 12, 12, 14, 15,
                         0, 17, 17, 19, 20]
    linkJointTypes = [4,
                      0, 4, 0, 0, 4,
                      0, 4, 0, 0, 4,
                      0, 4, 0, 0, 4,
                      0, 4, 0, 0, 4]

    linkJointAxis = [(0.0, 0.0, 0.0),
                     (1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.0),
                     (1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.0),
                     (1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.0),
                     (1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.0)]

    obUid = p.createMultiBody(baseMass,
                              baseCollisionShapeIndex=baseCollisionShapeIndex,
                              baseVisualShapeIndex=baseVisualShapeIndex,
                              basePosition=basePosition,
                              baseOrientation=baseOrientation,
                              baseInertialFramePosition=[0.012731, 0.002186, 0.000515],
                              baseInertialFrameOrientation=p.getQuaternionFromEuler([0.0, -0.0, 0.0]),
                              linkMasses=linkMasses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=linkParentIndices,
                              linkJointTypes=linkJointTypes,
                              linkJointAxis=linkJointAxis)

    return obUid
