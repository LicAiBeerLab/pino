from copy import deepcopy
from dataclasses import dataclass
import time
import odio_urdf
from pinokla.closed_loop_kinematics import ForwardK, ForwardK1, closedLoopProximalMount
from pinokla.default_traj import get_simple_spline, convert_x_y_to_6d_traj, simple_traj_derivative
from pinokla.loader_tools import completeRobotLoader, completeRobotLoaderFromStr
from pinokla.calc_criterion import calc_force_ell_along_trj_trans, kinematic_simulation, search_workspace, set_end_effector, kinematic_test
from pinocchio.visualize import MeshcatVisualizer
import meshcat
import os
import pinocchio as pin
import numpy as np
from pinocchio.robot_wrapper import RobotWrapper
from itertools import product
import matplotlib.pyplot as plt
import random
import os
import sys

from create_robot_example import (
    link1_auto,
    link2_auto,
    base_link,
    link3_auto,
    link4_auto,
    joint1,
    joint2,
    joint3,
    joint4,
    joint5,
    joint6,
    link5_psedo_auto,
    link6_psedo_auto,
    joint_description,
    loop_description,
)
from pinokla.robot_utils import freezeJoints

my_robot = odio_urdf.Robot(
    "basic_robot",
    link1_auto,
    link2_auto,
    base_link,
    link3_auto,
    link4_auto,
    joint1,
    joint2,
    joint3,
    joint4,
    joint5,
    joint6,
    link6_psedo_auto,
    link5_psedo_auto,
)

with open("basic_robot.urdf", "w") as file:
    file.write(str(my_robot))

model, constraint_models, actuation_model, visual_mode = completeRobotLoaderFromStr(
    str(my_robot),
    joint_description=joint_description,
    loop_description=loop_description,
)

constraint_data = [c.createData() for c in constraint_models]
data = model.createData()

q0 = closedLoopProximalMount(
    model,
    data,
    constraint_models,
    constraint_data,
    actuation_model,
    np.array([np.pi / 2, 0, 0, -np.pi / 2, 0], dtype=np.float64),
    max_it=10,
)

# q3, error = ForwardK(
#     model,
#     constraint_models,
#     actuation_model,
#     np.array([np.pi / 2.5, 0, 0, -np.pi / 2, 0], dtype=np.float64),
#     100,
# )

viz = MeshcatVisualizer(model, visual_mode, visual_mode)
idpied = model.getFrameId("link6_psedo")
viz.viewer = meshcat.Visualizer().open()
viz.clean()
viz.loadViewerModel()

# pin.framesForwardKinematics(model, data, q0)
# pin.computeAllTerms(
#     model,
#     data,
#     np.array([np.pi / 2, 0, 0, -np.pi / 2, 0], dtype=np.float64),
#     np.array([0, 0, 0, 0, 0], dtype=np.float64),
# )

EFFECTOR_NAME = 'link5_psedo'
BASE_FRAME = 'base_link'
# q_space_mot_1 = np.linspace(-np.pi / 2, np.pi / 2, 100)
# q_space_mot_2 = np.linspace(-np.pi / 2 , np.pi / 2, 50)
# q_mot_double_space = list(product(q_space_mot_1, q_space_mot_2))

# c = 0
# q_start = pin.neutral(model)
# workspace = np.empty((len(q_mot_double_space), 3))
# for q_sample in q_mot_double_space:

#     q_dict_mot = zip(actuation_model.idqmot, q_sample)
#     for key, value in q_dict_mot:
#         q_start[key] = value
#     q3, error = ForwardK(
#         model,
#         constraint_models,
#         actuation_model,
#         q_start,
#         150,
#     )

#     if error < 1e-11:
#         # viz.display(q3)
#         # time.sleep(0.01)
#         q_start = q3
#         pin.framesForwardKinematics(model, data, q3)
#         id_effector = model.getFrameId(EFFECTOR_NAME)
#         id_base = model.getFrameId(BASE_FRAME)
#         effector_pos = data.oMf[id_effector].translation
#         base_pos = data.oMf[id_base].translation
#         transformed_pos = effector_pos - base_pos
#         if transformed_pos[1] > 0:
#             workspace[c] = transformed_pos
#             c+= 1

# plt.scatter(list([x[0]for x in workspace]),  list([x[1]for x in workspace]))
# plt.show()
# natural_start = [0.67397308, 0.99271336, 0., 0, 0, 0]
x_traj, y_traj = get_simple_spline()
traj_6d = convert_x_y_to_6d_traj(x_traj, y_traj)
traj_6d_v = simple_traj_derivative(traj_6d, 0.0001)
q_start_traj = set_end_effector(model, data, constraint_models,
                                constraint_data, actuation_model, traj_6d[0],
                                BASE_FRAME, EFFECTOR_NAME, q0, viz=viz)

# q_space_mot_1 = np.linspace(-np.pi , np.pi , 50)
# q_space_mot_2 = np.linspace(-np.pi  , np.pi , 50)
# q_mot_double_space = list(product(q_space_mot_1, q_space_mot_2))
# workspace_xyz, available_q = search_workspace(model, data, EFFECTOR_NAME, BASE_FRAME, np.array(q_mot_double_space), actuation_model, constraint_models)
# traj_M, traj_J_closed, traj_dq = kinematic_simulation(model, data, actuation_model, constraint_models, constraint_data, EFFECTOR_NAME, BASE_FRAME, available_q, False)
# traj_force_cap = calc_force_ell_along_trj_trans(traj_J_closed)

# plt.scatter(res_traj[:,0],  res_traj[:,1])
# plt.scatter(x_traj,  y_traj)
# plt.show()
pass
