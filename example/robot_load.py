import pinocchio as pin
import numpy as np
import meshcat
from pinocchio.visualize import MeshcatVisualizer


import os
import sys

script_dir = os.path.dirname(__file__)
mymodule_dir = os.path.join(script_dir, "../utils")
sys.path.append(mymodule_dir)

from pinokla.loader_tools import *
from pinokla.closed_loop_kinematics import *
from pinokla.closed_loop_jacobian import *




path_to_robot=os.getcwd() + "/robots/cassie_like"
name_yaml='robot.yaml'
name_urdf='robot.urdf'


model,constraint_model,actuation_model,visual_model= completeRobotLoader(path_to_robot,name_urdf,name_yaml)
constraint_data = [c.createData() for c in constraint_model]
data = model.createData()



viz = MeshcatVisualizer(model, visual_model, visual_model)
viz.viewer = meshcat.Visualizer().open()
viz.clean()
viz.loadViewerModel()

q=closedLoopProximalMount(model,data,constraint_model,constraint_data,actuation_model)
viz.display(q)
pass





