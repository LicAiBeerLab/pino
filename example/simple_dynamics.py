import pinocchio as pin
import numpy as np
import meshcat
from pinocchio.visualize import MeshcatVisualizer
from numpy.linalg import norm

import os
import sys

script_dir = os.path.dirname(__file__)
mymodule_dir = os.path.join(script_dir, "../utils")
sys.path.append(mymodule_dir)

from pinokla.loader_tools import completeRobotLoader
from pinokla.closed_loop_kinematics import closedLoopProximalMount


## Load robot

path_to_robot=os.getcwd() + "/robots/digit_like"
name_yaml='robot.yaml'
name_urdf='robot.urdf'


model,constraint_model,actuation_model,visual_model= completeRobotLoader(path_to_robot,name_urdf,name_yaml)
constraint_data = [c.createData() for c in constraint_model]
data = model.createData()


## Visualizer

viz = MeshcatVisualizer(model, visual_model, visual_model)
viz.viewer = meshcat.Visualizer().open()
viz.clean()
viz.loadViewerModel()


## Montable configuration
q=closedLoopProximalMount(model,data,constraint_model,constraint_data,actuation_model)
viz.display(q)

## free fall dynamics
pin.initConstraintDynamics(model, data, constraint_model)
DT=1e-4
N_it=10000
tauq=np.zeros(model.nv)
vq=np.zeros(model.nv)

accuracy=1e-8
mu_sim=1e-8
max_it=100
dyn_set = pin.ProximalSettings(accuracy, mu_sim, max_it)


for i in range(N_it):
    a = pin.constraintDynamics(model,data,q,vq,tauq,constraint_model,constraint_data,dyn_set)
    vq+=a*DT
    q = pin.integrate(model, q, vq * DT)
    viz.display(q)

## Check constraint 

err=np.sum([norm(pin.log(cd.c1Mc2).np[:cm.size()]) for (cd,cm) in zip(constraint_data,constraint_model)])
print(err)