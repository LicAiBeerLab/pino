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
from pinokla.closed_loop_jacobian import inverseConstraintKinematicsSpeed,dq_dqmot

## Load robot
 
path_to_robot=os.getcwd() + "/robots/kangaroo_like"
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


## articular speed to comply with the given foot speed : 
idfoot=model.getFrameId('bout_pied')
vfoot_world=np.array([1,0,0,0,0,0])
vfoot_foot=data.oMf[idfoot].action@vfoot_world
vq,Jclosed=inverseConstraintKinematicsSpeed(model,data,constraint_model,constraint_data,actuation_model,q,idfoot,vfoot_foot)

DT=1e-3
for i in range (100):
    vq,Jclosed=inverseConstraintKinematicsSpeed(model,data,constraint_model,constraint_data,actuation_model,q,idfoot,vfoot_foot)
    q = pin.integrate(model, q, vq * DT)
    viz.display(q)


