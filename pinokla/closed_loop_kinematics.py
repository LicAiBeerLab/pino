"""
-*- coding: utf-8 -*-
Ludovic DE MATTEIS & Virgile BATTO, April 2023

Tools to compute the forwark and inverse kinematics of a robot with  closed loop 

"""

from copy import deepcopy
import pinocchio as pin
import numpy as np
from pinokla.robot_utils import freezeJoints, freezeJointsWithoutVis
from pinocchio.visualize import GepettoVisualizer
from pinocchio.visualize import MeshcatVisualizer
import meshcat


def closedLoopInverseKinematicsProximal(
    rmodel,
    rdata,
    rconstraint_model,
    rconstraint_data,
    target_pos,
    ideff,
    q_start = None,
    onlytranslation=False,
    max_it=500,
    eps=1e-11,
    rho=1e-10,
    mu=1e-3,
):
    """
    q=inverseGeomProximalSolver(rmodel,rdata,rconstraint_model,rconstraint_data,idframe,pos,only_translation=False,max_it=100,eps=1e-12,rho=1e-10,mu=1e-4)

    Perform inverse kinematics with a proximal solver.

    Args:
        rmodel (pinocchio.Model): Pinocchio model.
        rdata (pinocchio.Data): Pinocchio data.
        rconstraint_model (list): List of constraint models.
        rconstraint_data (list): List of constraint data.
        target_pos (np.array): Target position.
        name_eff (str, optional): Name of the frame. Defaults to "effecteur".
        onlytranslation (bool, optional): Only consider translation. Defaults to False.
        max_it (int, optional): Maximum number of iterations. Defaults to 100.
        eps (float, optional): Convergence threshold for primal and dual feasibility. Defaults to 1e-12.
        rho (float, optional): Scaling factor for the identity matrix. Defaults to 1e-10.
        mu (float, optional): Penalty parameter. Defaults to 1e-4.

    Returns:
        np.array: Joint positions that achieve the desired target position.

    raw here (L84-126):https://gitlab.inria.fr/jucarpen/pinocchio/-/blob/pinocchio-3x/examples/simulation-closed-kinematic-chains.py
    """
    
    model = pin.Model(rmodel)
    constraint_model = [pin.RigidConstraintModel(x) for x in  rconstraint_model]
    # add a contact constraint
    target_SE3 = pin.SE3.Identity()
    target_SE3.translation = np.array(target_pos[0:3])
    frame_constraint = model.frames[ideff]
    parent_joint = frame_constraint.parentJoint
    placement = frame_constraint.placement
    if onlytranslation:
        final_constraint = pin.RigidConstraintModel(pin.ContactType.CONTACT_3D,
                                                    model, parent_joint,
                                                    placement, 0, target_SE3,
                                                    pin.ReferenceFrame.LOCAL)
    else:
        final_constraint = pin.RigidConstraintModel(
            pin.ContactType.CONTACT_6D, model, parent_joint, placement,
            model.getJointId("universel"), target_pos,
            pin.ReferenceFrame.LOCAL)
    constraint_model.append(final_constraint)

    data = model.createData()
    constraint_data = [cm.createData() for cm in constraint_model]

    # proximal solver (black magic)
    if q_start is None:
        q = pin.neutral(model)
    else:
        q = q_start
    constraint_dim = 0
    for cm in constraint_model:
        constraint_dim += cm.size()
    is_reach = False
    y = np.ones((constraint_dim))
    data.M = np.eye(model.nv) * rho
    kkt_constraint = pin.ContactCholeskyDecomposition(model, constraint_model)
    primal_feas_array = np.zeros(max_it)
    q_array = np.zeros((max_it, len(q)))
    for k in range(max_it):
        pin.computeJointJacobians(model, data, q)
        kkt_constraint.compute(model, data, constraint_model, constraint_data,
                               mu)

        constraint_value = np.concatenate([
            (pin.log(cd.c1Mc2).np[:cm.size()])
            for (cd, cm) in zip(constraint_data, constraint_model)
        ])

        LJ = []
        for cm, cd in zip(constraint_model, constraint_data):
            Jc = pin.getConstraintJacobian(model, data, cm, cd)
            LJ.append(Jc)
        J = np.concatenate(LJ)

        primal_feas = np.linalg.norm(constraint_value, np.inf)
        primal_feas_array[k] = primal_feas
        q_array[k] = q
        dual_feas = np.linalg.norm(J.T.dot(constraint_value + y), np.inf)
        if primal_feas < eps and dual_feas < eps:
            is_reach = True
            break
            

        rhs = np.concatenate([-constraint_value - y * mu, np.zeros(model.nv)])

        dz = kkt_constraint.solve(rhs)
        dy = dz[:constraint_dim]
        dq = dz[constraint_dim:]

        alpha = 0.5
        q = pin.integrate(model, q, -alpha * dq)
        y -= alpha * (-dy + y)
    
    pin.framesForwardKinematics(model, data, q)
    id_frame = model.getFrameId("link5_psedo")
    pos_e = np.linalg.norm(data.oMf[id_frame].translation -
                        np.array(target_pos[0:3]))
    min_feas = primal_feas
    if not is_reach:
        for_sort = np.column_stack((primal_feas_array, q_array))
        key_sort = lambda x: x[0]
        for_sort = sorted(for_sort, key=key_sort)
        finish_q = for_sort[0][1:]
        q = finish_q
        min_feas = for_sort[0][0]
        pin.framesForwardKinematics(model, data, q)
        pos_e = np.linalg.norm(data.oMf[id_frame].translation -
                    np.array(target_pos[0:3]))
    return q, min_feas, pos_e


def closedLoopProximalMount(
    model,
    data,
    constraint_model,
    constraint_data,
    actuation_model,
    q_prec=None,
    max_it=100,
    eps=1e-12,
    rho=1e-10,
    mu=1e-4,
):
    """
    q=proximalSolver(model,data,constraint_model,constraint_data,max_it=100,eps=1e-12,rho=1e-10,mu=1e-4)

    Build the robot in respect to the constraints using a proximal solver.

    Args:
        model (pinocchio.Model): Pinocchio model.
        data (pinocchio.Data): Pinocchio data.
        constraint_model (list): List of constraint models.
        constraint_data (list): List of constraint data.
        actuation_model (ActuationModelFreeFlyer): Actuation model.
        q_prec (list or np.array, optional): Initial guess for joint positions. Defaults to [].
        max_it (int, optional): Maximum number of iterations. Defaults to 100.
        eps (float, optional): Convergence threshold for primal and dual feasibility. Defaults to 1e-12.
        rho (float, optional): Scaling factor for the identity matrix. Defaults to 1e-10.
        mu (float, optional): Penalty parameter. Defaults to 1e-4.

    Returns:
        np.array: Joint positions of the robot respecting the constraints.

    raw here (L84-126):https://gitlab.inria.fr/jucarpen/pinocchio/-/blob/pinocchio-3x/examples/simulation-closed-kinematic-chains.py
    """

    Lid = actuation_model.idqmot
    if q_prec is None:
        q_prec = pin.neutral(model)
    q = q_prec

    constraint_dim = 0
    for cm in constraint_model:
        constraint_dim += cm.size()

    y = np.ones((constraint_dim))
    data.M = np.eye(model.nv) * rho
    kkt_constraint = pin.ContactCholeskyDecomposition(model, constraint_model)

    for k in range(max_it):
        pin.computeJointJacobians(model, data, q)
        kkt_constraint.compute(model, data, constraint_model, constraint_data, mu)

        constraint_value = np.concatenate(
            [
                (pin.log(cd.c1Mc2).np[: cm.size()])
                for (cd, cm) in zip(constraint_data, constraint_model)
            ]
        )

        LJ = []
        for cm, cd in zip(constraint_model, constraint_data):
            Jc = pin.getConstraintJacobian(model, data, cm, cd)
            LJ.append(Jc)
        J = np.concatenate(LJ)

        primal_feas = np.linalg.norm(constraint_value, np.inf)
        dual_feas = np.linalg.norm(J.T.dot(constraint_value + y), np.inf)
        if primal_feas < eps and dual_feas < eps:
            print("Convergence achieved")
            break
        print("constraint_value:", np.linalg.norm(constraint_value))
        rhs = np.concatenate([-constraint_value - y * mu, np.zeros(model.nv)])

        dz = kkt_constraint.solve(rhs)
        dy = dz[:constraint_dim]
        dq = dz[constraint_dim:]

        alpha = 1.0
        q = pin.integrate(model, q, -alpha * dq)
        y -= alpha * (-dy + y)
    return q


def ForwardK(
    model,
    constraint_model,
    actuation_model,
    q_prec=None,
    max_it=100,
    alpha = 0.7,
    eps=1e-12,
    rho=1e-10,
    mu=1e-4,

):
    """
    q=proximalSolver(model,data,constraint_model,constraint_data,max_it=100,eps=1e-12,rho=1e-10,mu=1e-4)

    Build the robot in respect to the constraints using a proximal solver.

    Args:
        model (pinocchio.Model): Pinocchio model.
        data (pinocchio.Data): Pinocchio data.
        constraint_model (list): List of constraint models.
        constraint_data (list): List of constraint data.
        actuation_model (ActuationModelFreeFlyer): Actuation model.
        q_prec (list or np.array, optional): Initial guess for joint positions. Defaults to [].
        max_it (int, optional): Maximum number of iterations. Defaults to 100.
        eps (float, optional): Convergence threshold for primal and dual feasibility. Defaults to 1e-12.
        rho (float, optional): Scaling factor for the identity matrix. Defaults to 1e-10.
        mu (float, optional): Penalty parameter. Defaults to 1e-4.

    Returns:
        np.array: Joint positions of the robot respecting the constraints.

    raw here (L84-126):https://gitlab.inria.fr/jucarpen/pinocchio/-/blob/pinocchio-3x/examples/simulation-closed-kinematic-chains.py
    """

    Lid = actuation_model.idMotJoints
    Lid_q = actuation_model.idqmot

    (reduced_model, reduced_constraint_models) = freezeJointsWithoutVis(
        model, constraint_model, None, Lid, q_prec
    )

    reduced_data = reduced_model.createData()
    reduced_constraint_data = [c.createData() for c in reduced_constraint_models]

    q = np.delete(q_prec, Lid_q, axis=0)
    constraint_dim = 0
    for cm in reduced_constraint_models:
        constraint_dim += cm.size()

    y = np.ones((constraint_dim))
    reduced_data.M = np.eye(reduced_model.nv) * rho
    kkt_constraint = pin.ContactCholeskyDecomposition(
        reduced_model, reduced_constraint_models
    )

    for k in range(max_it):
        pin.computeJointJacobians(reduced_model, reduced_data, q)
        kkt_constraint.compute(
            reduced_model,
            reduced_data,
            reduced_constraint_models,
            reduced_constraint_data,
            mu,
        )

        constraint_value = np.concatenate(
            [
                (pin.log(cd.c1Mc2).np[: cm.size()])
                for (cd, cm) in zip(reduced_constraint_data, reduced_constraint_models)
            ]
        )

        # LJ = []
        # for cm, cd in zip(reduced_constraint_models, reduced_constraint_data):
        #     Jc = pin.getConstraintJacobian(reduced_model, reduced_data, cm, cd)
        #     LJ.append(Jc)
        # J = np.concatenate(LJ)

        primal_feas = np.linalg.norm(constraint_value, np.inf)
        # dual_feas = np.linalg.norm(J.T.dot(constraint_value + y), np.inf)
        if primal_feas < eps:
            # print("Convergence achieved")
            break
        # print("constraint_value:", np.linalg.norm(constraint_value))
        rhs = np.concatenate([-constraint_value - y * mu, np.zeros(reduced_model.nv)])

        dz = kkt_constraint.solve(rhs)
        dy = dz[:constraint_dim]
        dq = dz[constraint_dim:]


        q = pin.integrate(reduced_model, q, -alpha * dq)
        y -= alpha * (-dy + y)

    q_final = q_prec
    free_q_dict = zip(actuation_model.idqfree, q)
    for index, value in free_q_dict:
        q_final[index] = value
    return q_final, primal_feas


def ForwardK1(
    model,
    visual_model,
    constraint_model,
    collision_model,
    actuation_model,
    q_prec=None,
    max_it=100,
    eps=1e-12,
    rho=1e-10,
    mu=1e-4,
):
    Lid = actuation_model.idMotJoints

    Lid_q = actuation_model.idqmot
    q_prec2 = np.delete(q_prec, Lid, axis=0)
    model2 = model.copy()
    constraint_model2 = constraint_model.copy()
    reduced_model, reduced_constraint_models, reduced_actuation_model, reduced_visual_model, reduced_collision_model = freezeJoints(model2,
    constraint_model2,
    actuation_model,
    visual_model,
    collision_model,
    Lid,
    q_prec,
    )

    reduced_data = reduced_model.createData()
    data = model.createData()
    reduced_constraint_data = [c.createData() for c in reduced_constraint_models]
    constraint_data = [c.createData() for c in constraint_model2]

    pin.framesForwardKinematics(reduced_model, reduced_data, q_prec2)
    pin.computeAllTerms(
    reduced_model,
    reduced_data,
    q_prec2,
    q_prec2)

    q_ooo = closedLoopProximalMount(
        reduced_model,
        reduced_data,
        reduced_constraint_models,
        reduced_constraint_data,
        reduced_actuation_model,
        q_prec2,
        max_it=4,
        rho=1e-8,
        mu=1e-3
    )

    return q_ooo
