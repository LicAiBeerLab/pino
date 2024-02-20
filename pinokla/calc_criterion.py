import time
from matplotlib.pylab import LinAlgError
import pinocchio as pin
from numpy.linalg import norm
import numpy as np

from pinokla.closed_loop_jacobian import dq_dqmot, inverseConstraintKinematicsSpeed
from pinokla.closed_loop_kinematics import ForwardK


def kinematic_test(
    model, data, constraint_models, constraint_data, actuation_model, end_effector_frame: str, base_frame, traj_6d: np.ndarray, traj_6d_v: np.ndarray, q_start, viz=None
):
    DT = 1e-3
    Kp = 10
    Kd = 2
    q = q_start
    ee_frame_id = model.getFrameId(end_effector_frame)
    id_base = model.getFrameId(base_frame)
    base_coord_t = data.oMf[id_base].translation
    pin.framesForwardKinematics(model, data, q)
    old_pos_ee = np.concatenate([data.oMf[ee_frame_id].translation -
                                 base_coord_t, pin.log(data.oMf[ee_frame_id]).angular])

    v_ee_world = np.array([1, 0, 0, 0, 0, 0])
    vee_foot = data.oMf[ee_frame_id].action@v_ee_world

    vq, Jclosed = inverseConstraintKinematicsSpeed(
        model, data, constraint_models, constraint_data, actuation_model, q, ee_frame_id, vee_foot)

    traj_6d_ee = np.zeros(traj_6d.shape)

    for i in range(len(traj_6d)):
        target_ee_pos = traj_6d[i]
        target_ee_v = traj_6d_v[i]

        pos_ee = np.concatenate([data.oMf[ee_frame_id].translation -
                                 base_coord_t, pin.log(data.oMf[ee_frame_id]).angular])
        traj_6d_ee[i] = pos_ee
        vreel = (pos_ee-old_pos_ee)
        old_pos_ee = pos_ee.copy()

        va = Kp*(target_ee_pos-pos_ee) + Kd*(target_ee_v-vreel)
        vq, Jf36_closed = inverseConstraintKinematicsSpeed(
            model, data, constraint_models, constraint_data, actuation_model, q, ee_frame_id, data.oMf[ee_frame_id].action@va)
        q = pin.integrate(model, q, vq * DT)
        pin.framesForwardKinematics(model, data, q)
        pin.computeAllTerms(model, data, q, vq)
        pin.computeJointJacobians(model, data, q)
        if viz:
            viz.display(q)
            time.sleep(0.01)
            print(norm(target_ee_pos[0:3] - pos_ee[0:3]))
    return traj_6d_ee


def set_end_effector(model,
                     data,
                     constraint_model,
                     constraint_data,
                     actuation_model,
                     target_6d: np.ndarray,
                     base_frame: str,
                     frame_name_ee: str,
                     starting_q=None,
                     only_trans=True,
                     viz=None):
    if not only_trans:
        raise ("Rotation not implemented")
    Kp = 10
    Kd = 1
    DT = 0.001
    QUALITY = 0.04
    id_ee = model.getFrameId(frame_name_ee)
    id_base = model.getFrameId(base_frame)
    q = starting_q
    pin.framesForwardKinematics(model, data, starting_q)
    base_coord_t = data.oMf[id_base].translation
    ee_coord_t = data.oMf[id_ee].translation
    oldpos_t = ee_coord_t - base_coord_t
    error = np.zeros(6)
    for i in range(200):
        base_coord_trans = data.oMf[id_base].translation
        ee_coord_trans = data.oMf[id_ee].translation
        ee_coord_trans_in_base = ee_coord_trans - base_coord_trans
        ee_coord_in_base = np.concatenate(
            (ee_coord_trans_in_base, pin.log(data.oMf[id_ee]).angular))

        prev_error = error
        error = target_6d - ee_coord_in_base
        # vreel = target_6d - oldpos_t
        va = (Kp * error * 0.8 + Kd * (error - prev_error)) 
        vq, Jf36_closed = inverseConstraintKinematicsSpeed(
            model,
            data,
            constraint_model,
            constraint_data,
            actuation_model,
            q,
            id_ee,
            data.oMf[id_ee].action @ va,
        )
        q = pin.integrate(model, q, vq * DT)
        pin.framesForwardKinematics(model, data, q)

        err = np.sum(
            [norm(cd.contact_placement_error.np) for cd in constraint_data])

        error = norm(ee_coord_in_base[0:3] - target_6d[0:3])
        if viz:
            viz.display(q)
            time.sleep(0.01)
            print(error)

        if error < QUALITY:
            break

    return q


def search_workspace(model, data, effector_frame_name: str,  base_frame_name: str,  q_space: np.ndarray, actuation_model, constraint_models, viz=None):
    c = 0
    q_start = pin.neutral(model)
    workspace_xyz = np.empty((len(q_space), 3))
    available_q = np.empty((len(q_space), len(q_start)))
    for q_sample in q_space:

        q_dict_mot = zip(actuation_model.idqmot, q_sample)
        for key, value in q_dict_mot:
            q_start[key] = value
        q3, error = ForwardK(
            model,
            constraint_models,
            actuation_model,
            q_start,
            150,
        )

        if error < 1e-11:
            if viz:
                viz.display(q3)
                time.sleep(0.005)
            q_start = q3
            pin.framesForwardKinematics(model, data, q3)
            id_effector = model.getFrameId(effector_frame_name)
            id_base = model.getFrameId(base_frame_name)
            effector_pos = data.oMf[id_effector].translation
            base_pos = data.oMf[id_base].translation
            transformed_pos = effector_pos - base_pos

            workspace_xyz[c] = transformed_pos
            available_q[c] = q3
            c += 1
    return (workspace_xyz[0:c], available_q[0:c])


def convert_full_J_to_planar_xz(full_J: np.ndarray):
    ret = np.row_stack((full_J[0], full_J[2], full_J[4]))
    return ret


def calc_manipulability(jacob: np.ndarray):
    ret1 = np.linalg.det(jacob@jacob.T)
    # Alternative
    # manip=np.linalg.svd(jacob@jacob.T)[1]
    # np.product(manip)**0.5
    return ret1


def calc_force_ellips(jacob: np.ndarray):
    try:
        ret1 = np.linalg.det(np.linalg.inv(jacob).T@np.linalg.inv(jacob))
    except LinAlgError:
        ret1 = 0
    return ret1


def kinematic_simulation(model, data, actuation_model, constraint_models, constraint_data, frame_name_ee: str,  base_frame_name: str,  q_space: np.ndarray, compute_all_terms=True, viz=None):
    id_ee = model.getFrameId(frame_name_ee)
    # Zero iterate for precise allocate
    # Not the best soluition
    pin.framesForwardKinematics(model, data, q_space[0])
    if compute_all_terms:
        pin.computeAllTerms(model, data, q_space[0], np.zeros(len(q_space[0])))

    pin.computeJointJacobians(model, data, q_space[0])

    vq, J_closed = inverseConstraintKinematicsSpeed(
        model, data, constraint_models, constraint_data, actuation_model, q_space[0], id_ee, data.oMf[id_ee].action@np.zeros(6))
    LJ = []
    for (cm, cd) in zip(constraint_models, constraint_data):
        Jc = pin.getConstraintJacobian(model, data, cm, cd)
        LJ.append(Jc)

    M = pin.crba(model, data, q_space[0])
    dq = dq_dqmot(model, actuation_model, LJ)

    traj_M = np.zeros((len(q_space), *M.shape))
    traj_J_closed = np.zeros((len(q_space), *J_closed.shape))
    traj_dq = np.zeros((len(q_space), *dq.shape))

    for num, q in enumerate(q_space):
        pin.framesForwardKinematics(model, data, q)
        if compute_all_terms:
            pin.computeAllTerms(model, data, q, np.zeros(len(q)))
        pin.computeJointJacobians(model, data, q)

        vq, J_closed = inverseConstraintKinematicsSpeed(
            model, data, constraint_models, constraint_data, actuation_model, q, id_ee, data.oMf[id_ee].action@np.zeros(6))
        LJ = []
        for (cm, cd) in zip(constraint_models, constraint_data):
            Jc = pin.getConstraintJacobian(model, data, cm, cd)
            LJ.append(Jc)

        M = pin.crba(model, data, q)
        dq = dq_dqmot(model, actuation_model, LJ)

        traj_M[num] = M
        traj_J_closed[num] = J_closed
        traj_dq[num] = dq

    return traj_M, traj_J_closed, traj_dq


def calc_manipulability_along_trj(traj_J_closed):
    array_manip = np.zeros(len(traj_J_closed))
    for num, J in enumerate(traj_J_closed):
        planar_J = convert_full_J_to_planar_xz(J)
        manip = calc_manipulability(planar_J)
        array_manip[num] = manip

    return array_manip


def calc_manipulability_along_trj_trans(traj_J_closed):
    array_manip = np.zeros(len(traj_J_closed))
    for num, J in enumerate(traj_J_closed):
        planar_J = convert_full_J_to_planar_xz(J)
        trans_planar_J = planar_J[:2, :2]
        manip = calc_manipulability(trans_planar_J)
        array_manip[num] = manip

    return array_manip


def calc_force_ell_along_trj_trans(traj_J_closed):
    array_force_cap = np.zeros(len(traj_J_closed))
    for num, J in enumerate(traj_J_closed):
        planar_J = convert_full_J_to_planar_xz(J)
        trans_planar_J = planar_J[:2, :2]
        force_cap = calc_force_ellips(trans_planar_J)
        array_force_cap[num] = force_cap

    return array_force_cap


def calc_IMF(M_free: np.ndarray, dq_free: np.ndarray,  J_closed_free: np.ndarray):
    y = np.array([0, 1, 0, 0, 0, 0])
    Mmot_free = dq_free.T@M_free@dq_free
    Lambda_free = np.linalg.inv(
        J_closed_free@np.linalg.inv(Mmot_free)@J_closed_free.T)
    Lambda_free_lock = np.linalg.inv(
        J_closed_free[:6, :6]@np.linalg.inv(Mmot_free[:6, :6])@J_closed_free[:6, :6].T)

    IMF = np.linalg.det(np.identity(6)-Lambda_free @
                        np.linalg.inv(Lambda_free_lock))
    yimf = 1-(y.T@Lambda_free@y)/(y.T@Lambda_free_lock@y)
    return yimf, IMF


def calc_IMF_along_traj(M_traj_free: np.ndarray, dq_traj_free: np.ndarray,  J_traj_closed_free: np.ndarray):
    imf_array = np.zeros(len(M_traj_free))
    for M_free, dq_free, J_closed_free, num in zip(M_traj_free, dq_traj_free, J_traj_closed_free, range(len(M_traj_free))):
        yimf, IMF = calc_IMF(M_free, dq_free, J_closed_free)
        imf_array[num] = yimf
    return imf_array


def calc_foot_inertia(M: np.ndarray, dq: np.ndarray,  J_closed: np.ndarray):
    Mmot = dq.T@M@dq
    Lambda = np.linalg.inv(J_closed@np.linalg.inv(Mmot)@J_closed.T)
    return Lambda


def calc_foot_inertia_along_traj(M_traj: np.ndarray, dq_traj: np.ndarray,  J_traj_closed: np.ndarray):
    foot_inertia_array = np.zeros(len(M_traj))
    for M, dq, J_closed, num in zip(M_traj, dq_traj, J_traj_closed, range(len(M_traj))):
        try:
            planar_J = convert_full_J_to_planar_xz(J_closed)
            Lambda = calc_foot_inertia(M, dq, planar_J)
            # [0,0]xx [1,1] yy [2,2] zz
            foot_inertia_array[num] = Lambda[1, 1]
        except:
            foot_inertia_array[num] = None
    return foot_inertia_array


def all_mean(trjs: tuple[np.ndarray]):
    means = np.zeros(len(trjs))
    for num, tr in enumerate(trjs):
        means[num] = tr.mean()
    return means

def mean_max_min_std(trj: np.ndarray):
    mean = trj.mean()
    max_value = trj.max()
    min_value = trj.min()
    trj.std()
    pass
