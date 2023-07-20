from dqrobotics import *
import dqrobotics as dql
import numpy as np
from dqrobotics.robot_modeling import DQ_Kinematics


def closest_invariant_rotation_error(x, xd):
    er_plus_norm = np.linalg.norm(dql.vec4(dql.conj(dql.rotation(x)) * dql.rotation(xd) - 1))
    er_minus_norm = np.linalg.norm(dql.vec4(dql.conj(dql.rotation(x)) * dql.rotation(xd) + 1))

    if er_plus_norm < er_minus_norm:
        er = dql.conj(dql.rotation(x)) * dql.rotation(xd) - 1

    else:
        er = dql.conj(dql.rotation(x)) * dql.rotation(xd) + 1

    return er


def closest_invariant_rotation_error_scalar(x, xd):
    err = closest_invariant_rotation_error(x, xd)
    return np.linalg.norm(err.vec4())


#################################
# pose Error
#################################
def translation_error(x, xd):
    err = dql.vec3(dql.translation(x) - dql.translation(xd))
    return np.linalg.norm(err)


def get_weighted_pose_error(x, xd, Jt, Jr, alpha):
    t_err_vec = dql.vec4(dql.translation(x) - dql.translation(xd))
    r_err_vec = dql.vec4(closest_invariant_rotation_error(x, xd))

    c_t = t_err_vec.T @ Jt
    c_r = r_err_vec.T @ Jr

    return alpha * c_t + (1 - alpha) * c_r


def get_modified_pose_error(x, xd):
    err_minus = dql.vec8(dql.conj(x) * xd - 1)
    err_plus = dql.vec8(dql.conj(x) * xd + 1)

    norm_minus = np.linalg.norm(err_minus)
    norm_plus = np.linalg.norm(err_plus)

    if norm_minus < norm_plus:
        return err_minus
    else:
        return err_plus


def get_pose_error(x, xd):
    return dql.vec8(x) - dql.vec8(xd)


#################################
# Feed Forward
#################################
def get_modified_feed_forward(x, xd_dot):
    return dql.vec8(dql.conj(x) * xd_dot)


def get_feed_forward(x, xd_dot):
    return (dql.vec8(x) - dql.vec8(xd_dot))

    x_t = dql.translation(x)
    x_r = dql.rotation(x)
    xd_dot_r = dql.P(xd_dot)
    xd_dot_D = dql.D(xd_dot)
    xd_dot_t = (2 * xd_dot_D * dql.conj(xd_dot_r))

    # print(x_t, ":", x_r, ":", xd_dot_r, ":",xd_dot_t)

    return dql.hamiplus8(dql.conj(x)) @ dql.vec8(xd_dot)


#################################
# Jacobian
#################################
def get_modified_jacobian(Jx, xd):
    return dql.haminus8(xd) @ dql.C8() @ Jx


def get_jacobian(Jx, xd):
    return Jx


#################################
# interpolation
#################################
def calculate_next_trajectory_step(x, xf, sampling_time, speed):
    # print(x.q, xf.q, sampling_time, speed)
    er_plus_norm = dql.norm(dql.conj(x) * xf - 1).q[0] / speed
    er_minus_norm = dql.norm(dql.conj(x) * xf + 1).q[0] / speed

    if er_plus_norm < er_minus_norm:
        tmax = er_plus_norm
    else:
        tmax = er_minus_norm

    if (tmax < sampling_time):
        return xf

    if er_plus_norm < er_minus_norm:
        return x * dql.exp((sampling_time / tmax) * dql.log(dql.conj(x) * (xf)))
    else:
        return x * dql.exp((sampling_time / tmax) * dql.log(dql.conj(x) * (-xf)))


#################################
# Constraints calculation
#################################
def robotPointToPointEllipsoidActiveConstraint(t, Jt,
                                               p, Jp,
                                               a, b, c,
                                               ref_x, J_ref_x, ):
    # a, b,c represent the size of the ellipsoid

    kinv = dql.inv(ref_x)
    kinv_r = dql.rotation(kinv)
    Jr = DQ_Kinematics.rotation_jacobian(J_ref_x)

    ellipsoid = np.array([[0, 0, 0, 0],
                          [0, a * a, 0, 0],
                          [0, 0, b * b, 0],
                          [0, 0, 0, c * c]])

    jacobian = 2 * dql.vec4(kinv_r * (t - p) * kinv_r.conj()).T @ ellipsoid @ \
               (dql.haminus4((t - p) * kinv_r.conj()) @ dql.C4() @ Jr
                - dql.haminus4(kinv_r.conj()) @ dql.hamiplus4(kinv_r) @ Jp
                + dql.haminus4(kinv_r.conj()) @ dql.hamiplus4(kinv_r) @ Jt
                + dql.hamiplus4(kinv_r * (t - p)) @ Jr)

    distance = np.linalg.norm(dql.vec4(t - p)) ** 2

    return jacobian, distance


#################################
# Pathology cut scoring calculation
#################################
def get_plucker_from_pose_and_source_vec(pose, pose_direction):
    pose_rot = dql.rotation(pose)
    pose_pt = dql.translation(pose)
    pos_vec = pose_rot * pose_direction * dql.conj(pose_rot)
    return pos_vec + dql.DQ.E * dql.cross(pose_pt, pos_vec)


def project_plucker_onto_plane(line, plane):
    # check for line is not vertical to plane
    plane_norm = dql.P(plane)
    line_dir = dql.P(line)
    assert dql.norm(dql.cross(plane_norm, line_dir)).q[0] != 0, "The given line is vertical to the plane"
    line_m = dql.D(line)
    S = np.array([[0, -line_dir.q[3], line_dir.q[2]],
                  [line_dir.q[3], 0, -line_dir.q[1]],
                  [-line_dir.q[2], line_dir.q[1], 0]])
    p1 = dql.DQ(-np.linalg.pinv(S) @ line_m.vec3())
    p2 = p1 + line_dir
    # construct 1   line base on two point projected onto plane
    p1_p = project_point_onto_plane(p1, plane)
    p2_p = project_point_onto_plane(p2, plane)
    l_d = (p2_p - p1_p).normalize()
    l_m = dql.cross(p1_p, l_d)
    return l_d + dql.DQ.E * l_m


def project_point_onto_plane(point, plane):
    assert dql.is_pure(point), "input point is not pure"
    plane_d = dql.D(plane)
    plane_n = dql.P(plane).normalize()
    v = point - plane_d * plane_n
    dist = dql.dot(v, plane_n)
    return point - dist * plane_n
