import math


def quat_normalize_wxyz(q):
    q = tuple(float(v) for v in q)
    n = math.sqrt(sum(v * v for v in q))
    if n < 1.0e-12:
        raise ValueError("zero quaternion")
    return tuple(v / n for v in q)


def quat_to_rot_wxyz(q):
    w, x, y, z = quat_normalize_wxyz(q)
    return (
        (1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w), 2.0*(x*z + y*w)),
        (2.0*(x*y + z*w), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)),
        (2.0*(x*z - y*w), 2.0*(y*z + x*w), 1.0 - 2.0*(x*x + y*y)),
    )


def mat3_vec(R, v):
    return tuple(sum(float(R[i][j])*float(v[j]) for j in range(3)) for i in range(3))


def mat3_mul(A, B):
    return tuple(tuple(sum(float(A[i][k])*float(B[k][j]) for k in range(3)) for j in range(3)) for i in range(3))


def mat3_transpose(A):
    return tuple(tuple(float(A[j][i]) for j in range(3)) for i in range(3))


def vec_add(a, b):
    return tuple(float(a[i]) + float(b[i]) for i in range(3))


def vec_sub(a, b):
    return tuple(float(a[i]) - float(b[i]) for i in range(3))


def vec_norm(v):
    return math.sqrt(sum(float(x)*float(x) for x in v))


def rigid_inverse(p_ab, q_ab):
    R_ab = quat_to_rot_wxyz(q_ab)
    R_ba = mat3_transpose(R_ab)
    p_ba = mat3_vec(R_ba, tuple(-float(x) for x in p_ab))
    return p_ba, R_ba


def compose_pose_matrix_parts(p_ab, R_ab, p_bc, R_bc):
    p_ac = vec_add(p_ab, mat3_vec(R_ab, p_bc))
    R_ac = mat3_mul(R_ab, R_bc)
    return p_ac, R_ac


def transform_msg_to_pose(transform):
    t = transform.translation
    q = transform.rotation
    return (
        (float(t.x), float(t.y), float(t.z)),
        (float(q.w), float(q.x), float(q.y), float(q.z)),
    )


def compute_base_to_tip_tag(p_base_ref, q_base_ref, p_camera_ref, q_camera_ref, p_camera_tip, q_camera_tip):
    # ^B T_C = ^B T_ref * inv(^C T_ref)
    # ^B T_tipTag = ^B T_C * ^C T_tipTag
    R_b_ref = quat_to_rot_wxyz(q_base_ref)
    p_ref_c, R_ref_c = rigid_inverse(p_camera_ref, q_camera_ref)
    p_b_c, R_b_c = compose_pose_matrix_parts(p_base_ref, R_b_ref, p_ref_c, R_ref_c)
    R_c_tip = quat_to_rot_wxyz(q_camera_tip)
    return compose_pose_matrix_parts(p_b_c, R_b_c, p_camera_tip, R_c_tip)


def compensate_tip_tag_offset(p_base_tag, q_base_tip_fused, tip_to_tag_xyz_m):
    R = quat_to_rot_wxyz(q_base_tip_fused)
    offset_base = mat3_vec(R, tip_to_tag_xyz_m)
    return vec_sub(p_base_tag, offset_base)


def fuse_position_xyz(p_model, p_vision, weights_xyz):
    return tuple(float(p_model[i]) + float(weights_xyz[i])*(float(p_vision[i]) - float(p_model[i])) for i in range(3))
