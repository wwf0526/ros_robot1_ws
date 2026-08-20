import math


def quat_normalize(q):
    values = tuple(float(v) for v in q)
    n = math.sqrt(sum(v * v for v in values))
    if n < 1.0e-12:
        raise ValueError("Quaternion norm is zero")
    return tuple(v / n for v in values)


def quat_canonical(q):
    q = quat_normalize(q)
    if q[0] < 0.0:
        q = tuple(-v for v in q)
    return q


def quat_conjugate(q):
    w, x, y, z = quat_normalize(q)
    return (w, -x, -y, -z)


def quat_inverse(q):
    return quat_conjugate(q)


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b

    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def quat_mean(samples):
    if not samples:
        raise ValueError("No quaternion samples")

    normalized = [quat_normalize(q) for q in samples]
    ref = normalized[0]
    aligned = []

    for q in normalized:
        if quat_dot(q, ref) < 0.0:
            q = tuple(-v for v in q)
        aligned.append(q)

    summed = tuple(sum(q[i] for q in aligned) for i in range(4))
    return quat_canonical(summed)


def quat_angular_distance_deg(q0, q1):
    a = quat_normalize(q0)
    b = quat_normalize(q1)

    d = abs(quat_dot(a, b))
    d = max(-1.0, min(1.0, d))

    return math.degrees(2.0 * math.acos(d))


def quat_max_dispersion_deg(samples, mean_q=None):
    if not samples:
        raise ValueError("No quaternion samples")

    if mean_q is None:
        mean_q = quat_mean(samples)

    return max(
        quat_angular_distance_deg(mean_q, q)
        for q in samples
    )


def quat_slerp(q0, q1, alpha):
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)

    alpha = max(0.0, min(1.0, float(alpha)))

    d = quat_dot(q0, q1)

    if d < 0.0:
        q1 = tuple(-v for v in q1)
        d = -d

    d = max(-1.0, min(1.0, d))

    if d > 0.9995:
        q = tuple(
            (1.0 - alpha) * a + alpha * b
            for a, b in zip(q0, q1)
        )
        return quat_canonical(q)

    theta0 = math.acos(d)
    sin_theta0 = math.sin(theta0)

    s0 = math.sin((1.0 - alpha) * theta0) / sin_theta0
    s1 = math.sin(alpha * theta0) / sin_theta0

    q = tuple(
        s0 * a + s1 * b
        for a, b in zip(q0, q1)
    )

    return quat_canonical(q)


def quat_from_euler_zyx(roll_rad, pitch_rad, yaw_rad):
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)

    return quat_canonical((
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ))


def quat_to_euler_rad(q):
    w, x, y, z = quat_normalize(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quat_to_euler_deg(q):
    roll, pitch, yaw = quat_to_euler_rad(q)

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


def align_imu_quaternion(q_raw, q_zero, q_mount):
    q_raw = quat_normalize(q_raw)
    q_zero = quat_normalize(q_zero)
    q_mount = quat_normalize(q_mount)

    q_rel = quat_multiply(
        quat_inverse(q_zero),
        q_raw,
    )

    q_robot = quat_multiply(
        quat_multiply(q_mount, q_rel),
        quat_inverse(q_mount),
    )

    return quat_canonical(q_robot)


def wrap_deg(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0
