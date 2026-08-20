import math
import time
from collections import deque

import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from robot_interfaces.msg import MotorState, TendonState, ContinuumState, SafetyState
from robot_interfaces.srv import SetImuZero

from continuum_model.pcc_model import compute_section_curvature
from continuum_pccmodel.geometry import load_pcc_geometry_from_cfg
from continuum_pccmodel.forward_kinematics import forward_kinematics_from_dl_mm

from .orientation_fusion import (
    align_imu_quaternion,
    quat_angular_distance_deg,
    quat_canonical,
    quat_from_euler_zyx,
    quat_max_dispersion_deg,
    quat_mean,
    quat_normalize,
    quat_slerp,
    quat_to_euler_deg,
)

from .vision_fusion import (
    compute_base_to_tip_tag,
    compensate_tip_tag_offset,
    fuse_position_xyz,
    transform_msg_to_pose,
    vec_norm,
    vec_sub,
)


IDENTITY_Q = (1.0, 0.0, 0.0, 0.0)


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__("continuum_state_estimator")

        self.declare_parameter(
            "calibration_file",
            "/home/wangwenfeng/ros_robot1_ws/src/robot_bringup/config/robot_calibration.yaml",
        )
        self.calibration_file = self.get_parameter("calibration_file").value

        self.motor_position_deg = {}
        self.last_motor_time = {}
        self.emergency_stop_active = False

        self.imu_raw = {"imu1": None, "imu2": None}
        self.last_imu_time = {"imu1": None, "imu2": None}
        self.imu_buffers = {
            "imu1": deque(maxlen=1000),
            "imu2": deque(maxlen=1000),
        }
        self.q_zero = {"imu1": None, "imu2": None}

        self.last_pcc_model_valid = False
        self.last_orientation_residual_deg = {
            "section1": -1.0,
            "section2": -1.0,
        }
        self.last_vision_extrinsic_ready = False
        self.last_vision_tip_valid = False
        self.last_vision_position_residual_m = -1.0

        self.load_calibration()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tendon_pub = self.create_publisher(
            TendonState,
            self.state_topics["tendon_state"],
            10,
        )
        self.continuum_pub = self.create_publisher(
            ContinuumState,
            self.state_topics["continuum_state"],
            10,
        )
        self.safety_pub = self.create_publisher(
            SafetyState,
            self.state_topics["safety_state"],
            10,
        )

        self.motor_sub = self.create_subscription(
            MotorState,
            "/motor/state",
            self.motor_callback,
            10,
        )
        self.imu1_sub = self.create_subscription(
            Imu,
            self.imu_topics["imu1"],
            lambda msg: self.imu_callback("imu1", msg),
            10,
        )
        self.imu2_sub = self.create_subscription(
            Imu,
            self.imu_topics["imu2"],
            lambda msg: self.imu_callback("imu2", msg),
            10,
        )
        self.estop_sub = self.create_subscription(
            Bool,
            "/motor/emergency_stop_active",
            self.estop_callback,
            10,
        )

        self.imu_zero_srv = self.create_service(
            SetImuZero,
            "/continuum/set_imu_zero",
            self.set_imu_zero_callback,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_states,
        )

        self.get_logger().info(
            "continuum_state_estimator started: "
            "PCC + dual-IMU quaternion fusion enabled; "
            "runtime IMU zero is required before IMU correction is used"
        )

    def load_calibration(self):
        with open(self.calibration_file, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        self.cfg = cfg
        self.motor_ids = [int(x) for x in cfg["motor"]["motor_ids"]]
        self.sign = cfg["motor"]["sign"]
        self.spool_radius_mm = cfg["motor"]["spool_radius_mm"]
        self.motor_to_tendon = cfg["tendon"]["motor_to_tendon"]
        self.slack_threshold_mm = float(cfg["tendon"]["slack_threshold_mm"])

        state_cfg = cfg.get("state_estimation", {})
        self.publish_rate_hz = float(state_cfg.get("publish_rate_hz", 50.0))
        self.state_topics = state_cfg.get(
            "publish_topics",
            {
                "tendon_state": "/continuum/tendon_state",
                "continuum_state": "/continuum/state",
                "safety_state": "/continuum/safety_state",
            },
        )

        imu_cfg = cfg["imu"]
        self.imu_topics = imu_cfg["imu_topics"]

        mount_cfg = imu_cfg["mount_quaternion_wxyz"]
        self.q_mount = {
            name: quat_normalize(tuple(float(v) for v in mount_cfg[name]))
            for name in ("imu1", "imu2")
        }

        fusion_cfg = cfg.get("orientation_fusion", {})
        self.use_imu_correction = bool(
            state_cfg.get("use_imu_correction", True)
            and fusion_cfg.get("enabled", True)
        )
        self.section1_imu_weight = float(
            fusion_cfg.get("section1_imu_weight", 0.8)
        )
        self.section2_imu_weight = float(
            fusion_cfg.get("section2_imu_weight", 0.8)
        )
        self.zero_window_sec = float(
            fusion_cfg.get("zero_window_sec", 3.0)
        )
        self.zero_min_samples = int(
            fusion_cfg.get("zero_min_samples", 50)
        )
        self.zero_max_dispersion_deg = float(
            fusion_cfg.get("zero_max_dispersion_deg", 0.5)
        )
        self.imu_timeout_sec = float(
            fusion_cfg.get("imu_timeout_sec", 0.5)
        )
        self.motor_timeout_sec = float(
            fusion_cfg.get("motor_timeout_sec", 0.5)
        )
        self.residual_limit_deg = float(
            fusion_cfg.get("residual_limit_deg", 10.0)
        )

        vision_cfg = cfg.get("vision_fusion", {})
        self.vision_enabled = bool(vision_cfg.get("enabled", True))
        self.camera_frame = str(vision_cfg.get("camera_frame", "camera"))
        self.tip_tag_frame = str(vision_cfg.get("tip_tag_frame", "continuum_tip_tag"))
        self.base_tag_frame = str(vision_cfg.get("base_tag_frame", "continuum_base_tag"))
        base_tag_pose = vision_cfg.get("base_tag_pose_in_base", {})
        self.base_tag_translation_m = tuple(float(v) for v in base_tag_pose.get("translation_xyz_m", [0.0, 0.0, 0.0]))
        self.base_tag_quaternion_wxyz = quat_normalize(tuple(float(v) for v in base_tag_pose.get("quaternion_wxyz", [1.0, 0.0, 0.0, 0.0])))
        self.tip_to_tag_xyz_m = tuple(float(v) for v in vision_cfg.get("tip_to_tag_xyz_m", [0.0, 0.0, 0.015]))
        self.position_weight_xyz = tuple(float(v) for v in vision_cfg.get("position_weight_xyz", [0.5, 0.5, 0.5]))
        if len(self.position_weight_xyz) != 3 or any(v < 0.0 or v > 1.0 for v in self.position_weight_xyz):
            raise ValueError("vision position weights must be three values in [0,1]")
        self.vision_timeout_sec = float(vision_cfg.get("vision_timeout_sec", 0.5))
        self.position_residual_limit_m = float(vision_cfg.get("position_residual_limit_m", 0.05))

        pcc_cfg = cfg.get("pcc_model", {})
        self.pcc_enabled = bool(pcc_cfg.get("enabled", False))
        self.pcc_geometry = (
            load_pcc_geometry_from_cfg(cfg)
            if self.pcc_enabled
            else None
        )
        self.pcc_chain_order = list(
            pcc_cfg.get("chain_order", ["section2", "section1"])
        )
        self.pcc_sections_cfg = pcc_cfg.get("sections", {})

        section_imu_map = cfg.get("section_imu_map", {})
        if (
            section_imu_map.get("section1") != "imu1"
            or section_imu_map.get("section2") != "imu2"
        ):
            raise ValueError(
                "Expected section1->imu1 and section2->imu2 in section_imu_map"
            )

        for mid in self.motor_ids:
            self.motor_position_deg[mid] = 0.0
            self.last_motor_time[mid] = None

        self.get_logger().info(
            f"Loaded calibration file: {self.calibration_file}"
        )

    def motor_callback(self, msg: MotorState):
        motor_id = int(msg.motor_id)
        self.motor_position_deg[motor_id] = float(msg.position_deg)
        self.last_motor_time[motor_id] = self.get_clock().now()

    def imu_callback(self, name, msg: Imu):
        q = msg.orientation
        try:
            q_raw = quat_normalize(
                (float(q.w), float(q.x), float(q.y), float(q.z))
            )
        except ValueError:
            self.get_logger().warn(f"{name}: invalid quaternion")
            return

        self.imu_raw[name] = q_raw
        self.last_imu_time[name] = self.get_clock().now()
        self.imu_buffers[name].append((time.monotonic(), q_raw))

    def estop_callback(self, msg: Bool):
        self.emergency_stop_active = bool(msg.data)

    def set_imu_zero_callback(self, request, response):
        window_sec = float(request.window_sec)
        if window_sec <= 0.0:
            window_sec = self.zero_window_sec

        cutoff = time.monotonic() - window_sec
        candidate = {}

        for name in ("imu1", "imu2"):
            samples = [
                q for stamp, q in self.imu_buffers[name]
                if stamp >= cutoff
            ]

            if len(samples) < self.zero_min_samples:
                response.success = False
                response.message = (
                    f"{name}: only {len(samples)} samples in "
                    f"{window_sec:.2f}s, need >= {self.zero_min_samples}"
                )
                return response

            q_mean = quat_mean(samples)
            dispersion = quat_max_dispersion_deg(samples, q_mean)

            if dispersion > self.zero_max_dispersion_deg:
                response.success = False
                response.message = (
                    f"{name}: robot/IMU not stationary enough; "
                    f"max dispersion={dispersion:.4f} deg > "
                    f"{self.zero_max_dispersion_deg:.4f} deg"
                )
                return response

            candidate[name] = (q_mean, dispersion)

        self.q_zero["imu1"] = candidate["imu1"][0]
        self.q_zero["imu2"] = candidate["imu2"][0]

        self._assign_quaternion(
            response.imu1_zero_orientation,
            self.q_zero["imu1"],
        )
        self._assign_quaternion(
            response.imu2_zero_orientation,
            self.q_zero["imu2"],
        )
        response.imu1_max_dispersion_deg = float(candidate["imu1"][1])
        response.imu2_max_dispersion_deg = float(candidate["imu2"][1])
        response.success = True
        response.message = "Runtime IMU zero established for imu1 and imu2"

        self.get_logger().info(
            "Runtime IMU zero established: "
            f"imu1 max dispersion={candidate['imu1'][1]:.4f} deg, "
            f"imu2 max dispersion={candidate['imu2'][1]:.4f} deg"
        )
        return response

    @staticmethod
    def _assign_quaternion(field, q):
        w, x, y, z = quat_canonical(q)
        field.w = float(w)
        field.x = float(x)
        field.y = float(y)
        field.z = float(z)

    @staticmethod
    def _assign_point(field, xyz):
        field.x = float(xyz[0])
        field.y = float(xyz[1])
        field.z = float(xyz[2])

    def imu_zero_ready(self):
        return (
            self.q_zero["imu1"] is not None
            and self.q_zero["imu2"] is not None
        )

    def imu_stream_valid(self, name, now=None):
        if self.last_imu_time[name] is None:
            return False
        if now is None:
            now = self.get_clock().now()
        age = (now - self.last_imu_time[name]).nanoseconds / 1e9
        return age <= self.imu_timeout_sec

    def motor_stream_valid(self, now=None):
        if now is None:
            now = self.get_clock().now()

        for mid in self.motor_ids:
            stamp = self.last_motor_time.get(mid)
            if stamp is None:
                return False

            age = (now - stamp).nanoseconds / 1e9
            if age > self.motor_timeout_sec:
                return False

        return True

    def compute_tendon_state(self):
        tendon_length_mm = [0.0] * 6
        motor_position_deg = [0.0] * 6
        tendon_slack = [False] * 6

        for motor_id in self.motor_ids:
            pos_deg = self.motor_position_deg.get(motor_id, 0.0)
            theta_rad = math.radians(pos_deg)

            sign = float(self.sign[motor_id])
            radius = float(self.spool_radius_mm[motor_id])
            delta_l = sign * radius * theta_rad

            tendon_id = int(self.motor_to_tendon[motor_id])
            index = tendon_id - 1

            tendon_length_mm[index] = float(delta_l)
            motor_position_deg[index] = float(pos_deg)

            # 目前没有直接张力传感器，不把长度接近零误判为物理松绳。
            tendon_slack[index] = False

        return tendon_length_mm, motor_position_deg, tendon_slack

    def build_pcc_input_from_tendon_lengths(self, tendon_length_mm):
        dl_sections_mm = []
        phase_offsets_rad = []
        dl_by_section = {}

        for section_name in self.pcc_chain_order:
            section_cfg = self.pcc_sections_cfg[section_name]
            tendon_ids = [int(x) for x in section_cfg["tendon_ids"]]
            dl_signs = [
                float(x)
                for x in section_cfg.get(
                    "dl_signs",
                    [1.0] * len(tendon_ids),
                )
            ]

            if len(tendon_ids) != 3 or len(dl_signs) != 3:
                raise ValueError(
                    f"{section_name}: exactly 3 tendon_ids and dl_signs required"
                )

            dl_mm = [
                float(tendon_length_mm[tendon_id - 1]) * dl_sign
                for tendon_id, dl_sign in zip(tendon_ids, dl_signs)
            ]

            dl_sections_mm.append(dl_mm)
            phase_offsets_rad.append(
                float(section_cfg.get("phase_offset_rad", 0.0))
            )
            dl_by_section[section_name] = dl_mm

        return dl_sections_mm, phase_offsets_rad, dl_by_section

    def fill_pcc_model_fields(self, msg: ContinuumState, tendon_length_mm):
        msg.pcc_model_valid = False

        if not self.pcc_enabled or self.pcc_geometry is None:
            return

        try:
            dl_sections_mm, phase_offsets_rad, dl_by_section = (
                self.build_pcc_input_from_tendon_lengths(tendon_length_mm)
            )

            fk = forward_kinematics_from_dl_mm(
                dl_sections_mm=dl_sections_mm,
                geometry=self.pcc_geometry,
                phase_offsets_rad=phase_offsets_rad,
                section_names=self.pcc_chain_order,
            )

            pose_by_section = {
                section_name: fk["section_poses"][i]
                for i, section_name in enumerate(self.pcc_chain_order)
            }

            section1_dl = dl_by_section.get("section1", [0.0, 0.0, 0.0])
            section2_dl = dl_by_section.get("section2", [0.0, 0.0, 0.0])

            msg.section1_pcc_dl1_mm = float(section1_dl[0])
            msg.section1_pcc_dl2_mm = float(section1_dl[1])
            msg.section1_pcc_dl3_mm = float(section1_dl[2])

            msg.section2_pcc_dl1_mm = float(section2_dl[0])
            msg.section2_pcc_dl2_mm = float(section2_dl[1])
            msg.section2_pcc_dl3_mm = float(section2_dl[2])

            if "section1" in pose_by_section:
                p = pose_by_section["section1"]
                msg.section1_model_theta_rad = float(p["theta"])
                msg.section1_model_phi_rad = float(p["phi"])
                msg.section1_model_kappa_1pm = float(p["kappa"])
                msg.section1_model_arc_length_m = float(p["arc_length"])
                msg.section1_model_center_length_m = float(p["center_length"])
                msg.section1_model_l0_m = float(p["L0"])
                msg.section1_model_px_m = float(p["px"])
                msg.section1_model_py_m = float(p["py"])
                msg.section1_model_pz_m = float(p["pz"])
                msg.section1_model_yaw_rad = float(p["yaw"])
                msg.section1_model_pitch_rad = float(p["pitch"])
                msg.section1_model_roll_rad = float(p["roll"])

            if "section2" in pose_by_section:
                p = pose_by_section["section2"]
                msg.section2_model_theta_rad = float(p["theta"])
                msg.section2_model_phi_rad = float(p["phi"])
                msg.section2_model_kappa_1pm = float(p["kappa"])
                msg.section2_model_arc_length_m = float(p["arc_length"])
                msg.section2_model_center_length_m = float(p["center_length"])
                msg.section2_model_l0_m = float(p["L0"])
                msg.section2_model_px_m = float(p["px"])
                msg.section2_model_py_m = float(p["py"])
                msg.section2_model_pz_m = float(p["pz"])
                msg.section2_model_yaw_rad = float(p["yaw"])
                msg.section2_model_pitch_rad = float(p["pitch"])
                msg.section2_model_roll_rad = float(p["roll"])

            msg.end_model_px_m = float(fk["end_px"])
            msg.end_model_py_m = float(fk["end_py"])
            msg.end_model_pz_m = float(fk["end_pz"])
            msg.end_model_yaw_rad = float(fk["end_yaw"])
            msg.end_model_pitch_rad = float(fk["end_pitch"])
            msg.end_model_roll_rad = float(fk["end_roll"])

            msg.pcc_model_valid = True

        except Exception as exc:
            msg.pcc_model_valid = False
            self.get_logger().warn(f"PCC model update failed: {exc}")

    @staticmethod
    def _transform_stamp_age_sec(transform_stamped, now):
        stamp = transform_stamped.header.stamp
        stamp_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return float("inf")
        return max(0.0, (now.nanoseconds - stamp_ns) * 1.0e-9)

    def apply_vision_position_fusion(self, msg: ContinuumState):
        p_model = (float(msg.end_model_px_m), float(msg.end_model_py_m), float(msg.end_model_pz_m))
        msg.vision_extrinsic_ready = False
        msg.vision_tip_valid = False
        msg.vision_position_used = False
        self._assign_point(msg.end_vision_position_m, (0.0, 0.0, 0.0))
        self._assign_point(msg.end_fused_position_m, p_model)
        msg.vision_position_residual_x_m = 0.0
        msg.vision_position_residual_y_m = 0.0
        msg.vision_position_residual_z_m = 0.0
        msg.vision_position_residual_m = -1.0
        self.last_vision_extrinsic_ready = False
        self.last_vision_tip_valid = False
        self.last_vision_position_residual_m = -1.0
        if not self.vision_enabled or not msg.pcc_model_valid:
            return
        now = self.get_clock().now()
        try:
            tf_camera_ref = self.tf_buffer.lookup_transform(self.camera_frame, self.base_tag_frame, Time())
            tf_camera_tip = self.tf_buffer.lookup_transform(self.camera_frame, self.tip_tag_frame, Time())
        except TransformException:
            return
        age_ref = self._transform_stamp_age_sec(tf_camera_ref, now)
        age_tip = self._transform_stamp_age_sec(tf_camera_tip, now)
        if age_ref > self.vision_timeout_sec:
            return
        msg.vision_extrinsic_ready = True
        self.last_vision_extrinsic_ready = True
        if age_tip > self.vision_timeout_sec:
            return
        p_c_ref, q_c_ref = transform_msg_to_pose(tf_camera_ref.transform)
        p_c_tip, q_c_tip = transform_msg_to_pose(tf_camera_tip.transform)
        try:
            p_b_tag, _ = compute_base_to_tip_tag(self.base_tag_translation_m, self.base_tag_quaternion_wxyz, p_c_ref, q_c_ref, p_c_tip, q_c_tip)
            q_tip_fused = (float(msg.end_fused_orientation.w), float(msg.end_fused_orientation.x), float(msg.end_fused_orientation.y), float(msg.end_fused_orientation.z))
            p_vision = compensate_tip_tag_offset(p_b_tag, q_tip_fused, self.tip_to_tag_xyz_m)
        except ValueError:
            return
        residual = vec_sub(p_vision, p_model)
        residual_norm = vec_norm(residual)
        self._assign_point(msg.end_vision_position_m, p_vision)
        msg.vision_position_residual_x_m = float(residual[0])
        msg.vision_position_residual_y_m = float(residual[1])
        msg.vision_position_residual_z_m = float(residual[2])
        msg.vision_position_residual_m = float(residual_norm)
        msg.vision_tip_valid = True
        self.last_vision_tip_valid = True
        self.last_vision_position_residual_m = float(residual_norm)
        if residual_norm > self.position_residual_limit_m:
            return
        p_fused = fuse_position_xyz(p_model, p_vision, self.position_weight_xyz)
        self._assign_point(msg.end_fused_position_m, p_fused)
        msg.vision_position_used = True

    def apply_orientation_fusion(self, msg: ContinuumState):
        msg.imu_zero_ready = bool(self.imu_zero_ready())
        msg.imu1_aligned_valid = False
        msg.imu2_aligned_valid = False
        msg.section1_fused_valid = False
        msg.section2_fused_valid = False
        msg.section1_orientation_residual_deg = -1.0
        msg.section2_orientation_residual_deg = -1.0

        self._assign_quaternion(msg.section1_model_orientation, IDENTITY_Q)
        self._assign_quaternion(msg.section2_model_orientation, IDENTITY_Q)
        self._assign_quaternion(msg.end_model_orientation, IDENTITY_Q)
        self._assign_quaternion(msg.imu1_aligned_orientation, IDENTITY_Q)
        self._assign_quaternion(msg.imu2_aligned_orientation, IDENTITY_Q)
        self._assign_quaternion(msg.section1_fused_orientation, IDENTITY_Q)
        self._assign_quaternion(msg.section2_fused_orientation, IDENTITY_Q)
        self._assign_quaternion(msg.end_fused_orientation, IDENTITY_Q)

        msg.imu1_roll = 0.0
        msg.imu1_pitch = 0.0
        msg.imu1_yaw = 0.0
        msg.imu2_roll = 0.0
        msg.imu2_pitch = 0.0
        msg.imu2_yaw = 0.0

        # 视觉接口预留；当前 fused position 先使用 PCC。
        msg.vision_tip_valid = False
        self._assign_point(msg.end_vision_position_m, (0.0, 0.0, 0.0))
        self._assign_point(
            msg.end_fused_position_m,
            (
                msg.end_model_px_m,
                msg.end_model_py_m,
                msg.end_model_pz_m,
            ),
        )
        msg.vision_position_residual_m = -1.0

        if not msg.pcc_model_valid:
            self.last_pcc_model_valid = False
            self.last_orientation_residual_deg = {
                "section1": -1.0,
                "section2": -1.0,
            }
            return

        q_model1 = quat_from_euler_zyx(
            msg.section1_model_roll_rad,
            msg.section1_model_pitch_rad,
            msg.section1_model_yaw_rad,
        )
        q_model2 = quat_from_euler_zyx(
            msg.section2_model_roll_rad,
            msg.section2_model_pitch_rad,
            msg.section2_model_yaw_rad,
        )
        q_end_model = quat_from_euler_zyx(
            msg.end_model_roll_rad,
            msg.end_model_pitch_rad,
            msg.end_model_yaw_rad,
        )

        self._assign_quaternion(msg.section1_model_orientation, q_model1)
        self._assign_quaternion(msg.section2_model_orientation, q_model2)
        self._assign_quaternion(msg.end_model_orientation, q_end_model)

        now = self.get_clock().now()

        imu1_ok = (
            self.use_imu_correction
            and self.q_zero["imu1"] is not None
            and self.imu_raw["imu1"] is not None
            and self.imu_stream_valid("imu1", now)
        )
        imu2_ok = (
            self.use_imu_correction
            and self.q_zero["imu2"] is not None
            and self.imu_raw["imu2"] is not None
            and self.imu_stream_valid("imu2", now)
        )

        q_imu1 = None
        q_imu2 = None

        if imu1_ok:
            q_imu1 = align_imu_quaternion(
                self.imu_raw["imu1"],
                self.q_zero["imu1"],
                self.q_mount["imu1"],
            )
            self._assign_quaternion(msg.imu1_aligned_orientation, q_imu1)
            r, p, y = quat_to_euler_deg(q_imu1)
            msg.imu1_roll = float(r)
            msg.imu1_pitch = float(p)
            msg.imu1_yaw = float(y)
            msg.imu1_aligned_valid = True

        if imu2_ok:
            q_imu2 = align_imu_quaternion(
                self.imu_raw["imu2"],
                self.q_zero["imu2"],
                self.q_mount["imu2"],
            )
            self._assign_quaternion(msg.imu2_aligned_orientation, q_imu2)
            r, p, y = quat_to_euler_deg(q_imu2)
            msg.imu2_roll = float(r)
            msg.imu2_pitch = float(p)
            msg.imu2_yaw = float(y)
            msg.imu2_aligned_valid = True

        if q_imu1 is not None:
            q_fused1 = quat_slerp(
                q_model1,
                q_imu1,
                self.section1_imu_weight,
            )
            residual1 = quat_angular_distance_deg(q_model1, q_imu1)
        else:
            q_fused1 = q_model1
            residual1 = -1.0

        if q_imu2 is not None:
            q_fused2 = quat_slerp(
                q_model2,
                q_imu2,
                self.section2_imu_weight,
            )
            residual2 = quat_angular_distance_deg(q_model2, q_imu2)
        else:
            q_fused2 = q_model2
            residual2 = -1.0

        self._assign_quaternion(msg.section1_fused_orientation, q_fused1)
        self._assign_quaternion(msg.section2_fused_orientation, q_fused2)

        # section1 是末端段，累计到 section1 末端即 tip 姿态。
        self._assign_quaternion(msg.end_fused_orientation, q_fused1)

        msg.section1_fused_valid = True
        msg.section2_fused_valid = True
        msg.section1_orientation_residual_deg = float(residual1)
        msg.section2_orientation_residual_deg = float(residual2)

        self.last_pcc_model_valid = True
        self.last_orientation_residual_deg = {
            "section1": float(residual1),
            "section2": float(residual2),
        }

    def publish_states(self):
        tendon_length_mm, motor_position_deg, tendon_slack = (
            self.compute_tendon_state()
        )

        tendon_msg = TendonState()
        tendon_msg.header.stamp = self.get_clock().now().to_msg()
        tendon_msg.header.frame_id = "continuum_base"
        tendon_msg.tendon_length_mm = tendon_length_mm
        tendon_msg.motor_position_deg = motor_position_deg
        tendon_msg.tendon_slack = tendon_slack
        self.tendon_pub.publish(tendon_msg)

        self.publish_continuum_state(tendon_length_mm, tendon_slack)
        self.publish_safety_state(motor_position_deg)

    def publish_continuum_state(self, tendon_length_mm, tendon_slack):
        tendon_angles = self.cfg["tendon"]["angle_deg"]
        section1 = self.cfg["sections"]["section1"]
        section2 = self.cfg["sections"]["section2"]

        radius1 = float(section1["tendon_radius_mm"])
        radius2 = float(section2["tendon_radius_mm"])

        section1_ids = [int(x) for x in section1["tendon_ids"]]
        section2_ids = [int(x) for x in section2["tendon_ids"]]

        section1_lengths = {
            tid: tendon_length_mm[tid - 1]
            for tid in section1_ids
        }
        section2_lengths = {
            tid: tendon_length_mm[tid - 1]
            for tid in section2_ids
        }

        section1_angles = {
            tid: float(tendon_angles[tid])
            for tid in section1_ids
        }
        section2_angles = {
            tid: float(tendon_angles[tid])
            for tid in section2_ids
        }

        section1_kx, section1_ky = compute_section_curvature(
            section1_lengths,
            section1_angles,
            radius1,
        )
        section2_kx, section2_ky = compute_section_curvature(
            section2_lengths,
            section2_angles,
            radius2,
        )

        msg = ContinuumState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "continuum_base"

        msg.section1_kx = float(section1_kx)
        msg.section1_ky = float(section1_ky)
        msg.section2_kx = float(section2_kx)
        msg.section2_ky = float(section2_ky)

        msg.tendon_length_mm = tendon_length_mm
        msg.tendon_slack = tendon_slack

        self.fill_pcc_model_fields(msg, tendon_length_mm)
        self.apply_orientation_fusion(msg)
        self.apply_vision_position_fusion(msg)

        self.continuum_pub.publish(msg)

    def publish_safety_state(self, motor_position_deg):
        now = self.get_clock().now()

        tendon_slack_detected = False

        imu_timeout = False
        if self.use_imu_correction:
            imu_timeout = not (
                self.imu_stream_valid("imu1", now)
                and self.imu_stream_valid("imu2", now)
            )

        motor_timeout = not self.motor_stream_valid(now)

        # runtime IMU zero 只对当前连续有效的 IMU/AHRS 会话成立。
        # 任一路 IMU 超时后，AHRS 重启可能改变 world/yaw 参考，
        # 因此必须作废旧 q_zero，恢复后要求重新标准零位标定。
        if (
            self.use_imu_correction
            and imu_timeout
            and self.imu_zero_ready()
        ):
            self.q_zero["imu1"] = None
            self.q_zero["imu2"] = None
            self.get_logger().warn(
                "IMU timeout detected: runtime IMU zero invalidated; "
                "call /continuum/set_imu_zero again after IMU recovery."
            )

        imu_zero_ready = self.imu_zero_ready()

        motor_limit_reached = False
        limits = self.cfg["motor"]["position_limit_deg"]

        for motor_id in self.motor_ids:
            tendon_id = int(self.motor_to_tendon[motor_id])
            index = tendon_id - 1
            low = float(limits[motor_id][0])
            high = float(limits[motor_id][1])
            pos = float(motor_position_deg[index])

            if pos <= low or pos >= high:
                motor_limit_reached = True
                break

        valid_residuals = [
            value
            for value in self.last_orientation_residual_deg.values()
            if value >= 0.0
        ]
        max_residual_deg = max(valid_residuals) if valid_residuals else -1.0

        residual_too_large = (
            max_residual_deg >= 0.0
            and max_residual_deg > self.residual_limit_deg
        )

        vision_extrinsic_ready = ((not self.vision_enabled) or self.last_vision_extrinsic_ready)
        vision_timeout = (self.vision_enabled and not self.last_vision_tip_valid)
        vision_position_residual_too_large = (
            self.vision_enabled
            and self.last_vision_position_residual_m >= 0.0
            and self.last_vision_position_residual_m > self.position_residual_limit_m
        )

        safe_to_control = not (
            self.emergency_stop_active
            or motor_timeout
            or imu_timeout
            or (self.use_imu_correction and not imu_zero_ready)
            or not self.last_pcc_model_valid
            or motor_limit_reached
            or tendon_slack_detected
            or residual_too_large
            or not vision_extrinsic_ready
            or vision_timeout
            or vision_position_residual_too_large
        )

        msg = SafetyState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "continuum_base"

        msg.safe_to_control = bool(safe_to_control)
        msg.tendon_slack_detected = bool(tendon_slack_detected)
        msg.motor_limit_reached = bool(motor_limit_reached)
        msg.imu_timeout = bool(imu_timeout)
        msg.motor_timeout = bool(motor_timeout)
        msg.imu_zero_ready = bool(imu_zero_ready)
        msg.pcc_model_valid = bool(self.last_pcc_model_valid)
        msg.residual_too_large = bool(residual_too_large)
        msg.emergency_stop_active = bool(self.emergency_stop_active)
        msg.max_orientation_residual_deg = float(max_residual_deg)
        msg.vision_extrinsic_ready = bool(vision_extrinsic_ready)
        msg.vision_timeout = bool(vision_timeout)
        msg.vision_position_residual_too_large = bool(vision_position_residual_too_large)
        msg.max_position_residual_m = float(self.last_vision_position_residual_m)

        unsafe_reasons = []

        unsafe_reasons = []
        if self.emergency_stop_active:
            unsafe_reasons.append("emergency stop active")
        if motor_timeout:
            unsafe_reasons.append("motor feedback timeout")
        if imu_timeout:
            unsafe_reasons.append("IMU timeout")
        if self.use_imu_correction and not imu_zero_ready:
            unsafe_reasons.append("runtime IMU zero not established")
        if not self.last_pcc_model_valid:
            unsafe_reasons.append("PCC model invalid")
        if motor_limit_reached:
            unsafe_reasons.append("motor limit reached")
        if tendon_slack_detected:
            unsafe_reasons.append("tendon slack detected")
        if residual_too_large:
            unsafe_reasons.append("PCC/IMU orientation residual too large")
        if not vision_extrinsic_ready:
            unsafe_reasons.append("vision base/camera extrinsic unavailable")
        if vision_timeout:
            unsafe_reasons.append("vision tip timeout")
        if vision_position_residual_too_large:
            unsafe_reasons.append("PCC/vision position residual too large")
        if unsafe_reasons:
            msg.status = "UNSAFE: " + "; ".join(unsafe_reasons)
        else:
            msg.status = "SAFE"

        self.safety_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
