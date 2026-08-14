import math
import yaml
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from robot_interfaces.msg import MotorState, TendonState, ContinuumState, SafetyState, ModelResidual
from continuum_model.pcc_model import compute_section_curvature
from continuum_pccmodel.geometry import load_pcc_geometry_from_cfg
from continuum_pccmodel.forward_kinematics import forward_kinematics_from_dl_mm
from std_msgs.msg import Bool

def quaternion_to_euler_deg(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__("continuum_state_estimator")

        self.declare_parameter(
            "calibration_file",
            "/home/wangwenfeng/ros_robot1_ws/src/robot_bringup/config/robot_calibration.yaml",
        )

        self.calibration_file = self.get_parameter("calibration_file").value

        self.motor_position_deg = {}
        self.imu1_euler = None
        self.imu2_euler = None

        self.load_calibration()
        
        #SafetyState参数限制范围
        self.residual_limit_deg = 10.0  #残差安全阀值：10度
        self.residual_valid = False
        self.emergency_stop_active = False
        self.last_imu1_time = None      #IMU1最后接收时间（判断是否掉线）
        self.last_imu2_time = None      #IMU2最后接收时间
        self.imu_timeout_sec = 1.0      #IMU超时时间：1s

        self.tendon_pub = self.create_publisher(
            TendonState,
            "/continuum/tendon_state",
            10,
        )

        self.continuum_pub = self.create_publisher(
            ContinuumState,
            "/continuum/state",
            10,
        )

        self.safety_pub = self.create_publisher(
            SafetyState,
            "/continuum/safety_state",
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
            "/imu1/data_raw",
            self.imu1_callback,
            10,
        )

        self.imu2_sub = self.create_subscription(
            Imu,
            "/imu2/data_raw",
            self.imu2_callback,
            10,
        )

        self.estop_sub = self.create_subscription(
            Bool,
            "/motor/emergency_stop_active",
            self.estop_callback,
            10,
        )
        
        self.residual_sub = self.create_subscription(
            ModelResidual,
            "/continuum/model_residual",
            self.residual_callback,
            10,
        )
        
        self.timer = self.create_timer(0.02, self.publish_states)

        self.get_logger().info("continuum_state_estimator started")

    def load_calibration(self):
        with open(self.calibration_file, "r") as f:
            cfg = yaml.safe_load(f)

        self.cfg = cfg

        self.motor_ids = cfg["motor"]["motor_ids"]
        self.sign = cfg["motor"]["sign"]
        self.spool_radius_mm = cfg["motor"]["spool_radius_mm"]
        self.motor_to_tendon = cfg["tendon"]["motor_to_tendon"]
        self.slack_threshold_mm = float(cfg["tendon"]["slack_threshold_mm"])
        self.imu_offset = cfg["imu"]["mounting_offset_deg"]
        # PCC 正运动学模型参数
        pcc_cfg = cfg.get("pcc_model", {})
        self.pcc_enabled = bool(pcc_cfg.get("enabled", False))
        self.pcc_geometry = load_pcc_geometry_from_cfg(cfg) if self.pcc_enabled else None

        # 真实结构顺序：基座 -> section2 -> section1 -> 末端
        self.pcc_chain_order = list(pcc_cfg.get("chain_order", ["section2", "section1"]))
        self.pcc_sections_cfg = pcc_cfg.get("sections", {})
        for mid in self.motor_ids:
            self.motor_position_deg[int(mid)] = 0.0

        self.get_logger().info(f"Loaded calibration file: {self.calibration_file}")

    def motor_callback(self, msg: MotorState):
        self.motor_position_deg[int(msg.motor_id)] = float(msg.position_deg)

    def imu1_callback(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)

        offset = self.imu_offset["imu1"]

        self.imu1_euler = (
            roll - float(offset["roll"]),
            pitch - float(offset["pitch"]),
            yaw - float(offset["yaw"]),
        )

        self.last_imu1_time = self.get_clock().now()


    def imu2_callback(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)

        offset = self.imu_offset["imu2"]

        self.imu2_euler = (
            roll - float(offset["roll"]),
            pitch - float(offset["pitch"]),
            yaw - float(offset["yaw"]),
        )

        self.last_imu2_time = self.get_clock().now()

    def residual_callback(self, msg: ModelResidual):
        self.residual_valid = bool(msg.residual_valid)

        self.max_residual_deg = max(
            abs(float(msg.section1_roll_residual)),
            abs(float(msg.section1_pitch_residual)),
            abs(float(msg.section2_roll_residual)),
            abs(float(msg.section2_pitch_residual)),
        )
    
    def estop_callback(self, msg: Bool):
        self.emergency_stop_active = bool(msg.data)
    
    def compute_tendon_state(self):
        tendon_length_mm = [0.0] * 6
        motor_position_deg = [0.0] * 6
        tendon_slack = [False] * 6

        for motor_id in self.motor_ids:
            motor_id = int(motor_id)

            pos_deg = self.motor_position_deg.get(motor_id, 0.0)
            theta_rad = math.radians(pos_deg)

            sign = float(self.sign[motor_id])
            radius = float(self.spool_radius_mm[motor_id])

            delta_l = sign * radius * theta_rad

            tendon_id = int(self.motor_to_tendon[motor_id])
            index = tendon_id - 1

            tendon_length_mm[index] = float(delta_l)
            motor_position_deg[index] = float(pos_deg)
            tendon_slack[index] = abs(delta_l) < self.slack_threshold_mm

        return tendon_length_mm, motor_position_deg, tendon_slack

    def build_pcc_input_from_tendon_lengths(self, tendon_length_mm):
        """
        根据 YAML 中的 pcc_model.sections 映射，把真实绳长变化组装成 PCC 模型输入。

        最终 YAML 中：
            section2: [DL1, DL2, DL3] = [绳4, 绳2, 绳6]
            section1: [DL1, DL2, DL3] = [绳5, 绳3, 绳1]

        计算顺序由 chain_order 决定：
            [section2, section1]
        """
        dl_sections_mm = []
        phase_offsets_rad = []
        dl_by_section = {}

        for section_name in self.pcc_chain_order:
            section_cfg = self.pcc_sections_cfg[section_name]

            tendon_ids = [int(x) for x in section_cfg["tendon_ids"]]
            dl_signs = [
                float(x)
                for x in section_cfg.get("dl_signs", [1.0] * len(tendon_ids))
            ]

            if len(tendon_ids) != 3:
                raise ValueError(f"{section_name} tendon_ids must contain 3 tendons")

            if len(dl_signs) != 3:
                raise ValueError(f"{section_name} dl_signs must contain 3 values")

            dl_mm = []

            for tendon_id, dl_sign in zip(tendon_ids, dl_signs):
                # tendon_length_mm 按真实绳号 1~6 存储，所以索引是 tendon_id - 1
                dl_value_mm = float(tendon_length_mm[tendon_id - 1]) * dl_sign
                dl_mm.append(dl_value_mm)

            dl_sections_mm.append(dl_mm)
            phase_offsets_rad.append(float(section_cfg.get("phase_offset_rad", 0.0)))
            dl_by_section[section_name] = dl_mm

        return dl_sections_mm, phase_offsets_rad, dl_by_section

    def fill_pcc_model_fields(self, msg: ContinuumState, tendon_length_mm):
        """
        调用 continuum_pccmodel 正运动学，并把结果写入 ContinuumState 新增字段。
        """
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

            # fk["section_poses"] 的顺序和 self.pcc_chain_order 一致
            pose_by_section = {}

            for i, section_name in enumerate(self.pcc_chain_order):
                pose_by_section[section_name] = fk["section_poses"][i]

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

        except Exception as e:
            msg.pcc_model_valid = False
            self.get_logger().warn(f"PCC model update failed: {e}")

    def publish_states(self):
        tendon_length_mm, motor_position_deg, tendon_slack = self.compute_tendon_state()

        tendon_msg = TendonState()
        tendon_msg.header.stamp = self.get_clock().now().to_msg()
        tendon_msg.header.frame_id = "continuum_base"
        tendon_msg.tendon_length_mm = tendon_length_mm
        tendon_msg.motor_position_deg = motor_position_deg
        tendon_msg.tendon_slack = tendon_slack
        self.tendon_pub.publish(tendon_msg)
        self.publish_continuum_state(tendon_length_mm, tendon_slack)
        self.publish_safety_state(tendon_length_mm, motor_position_deg, tendon_slack)

    def publish_continuum_state(self, tendon_length_mm, tendon_slack):
        # 即使 IMU 暂时没有数据，也继续发布 PCC 模型状态，
        # 这样 MuJoCo viewer 可以只依赖电机反馈/绳长变化实时显示。
        # IMU 未连接时，IMU 欧拉角字段暂时填 0.0。

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

        if self.imu1_euler is None:
            imu1_roll, imu1_pitch, imu1_yaw = 0.0, 0.0, 0.0
        else:
            imu1_roll, imu1_pitch, imu1_yaw = self.imu1_euler

        if self.imu2_euler is None:
            imu2_roll, imu2_pitch, imu2_yaw = 0.0, 0.0, 0.0
        else:
            imu2_roll, imu2_pitch, imu2_yaw = self.imu2_euler

        msg = ContinuumState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "continuum_base"

        msg.section1_kx = float(section1_kx)
        msg.section1_ky = float(section1_ky)

        msg.section2_kx = float(section2_kx)
        msg.section2_ky = float(section2_ky)

        msg.imu1_roll = float(imu1_roll)
        msg.imu1_pitch = float(imu1_pitch)
        msg.imu1_yaw = float(imu1_yaw)

        msg.imu2_roll = float(imu2_roll)
        msg.imu2_pitch = float(imu2_pitch)
        msg.imu2_yaw = float(imu2_yaw)

        msg.tendon_length_mm = tendon_length_mm
        msg.tendon_slack = tendon_slack

        self.fill_pcc_model_fields(msg, tendon_length_mm)

        self.continuum_pub.publish(msg)

    def publish_safety_state(self, tendon_length_mm, motor_position_deg, tendon_slack):
        now = self.get_clock().now()

        tendon_slack_detected = False

        imu_timeout = False
        if self.last_imu1_time is None or self.last_imu2_time is None:
            imu_timeout = True
        else:
            dt1 = (now - self.last_imu1_time).nanoseconds / 1e9
            dt2 = (now - self.last_imu2_time).nanoseconds / 1e9
            imu_timeout = dt1 > self.imu_timeout_sec or dt2 > self.imu_timeout_sec

        motor_limit_reached = False
        try:
            limits = self.cfg["motor"]["position_limit_deg"]
            for motor_id in self.motor_ids:
                motor_id = int(motor_id)
                tendon_id = int(self.motor_to_tendon[motor_id])
                index = tendon_id - 1

                low = float(limits[motor_id][0])
                high = float(limits[motor_id][1])
                pos = float(motor_position_deg[index])

                if pos <= low or pos >= high:
                    motor_limit_reached = True
                    break
        except Exception:
            motor_limit_reached = False

        residual_too_large = (
            self.residual_valid
            and self.max_residual_deg > self.residual_limit_deg
        )

        emergency_stop_active = self.emergency_stop_active

        safe_to_control = not (
            tendon_slack_detected
            or imu_timeout
            or motor_limit_reached
            or residual_too_large
            or emergency_stop_active
        )

        msg = SafetyState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "continuum_base"

        msg.safe_to_control = bool(safe_to_control)
        msg.tendon_slack_detected = bool(tendon_slack_detected)
        msg.imu_timeout = bool(imu_timeout)
        msg.motor_limit_reached = bool(motor_limit_reached)
        msg.residual_too_large = bool(residual_too_large)
        msg.emergency_stop_active = bool(emergency_stop_active)

        if safe_to_control:
            msg.status = "SAFE"
        elif imu_timeout:
            msg.status = "UNSAFE: IMU timeout"
        elif motor_limit_reached:
            msg.status = "UNSAFE: motor limit reached"
        elif tendon_slack_detected:
            msg.status = "UNSAFE: tendon slack detected"
        elif residual_too_large:
            msg.status = "UNSAFE: residual too large"
        elif emergency_stop_active:
            msg.status = "UNSAFE: emergency stop active"
        else:
            msg.status = "UNSAFE"

        self.safety_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
