#ROS2 节点，订阅 /motor/command、发布 /motor/state，调用ms42ddc_driver.py和 ms42ddc_driver.py
from __future__ import annotations
import rclpy
import json
import os
from rclpy.node import Node
from robot_interfaces.msg import MotorCommand, MotorCommandArray, MotorState
from .ms42ddc_driver import MS42DDCDriver
from robot_interfaces.srv import (SetZero, HomeMotors, EmergencyStop, ClearEmergencyStop)


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")
        self.estop_active = False	#False：当前没有急停，可以控制电机
        #declare_parameter：声明参数
        self.declare_parameter("interface", "socketcan")
        self.declare_parameter("primary_channel", "can0")
        self.declare_parameter("secondary_channel", "can1")
        self.declare_parameter("bitrate", 1000000)
        self.declare_parameter("feedback_id", 10)
        self.declare_parameter("microstep", 32)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("motor_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("secondary_ids", [4, 5, 6])
        self.declare_parameter("command_topic", "/motor/command")
        self.declare_parameter("command_array_topic", "/motor/command_array")
        self.declare_parameter("state_topic", "/motor/state")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("default_speed_rad_s", 3.0)
        self.declare_parameter("zero_tolerance_deg", 0.2)
        self.declare_parameter(
            "zero_file",
            "/tmp/ms42ddc_zero_offsets.json",
        )
        
        #get_parameter：读取参数
        self.interface = self.get_parameter("interface").value
        self.primary_channel = self.get_parameter("primary_channel").value
        self.secondary_channel = self.get_parameter("secondary_channel").value
        self.bitrate = int(self.get_parameter("bitrate").value)
        self.feedback_id = int(self.get_parameter("feedback_id").value)
        self.microstep = int(self.get_parameter("microstep").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.motor_ids = list(self.get_parameter("motor_ids").value)
        self.secondary_ids = list(self.get_parameter("secondary_ids").value)
        self.command_topic = self.get_parameter("command_topic").value
        self.command_array_topic = self.get_parameter("command_array_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.default_speed_rad_s = float(
            self.get_parameter("default_speed_rad_s").value
        )
        self.zero_tolerance_deg = float(
            self.get_parameter("zero_tolerance_deg").value
        )
        self.zero_file = self.get_parameter("zero_file").value
        
        #读取零点文件
        self.zero_offsets = {mid:0.0 for mid in self.motor_ids}
        if os.path.exists(self.zero_file):
            try:
                with open(self.zero_file, "r") as f:
                    data = json.load(f)          # <- 缩进到 with 内部
                for k, v in data.items():
                    self.zero_offsets[int(k)] = float(v)
                self.get_logger().info(f"Loaded zero offsets: {self.zero_offsets}")
            except Exception as exc:
                self.get_logger().warn(f"Failed to load zero offsets: {exc}")
        else:
            self.get_logger().info("No zero offset file found, using zeros.")

        #创建自定义服务
        self.set_zero_srv = self.create_service(SetZero, "/motor/set_zero", self.set_zero_callback)
        self.home_srv = self.create_service(HomeMotors, "/motor/home_motors", self.home_motors_callback)
        self.estop_srv = self.create_service(EmergencyStop, "/motor/emergency_stop", self.emergency_stop_callback)
        self.clear_estop_srv = self.create_service(ClearEmergencyStop, "/motor/clear_estop", self.clear_estop_callback)
        
        #创建底层驱动对象
        self.driver = MS42DDCDriver(
            interface=self.interface,
            primary_channel=self.primary_channel,
            secondary_channel=self.secondary_channel,
            bitrate=self.bitrate,
            feedback_id=self.feedback_id,
            microstep=self.microstep,
            timeout=self.timeout,
            motor_ids=self.motor_ids,
            secondary_ids=self.secondary_ids,
            zero_file=self.zero_file,
        )
        
        #打开 CAN
        try:
            self.driver.open()
            self.get_logger().info(
                f"Motor CAN opened: {self.primary_channel}, {self.secondary_channel}, "
                f"interface={self.interface}, bitrate={self.bitrate}"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to open CAN bus: {exc}")
            raise
        
        #创建单电机订阅器
        self.command_sub = self.create_subscription(
            MotorCommand,
            self.command_topic,
            self.command_callback,
            10,
        )

        #创建多电机订阅器
        self.command_array_sub = self.create_subscription(
            MotorCommandArray,
            self.command_array_topic,
            self.command_array_callback,
            10,
        )
        
        #创建状态发布器
        self.state_pub = self.create_publisher(
            MotorState,
            self.state_topic,
            10,
        )
        #创建定时器
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_state)

    #command_callback：单电机命令回调
    def command_callback(self, msg: MotorCommand):
        if self.estop_active:
            self.get_logger().warn(
                "Command rejected: emergency stop is active."
            )
            return
            
        motor_id = int(msg.motor_id)
        mode = int(msg.mode)
        target_deg = float(msg.target_deg)
        speed_rad_s = float(msg.speed_rad_s)
        
        if motor_id not in self.motor_ids:
            self.get_logger().warn(f"Unknown motor id: {motor_id}")
            return

        try:
            if mode == 1:
                # 速度模式：以后需要在 driver 中实现 send_speed()
                self.get_logger().warn(
                    "mode=1 speed mode is not implemented yet"
                )

            elif mode == 2:
                # 位置模式：当前已支持
                self.driver.send_position(
                    motor_id,
                    target_deg,
                    speed_rad_s
                )
                self.get_logger().info(
                    f"motor {motor_id}: position mode, "
                    f"target={target_deg:.3f} deg, speed={speed_rad_s:.3f} rad/s"
                )

            elif mode == 3:
                # 力矩/电流模式：以后需要在 driver 中实现 send_torque/current()
                self.get_logger().warn(
                    "mode=3 torque/current mode is not implemented yet"
                )

            elif mode == 4:
                # 单圈绝对角度模式：以后需要在 driver 中实现 send_absolute_position()
                self.get_logger().warn(
                    "mode=4 absolute position mode is not implemented yet"
                )

            else:
                self.get_logger().warn(
                    f"Unsupported mode: {mode}. "
                    "Supported modes: 1=speed, 2=position, 3=current, 4=absolute"
                )

        except Exception as exc:
            self.get_logger().warn(
                f"motor {motor_id} command failed: {exc}"
            )
            
    #command_array_callback：多电机命令回调
    def command_array_callback(self, msg: MotorCommandArray):
        if self.estop_active:
            self.get_logger().warn(
                "Command rejected: emergency stop is active."
            )
            return
            
        if not msg.commands:
            self.get_logger().warn("Received empty MotorCommandArray")
            return

        self.get_logger().info(f"Received MotorCommandArray with {len(msg.commands)} commands")

        for cmd in msg.commands:
            motor_id = int(cmd.motor_id)
            mode = int(cmd.mode)
            target_deg = float(cmd.target_deg)
            speed_rad_s = float(cmd.speed_rad_s)

            if motor_id not in self.motor_ids:
                self.get_logger().warn(f"Unknown motor id in array: {motor_id}")
                continue

            try:
                if mode == 2:
                    # 位置模式
                    self.driver.send_position(motor_id, target_deg, speed_rad_s)
                    self.get_logger().info(
                        f"array motor {motor_id}: target={target_deg:.3f} deg, "
                        f"speed={speed_rad_s:.3f} rad/s"
                    )
                else:
                    self.get_logger().warn(
                        f"Unsupported mode in array: {mode}. Currently only mode=2 is supported."
                    )

            except Exception as exc:
                self.get_logger().warn(
                    f"array motor {motor_id} command failed: {exc}"
                )

    #publish_state：发布电机状态
    def publish_state(self):
        for motor_id in self.motor_ids:
            try:
                status = self.driver.read_status(motor_id)
                rel_deg = self.driver.relative_position(status)

                msg = MotorState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "motor_base"

                msg.motor_id = int(status.motor_id)
                msg.raw_deg = float(status.raw_deg)
                msg.position_deg = float(rel_deg)
                msg.speed_rad_s = float(status.speed_rad_s)
                msg.reached = bool(status.reached)
                msg.channel = self.driver.channel_for_motor(motor_id)

                self.state_pub.publish(msg)

            except Exception as exc:
                self.get_logger().warn(f"read motor {motor_id} failed: {exc}")

    #set_zero_callback：设置零点服务
    def set_zero_callback(self, request, response):
        motors_to_set = request.motor_ids or self.motor_ids
        success_ids = []
        for mid in motors_to_set:
            try:
                status = self.driver.read_status(mid)
                self.zero_offsets[mid] = float(status.raw_deg)
                self.driver.zero_offsets[mid] = float(status.raw_deg)
                success_ids.append(mid)
            except Exception as exc:
                self.get_logger().warn(
                    f"set zero motor {mid} failed: {exc}"
                )
        if success_ids:
            with open(self.zero_file, "w") as f:
                json.dump(self.zero_offsets, f)
            response.success = True
            response.message = (
                f"Zero offsets saved for motors: {success_ids}"
            )
        else:
            response.success = False
            response.message = (
                "No motor feedback received."
            )
        return response
    
    #home_motors_callback：回零服务
    def home_motors_callback(self, request, response):
        if self.estop_active:
            response.success = False
            response.message = "Home rejected: emergency stop is active."
            self.get_logger().warn(response.message)
            return response
    
        motors_to_home = request.motor_ids or self.motor_ids
        speed = request.speed_rad_s or self.default_speed_rad_s
        for mid in motors_to_home:
            try:
                status = self.driver.read_status(mid)
                current_raw_deg = float(status.raw_deg)
                zero_deg = self.zero_offsets.get(mid, 0.0)
                # 考虑方向修正
                delta_deg = zero_deg - current_raw_deg
                self.driver.send_position(mid, delta_deg, speed)
            except Exception as exc:
                self.get_logger().warn(f"home motor {mid} failed: {exc}")
        response.success = True
        response.message =( f"Motors moved to zero positions: {motors_to_home}")
        return response

    #emergency_stop_callback：急停服务
    def emergency_stop_callback(self, request, response):
        self.estop_active = True

        try:
            self.driver.stop_all()
            response.success = True
            response.message = "Emergency stop activated. All motors stopped."
            self.get_logger().warn(response.message)

        except Exception as exc:
            response.success = False
            response.message = f"Emergency stop failed: {exc}"
            self.get_logger().error(response.message)

        return response

    #clear_estop_callback：清除急停服务
    def clear_estop_callback(self, request, response):
        self.estop_active = False

        response.success = True
        response.message = "Emergency stop cleared. Motor commands are enabled."
        self.get_logger().info(response.message)

        return response
	

    #destroy_node：关闭节点
    def destroy_node(self):
        try:
            self.driver.stop_all()
            self.driver.close()
        except Exception:
            pass

        super().destroy_node()
    
#main：程序入口
def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()  # 使用 try_shutdown 避免重复调用
        except Exception:
            pass


if __name__ == "__main__":
    main()
