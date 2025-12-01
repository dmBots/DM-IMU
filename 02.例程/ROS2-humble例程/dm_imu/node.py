import math
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, PoseStamped

# 使用你的串口实现（保持路径）
from .modules.dm_serial import DM_Serial


def euler_rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """ZYX intrinsic (yaw->pitch->roll); roll/pitch/yaw in radians."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class DmImuNode(Node):
    def __init__(self):
        super().__init__('dm_imu')

        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rpy_in_degree', True) # 若希望 /imu/rpy 以“度”发布，把 False 改 True
        self.declare_parameter('verbose', True)          # 终端打印
        self.declare_parameter('qos_reliable', True)     # 发布端 QoS（默认 Reliable，RViz 直接可见）
        # 新增：三个话题的开关（默认全开）
        self.declare_parameter('publish_imu_data', False)  # /imu/data
        self.declare_parameter('publish_rpy', True)       # /imu/rpy
        self.declare_parameter('publish_pose', False)      # /imu/pose

        def _p(name, default=None):
            try:
                v = self.get_parameter(name).value
                return default if v in (None, '') else v
            except Exception:
                return default

        self.port = _p('port', '/dev/ttyACM0')
        self.frame_id = _p('frame_id', 'imu_link')
        self.publish_rpy = bool(_p('publish_rpy', True))
        self.publish_rpy_in_degree = bool(_p('publish_rpy_in_degree', True))
        self.verbose = bool(_p('verbose', True))
        qos_reliable = bool(_p('qos_reliable', True))
        # 新增：三个话题的总开关
        self.publish_imu_data = bool(_p('publish_imu_data', False))
        self.publish_rpy = bool(_p('publish_rpy', True))
        self.publish_pose = bool(_p('publish_pose', False))

        baud = _p('baudrate', 921600)
        try:
            self.baudrate = int(baud)
        except (TypeError, ValueError):
            self.get_logger().warn(f'Invalid baudrate "{baud}", fallback to 921600')
            self.baudrate = 921600

        # ---------- QoS ----------
        if qos_reliable:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                durability=DurabilityPolicy.VOLATILE,
            )
        else:
            from rclpy.qos import qos_profile_sensor_data  # BestEffort
            qos = qos_profile_sensor_data

        # ---------- Publishers ----------
        self.pub_imu  = self.create_publisher(Imu, 'imu/data', qos)                 if self.publish_imu_data else None
        self.pub_rpy  = self.create_publisher(Vector3Stamped, 'imu/rpy', qos)       if self.publish_rpy      else None
        self.pub_pose = self.create_publisher(PoseStamped, 'imu/pose', 10)          if self.publish_pose     else None

        # ---------- Serial ----------
        try:
            # 按你的要求：初始化不传 timeout
            self.ser = DM_Serial(self.port, baudrate=self.baudrate)
            self.ser.start_reader()  # 后台读线程交给你的类
            self.get_logger().info(f'Opened serial {self.port} @ {self.baudrate}')
        except Exception as e:
            self.get_logger().fatal(f'Init serial failed: {e}')
            raise

        # ---------- Timers ----------
        # 用时间戳做去重；拿不到就不去重
        self._last_stamp_ts: Optional[float] = None
        self._closing = threading.Event()
        self._logged_bad_fmt_once = False
        self._no_frame_ticks = 0
        self._pub_count = 0

        # 200 Hz 轮询
        self.timer_pub = self.create_timer(0.005, self._on_timer_publish)
        # 每 2s 打一次统计（若你的类提供 get_stats）
        self.timer_stat = self.create_timer(2.0, self._on_timer_stats)

    # ----------- Timers -----------
    def _on_timer_publish(self):
        try:
            latest = self.ser.get_latest()
        except Exception as e:
            if self.verbose:
                self.get_logger().warn(f'get_latest() exception: {e}')
            return

        if latest is None:
            self._no_frame_ticks += 1
            if self._no_frame_ticks % 200 == 0 and self.verbose:  # ≈每秒一次
                self.get_logger().warn('No frames yet from serial (≈1s). Check IMU streaming/baud/crc.')
            return

        ok, stamp_ts, r_deg, p_deg, y_deg = self._extract_latest(latest)  # 提取数据!!
        if not ok:
            if not self._logged_bad_fmt_once and self.verbose:
                self.get_logger().warn(f'Unknown latest frame format; example: {repr(latest)}')
                self._logged_bad_fmt_once = True
            return

        # 去重（优先用时间戳；没有就不去重）
        if stamp_ts is not None and stamp_ts == self._last_stamp_ts:
            return
        self._last_stamp_ts = stamp_ts

        # 度→弧度
        r = r_deg * math.pi / 180.0
        p = p_deg * math.pi / 180.0
        y = y_deg * math.pi / 180.0

        stamp = self.get_clock().now().to_msg()

        # /imu/data
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        qx, qy, qz, qw = euler_rpy_to_quat(r, p, y)

        # /imu/rpy
        if self.pub_rpy is not None:
            rpy_msg = Vector3Stamped()
            rpy_msg.header.stamp = stamp
            rpy_msg.header.frame_id = self.frame_id
            if self.publish_rpy_in_degree:
                rpy_msg.vector.x = float(r_deg)
                rpy_msg.vector.y = float(p_deg)
                rpy_msg.vector.z = float(y_deg)
            else:
                rpy_msg.vector.x = float(r)
                rpy_msg.vector.y = float(p)
                rpy_msg.vector.z = float(y)
            # /imu/rpy
            self.pub_rpy.publish(rpy_msg)

        if self.publish_imu_data == False and self.publish_pose == False:
            return
        # 有效性检查 + 归一化（防 Unvisualizable）
        def _finite(*vals):
            return all(math.isfinite(v) for v in vals)

        if not _finite(qx, qy, qz, qw):
            if self.verbose:
                self.get_logger().warn('Quaternion has NaN/Inf, publishing identity (0,0,0,1)')
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            n = (qx*qx + qy*qy + qz*qz + qw*qw) ** 0.5
            if n < 1e-6:
                if self.verbose:
                    self.get_logger().warn('Quaternion norm ~0, publishing identity (0,0,0,1)')
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

        # /imu/data
        if self.pub_imu is not None:
            imu.orientation.x, imu.orientation.y = qx, qy
            imu.orientation.z, imu.orientation.w = qz, qw
            imu.orientation_covariance[0] = 0.02
            imu.orientation_covariance[4] = 0.02
            imu.orientation_covariance[8] = 0.02
            for i in range(9):
                imu.angular_velocity_covariance[i] = -1.0
                imu.linear_acceleration_covariance[i] = -1.0
            self.pub_imu.publish(imu)

        # /imu/pose（原点+IMU姿态），便于 RViz 直接看 Pose
        if self.pub_pose is not None:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pub_pose.publish(pose)

        # 终端打印
        self._pub_count += 1
        self._no_frame_ticks = 0
        if self.verbose:
            self.get_logger().info(
                f'#{self._pub_count} RPY(deg)=({r_deg:.2f}, {p_deg:.2f}, {y_deg:.2f}) '
                f'RPY(rad)=({r:.3f}, {p:.3f}, {y:.3f})'
            )

    def _on_timer_stats(self):
        try:
            if hasattr(self.ser, 'get_stats'):
                stats = self.ser.get_stats()
                msg = " ".join([f"{k}={v}" for k, v in stats.items()]) if isinstance(stats, dict) else str(stats)
                if self.verbose:
                    self.get_logger().info(f'[stats] {msg}')
        except Exception:
            pass  # 统计异常不影响主流程

    # ----------- Helpers -----------
    def _extract_latest(self, latest) -> Tuple[bool, Optional[float], float, float, float]:
        """
        返回 (ok, stamp_ts, roll_deg, pitch_deg, yaw_deg)
        - 兼容你的嵌套：((rid, (r,p,y)), ts, extra)
        - 也兼容 dict / 扁平 tuple / 对象字段
        """
        try:
            # dict
            if isinstance(latest, dict):
                r = latest.get('roll') or latest.get('r') or latest.get('Roll')
                p = latest.get('pitch') or latest.get('p') or latest.get('Pitch')
                y = latest.get('yaw') or latest.get('y') or latest.get('Yaw')
                ts = latest.get('ts') or latest.get('timestamp') or latest.get('time') or None
                if r is not None and p is not None and y is not None:
                    return True, (float(ts) if ts is not None else None), float(r), float(p), float(y)

            # tuple/list
            if isinstance(latest, (tuple, list)):
                # ((rid, (r,p,y)), ts, extra)
                if len(latest) >= 2 and isinstance(latest[0], (tuple, list)):
                    rid_part = latest[0]
                    ts = latest[1] if isinstance(latest[1], (int, float)) else None
                    if len(rid_part) == 2 and isinstance(rid_part[1], (tuple, list)) and len(rid_part[1]) >= 3:
                        r, p, y = rid_part[1][0], rid_part[1][1], rid_part[1][2]
                        return True, (float(ts) if ts is not None else None), float(r), float(p), float(y)
                # (rid, r, p, y)
                if len(latest) >= 4 and not isinstance(latest[0], (tuple, list)):
                    _, r, p, y = latest[0], latest[1], latest[2], latest[3]
                    return True, None, float(r), float(p), float(y)
                # (r, p, y)
                if len(latest) == 3:
                    r, p, y = latest[0], latest[1], latest[2]
                    return True, None, float(r), float(p), float(y)

            # 对象字段
            r = getattr(latest, 'roll', None)
            p = getattr(latest, 'pitch', None)
            y = getattr(latest, 'yaw', None)
            ts = getattr(latest, 'ts', None) or getattr(latest, 'timestamp', None) or None
            if r is not None and p is not None and y is not None:
                return True, (float(ts) if ts is not None else None), float(r), float(p), float(y)

            return False, None, 0.0, 0.0, 0.0
        except Exception as e:
            if self.verbose:
                self.get_logger().debug(f'_extract_latest exception: {e}')
            return False, None, 0.0, 0.0, 0.0

    # ----------- Shutdown -----------
    def destroy_node(self):
        if getattr(self, '_closing', None) is None or self._closing.is_set():
            try:
                super().destroy_node()
            except Exception:
                pass
            return
        self._closing.set()
        try:
            if hasattr(self.ser, 'stop_reader'):
                self.ser.stop_reader()
        except Exception:
            pass
        try:
            if hasattr(self.ser, 'close'):
                self.ser.close()
        except Exception:
            pass
        try:
            super().destroy_node()
        except Exception:
            pass


def main():
    rclpy.init()
    node = DmImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
