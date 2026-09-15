#!/usr/bin/env python3
"""
基于 HSV 颜色分割的目标检测节点。

订阅 /camera/image_raw，检测世界中的彩色障碍物，
发布标注图像和检测结果，并在 RViz 中显示三维标记。
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# 颜色定义：名称 -> (HSV 下界列表, HSV 上界列表, BGR 画框颜色)
# 红色跨越 H=0，所以需要两段区间
COLOR_TABLE = {
    'red': ([(0, 120, 70), (170, 120, 70)],
            [(10, 255, 255), (179, 255, 255)], (0, 0, 255)),
    'blue': ([(100, 120, 70)], [(130, 255, 255)], (255, 0, 0)),
    'green': ([(40, 100, 70)], [(85, 255, 255)], (0, 255, 0)),
    'yellow': ([(20, 120, 100)], [(35, 255, 255)], (0, 255, 255)),
}


class ColorDetector(Node):
    """用 HSV 颜色分割检测仿真世界中的彩色障碍物。"""

    def __init__(self):
        super().__init__('color_detector')

        self.declare_parameter('min_area', 400)        # 最小连通域面积（像素）
        self.declare_parameter('publish_debug', True)
        self._min_area = self.get_parameter('min_area').value
        self._publish_debug = self.get_parameter('publish_debug').value

        self._bridge = CvBridge()
        self._depth = None       # 最近一帧深度图

        # 相机话题是传感器数据，用 SensorData QoS（BEST_EFFORT）
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._on_depth, qos_profile_sensor_data)

        self._img_pub = self.create_publisher(Image, '/detection/image', 10)
        self._txt_pub = self.create_publisher(String, '/detection/result', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/detection/markers', 10)

        self.get_logger().info(
            f'颜色检测器已启动，检测 {list(COLOR_TABLE)}，'
            f'最小面积 {self._min_area} 像素')

    # ---------------- 回调 ----------------

    def _on_depth(self, msg: Image):
        """缓存深度图，用于估计目标距离。"""
        try:
            self._depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().warn(f'深度图转换失败: {exc}', throttle_duration_sec=5.0)

    def _on_image(self, msg: Image):
        try:
            # Gazebo 发 rgb8，OpenCV 用 bgr8 —— 这里必须显式转换
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'图像转换失败: {exc}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        for name, (lowers, uppers, draw_bgr) in COLOR_TABLE.items():
            mask = self._build_mask(hsv, lowers, uppers)
            for box in self._find_boxes(mask):
                x, y, w, h = box
                dist = self._estimate_distance(x + w // 2, y + h // 2)
                detections.append((name, box, dist))
                self._draw(frame, name, box, dist, draw_bgr)

        self._publish(msg, frame, detections)

    # ---------------- 图像处理 ----------------

    @staticmethod
    def _build_mask(hsv, lowers, uppers):
        """按颜色区间生成二值掩码，并做形态学去噪。"""
        mask = None
        for lo, hi in zip(lowers, uppers):
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        # 开运算去除孤立噪点，闭运算填补目标内部空洞
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _find_boxes(self, mask):
        """从掩码里提取足够大的连通域外接矩形。"""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            if cv2.contourArea(c) < self._min_area:
                continue
            boxes.append(cv2.boundingRect(c))
        return boxes

    def _estimate_distance(self, cx, cy):
        """用深度图估计目标中心的距离（米）。取邻域中位数抗噪。"""
        if self._depth is None:
            return None
        h, w = self._depth.shape[:2]
        if not (0 <= cx < w and 0 <= cy < h):
            return None

        r = 5
        patch = self._depth[max(0, cy - r):cy + r, max(0, cx - r):cx + r]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    @staticmethod
    def _draw(frame, name, box, dist, color):
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        label = name if dist is None else f'{name} {dist:.2f}m'
        cv2.putText(frame, label, (x, max(15, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # ---------------- 发布 ----------------

    def _publish(self, src_msg, frame, detections):
        if self._publish_debug:
            out = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = src_msg.header
            self._img_pub.publish(out)

        if detections:
            parts = []
            for name, (x, y, w, h), dist in detections:
                d = 'unknown' if dist is None else f'{dist:.2f}m'
                parts.append(f'{name}@({x + w // 2},{y + h // 2}) {d}')
            text = ' | '.join(parts)
            self._txt_pub.publish(String(data=text))
            self.get_logger().info(f'检测到 {len(detections)} 个目标: {text}',
                                   throttle_duration_sec=2.0)

        self._publish_markers(src_msg, detections)

    def _publish_markers(self, src_msg, detections):
        """在 RViz 中用球体标记检测到的目标（粗略位置）。"""
        arr = MarkerArray()

        clear = Marker()
        clear.header = src_msg.header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, (name, (x, y, w, h), dist) in enumerate(detections):
            if dist is None:
                continue
            m = Marker()
            m.header = src_msg.header      # frame 是 camera_depth_optical_frame
            m.ns = 'detections'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            # 光学坐标系：z 向前、x 向右、y 向下
            m.pose.position = Point(x=0.0, y=0.0, z=dist)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.a = 0.8
            m.color.r = 1.0 if name in ('red', 'yellow') else 0.0
            m.color.g = 1.0 if name in ('green', 'yellow') else 0.0
            m.color.b = 1.0 if name == 'blue' else 0.0
            arr.markers.append(m)

        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()