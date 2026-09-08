#!/usr/bin/env python3
"""
多目标点自主导航节点。

从 YAML 配置读取一串航点，依次通过 Nav2 的 navigate_to_pose 动作
接口发送，等待每一个到达后再发下一个，并输出完整的状态日志。
"""

import math
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float):
    """把绕 z 轴的偏航角（弧度）转成四元数 (z, w) 分量。

    绕单一轴 z 旋转时，四元数简化为:
        q = (0, 0, sin(yaw/2), cos(yaw/2))
    x 和 y 分量恒为 0，所以只返回 z 和 w。
    """
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WaypointNavigator(Node):
    """依次导航到一组预设航点。"""

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('waypoints_file', '')
        wp_file = self.get_parameter('waypoints_file').value
        if not wp_file:
            raise ValueError(
                '必须通过参数 waypoints_file 指定航点配置文件路径')

        self._config = self._load_config(wp_file)
        self._waypoints = self._config['waypoints']
        self._frame_id = self._config.get('frame_id', 'map')
        self._goal_timeout = float(self._config.get('goal_timeout_sec', 180.0))
        self._settle_time = float(self._config.get('settle_time_sec', 1.0))

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._last_feedback_log = 0.0

        self.get_logger().info(
            f'已加载 {len(self._waypoints)} 个航点，坐标系 [{self._frame_id}]')

    # ---------------- 配置加载 ----------------

    def _load_config(self, path: str) -> dict:
        """读取并校验 YAML 航点配置。"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f'航点配置文件不存在: {path}')
        except yaml.YAMLError as exc:
            raise ValueError(f'航点配置 YAML 解析失败: {exc}')

        if not isinstance(cfg, dict) or 'waypoints' not in cfg:
            raise ValueError('配置文件缺少顶层 waypoints 字段')

        wps = cfg['waypoints']
        if not isinstance(wps, list) or not wps:
            raise ValueError('waypoints 必须是非空列表')

        for i, wp in enumerate(wps):
            for key in ('x', 'y'):
                if key not in wp:
                    raise ValueError(f'第 {i + 1} 个航点缺少字段 "{key}"')
            wp.setdefault('yaw', 0.0)
            wp.setdefault('name', f'waypoint_{i + 1}')

        return cfg

    # ---------------- 动作交互 ----------------

    def wait_for_nav2(self, timeout_sec: float = 60.0) -> bool:
        """等待 Nav2 的 navigate_to_pose 动作服务端上线。"""
        self.get_logger().info('等待 Nav2 动作服务端 navigate_to_pose ...')
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(
                f'{timeout_sec:.0f} 秒内未等到 Nav2。'
                '请确认导航栈已启动且各节点处于 active 状态。')
            return False
        self.get_logger().info('Nav2 动作服务端已就绪')
        return True

    def _build_goal(self, wp: dict) -> NavigateToPose.Goal:
        """把一个航点字典转换成 NavigateToPose 目标消息。"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        goal.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(float(wp['yaw']))
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal

    def _on_feedback(self, feedback_msg):
        """限流打印导航反馈，避免刷屏。"""
        now = time.monotonic()
        if now - self._last_feedback_log < 2.0:
            return
        self._last_feedback_log = now

        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  剩余距离 {fb.distance_remaining:.2f} m | '
            f'已用时 {fb.navigation_time.sec} s | '
            f'恢复行为触发 {fb.number_of_recoveries} 次')

    def _spin_until(self, future, timeout_sec: float) -> bool:
        """自旋等待 future 完成，返回是否在超时前完成。"""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def go_to_waypoint(self, wp: dict, index: int, total: int) -> bool:
        """发送单个目标点并阻塞等待结果。成功返回 True。"""
        name = wp['name']
        self.get_logger().info(
            f'[{index}/{total}] 前往 "{name}" '
            f'-> x={wp["x"]:.2f}, y={wp["y"]:.2f}, yaw={wp["yaw"]:.2f}')

        goal = self._build_goal(wp)

        # 第一步：发送目标，等待服务端接受或拒绝
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        if not self._spin_until(send_future, timeout_sec=10.0):
            self.get_logger().error(f'  "{name}": 发送目标超时')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'  "{name}": 目标被 Nav2 拒绝。'
                '通常是目标点落在障碍物或膨胀区内。')
            return False

        self.get_logger().info(f'  "{name}": 目标已接受，导航中 ...')

        # 第二步：等待最终结果（这才是"走到了没有"）
        result_future = goal_handle.get_result_async()
        if not self._spin_until(result_future, timeout_sec=self._goal_timeout):
            self.get_logger().error(
                f'  "{name}": 超过 {self._goal_timeout:.0f} 秒未完成，取消目标')
            cancel_future = goal_handle.cancel_goal_async()
            self._spin_until(cancel_future, timeout_sec=5.0)
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'  ✓ "{name}": 已到达')
            return True

        reason = {
            GoalStatus.STATUS_ABORTED: '导航失败（规划或控制中止）',
            GoalStatus.STATUS_CANCELED: '目标被取消',
        }.get(status, f'未知状态码 {status}')
        self.get_logger().error(f'  ✗ "{name}": {reason}')
        return False

    # ---------------- 主流程 ----------------

    def run(self) -> int:
        """依次访问全部航点，返回失败数量。"""
        if not self.wait_for_nav2():
            return len(self._waypoints)

        total = len(self._waypoints)
        succeeded, failed = [], []

        for i, wp in enumerate(self._waypoints, start=1):
            ok = self.go_to_waypoint(wp, i, total)
            (succeeded if ok else failed).append(wp['name'])

            if not ok:
                self.get_logger().warn(
                    f'  跳过 "{wp["name"]}"，继续下一个目标点')

            # 停顿一下让机器人稳定，避免上一个目标的残余速度影响下一次规划
            if i < total:
                end = time.monotonic() + self._settle_time
                while rclpy.ok() and time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info('=' * 52)
        self.get_logger().info(f'导航任务结束：成功 {len(succeeded)}/{total}')
        if succeeded:
            self.get_logger().info(f'  成功: {", ".join(succeeded)}')
        if failed:
            self.get_logger().warn(f'  失败: {", ".join(failed)}')
        self.get_logger().info('=' * 52)
        return len(failed)


def main(args=None):
    rclpy.init(args=args)

    node = None
    exit_code = 0
    try:
        node = WaypointNavigator()
        exit_code = 1 if node.run() > 0 else 0
    except (ValueError, FileNotFoundError) as exc:
        print(f'[waypoint_navigator] 配置错误: {exc}', file=sys.stderr)
        exit_code = 2
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('收到中断信号，退出')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()