"""DexHand021S 手部控制 Action Client 示例."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hands_control_interfaces.action import HandControl, ResetHand


class HandControlClient(Node):
    """灵巧手控制客户端."""

    def __init__(self):
        """初始化客户端节点."""
        super().__init__('hand_control_client')

        self._hand_control_clients = {
            0: ActionClient(self, HandControl, '/left/hand_control'),
            1: ActionClient(self, HandControl, '/right/hand_control'),
        }
        self._reset_hand_clients = {
            0: ActionClient(self, ResetHand, '/left/reset_hand'),
            1: ActionClient(self, ResetHand, '/right/reset_hand'),
        }

    def _get_clients(self, adapter_index):
        """根据 adapter_index 获取对应的 action clients."""
        if adapter_index not in [0, 1]:
            raise ValueError(f'无效的 adapter_index: {adapter_index} (只支持 0 或 1)')
        return self._hand_control_clients[adapter_index], self._reset_hand_clients[adapter_index]

    def send_hand_control_goal(self, adapter_index, finger_id, position, speed=600, force=0x55, wait_time=10):
        """发送手部控制目标."""
        hand_client, _ = self._get_clients(adapter_index)
        goal_msg = HandControl.Goal()
        goal_msg.adapter_index = adapter_index
        goal_msg.finger_id = finger_id
        goal_msg.position = position
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.wait_time = wait_time

        hand_name = '左手' if adapter_index == 0 else '右手'
        self.get_logger().info(
            f'发送控制目标: {hand_name} - 手指 {finger_id}, '
            f'位置={position}, 速度={speed}'
        )

        hand_client.wait_for_server()

        self._send_goal_future = hand_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_reset_goal(self, adapter_index):
        """发送手部重置目标."""
        _, reset_client = self._get_clients(adapter_index)
        goal_msg = ResetHand.Goal()
        goal_msg.adapter_index = adapter_index

        hand_name = '左手' if adapter_index == 0 else '右手'
        self.get_logger().info(f'发送重置目标: {hand_name}')

        reset_client.wait_for_server()

        self._send_goal_future = reset_client.send_goal_async(
            goal_msg,
            feedback_callback=self.reset_feedback_callback
        )

        self._send_goal_future.add_done_callback(self.reset_goal_response_callback)

    def goal_response_callback(self, future):
        """处理目标响应."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return

        self.get_logger().info('目标已接受')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def reset_goal_response_callback(self, future):
        """处理重置目标响应."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('重置目标被拒绝')
            return

        self.get_logger().info('重置目标已接受')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_reset_result_callback)

    def get_result_callback(self, future):
        """获取执行结果."""
        result = future.result().result
        self.get_logger().info(
            f'结果: {"成功" if result.success else "失败"} - {result.message}'
        )
        self.get_logger().info(f'最终位置: {result.final_positions}')

    def get_reset_result_callback(self, future):
        """获取重置结果."""
        result = future.result().result
        self.get_logger().info(
            f'重置结果: {"成功" if result.success else "失败"} - {result.message}'
        )

    def feedback_callback(self, feedback_msg):
        """接收反馈信息."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'反馈: 进度={feedback.progress:.1%}, '
            f'当前位置={feedback.current_positions}'
        )

    def reset_feedback_callback(self, feedback_msg):
        """接收重置反馈."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'重置反馈: {feedback.status}')


def main(args=None):
    """主函数 - 演示控制流程."""
    rclpy.init(args=args)
    client = HandControlClient()

    try:
        # 示例 1: 重置左手
        client.get_logger().info('=== 示例 1: 重置左手 ===')
        client.send_reset_goal(adapter_index=0)
        rclpy.spin_once(client, timeout_sec=3.0)

        # 示例 2: 左手手指 1 移动到位置 1000
        client.get_logger().info('\n=== 示例 2: 左手手指 1 移动 ===')
        client.send_hand_control_goal(
            adapter_index=0,
            finger_id=0x01,
            position=1000,
            speed=600
        )
        rclpy.spin_once(client, timeout_sec=3.0)

        # 示例 3: 右手手指 2 移动到位置 800
        client.get_logger().info('\n=== 示例 3: 右手手指 2 移动 ===')
        client.send_hand_control_goal(
            adapter_index=1,
            finger_id=0x02,
            position=800,
            speed=500
        )
        rclpy.spin_once(client, timeout_sec=3.0)

        # 示例 4: 重置右手
        client.get_logger().info('\n=== 示例 4: 重置右手 ===')
        client.send_reset_goal(adapter_index=1)
        rclpy.spin_once(client, timeout_sec=3.0)

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
