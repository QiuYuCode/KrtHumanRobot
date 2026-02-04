"""DexHand021S 双手控制 Action Server."""
import time
import threading
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

try:
    from dexhand import DexHand021S
    from dexhand.dexhand import AdapterType
    DEXHAND_AVAILABLE = True
except ImportError:
    DEXHAND_AVAILABLE = False

from hands_control_interfaces.action import HandControl, ResetHand


class HandControlServer(Node):
    """灵巧手控制服务器，支持双手独立控制."""

    def __init__(self):
        """初始化节点和灵巧手连接."""
        super().__init__('hand_control_server')

        if not DEXHAND_AVAILABLE:
            self.get_logger().error('dexhand 模块未安装，请先安装 dexhand_sdk_python')
            return

        # 参数声明
        self.declare_parameter('adapter_type', 'ZLG_MINI')
        self.declare_parameter('left_hand_device_id', 0x01)
        self.declare_parameter('right_hand_device_id', 0x02)

        adapter_type_str = self.get_parameter('adapter_type').value
        self.left_device_id = self.get_parameter('left_hand_device_id').value
        self.right_device_id = self.get_parameter('right_hand_device_id').value

        # 转换 adapter_type
        self.adapter_type = self._parse_adapter_type(adapter_type_str)

        # 初始化两只手
        self.hands = {}
        self.hand_locks = {}

        try:
            # 左手 (adapter_index=0)
            self.hands[0] = DexHand021S(
                adapter_type=self.adapter_type,
                adapter_index=0
            )
            self.hands[0].listen(enable=True)
            self.hands[0].enable_realtime_response(
                device_id=self.left_device_id,
                enable=True
            )
            self.hand_locks[0] = threading.Lock()
            self.get_logger().info(f'左手初始化成功 (adapter_index=0, device_id={self.left_device_id})')

            # 右手 (adapter_index=1)
            self.hands[1] = DexHand021S(
                adapter_type=self.adapter_type,
                adapter_index=1
            )
            self.hands[1].listen(enable=True)
            self.hands[1].enable_realtime_response(
                device_id=self.right_device_id,
                enable=True
            )
            self.hand_locks[1] = threading.Lock()
            self.get_logger().info(f'右手初始化成功 (adapter_index=1, device_id={self.right_device_id})')

        except Exception as e:
            self.get_logger().error(f'灵巧手初始化失败: {str(e)}')
            return

        # 创建 action servers
        callback_group = ReentrantCallbackGroup()

        self._hand_control_server = ActionServer(
            self,
            HandControl,
            'hand_control',
            self._execute_hand_control,
            callback_group=callback_group
        )

        self._reset_hand_server = ActionServer(
            self,
            ResetHand,
            'reset_hand',
            self._execute_reset_hand,
            callback_group=callback_group
        )

        self.get_logger().info('手部控制 Action Server 已启动')

    def _parse_adapter_type(self, adapter_type_str):
        """解析适配器类型字符串."""
        adapter_map = {
            'ZLG_200U': AdapterType.ZLG_200U,
            'ZLG_MINI': AdapterType.ZLG_MINI,
        }
        return adapter_map.get(adapter_type_str, AdapterType.ZLG_MINI)

    def _get_device_id(self, adapter_index):
        """获取设备 ID."""
        return self.left_device_id if adapter_index == 0 else self.right_device_id

    def _read_all_positions(self, hand, device_id):
        """读取所有手指位置，强制转换为 int."""
        positions = []
        for finger_id in [0x01, 0x02, 0x03]:
            try:
                positions.append(int(hand.get_joint_degree(device_id, finger_id)))
            except Exception:
                positions.append(0)
        return positions

    def _execute_hand_control(self, goal_handle):
        """执行手部控制 action."""
        self.get_logger().info('收到手部控制请求')

        goal = goal_handle.request
        feedback_msg = HandControl.Feedback()
        result = HandControl.Result()

        adapter_index = goal.adapter_index
        finger_id = goal.finger_id
        position = goal.position
        speed = goal.speed
        force = goal.force
        wait_time = goal.wait_time

        # 验证参数
        if adapter_index not in [0, 1]:
            result.success = False
            result.message = f'无效的 adapter_index: {adapter_index} (只支持 0 或 1)'
            goal_handle.abort()
            return result

        if adapter_index not in self.hands:
            result.success = False
            result.message = f'手部 {adapter_index} 未初始化'
            goal_handle.abort()
            return result

        if finger_id not in [0, 1, 2, 3]:
            result.success = False
            result.message = f'无效的 finger_id: {finger_id} (支持 0,1,2,3)'
            goal_handle.abort()
            return result

        hand = self.hands[adapter_index]
        device_id = self._get_device_id(adapter_index)
        hand_name = '左手' if adapter_index == 0 else '右手'
        target_fingers = [0x01, 0x02, 0x03] if finger_id == 0 else [finger_id]

        try:
            with self.hand_locks[adapter_index]:
                # 清除错误并移动手指
                for fid in target_fingers:
                    hand.clear_error(device_id, fid)

                self.get_logger().info(
                    f'{hand_name} - 手指 {target_fingers} 移动到位置 {position}, '
                    f'速度={speed}, 力度={force:#x}'
                )
                for fid in target_fingers:
                    hand.move_finger(
                        device_id,
                        fid,
                        position,
                        speed,
                        force,
                        wait_time
                    )

                # 等待移动完成并发送反馈
                total_time = wait_time * 0.1  # wait_time 单位是 100ms
                steps = 10
                for i in range(steps):
                    time.sleep(total_time / steps)

                    # 获取当前位置
                    feedback_msg.current_positions = self._read_all_positions(
                        hand, device_id
                    )

                    feedback_msg.progress = (i + 1) / steps
                    goal_handle.publish_feedback(feedback_msg)

                # 获取最终位置
                joint1, joint2, joint3 = self._read_all_positions(hand, device_id)

                result.success = True
                result.message = f'{hand_name} 手指 {target_fingers} 移动完成'
                result.final_positions = [joint1, joint2, joint3]

                self.get_logger().info(
                    f'{hand_name} 当前位置: [{joint1}, {joint2}, {joint3}]'
                )

        except Exception as e:
            result.success = False
            result.message = f'{hand_name} 控制失败: {str(e)}'
            result.final_positions = [0, 0, 0]
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        goal_handle.succeed()
        return result

    def _execute_reset_hand(self, goal_handle):
        """执行手部重置 action."""
        self.get_logger().info('收到手部重置请求')

        goal = goal_handle.request
        feedback_msg = ResetHand.Feedback()
        result = ResetHand.Result()

        adapter_index = goal.adapter_index

        # 验证参数
        if adapter_index not in [0, 1]:
            result.success = False
            result.message = f'无效的 adapter_index: {adapter_index}'
            goal_handle.abort()
            return result

        if adapter_index not in self.hands:
            result.success = False
            result.message = f'手部 {adapter_index} 未初始化'
            goal_handle.abort()
            return result

        hand = self.hands[adapter_index]
        device_id = self._get_device_id(adapter_index)
        hand_name = '左手' if adapter_index == 0 else '右手'

        try:
            with self.hand_locks[adapter_index]:
                # 清除所有手指的错误
                feedback_msg.status = f'清除 {hand_name} 错误'
                goal_handle.publish_feedback(feedback_msg)

                for finger_id in [0x01, 0x02, 0x03]:
                    hand.clear_error(device_id, finger_id)

                # 重置关节
                feedback_msg.status = f'重置 {hand_name} 关节'
                goal_handle.publish_feedback(feedback_msg)

                hand.reset_joints(device_id)
                time.sleep(1)

                result.success = True
                result.message = f'{hand_name} 重置完成'
                self.get_logger().info(result.message)

        except Exception as e:
            result.success = False
            result.message = f'{hand_name} 重置失败: {str(e)}'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        goal_handle.succeed()
        return result


def main(args=None):
    """主函数."""
    rclpy.init(args=args)
    node = HandControlServer()

    if not DEXHAND_AVAILABLE:
        node.get_logger().error('无法启动节点，dexhand 模块缺失')
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
