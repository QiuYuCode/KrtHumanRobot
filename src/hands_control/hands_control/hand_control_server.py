"""DexHand021S 双手控制 Action Server."""
import time
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

try:
    from dexhand import DexHand021S
    from dexhand.dexhand import AdapterType
    DEXHAND_AVAILABLE = True
except ImportError:
    DEXHAND_AVAILABLE = False

from hands_control_interfaces.action import HandControl, ResetHand
from hands_control_interfaces.srv import (
    GetApproachingValue,
    GetDeviceId,
    GetDeviceString,
    GetFingerValue,
    GetNormalPressure,
    GetTangentPressure,
    SetFingerValue,
)


class HandControlServer(LifecycleNode):
    """灵巧手控制服务器，单手实例."""

    def __init__(self):
        """初始化节点和灵巧手连接."""
        super().__init__('hand_control_server')

        # 参数声明
        self.declare_parameter('adapter_type', 'ZLG_MINI')
        self.declare_parameter('adapter_index', 0)
        self.declare_parameter('device_id', 0x01)
        self.declare_parameter('hand_name', '')
        self.declare_parameter('listen_enabled', False)
        self.declare_parameter('realtime_response_enabled', False)
        self.declare_parameter('has_pressure_sensor', False)
        self.declare_parameter('feedback_topic', 'feedback/hand_joint_states')
        self.declare_parameter('control_topic', 'control/joint_states')
        self.declare_parameter('state_publish_rate_hz', 50.0)
        self.declare_parameter('stream_speed', 500)
        self.declare_parameter('stream_force', 85)
        self.declare_parameter('stream_wait_time', 0)

        self.comm_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()
        self._parameter_callback = self.add_on_set_parameters_callback(
            self._handle_parameter_update
        )
        self._stopping = False
        self._interfaces_active = False
        self._control_services = []
        self._state_publisher = None
        self._state_subscription = None
        self._state_timer = None
        self.listen_enabled = bool(self.get_parameter('listen_enabled').value)
        self.realtime_response_enabled = bool(
            self.get_parameter('realtime_response_enabled').value
        )

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Allocate the SDK object while entering the inactive state."""
        if not DEXHAND_AVAILABLE:
            self.get_logger().error('dexhand 模块未安装，请先安装 dexhand_sdk_python')
            return TransitionCallbackReturn.FAILURE
        try:
            adapter_type = str(self.get_parameter('adapter_type').value)
            self.adapter_type = self._parse_adapter_type(adapter_type)
            self.adapter_index = int(self.get_parameter('adapter_index').value)
            self.device_id = int(self.get_parameter('device_id').value)
            self.hand_name = str(self.get_parameter('hand_name').value)
            if not self.hand_name:
                self.hand_name = '左手' if self.adapter_index == 0 else '右手'
            self.has_pressure_sensor = bool(
                self.get_parameter('has_pressure_sensor').value
            )
            self.listen_enabled = bool(self.get_parameter('listen_enabled').value)
            self.realtime_response_enabled = bool(
                self.get_parameter('realtime_response_enabled').value
            )
            self.feedback_topic = str(self.get_parameter('feedback_topic').value)
            self.control_topic = str(self.get_parameter('control_topic').value)
            self.state_publish_rate_hz = float(
                self.get_parameter('state_publish_rate_hz').value
            )
            if self.state_publish_rate_hz <= 0.0:
                raise ValueError('state_publish_rate_hz 必须大于 0')
            if not hasattr(self, 'hand'):
                self.hand = DexHand021S(
                    adapter_type=self.adapter_type,
                    adapter_index=self.adapter_index,
                )
            self.get_logger().info(
                f'{self.hand_name}配置完成 '
                f'(adapter_index={self.adapter_index}, device_id={self.device_id})'
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            if hasattr(self, 'hand'):
                del self.hand
            self.get_logger().error(f'灵巧手配置失败: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, _state: State) -> TransitionCallbackReturn:
        """Expose control interfaces and enable configured runtime streams."""
        try:
            self._stopping = False
            self._create_control_interfaces()
            self._apply_runtime_state(
                listen_enabled=self.listen_enabled,
                realtime_response_enabled=self.realtime_response_enabled,
                force=True,
            )
            self.get_logger().info('手部控制已激活')
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self._destroy_control_interfaces()
            self.get_logger().error(f'灵巧手激活失败: {exc}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, _state: State) -> TransitionCallbackReturn:
        """Stop accepting commands and wait briefly for SDK access to finish."""
        self._stopping = True
        if not self.comm_lock.acquire(timeout=3.0):
            self._stopping = False
            self.get_logger().error('夹爪动作未在 3 秒内退出，拒绝停用')
            return TransitionCallbackReturn.FAILURE
        self.comm_lock.release()
        try:
            self._apply_runtime_state(False, False, force=True)
        except Exception as exc:
            self.get_logger().error(f'关闭夹爪监听失败: {exc}')
            self._stopping = False
            return TransitionCallbackReturn.FAILURE
        self._destroy_control_interfaces()
        self.get_logger().info('手部控制已停用')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Clear control interfaces and return to unconfigured."""
        self._destroy_control_interfaces()
        # ponytail: SDK has no destroy API; reuse the handle until process exit.
        self._stopping = False
        self.get_logger().info('手部控制接口已清理，硬件连接保留')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Best-effort safe shutdown from any lifecycle state."""
        if hasattr(self, 'hand'):
            try:
                self._apply_runtime_state(False, False, force=True)
            except Exception:
                pass
        return self.on_cleanup(state)

    def on_error(self, state: State) -> TransitionCallbackReturn:
        """Release partial resources so configure can be retried."""
        return self.on_cleanup(state)

    def _create_control_interfaces(self):
        if self._interfaces_active:
            return
        self._hand_control_server = ActionServer(
            self,
            HandControl,
            'hand_control',
            self._execute_hand_control,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._reset_hand_server = ActionServer(
            self,
            ResetHand,
            'reset_hand',
            self._execute_reset_hand,
            callback_group=self._callback_group,
        )
        self._control_services = [
            self.create_service(GetDeviceId, 'get_device_id', self._handle_get_device_id),
            self.create_service(
                GetDeviceString, 'get_firmware_version', self._handle_get_firmware_version
            ),
            self.create_service(Trigger, 'clear_error', self._handle_clear_error),
        ]
        self._register_finger_services()
        self._control_services.extend(self._finger_get_services)
        self._control_services.extend(self._finger_set_services)
        if self.has_pressure_sensor:
            self._control_services.extend([
                self.create_service(
                    GetNormalPressure, 'get_normal_pressure', self._handle_normal_pressure
                ),
                self.create_service(
                    GetTangentPressure, 'get_tangent_pressure', self._handle_tangent_pressure
                ),
                self.create_service(
                    GetApproachingValue,
                    'get_approaching_value',
                    self._handle_approaching_value,
                ),
            ])
        self._state_publisher = self.create_publisher(
            JointState, self.feedback_topic, qos_profile_sensor_data
        )
        self._state_subscription = self.create_subscription(
            JointState,
            self.control_topic,
            self._joint_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._state_timer = self.create_timer(
            1.0 / self.state_publish_rate_hz,
            self._publish_joint_state,
            callback_group=self._callback_group,
        )
        self._interfaces_active = True

    def _destroy_control_interfaces(self):
        if not self._interfaces_active:
            return
        self._hand_control_server.destroy()
        self._reset_hand_server.destroy()
        if self._state_timer is not None:
            self.destroy_timer(self._state_timer)
        if self._state_subscription is not None:
            self.destroy_subscription(self._state_subscription)
        if self._state_publisher is not None:
            self.destroy_publisher(self._state_publisher)
        self._state_timer = None
        self._state_subscription = None
        self._state_publisher = None
        for service in self._control_services:
            self.destroy_service(service)
        self._control_services = []
        self._interfaces_active = False

    def _publish_joint_state(self):
        if self._stopping or self._state_publisher is None:
            return
        try:
            with self.comm_lock:
                positions = self._read_all_positions(self.hand, self.device_id)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['finger_1', 'finger_2', 'finger_3']
            msg.position = [float(value) for value in positions]
            self._state_publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'{self.hand_name} 状态发布失败: {exc}')

    def _joint_state_callback(self, msg: JointState):
        if self._stopping or not msg.name or not msg.position:
            return
        positions = dict(zip(msg.name, msg.position))
        commands = []
        for finger_id in (1, 2, 3):
            name = f'finger_{finger_id}'
            if name not in positions:
                continue
            value = int(round(float(positions[name])))
            if not 0 <= value <= 1000:
                self.get_logger().warning(
                    f'{self.hand_name} 忽略超范围位置: {name}={value}'
                )
                continue
            commands.append((finger_id, value))
        if not commands:
            return
        speed = int(self.get_parameter('stream_speed').value)
        force = int(self.get_parameter('stream_force').value)
        wait_time = int(self.get_parameter('stream_wait_time').value)
        try:
            with self.comm_lock:
                for finger_id, position in commands:
                    self.hand.move_finger(
                        self.device_id, finger_id, position, speed, force, wait_time
                    )
        except Exception as exc:
            self.get_logger().error(f'{self.hand_name} 流式控制失败: {exc}')

    def _parse_adapter_type(self, adapter_type_str):
        """解析适配器类型字符串."""
        adapter_map = {
            'ZLG_200U': AdapterType.ZLG_200U,
            'ZLG_MINI': AdapterType.ZLG_MINI,
        }
        return adapter_map.get(adapter_type_str, AdapterType.ZLG_MINI)

    def _read_all_positions(self, hand, device_id):
        """读取所有手指位置，强制转换为 int."""
        positions = []
        for finger_id in [0x01, 0x02, 0x03]:
            try:
                positions.append(int(hand.get_joint_degree(device_id, finger_id)))
            except Exception:
                positions.append(0)
        return positions

    def _validate_finger_id(self, finger_id):
        """校验手指编号."""
        return finger_id in [0x01, 0x02, 0x03]

    def _register_finger_services(self):
        """注册按手指读取/设置的 SDK 服务."""
        get_services = {
            'get_safe_current': self.hand.get_safe_current,
            'get_safe_temperature': self.hand.get_safe_temperature,
            'get_error_code': self.hand.get_error_code,
            'get_motor_current': self.hand.get_motor_current,
            'get_motor_velocity': self.hand.get_motor_velocity,
            'get_motor_temperature': self.hand.get_motor_temperature,
            'get_joint_degree': self.hand.get_joint_degree,
        }
        set_services = {
            'set_safe_current': (
                self.hand.set_safe_current,
                200,
                800,
                'max_current',
            ),
            'set_safe_temperature': (
                self.hand.set_safe_temperature,
                55,
                85,
                'max_temperature',
            ),
        }

        self._finger_get_services = [
            self.create_service(
                GetFingerValue,
                service_name,
                self._make_get_finger_value_handler(service_name, sdk_method),
            )
            for service_name, sdk_method in get_services.items()
        ]
        self._finger_set_services = [
            self.create_service(
                SetFingerValue,
                service_name,
                self._make_set_finger_value_handler(
                    service_name,
                    sdk_method,
                    min_value,
                    max_value,
                    value_name,
                ),
            )
            for service_name, (
                sdk_method,
                min_value,
                max_value,
                value_name,
            ) in set_services.items()
        ]

    def _handle_get_device_id(self, request, response):
        """读取指定通道上的设备 ID."""
        try:
            with self.comm_lock:
                response.device_id = int(self.hand.get_device_id(request.channel))
            response.success = True
            response.error_message = ''
        except Exception as e:
            response.success = False
            response.device_id = 0
            response.error_message = str(e)
        return response

    def _handle_get_firmware_version(self, request, response):
        """读取固件版本."""
        del request
        try:
            with self.comm_lock:
                response.value = str(
                    self.hand.get_firmware_version(self.device_id)
                )
            response.success = True
            response.error_message = ''
        except Exception as e:
            response.success = False
            response.value = ''
            response.error_message = str(e)
        return response

    def _handle_clear_error(self, request, response):
        """清除当前设备错误."""
        del request
        try:
            with self.comm_lock:
                self.hand.clear_error(self.device_id)
            response.success = True
            response.message = f'{self.hand_name} 已清除错误'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _make_get_finger_value_handler(self, service_name, sdk_method):
        """创建按手指读取数值的服务回调."""
        def handler(request, response):
            finger_id = int(request.finger_id)
            if not self._validate_finger_id(finger_id):
                response.success = False
                response.value = 0.0
                response.error_message = (
                    f'无效的 finger_id: {finger_id} (支持 1,2,3)'
                )
                return response

            try:
                with self.comm_lock:
                    response.value = float(
                        sdk_method(self.device_id, finger_id)
                    )
                response.success = True
                response.error_message = ''
            except Exception as e:
                self.get_logger().error(
                    f'{self.hand_name} {service_name} 失败 '
                    f'finger_id={finger_id}: {str(e)}'
                )
                response.success = False
                response.value = 0.0
                response.error_message = str(e)
            return response

        return handler

    def _make_set_finger_value_handler(
        self,
        service_name,
        sdk_method,
        min_value,
        max_value,
        value_name,
    ):
        """创建按手指设置数值的服务回调."""
        def handler(request, response):
            finger_id = int(request.finger_id)
            value = int(request.value)
            if not self._validate_finger_id(finger_id):
                response.success = False
                response.error_message = (
                    f'无效的 finger_id: {finger_id} (支持 1,2,3)'
                )
                return response
            if value < min_value or value > max_value:
                response.success = False
                response.error_message = (
                    f'{value_name} 超出范围: {value} '
                    f'(支持 {min_value}-{max_value})'
                )
                return response

            try:
                with self.comm_lock:
                    sdk_method(self.device_id, finger_id, value)
                response.success = True
                response.error_message = ''
            except Exception as e:
                self.get_logger().error(
                    f'{self.hand_name} {service_name} 失败 '
                    f'finger_id={finger_id}, value={value}: {str(e)}'
                )
                response.success = False
                response.error_message = str(e)
            return response

        return handler

    def _pressure_response(self, value, response):
        """填充压力服务成功响应."""
        response.success = True
        response.available = True
        response.value = float(value)
        response.error_message = ''
        return response

    def _pressure_error(self, message, response, available=True):
        """填充压力服务失败响应."""
        response.success = False
        response.available = available
        response.value = 0.0
        response.error_message = message
        return response

    def _handle_normal_pressure(self, request, response):
        """查询法向压力."""
        return self._handle_pressure_query('normal', request, response)

    def _handle_tangent_pressure(self, request, response):
        """查询切向压力."""
        return self._handle_pressure_query('tangent', request, response)

    def _handle_approaching_value(self, request, response):
        """查询接近觉读数."""
        return self._handle_pressure_query('approach', request, response)

    def _handle_pressure_query(self, metric, request, response):
        """读取指定压力传感器数据."""
        finger_id = int(request.finger_id)
        if not self._validate_finger_id(finger_id):
            return self._pressure_error(
                f'无效的 finger_id: {finger_id} (支持 1,2,3)',
                response,
            )

        try:
            with self.comm_lock:
                if metric == 'normal':
                    value = self.hand.get_normal_pressure(self.device_id, finger_id)
                elif metric == 'tangent':
                    value = self.hand.get_tangent_pressure(self.device_id, finger_id)
                elif metric == 'approach':
                    value = self.hand.get_approaching_value(self.device_id, finger_id)
                else:
                    return self._pressure_error(
                        f'不支持的压力类型: {metric}',
                        response,
                    )
            return self._pressure_response(value, response)
        except Exception as e:
            self.get_logger().error(
                f'{self.hand_name} 压力读取失败 metric={metric}, finger_id={finger_id}: {str(e)}'
            )
            return self._pressure_error(str(e), response)

    def _apply_runtime_state(
        self,
        listen_enabled,
        realtime_response_enabled,
        force=False,
    ):
        """同步底层监听和实时反馈状态."""
        with self.comm_lock:
            if force or listen_enabled != self.listen_enabled:
                self.hand.listen(enable=listen_enabled)
                self.listen_enabled = listen_enabled

            if force or realtime_response_enabled != self.realtime_response_enabled:
                self.hand.enable_realtime_response(
                    device_id=self.device_id,
                    enable=realtime_response_enabled
                )
                self.realtime_response_enabled = realtime_response_enabled

    def _handle_parameter_update(self, params):
        """处理运行时参数更新."""
        allowed_names = {
            'listen_enabled',
            'realtime_response_enabled',
        }
        static_names = {
            'adapter_type',
            'adapter_index',
            'device_id',
            'hand_name',
            'has_pressure_sensor',
        }

        requested_listen = self.listen_enabled
        requested_realtime = self.realtime_response_enabled

        state_label = self._state_machine.current_state[1]
        for param in params:
            if param.name in static_names:
                if state_label != 'unconfigured':
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} 只允许在未配置状态设置'
                    )
                if hasattr(self, 'hand'):
                    current_value = {
                        'adapter_type': self.adapter_type.name,
                        'adapter_index': self.adapter_index,
                        'device_id': self.device_id,
                        'hand_name': self.hand_name,
                        'has_pressure_sensor': self.has_pressure_sensor,
                    }[param.name]
                    if param.value != current_value:
                        return SetParametersResult(
                            successful=False,
                            reason=f'{param.name} 需要重启节点后修改'
                        )
                if param.name == 'adapter_type' and str(param.value) not in {
                    'ZLG_MINI', 'ZLG_200U'
                }:
                    return SetParametersResult(
                        successful=False, reason='adapter_type 不受支持'
                    )
                if param.name == 'adapter_index' and not 0 <= int(param.value) <= 15:
                    return SetParametersResult(
                        successful=False, reason='adapter_index 必须在 0 到 15 之间'
                    )
                if param.name == 'device_id' and not 1 <= int(param.value) <= 255:
                    return SetParametersResult(
                        successful=False, reason='device_id 必须在 1 到 255 之间'
                    )
                continue

            if param.name not in allowed_names:
                return SetParametersResult(
                    successful=False,
                    reason=f'不支持的参数: {param.name}'
                )

            if param.name == 'listen_enabled':
                requested_listen = bool(param.value)
            elif param.name == 'realtime_response_enabled':
                requested_realtime = bool(param.value)

        old_listen = self.listen_enabled
        old_realtime = self.realtime_response_enabled

        if state_label != 'active':
            self.listen_enabled = requested_listen
            self.realtime_response_enabled = requested_realtime
            return SetParametersResult(successful=True)

        try:
            self._apply_runtime_state(
                listen_enabled=requested_listen,
                realtime_response_enabled=requested_realtime,
                force=False,
            )
        except Exception as e:
            self.get_logger().error(
                f'{self.hand_name} 参数更新失败: {str(e)}'
            )
            try:
                self._apply_runtime_state(
                    listen_enabled=old_listen,
                    realtime_response_enabled=old_realtime,
                    force=True,
                )
            except Exception as rollback_error:
                self.get_logger().error(
                    f'{self.hand_name} 参数回滚失败: {str(rollback_error)}'
                )
            return SetParametersResult(
                successful=False,
                reason=str(e)
            )

        self.get_logger().info(
            f'{self.hand_name} 参数已更新: '
            f'listen_enabled={self.listen_enabled}, '
            f'realtime_response_enabled={self.realtime_response_enabled}'
        )
        return SetParametersResult(successful=True)

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
        if adapter_index != self.adapter_index:
            result.success = False
            result.message = (
                f'adapter_index 不匹配: 目标={adapter_index}, '
                f'节点={self.adapter_index}'
            )
            goal_handle.abort()
            return result

        if finger_id not in [0, 1, 2, 3]:
            result.success = False
            result.message = f'无效的 finger_id: {finger_id} (支持 0,1,2,3)'
            goal_handle.abort()
            return result

        hand = self.hand
        device_id = self.device_id
        hand_name = self.hand_name
        target_fingers = [0x01, 0x02, 0x03] if finger_id == 0 else [finger_id]

        try:
            with self.comm_lock:
                if self._stopping:
                    result.success = False
                    result.message = f'{hand_name} 正在停用'
                    goal_handle.abort()
                    return result
                # 清除错误并移动手指
                hand.clear_error(device_id)

                self.get_logger().info(
                    f'{hand_name} - 手指 {target_fingers} 移动到位置 {position}, '
                    f'速度={speed}, 力度={force:#x}'
                )
                for fid in target_fingers:
                    if goal_handle.is_cancel_requested or self._stopping:
                        result.success = False
                        result.message = f'{hand_name} 控制已取消'
                        result.final_positions = self._read_all_positions(
                            hand, device_id
                        )
                        goal_handle.canceled()
                        return result
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
                    # ponytail: SDK 无停止指令；取消只能阻止后续命令并结束反馈等待。
                    if goal_handle.is_cancel_requested or self._stopping:
                        result.success = False
                        result.message = f'{hand_name} 控制已取消'
                        result.final_positions = self._read_all_positions(
                            hand, device_id
                        )
                        goal_handle.canceled()
                        return result
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

    @staticmethod
    def _cancel_callback(_goal_handle):
        """接受客户端取消；执行循环负责尽快结束当前目标."""
        return CancelResponse.ACCEPT

    def _execute_reset_hand(self, goal_handle):
        """执行手部重置 action."""
        self.get_logger().info('收到手部重置请求')

        goal = goal_handle.request
        feedback_msg = ResetHand.Feedback()
        result = ResetHand.Result()

        adapter_index = goal.adapter_index

        # 验证参数
        if adapter_index != self.adapter_index:
            result.success = False
            result.message = (
                f'adapter_index 不匹配: 目标={adapter_index}, '
                f'节点={self.adapter_index}'
            )
            goal_handle.abort()
            return result

        hand = self.hand
        device_id = self.device_id
        hand_name = self.hand_name

        try:
            with self.comm_lock:
                if self._stopping:
                    result.success = False
                    result.message = f'{hand_name} 正在停用'
                    goal_handle.abort()
                    return result
                # 清除所有手指的错误
                feedback_msg.status = f'清除 {hand_name} 错误'
                goal_handle.publish_feedback(feedback_msg)

                hand.clear_error(device_id)

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

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'hand'):
            try:
                node._apply_runtime_state(
                    listen_enabled=False,
                    realtime_response_enabled=False,
                    force=True,
                )
            except Exception:
                pass
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
