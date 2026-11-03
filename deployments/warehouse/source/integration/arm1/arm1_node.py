"""Robot arm 1 관제 클라이언트. send_goal의 Future는 실제 Result를 반환합니다."""
from functools import partial
import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from host_pkg.action import Arm

try:
    from shared_data import shared_memory
except ModuleNotFoundError as exc:
    if exc.name != 'shared_data':
        raise
    # 팔1 단독 실행도 가능합니다. 전체 관제에서는 기존 shared_data를 사용합니다.
    shared_memory = SimpleNamespace()


COMMANDS = {'arm1': {'SELECT_A', 'SELECT_B'}}
CONTROLS = {'EMER_STOP', 'RESTART'}
ALIASES = {'estop': 'EMER_STOP', 'restart': 'RESTART'}


class ArmActionClient(Node):
    def __init__(self, arm):
        # 로봇 서버 노드와 이름이 겹치지 않도록 host 네임스페이스를 사용합니다.
        super().__init__(arm + '_client', namespace='host')
        self.arm = arm
        self._client = ActionClient(self, Arm, f'/{arm}/data')
        self._lock = threading.RLock()
        self.busy = False
        self.feedback = 'UNKNOWN'
        self.last_result = None
        self.last_control_result = None
        self._requests = []
        self._write_shared(False, end_flag=False, success=False, end_message='', feedback_msg='UNKNOWN')
        self._write_shared(True, end_flag=False, success=False, end_message='')

    def _write_shared(self, control, **values):
        prefix = self.arm + ('_control_' if control else '_')
        for key, value in values.items():
            setattr(shared_memory, prefix + key, value)

    def send_goal(self, command):
        command = ALIASES.get(command, command)
        completion = Future()
        control = command in CONTROLS
        request = {'command': command, 'control': control, 'future': completion}
        with self._lock:
            if command not in COMMANDS[self.arm] and not control:
                return self._reject_local(request, '이 팔에서 지원하지 않는 명령')
            if not control and self.busy:
                return self._reject_local(request, '이 팔의 이전 작업 결과를 기다리는 중')
            if not control:
                self.busy = True
                self.last_result = None
            self._write_shared(control, end_flag=False, success=False, end_message='')
            self._requests.append(request)
        if not self._client.server_is_ready():
            self._finish(request, False, 'ERROR', f'/{self.arm}/data 서버 연결 안 됨')
            return completion
        goal_msg = Arm.Goal()
        goal_msg.command = command
        self.get_logger().info(f'{self.arm} 요청: {command}')
        try:
            future = self._client.send_goal_async(
                goal_msg, feedback_callback=partial(self.feedback_callback, request=request),
            )
            future.add_done_callback(partial(self.goal_response_callback, request=request))
        except Exception as exc:
            self._finish(request, False, 'ERROR', f'요청 전송 오류: {exc}')
        return completion

    def _reject_local(self, request, reason):
        self.get_logger().error(f"{self.arm} {request['command']}: {reason}")
        # 기존 작업 상태를 덮어쓰지 않고 이번 요청의 Future에만 실패를 반환합니다.
        request['future'].set_result(SimpleNamespace(
            command=request['command'], success=False, message='ERROR', detail=reason,
        ))
        return request['future']

    def feedback_callback(self, feedback_data, *, request):
        message = feedback_data.feedback.message
        with self._lock:
            if request['future'].done():
                return
            if message not in ('IDLE', 'RUNNING', 'PAUSED', 'ERROR'):
                self.get_logger().warning(f'{self.arm}: 알 수 없는 Feedback {message}')
                return
            self._write_shared(request['control'], feedback_msg=message)
            if not request['control']:
                self.feedback = message
        self.get_logger().info(f"{self.arm} {request['command']} Feedback: {message}")

    def goal_response_callback(self, future, *, request):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._finish(request, False, 'ERROR', '팔1 서버가 요청을 거절함 (다른 작업 진행 중 등)')
                return
            self.get_logger().info(f"{self.arm} {request['command']}: 접수됨, 실제 완료 대기")
            goal_handle.get_result_async().add_done_callback(
                partial(self.get_result_callback, request=request),
            )
        except Exception as exc:
            self._finish(request, False, 'ERROR', f'요청 응답 오류: {exc}')

    def get_result_callback(self, future, *, request):
        try:
            response = future.result()
            result = response.result
            success = (response.status == GoalStatus.STATUS_SUCCEEDED
                       and result.success and result.message == 'IDLE')
            self._finish(request, success, 'IDLE' if success else 'ERROR', result.message)
        except Exception as exc:
            self._finish(request, False, 'ERROR', f'결과 수신 오류: {exc}')

    def _finish(self, request, success, message, detail):
        with self._lock:
            if request['future'].done():
                return
            result = SimpleNamespace(command=request['command'], success=bool(success),
                                     message=message, detail=detail)
            self._write_shared(request['control'], end_flag=True, success=result.success,
                               end_message=message)
            if request['control']:
                self.last_control_result = result
            else:
                self.busy = False
                self.feedback = message
                self.last_result = result
                self._write_shared(False, feedback_msg=message)
            self._requests.remove(request)
            self.get_logger().info(
                f"{self.arm} {request['command']} Result: success={result.success}, "
                f'message={message} ({detail})',
            )
            request['future'].set_result(result)


class Arm1ActionClient(ArmActionClient):
    def __init__(self):
        super().__init__('arm1')
