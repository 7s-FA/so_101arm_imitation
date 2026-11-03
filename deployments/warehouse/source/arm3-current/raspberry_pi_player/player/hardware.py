"""STS3215 transport. Only SRAM goal, torque and lock writes are permitted.

Register addresses and position conventions checked against installed LeRobot
0.6.1 (Apache-2.0), motors/feetech/tables.py and motors/motors_bus.py.
"""

import fcntl
from importlib.metadata import version
import os
from pathlib import Path
import stat
import subprocess
import termios
import time

from .program import JOINTS, PlayerError, check_startup_path
from temperature_guard import TemperatureGuard, TemperatureFault

REGISTERS = {
    "Model_Number": (3, 2), "Min_Position_Limit": (9, 2), "Max_Position_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1), "Phase": (18, 1), "Homing_Offset": (31, 2), "Operating_Mode": (33, 1),
    "Torque_Enable": (40, 1), "Goal_Position": (42, 2), "Goal_Time": (44, 2),
    "Goal_Velocity": (46, 2), "Lock": (55, 1), "Present_Position": (56, 2),
    "Present_Temperature": (63, 1), "Status": (65, 1),
}
WRITABLE = {"Torque_Enable", "Goal_Position", "Lock"}
STARTUP_SPEED_TICKS = 50
# Return profile doubled on 2026-10-20: 56 -> 112 encoder steps/s.
# Nominal 9.84375 deg/s; actual travel still depends on load and target limits.
RETURN_SPEED_TICKS = 112
GUI_SPEED_TICKS = 227  # Explicit GUI user request: floor(20deg/s *4096/360).
STARTUP_INSET_TICKS = 57  # About 5 degrees: gives the servo useful position error.
STARTUP_ARRIVAL_TICKS = 23  # About 2 degrees; still well inside the boundary.
STARTUP_BOUNDARY_CLEARANCE_TICKS = 8


def decode_sign(value, bit):
    return -(value & ~(1 << bit)) if value & (1 << bit) else value


class Cancelled(PlayerError):
    pass


def check_stop(stop):
    if stop.is_set():
        raise Cancelled("정지 요청")


class SerialBus:
    def __init__(self, config):
        self.config = config
        self.port = None
        self.packet = None

    def open(self):
        # Imports are deliberately deferred: inspect/simulate never load a serial SDK.
        if version("feetech-servo-sdk") != "1.0.0" or version("pyserial") != "3.5":
            raise PlayerError("requirements.txt의 SDK/pyserial 버전으로 설치하세요.")
        import serial
        import scservo_sdk as sdk
        path = Path(self.config.port)
        resolved = path.resolve(strict=True)
        if not stat.S_ISCHR(resolved.stat().st_mode):
            raise PlayerError("USB 경로가 직렬 장치가 아닙니다.")
        properties = subprocess.run(["udevadm", "info", "--query=property", "--name", str(resolved)],
                                    capture_output=True, text=True, timeout=3, check=True).stdout
        actual = dict(line.split("=", 1) for line in properties.splitlines() if "=" in line)
        if actual.get("ID_SERIAL_SHORT") != self.config.serial:
            raise PlayerError("USB 장치의 실제 시리얼 번호가 robot.json과 다릅니다.")
        busy = subprocess.run(["fuser", str(resolved)], capture_output=True, text=True, timeout=3)
        if busy.returncode == 0 or busy.stdout.strip():
            raise PlayerError(f"포트를 사용하는 다른 프로그램을 먼저 종료하세요: PID {busy.stdout.strip()}")
        if busy.returncode != 1:
            raise PlayerError("포트 점유 상태를 검사하지 못했습니다.")

        class BoundedPort(sdk.PortHandler):
            def setupPort(self, baud):
                if self.is_open:
                    raise PlayerError("열린 포트의 통신 설정은 변경하지 않습니다.")
                self.ser = serial.Serial(str(resolved), baudrate=baud, timeout=0,
                                         write_timeout=.1, exclusive=True)
                try:
                    fcntl.ioctl(self.ser.fileno(), termios.TIOCEXCL)
                    self.ser.reset_input_buffer()
                except BaseException:
                    self.ser.close()
                    raise
                self.is_open = True
                self.tx_time_per_byte = 1000.0 / baud * 10
                return True

            def getCurrentTime(self):
                return time.monotonic() * 1000

            def clearPort(self):
                # SDK's flush()/tcdrain can wait without a timeout on a failed USB.
                self.ser.reset_input_buffer()

        self.port = BoundedPort(str(resolved))
        self.packet = sdk.PacketHandler(0)
        try:
            if not self.port.setBaudRate(1_000_000):
                raise PlayerError("1 Mbps 직렬 포트 열기 실패")
        except BaseException:
            self.close()
            raise

    def _check(self, joint, comm, error):
        if comm != 0:
            raise PlayerError(f"{joint}: 통신 실패: {self.packet.getTxRxResult(comm)}")
        if error != 0:
            raise PlayerError(f"{joint}: 모터 알람: {self.packet.getRxPacketError(error)}")

    def _exchange(self, function, *args):
        try:
            return function(self.port, *args)
        finally:
            # SDK 1.0.0 can leave this flag set after a Python serial exception.
            # Otherwise subsequent emergency OFF packets would all be rejected as busy.
            self.port.is_using = False

    def read(self, joint, register):
        addr, length = REGISTERS[register]
        motor_id = JOINTS.index(joint) + 1
        function = self.packet.read1ByteTxRx if length == 1 else self.packet.read2ByteTxRx
        value, comm, error = self._exchange(function, motor_id, addr)
        self._check(joint, comm, error)
        if register == "Homing_Offset":
            value = decode_sign(value, 11)
        if register in ("Goal_Position", "Present_Position", "Goal_Velocity"):
            value = decode_sign(value, 15)
        return value

    def feedback(self, joint):
        data, comm, error = self._exchange(self.packet.readTxRx, JOINTS.index(joint) + 1, 56, 10)
        self._check(joint, comm, error)
        if len(data) != 10:
            raise PlayerError(f"{joint}: 불완전한 피드백 패킷")
        return {"raw": decode_sign(data[0] | data[1] << 8, 15),
                "temperature": data[7], "status": data[9]}

    def write(self, joint, register, value):
        if register not in WRITABLE:
            raise PlayerError(f"허용되지 않은 모터 쓰기: {register}")
        high = 4095 if register == "Goal_Position" else 1
        if type(value) is not int or not 0 <= value <= high:
            raise PlayerError(f"잘못된 모터 쓰기 값: {register}={value}")
        addr, length = REGISTERS[register]
        function = self.packet.write1ByteTxRx if length == 1 else self.packet.write2ByteTxRx
        comm, error = self._exchange(function, JOINTS.index(joint) + 1, addr, value)
        self._check(joint, comm, error)

    def write_position_profile(self, joint, goal, speed):
        """One acknowledged SRAM packet: position + zero time + speed.

        Matches FTServo WritePosEx's contiguous goal/time/speed fields, without
        changing acceleration. Avoid an intermediate goal using unlimited speed.
        """
        if type(goal) is not int or not 0 <= goal <= 4095 or type(speed) is not int or speed not in (0, STARTUP_SPEED_TICKS, RETURN_SPEED_TICKS, GUI_SPEED_TICKS):
            raise PlayerError("Invalid startup position/speed profile")
        data = [goal & 255, goal >> 8, 0, 0, speed & 255, speed >> 8]
        comm, error = self._exchange(self.packet.writeTxRx, JOINTS.index(joint) + 1, 42, 6, data)
        self._check(joint, comm, error)

    def close(self):
        if self.port is not None and self.port.ser is not None and self.port.ser.is_open:
            try:
                fcntl.ioctl(self.port.ser.fileno(), termios.TIOCNXCL)
            finally:
                self.port.closePort()


class Follower:
    def __init__(self, bus, calibration, motion, stop, clock=time.monotonic):
        self.bus, self.calibration, self.motion, self.stop, self.clock = bus, calibration, motion, stop, clock
        self.motion_attempted = False
        self.active = False
        self.temperature_limits = {}
        self.temperature_guard = TemperatureGuard(motion.temperature_duration_s, clock)
        self.last_health = -float("inf")
        self.latest = None

    def startup_plan(self, raw):
        """Plan direct inward recovery in valid single-turn encoder coordinates."""
        plan = {}
        for joint, value in raw.items():
            entry = self.calibration.data[joint]
            lo, hi = entry["range_min"], entry["range_max"]
            if lo <= value <= hi:
                continue
            if type(value) is not int or not 0 <= value <= 4095 or hi - lo < 2 * STARTUP_INSET_TICKS:
                raise PlayerError(f"{joint}: invalid single-turn position or calibration span: {value}")
            target = lo + STARTUP_INSET_TICKS if value < lo else hi - STARTUP_INSET_TICKS
            if abs(target - value) >= 2048:
                raise PlayerError(f"{joint}: recovery path crosses a half-turn; inspect encoder reference")
            plan[joint] = {"raw": value, "boundary": lo if value < lo else hi,
                           "target_raw": target, "speed_ticks_per_second": STARTUP_SPEED_TICKS}
        return plan

    def _check_raw(self, joint, raw, startup_limits=None):
        if startup_limits and joint in startup_limits:
            lo, hi = startup_limits[joint]
            if type(raw) is not int or not 0 <= raw <= 4095 or not lo <= raw <= hi:
                raise PlayerError(f"{joint}: startup recovery position escaped [{lo}, {hi}]: {raw}")
        else:
            self.calibration.check_raw(joint, raw)

    def preflight(self, allow_startup_recovery=False):
        """Read only, including when a motor is already powered by somebody else."""
        for joint in JOINTS:
            check_stop(self.stop)
            if self.bus.read(joint, "Model_Number") != 777:
                raise PlayerError(f"{joint}: STS3215 모델 번호(777)가 아닙니다.")
            entry = self.calibration.data[joint]
            for field, register in (("homing_offset", "Homing_Offset"), ("range_min", "Min_Position_Limit"),
                                    ("range_max", "Max_Position_Limit")):
                actual = self.bus.read(joint, register)
                if actual != entry[field]:
                    raise PlayerError(f"보정 불일치 {joint}.{field}: 파일={entry[field]}, 모터={actual}")
            if self.bus.read(joint, "Operating_Mode") != 0:
                raise PlayerError(f"{joint}: 위치 제어 모드가 아닙니다.")
            if self.bus.read(joint, "Phase") & 0x10:
                raise PlayerError(f"{joint}: 위치 피드백 모드가 기존 GUI와 다릅니다. 기존 설정을 점검하세요.")
            if self.bus.read(joint, "Goal_Time") != 0:
                raise PlayerError(f"{joint}: Goal_Time must be zero")
            # A cancelled recovery deliberately leaves the low speed intact.
            if self.bus.read(joint, "Goal_Velocity") not in (0, STARTUP_SPEED_TICKS):
                raise PlayerError(f"{joint}: unexpected Goal_Velocity; inspect existing settings")
            temperature_limit = self.bus.read(joint, "Max_Temperature_Limit")
            if not 1 <= temperature_limit <= 150:
                raise PlayerError(f"{joint}: 모터 온도 보호값 읽기 이상")
            self.temperature_limits[joint] = min(temperature_limit, self.motion.temperature_limit_raw)
        limits = None
        if allow_startup_recovery:
            limits = dict.fromkeys(JOINTS, (0, 4095))
        current = self.sample(expected_active=False, force_health=True, startup_limits=limits)
        current["startup_recovery"] = self.startup_plan(current["raw"])
        return current

    def sample(self, expected_active=None, force_health=False, startup_limits=None):
        check_stop(self.stop)
        started = self.clock()
        expected = self.active if expected_active is None else expected_active
        health = force_health or started - self.last_health >= .5
        positions, raw, temperatures = {}, {}, {}
        for joint in JOINTS:
            check_stop(self.stop)
            feedback = self.bus.feedback(joint)
            self._check_raw(joint, feedback["raw"], startup_limits)
            if feedback["status"] != 0:
                raise PlayerError(f"{joint}: Status={feedback['status']}")
            try:
                self.temperature_guard.observe(joint, feedback['temperature'], self.temperature_limits[joint], expected)
            except TemperatureFault as exc:
                raise PlayerError(f"온도 보호: {exc}") from exc
            if health and self.bus.read(joint, "Torque_Enable") != int(expected):
                raise PlayerError(f"{joint}: 예상과 다른 토크 상태. 다른 제어 프로그램과 전원을 확인하세요.")
            raw[joint] = feedback["raw"]
            positions[joint + ".pos"] = self.calibration.to_units(joint, feedback["raw"])
            temperatures[joint] = feedback["temperature"]
        if self.clock() - started > .5:
            raise PlayerError("전체 위치 읽기가 0.5초를 넘었습니다. 구동을 중단합니다.")
        if health:
            self.last_health = self.clock()
        self.latest = {"pose": positions, "raw": raw, "temperatures_raw": temperatures, "time": self.clock(),
                       "temperature_pending": self.temperature_guard.snapshot()}
        return self.latest

    def activate(self, approved_pose, first_goal=None, *, allow_startup_recovery=False,
                 wait=None, poll=None, report=None, approved_raw=None):
        # Recheck configuration after the potentially long human approval prompt.
        sample = self.preflight(allow_startup_recovery=allow_startup_recovery)
        if max(abs(sample["pose"][key] - approved_pose[key]) for key in approved_pose) > 2:
            raise PlayerError("승인 화면 이후 자세가 2단위 넘게 변했습니다. 새로 확인하세요.")
        if approved_raw is not None:
            for joint in JOINTS:
                ticks = abs(sample["raw"][joint] - approved_raw[joint])
                units = ticks * (100 / (self.calibration.data[joint]["range_max"] -
                                       self.calibration.data[joint]["range_min"]) if joint == "gripper" else 360 / 4095)
                if units > 2:
                    raise PlayerError(f"{joint}: raw position changed after approval; restart")
        if first_goal is not None:
            check_startup_path(sample["pose"], first_goal)
        plan = sample["startup_recovery"]
        limits = {j: (min(p["raw"], self.calibration.data[j]["range_min"]),
                      max(p["raw"], self.calibration.data[j]["range_max"])) for j, p in plan.items()}
        goals = {}
        if plan and report:
            report("startup_recovery_begin", plan=plan)
        for joint in JOINTS:
            if poll:
                poll()
            check_stop(self.stop)
            raw = self.bus.read(joint, "Present_Position")
            self._check_raw(joint, raw, limits)
            if joint in plan and abs(raw - sample["raw"][joint]) > 2:
                raise PlayerError(f"{joint}: startup position changed; restart the check")
            units = self.calibration.to_units(joint, raw)
            if abs(units - sample["pose"][joint + ".pos"]) > 2:
                raise PlayerError(f"{joint}: 활성화 도중 자세가 변했습니다.")
            for register in ("Torque_Enable", "Operating_Mode", "Status"):
                if self.bus.read(joint, register) != 0:
                    raise PlayerError(f"{joint}: 활성화 직전 {register}가 0이 아닙니다.")
            if self.bus.read(joint, "Present_Temperature") >= self.temperature_limits[joint]:
                raise PlayerError(f"{joint}: 활성화 직전 온도 경보")
            # A goal write can itself energize this hardware, even before Torque_Enable=1.
            check_stop(self.stop)
            # The initial goal must itself be inside the unchanged hardware limits.
            entry = self.calibration.data[joint]
            goal = max(entry["range_min"], min(entry["range_max"], raw))
            if joint in plan:
                goal = plan[joint]["target_raw"]
            goals[joint] = goal
            self.motion_attempted = True
            speed = STARTUP_SPEED_TICKS if joint in plan else 0
            commanded_at = self.clock()
            # Profile speed and the in-range goal travel in the SAME packet.
            if joint in plan or self.bus.read(joint, "Goal_Velocity") != 0:
                self.bus.write_position_profile(joint, goal, speed)
            else:
                self.bus.write(joint, "Goal_Position", goal)
            if self.bus.read(joint, "Goal_Velocity") != speed or self.bus.read(joint, "Goal_Time") != 0:
                raise PlayerError(f"{joint}: startup speed profile readback failed")
            if self.bus.read(joint, "Goal_Position") != goal:
                raise PlayerError(f"{joint}: 초기 목표 읽기 확인 실패")
            check_stop(self.stop)
            self.bus.write(joint, "Lock", 1)
            check_stop(self.stop)
            if self.bus.read(joint, "Torque_Enable") == 0:
                self.bus.write(joint, "Torque_Enable", 1)
            if self.bus.read(joint, "Torque_Enable") != 1 or self.bus.read(joint, "Lock") != 1:
                raise PlayerError(f"{joint}: 활성화 확인 실패")
            after = self.bus.feedback(joint)
            if joint in plan and abs(after["raw"] - raw) > 2 * STARTUP_SPEED_TICKS * (self.clock() - commanded_at) + 8:
                raise PlayerError(f"{joint}: startup movement exceeded low-speed envelope")
            self._check_raw(joint, after["raw"], limits)
            if (after["status"] != 0 or after["temperature"] >= self.temperature_limits[joint]
                    or abs(self.calibration.to_units(joint, after["raw"]) - units) > 5):
                raise PlayerError(f"{joint}: 초기 활성화 중 이상 감지")
        self.active = True
        current = self.sample(force_health=True, startup_limits=limits)
        if max(abs(current["pose"][key] - sample["pose"][key]) for key in approved_pose) > 5:
            raise PlayerError("초기 활성화 중 위치가 5단위 넘게 변했습니다.")
        if plan:
            current = self._recover_startup(sample, plan, limits, goals, wait or self.stop.wait, poll, report)
        if first_goal is not None:
            check_startup_path(current["pose"], first_goal)
        return current

    def _recover_startup(self, initial, plan, limits, goals, wait, poll, report):
        """Monitor hardware-limited inward travel; no EEPROM or out-of-range goals."""
        started = last_tick = self.clock()
        timeout = 10 + 2 * max(abs(p["target_raw"] - p["raw"]) / STARTUP_SPEED_TICKS for p in plan.values())
        settled = None
        interval = max(.05, 1 / self.motion.hz)
        previous = self.latest
        best = {j: abs(p["target_raw"] - previous["raw"][j]) for j, p in plan.items()}
        progressed_at = dict.fromkeys(plan, started)
        while True:
            if poll:
                poll()
            check_stop(self.stop)
            now = self.clock()
            if now - started >= timeout or not 0 <= now - last_tick <= .5:
                raise PlayerError("Startup recovery timeout; playback cancelled")
            current = self.sample(force_health=True, startup_limits=limits)
            for joint in JOINTS:
                if joint in plan:
                    target = plan[joint]["target_raw"]
                    raw = current["raw"][joint]
                    dt = current["time"] - previous["time"]
                    if abs(raw - previous["raw"][joint]) > 2 * STARTUP_SPEED_TICKS * dt + 8:
                        raise PlayerError(f"{joint}: startup movement exceeded low-speed envelope")
                    direction = 1 if target > plan[joint]["raw"] else -1
                    if direction * (raw - target) > 8:
                        raise PlayerError(f"{joint}: startup target overshoot")
                    error = abs(target - raw)
                    if error > best[joint] + 4:
                        raise PlayerError(f"{joint}: outward drift during startup recovery")
                    if error <= best[joint] - 2:
                        best[joint], progressed_at[joint] = error, now
                    arrived = (self.calibration.data[joint]["range_min"] + STARTUP_BOUNDARY_CLEARANCE_TICKS <= raw
                               <= self.calibration.data[joint]["range_max"] - STARTUP_BOUNDARY_CLEARANCE_TICKS
                               and error <= STARTUP_ARRIVAL_TICKS)
                    if not arrived and now - progressed_at[joint] >= 4:
                        raise PlayerError(f"{joint}: startup recovery stalled")
                    if (self.bus.read(joint, "Goal_Velocity") != STARTUP_SPEED_TICKS
                            or self.bus.read(joint, "Goal_Time") != 0
                            or self.bus.read(joint, "Goal_Position") != target):
                        raise PlayerError(f"{joint}: startup speed/goal changed")
                elif abs(current["pose"][joint + ".pos"] - initial["pose"][joint + ".pos"]) > 5:
                    raise PlayerError(f"{joint}: position changed during startup recovery")
            inside = all(self.calibration.data[j]["range_min"] + STARTUP_BOUNDARY_CLEARANCE_TICKS <= current["raw"][j]
                         <= self.calibration.data[j]["range_max"] - STARTUP_BOUNDARY_CLEARANCE_TICKS
                         and abs(current["raw"][j] - p["target_raw"]) <= STARTUP_ARRIVAL_TICKS
                         for j, p in plan.items())
            if report:
                report("startup_recovery_sample", **current, goals_raw=dict(goals))
            if inside:
                settled = now if settled is None else settled
                if now - settled >= .3:
                    current = self.sample(force_health=True)
                    # Reset velocity only with a fresh CURRENT goal in the same
                    # packet. Never release the cap while a distant goal remains.
                    for joint in plan:
                        if poll:
                            poll()
                        check_stop(self.stop)
                        fresh = self.sample(force_health=True)
                        raw = fresh["raw"][joint]
                        entry = self.calibration.data[joint]
                        if (abs(raw - plan[joint]["target_raw"]) > STARTUP_ARRIVAL_TICKS
                                or not entry["range_min"] + STARTUP_BOUNDARY_CLEARANCE_TICKS <= raw
                                <= entry["range_max"] - STARTUP_BOUNDARY_CLEARANCE_TICKS):
                            raise PlayerError(f"{joint}: position changed before restoring speed")
                        self.bus.write_position_profile(joint, raw, 0)
                        if (self.bus.read(joint, "Goal_Position") != raw
                                or self.bus.read(joint, "Goal_Velocity") != 0
                                or self.bus.read(joint, "Goal_Time") != 0):
                            raise PlayerError(f"{joint}: normal speed restore failed")
                    current = self.sample(force_health=True)
                    if report:
                        report("startup_recovery_complete", **current)
                    return current
            else:
                settled = None
            previous, last_tick = current, now
            wait(interval)

    def send(self, target):
        check_stop(self.stop)
        if not self.active:
            raise PlayerError("활성화되지 않은 팔에 목표를 보낼 수 없습니다.")
        target = self.calibration.validate_pose(target)
        if self.latest is None or self.clock() - self.latest["time"] > .5:
            raise PlayerError("위치 표본이 오래되어 목표 전송을 중단합니다.")
        for key, value in target.items():
            if abs(value - self.latest["pose"][key]) > self.motion.relative_limit + 1e-8:
                raise PlayerError(f"{key}: 현재 위치 대비 목표 차이가 제한을 넘었습니다.")
        sent = {}
        for joint in JOINTS:
            check_stop(self.stop)
            if self.clock() - self.latest["time"] > .5:
                raise PlayerError("목표 전송 중 위치 표본이 만료됐습니다.")
            raw = self.calibration.to_raw(joint, target[joint + ".pos"])
            self.bus.write(joint, "Goal_Position", raw)
            sent[joint + ".pos"] = self.calibration.to_units(joint, raw)
        return sent

    def close(self):
        errors = []
        try:
            if self.motion_attempted:
                # Ignore cancellation here; every motor needs its own OFF attempt and readback.
                for joint in JOINTS:
                    try:
                        self.bus.write(joint, "Torque_Enable", 0)
                    except BaseException as exc:
                        errors.append(f"{joint} 토크 OFF 쓰기: {exc}")
                    try:
                        if self.bus.read(joint, "Torque_Enable") != 0:
                            raise PlayerError("토크 OFF 읽기 확인 실패")
                    except BaseException as exc:
                        errors.append(f"{joint} 토크 OFF 확인: {exc}")
        finally:
            try:
                self.bus.close()
            except BaseException as exc:
                errors.append(f"포트 닫기: {exc}")
            self.active = False
        return errors
