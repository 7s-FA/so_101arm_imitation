"""Strict teaching/calibration validation and LeRobot 0.6.1 position units."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
KEYS = tuple(j + ".pos" for j in JOINTS)
UNITS = {k: "normalized_0_100" if k == "gripper.pos" else "degrees" for k in KEYS}
MAX_FILE_BYTES = 2 * 1024 * 1024


class PlayerError(Exception):
    pass


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PlayerError(f"중복 JSON 키: {key}")
        result[key] = value
    return result


def read_json(path):
    with Path(path).open("rb") as stream:
        raw = stream.read(MAX_FILE_BYTES + 1)
    if len(raw) > MAX_FILE_BYTES:
        raise PlayerError("JSON 파일은 2 MiB 이하여야 합니다.")
    try:
        return json.loads(raw, object_pairs_hook=_pairs,
                          parse_constant=lambda value: (_ for _ in ()).throw(PlayerError(f"잘못된 숫자: {value}")))
    except (ValueError, UnicodeError) as exc:
        raise PlayerError(f"JSON 읽기 실패: {exc}") from exc


def integer(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise PlayerError(f"{name}: 정수 {low}~{high} 필요")
    return value


def number(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high or not math.isfinite(value):
        raise PlayerError(f"{name}: 유한한 숫자 {low}~{high} 필요")
    return float(value)


@dataclass(frozen=True)
class Motion:
    hz: float = 20
    body_rate: float = 10
    gripper_rate: float = 10
    first_step_rate: float = 5
    relative_limit: float = 10
    temperature_limit_raw: int = 65
    temperature_duration_s: float = 5.0

    def __post_init__(self):
        for name, low, high in (("hz", 5, 30), ("body_rate", .1, 30), ("gripper_rate", .1, 40),
                                ("first_step_rate", .1, 5), ("relative_limit", .1, 10)):
            number(getattr(self, name), low, high, name)
        integer(self.temperature_limit_raw, 1, 65, "temperature_limit_raw")
        number(self.temperature_duration_s, 0, 5, "temperature_duration_s")


@dataclass(frozen=True)
class Config:
    port: str
    calibration_file: Path
    calibration_id: str
    motion: Motion

    @property
    def serial(self):
        return Path(self.port).name.removeprefix("usb-1a86_USB_Single_Serial_").removesuffix("-if00")

    @classmethod
    def load(cls, path):
        path = Path(path).expanduser().resolve()
        data = read_json(path)
        if not isinstance(data, dict) or set(data) - {"schema", "port", "calibration_file", "calibration_id", "motion"}:
            raise PlayerError("robot.json 설정 키를 확인하세요.")
        if type(data.get("schema")) is not int or data["schema"] != 1:
            raise PlayerError("robot.json schema는 1이어야 합니다.")
        port = data.get("port", "")
        if not isinstance(port, str) or not re.fullmatch(r"/dev/serial/by-id/usb-1a86_USB_Single_Serial_[A-Za-z0-9]+-if00", port):
            raise PlayerError("확인된 USB 보드의 /dev/serial/by-id/ 경로가 필요합니다.")
        cal_id = data.get("calibration_id")
        if cal_id not in ("follower_left", "follower_right"):
            raise PlayerError("calibration_id는 follower_left 또는 follower_right여야 합니다.")
        cal_file = data.get("calibration_file")
        if not isinstance(cal_file, str) or not cal_file.strip():
            raise PlayerError("calibration_file이 필요합니다.")
        try:
            motion = Motion(**data.get("motion", {}))
        except TypeError as exc:
            raise PlayerError(f"motion 설정 오류: {exc}") from exc
        return cls(port, (path.parent / Path(cal_file).expanduser()).resolve(), cal_id, motion)


class Calibration:
    def __init__(self, data):
        if not isinstance(data, dict) or set(data) != set(JOINTS):
            raise PlayerError("보정 파일에 정확히 6개 관절이 있어야 합니다.")
        for motor_id, joint in enumerate(JOINTS, 1):
            entry = data[joint]
            if not isinstance(entry, dict):
                raise PlayerError(f"잘못된 보정 항목: {joint}")
            integer(entry.get("id"), motor_id, motor_id, joint + ".id")
            integer(entry.get("drive_mode"), 0, 1, joint + ".drive_mode")
            integer(entry.get("homing_offset"), -2047, 2047, joint + ".homing_offset")
            lo = integer(entry.get("range_min"), 0, 4094, joint + ".range_min")
            integer(entry.get("range_max"), lo + 1, 4095, joint + ".range_max")
        self.data = data
        # Identical canonical serialization to the existing teaching GUI.
        self.sha256 = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
        self.bounds = {j + ".pos": tuple(sorted((self.to_units(j, data[j]["range_min"]),
                                                self.to_units(j, data[j]["range_max"])))) for j in JOINTS}

    @classmethod
    def load(cls, path):
        return cls(read_json(path))

    def to_units(self, joint, raw):
        entry = self.data[joint]
        lo, hi = entry["range_min"], entry["range_max"]
        if joint == "gripper":
            result = (min(hi, max(lo, raw)) - lo) * 100 / (hi - lo)
            return 100 - result if entry["drive_mode"] else result
        # Homing offset is already applied in the servo. Do not apply it twice.
        return (raw - (lo + hi) / 2) * 360 / 4095

    def to_raw(self, joint, value):
        entry = self.data[joint]
        lo, hi = entry["range_min"], entry["range_max"]
        if joint == "gripper":
            value = 100 - value if entry["drive_mode"] else value
            raw = int(min(100, max(0, value)) / 100 * (hi - lo) + lo)
        else:
            raw = int(value * 4095 / 360 + (lo + hi) / 2)
        return max(lo, min(hi, raw))

    def check_raw(self, joint, raw):
        entry = self.data[joint]
        if type(raw) is not int or not entry["range_min"] <= raw <= entry["range_max"]:
            raise PlayerError(f"{joint}: 실측 위치 {raw}가 보정 범위 [{entry['range_min']}, {entry['range_max']}] 밖입니다. "
                              "팔을 받치고 전원을 끈 상태에서 범위 안으로 옮긴 뒤 다시 확인하세요.")

    def validate_pose(self, pose):
        if not isinstance(pose, dict) or set(pose) != set(KEYS):
            raise PlayerError("스텝에는 정확히 6개 .pos 관절값이 필요합니다.")
        result = {}
        for key in KEYS:
            lo, hi = self.bounds[key]
            value = number(pose[key], max(lo, 0 if key == "gripper.pos" else -180),
                           min(hi, 100 if key == "gripper.pos" else 180), key)
            result[key] = value
        return result


@dataclass(frozen=True)
class Program:
    title: str
    steps: tuple
    source: Path
    sha256: str

    @classmethod
    def load(cls, path, config, calibration):
        data = read_json(path)
        if not isinstance(data, dict) or type(data.get("schema")) is not int or data["schema"] != 1:
            raise PlayerError("지원하는 티칭 파일은 schema=1입니다.")
        if data.get("kind") not in (None, "teaching"):
            raise PlayerError("에피소드 저장으로 만든 티칭 파일을 선택하세요. 세션 로그/초안은 재생하지 않습니다.")
        identity = data.get("identity")
        if not isinstance(identity, dict) or identity.get("demo") is not False:
            raise PlayerError("실물에서 저장한 파일만 사용할 수 있습니다. DEMO 파일은 거부합니다.")
        serials = identity.get("serials", {})
        recorded = serials.get("follower") if isinstance(serials, dict) else None
        if recorded not in (config.serial, Path(config.port).name):
            raise PlayerError(f"다른 팔로워의 파일입니다: 기록={recorded}, 설정={config.serial}")
        hashes = identity.get("calibration_sha256", {})
        if not isinstance(hashes, dict) or hashes.get(config.calibration_id) != calibration.sha256:
            raise PlayerError("티칭 파일과 현재 팔로워 보정값의 해시가 다릅니다. 해당 보정 파일 또는 새 티칭이 필요합니다.")
        if data.get("units") != UNITS:
            raise PlayerError("관절 단위가 다릅니다. 몸체=degrees, 그리퍼=normalized_0_100이 필요합니다.")
        title = data.get("title")
        if not isinstance(title, str) or not 1 <= len(title.strip()) <= 80:
            raise PlayerError("저장된 에피소드 제목(1~80자)이 필요합니다.")
        steps = data.get("steps")
        if not isinstance(steps, list) or not 1 <= len(steps) <= 500:
            raise PlayerError("스텝은 1~500개여야 합니다.")
        clean = []
        for step in steps:
            if not isinstance(step, dict) or not isinstance(step.get("name"), str) or not 1 <= len(step["name"].strip()) <= 80:
                raise PlayerError("각 스텝에 이름(1~80자)이 필요합니다.")
            clean.append({"name": step["name"], "pose": calibration.validate_pose(step.get("pose"))})
        check_wrist([step["pose"] for step in clean])
        digest = hashlib.sha256(json.dumps(data, sort_keys=True, ensure_ascii=False).encode()).hexdigest()
        return cls(title.strip(), tuple(clean), Path(path).resolve(), digest)

    def select(self, number_):
        if number_ is None:
            return self
        integer(number_, 1, len(self.steps), "step")
        return Program(self.title, (self.steps[number_ - 1],), self.source, self.sha256)


def check_wrist(poses):
    for start, end in zip(poses, poses[1:]):
        if abs(end["wrist_roll.pos"] - start["wrist_roll.pos"]) > 90:
            raise PlayerError("손목 회전 차이가 90도를 넘는 이동은 차단합니다. 중간 자세를 추가하세요.")


def check_startup_path(start, goal):
    """Allow a half-turn for initial alignment, with one encoder tick of rounding.

    Follow the signed difference in the existing calibrated coordinates; never
    wrap angles or change zero offsets. Ordinary step-to-step limits are separate.
    """
    for joint in JOINTS[:-1]:
        key = joint + ".pos"
        if abs(goal[key] - start[key]) > 180 + 360 / 4095 + 1e-8:
            raise PlayerError(f"{joint}: startup alignment exceeds 180 degrees; check the starting pose")
