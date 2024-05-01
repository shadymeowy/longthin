import struct
from dataclasses import dataclass
from enum import Enum


@dataclass
class Reserved:
    reserved: int

    @staticmethod
    def from_bytes(data):
        return Reserved(*reserved_struct.unpack(data))

    def to_bytes(self):
        return reserved_struct.pack(
            self.reserved,
        )

    @property
    def type(self):
        return LTPacketType.RESERVED



@dataclass
class Imu:
    qw: float
    qx: float
    qy: float
    qz: float
    dvx: float
    dvy: float
    dvz: float
    dt: float

    @staticmethod
    def from_bytes(data):
        return Imu(*imu_struct.unpack(data))

    def to_bytes(self):
        return imu_struct.pack(
            self.qw,
            self.qx,
            self.qy,
            self.qz,
            self.dvx,
            self.dvy,
            self.dvz,
            self.dt,
        )

    @property
    def type(self):
        return LTPacketType.IMU



@dataclass
class ImuRaw:
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    mag_x: float
    mag_y: float
    mag_z: float

    @staticmethod
    def from_bytes(data):
        return ImuRaw(*imu_raw_struct.unpack(data))

    def to_bytes(self):
        return imu_raw_struct.pack(
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
            self.mag_x,
            self.mag_y,
            self.mag_z,
        )

    @property
    def type(self):
        return LTPacketType.IMU_RAW



@dataclass
class Motor:
    left: float
    right: float

    @staticmethod
    def from_bytes(data):
        return Motor(*motor_struct.unpack(data))

    def to_bytes(self):
        return motor_struct.pack(
            self.left,
            self.right,
        )

    @property
    def type(self):
        return LTPacketType.MOTOR



@dataclass
class MotorRaw:
    left: int
    right: int

    @staticmethod
    def from_bytes(data):
        return MotorRaw(*motor_raw_struct.unpack(data))

    def to_bytes(self):
        return motor_raw_struct.pack(
            self.left,
            self.right,
        )

    @property
    def type(self):
        return LTPacketType.MOTOR_RAW



@dataclass
class Setpoint:
    vel: float
    yaw: float

    @staticmethod
    def from_bytes(data):
        return Setpoint(*setpoint_struct.unpack(data))

    def to_bytes(self):
        return setpoint_struct.pack(
            self.vel,
            self.yaw,
        )

    @property
    def type(self):
        return LTPacketType.SETPOINT



@dataclass
class Setparam:
    param: int
    value: float

    @staticmethod
    def from_bytes(data):
        return Setparam(*setparam_struct.unpack(data))

    def to_bytes(self):
        return setparam_struct.pack(
            self.param,
            self.value,
        )

    @property
    def type(self):
        return LTPacketType.SETPARAM



@dataclass
class Led:
    index: int
    state: int

    @staticmethod
    def from_bytes(data):
        return Led(*led_struct.unpack(data))

    def to_bytes(self):
        return led_struct.pack(
            self.index,
            self.state,
        )

    @property
    def type(self):
        return LTPacketType.LED



@dataclass
class ControlDebug:
    current_yaw: float
    desired_d: float
    desired_yaw: float
    u_v: float
    u_w: float
    u_r: float
    u_l: float

    @staticmethod
    def from_bytes(data):
        return ControlDebug(*control_debug_struct.unpack(data))

    def to_bytes(self):
        return control_debug_struct.pack(
            self.current_yaw,
            self.desired_d,
            self.desired_yaw,
            self.u_v,
            self.u_w,
            self.u_r,
            self.u_l,
        )

    @property
    def type(self):
        return LTPacketType.CONTROL_DEBUG



@dataclass
class Setparamu:
    param: int
    value: int

    @staticmethod
    def from_bytes(data):
        return Setparamu(*setparamu_struct.unpack(data))

    def to_bytes(self):
        return setparamu_struct.pack(
            self.param,
            self.value,
        )

    @property
    def type(self):
        return LTPacketType.SETPARAMU



@dataclass
class Setparami:
    param: int
    value: int

    @staticmethod
    def from_bytes(data):
        return Setparami(*setparami_struct.unpack(data))

    def to_bytes(self):
        return setparami_struct.pack(
            self.param,
            self.value,
        )

    @property
    def type(self):
        return LTPacketType.SETPARAMI



@dataclass
class MotorOutput:
    left: float
    right: float

    @staticmethod
    def from_bytes(data):
        return MotorOutput(*motor_output_struct.unpack(data))

    def to_bytes(self):
        return motor_output_struct.pack(
            self.left,
            self.right,
        )

    @property
    def type(self):
        return LTPacketType.MOTOR_OUTPUT



@dataclass
class Reboot:
    reserved: int

    @staticmethod
    def from_bytes(data):
        return Reboot(*reboot_struct.unpack(data))

    def to_bytes(self):
        return reboot_struct.pack(
            self.reserved,
        )

    @property
    def type(self):
        return LTPacketType.REBOOT



@dataclass
class EvPose:
    x: float
    y: float
    yaw: float

    @staticmethod
    def from_bytes(data):
        return EvPose(*ev_pose_struct.unpack(data))

    def to_bytes(self):
        return ev_pose_struct.pack(
            self.x,
            self.y,
            self.yaw,
        )

    @property
    def type(self):
        return LTPacketType.EV_POSE



@dataclass
class SimState:
    w: float
    v: float
    yaw: float
    x: float
    y: float
    rate: float

    @staticmethod
    def from_bytes(data):
        return SimState(*sim_state_struct.unpack(data))

    def to_bytes(self):
        return sim_state_struct.pack(
            self.w,
            self.v,
            self.yaw,
            self.x,
            self.y,
            self.rate,
        )

    @property
    def type(self):
        return LTPacketType.SIM_STATE



@dataclass
class EkfState:
    x: float
    y: float
    yaw: float

    @staticmethod
    def from_bytes(data):
        return EkfState(*ekf_state_struct.unpack(data))

    def to_bytes(self):
        return ekf_state_struct.pack(
            self.x,
            self.y,
            self.yaw,
        )

    @property
    def type(self):
        return LTPacketType.EKF_STATE



@dataclass
class SetpointPos:
    x: float
    y: float

    @staticmethod
    def from_bytes(data):
        return SetpointPos(*setpoint_pos_struct.unpack(data))

    def to_bytes(self):
        return setpoint_pos_struct.pack(
            self.x,
            self.y,
        )

    @property
    def type(self):
        return LTPacketType.SETPOINT_POS



@dataclass
class GoalVision:
    area: float
    center_x: float
    center_y: float

    @staticmethod
    def from_bytes(data):
        return GoalVision(*goal_vision_struct.unpack(data))

    def to_bytes(self):
        return goal_vision_struct.pack(
            self.area,
            self.center_x,
            self.center_y,
        )

    @property
    def type(self):
        return LTPacketType.GOAL_VISION



@dataclass
class LaneVision:
    mean_x: float
    min_y: float

    @staticmethod
    def from_bytes(data):
        return LaneVision(*lane_vision_struct.unpack(data))

    def to_bytes(self):
        return lane_vision_struct.pack(
            self.mean_x,
            self.min_y,
        )

    @property
    def type(self):
        return LTPacketType.LANE_VISION



@dataclass
class ButtonState:
    index: int
    state: int

    @staticmethod
    def from_bytes(data):
        return ButtonState(*button_state_struct.unpack(data))

    def to_bytes(self):
        return button_state_struct.pack(
            self.index,
            self.state,
        )

    @property
    def type(self):
        return LTPacketType.BUTTON_STATE



@dataclass
class EkfReset:
    reserved: int

    @staticmethod
    def from_bytes(data):
        return EkfReset(*ekf_reset_struct.unpack(data))

    def to_bytes(self):
        return ekf_reset_struct.pack(
            self.reserved,
        )

    @property
    def type(self):
        return LTPacketType.EKF_RESET



@dataclass
class LedControl:
    id: int
    default_state: int
    high_time: int
    low_time: int
    remaining_cycles: int

    @staticmethod
    def from_bytes(data):
        return LedControl(*led_control_struct.unpack(data))

    def to_bytes(self):
        return led_control_struct.pack(
            self.id,
            self.default_state,
            self.high_time,
            self.low_time,
            self.remaining_cycles,
        )

    @property
    def type(self):
        return LTPacketType.LED_CONTROL



@dataclass
class AdcRead:
    id: int
    value: float

    @staticmethod
    def from_bytes(data):
        return AdcRead(*adc_read_struct.unpack(data))

    def to_bytes(self):
        return adc_read_struct.pack(
            self.id,
            self.value,
        )

    @property
    def type(self):
        return LTPacketType.ADC_READ



reserved_struct = struct.Struct('B')
imu_struct = struct.Struct('ffffffff')
imu_raw_struct = struct.Struct('fffffffff')
motor_struct = struct.Struct('ff')
motor_raw_struct = struct.Struct('hh')
setpoint_struct = struct.Struct('ff')
setparam_struct = struct.Struct('if')
led_struct = struct.Struct('BB')
control_debug_struct = struct.Struct('fffffff')
setparamu_struct = struct.Struct('iI')
setparami_struct = struct.Struct('ii')
motor_output_struct = struct.Struct('ff')
reboot_struct = struct.Struct('B')
ev_pose_struct = struct.Struct('fff')
sim_state_struct = struct.Struct('ffffff')
ekf_state_struct = struct.Struct('fff')
setpoint_pos_struct = struct.Struct('ff')
goal_vision_struct = struct.Struct('fff')
lane_vision_struct = struct.Struct('ff')
button_state_struct = struct.Struct('BB')
ekf_reset_struct = struct.Struct('B')
led_control_struct = struct.Struct('BBIIi')
adc_read_struct = struct.Struct('Bf')

class LTPacketType(Enum):
    RESERVED = 0
    IMU = 1
    IMU_RAW = 2
    MOTOR = 6
    MOTOR_RAW = 7
    SETPOINT = 8
    SETPARAM = 10
    LED = 11
    CONTROL_DEBUG = 12
    SETPARAMU = 13
    SETPARAMI = 14
    MOTOR_OUTPUT = 15
    REBOOT = 16
    EV_POSE = 17
    SIM_STATE = 18
    EKF_STATE = 19
    SETPOINT_POS = 20
    GOAL_VISION = 21
    LANE_VISION = 22
    BUTTON_STATE = 23
    EKF_RESET = 24
    LED_CONTROL = 25
    ADC_READ = 26

    @staticmethod
    def from_type(type_):
        return type_map_rev[type_]
    
    def to_type(self):
        return type_map[self]

type_map = {
    LTPacketType.RESERVED: Reserved,
    LTPacketType.IMU: Imu,
    LTPacketType.IMU_RAW: ImuRaw,
    LTPacketType.MOTOR: Motor,
    LTPacketType.MOTOR_RAW: MotorRaw,
    LTPacketType.SETPOINT: Setpoint,
    LTPacketType.SETPARAM: Setparam,
    LTPacketType.LED: Led,
    LTPacketType.CONTROL_DEBUG: ControlDebug,
    LTPacketType.SETPARAMU: Setparamu,
    LTPacketType.SETPARAMI: Setparami,
    LTPacketType.MOTOR_OUTPUT: MotorOutput,
    LTPacketType.REBOOT: Reboot,
    LTPacketType.EV_POSE: EvPose,
    LTPacketType.SIM_STATE: SimState,
    LTPacketType.EKF_STATE: EkfState,
    LTPacketType.SETPOINT_POS: SetpointPos,
    LTPacketType.GOAL_VISION: GoalVision,
    LTPacketType.LANE_VISION: LaneVision,
    LTPacketType.BUTTON_STATE: ButtonState,
    LTPacketType.EKF_RESET: EkfReset,
    LTPacketType.LED_CONTROL: LedControl,
    LTPacketType.ADC_READ: AdcRead,
}
type_map_rev = {v: k for k, v in type_map.items()}