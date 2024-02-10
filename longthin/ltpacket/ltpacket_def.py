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
        return LTPACKET_TYPE.RESERVED



@dataclass
class Imu:
    roll: float
    pitch: float
    yaw: float
    vel: float

    @staticmethod
    def from_bytes(data):
        return Imu(*imu_struct.unpack(data))

    def to_bytes(self):
        return imu_struct.pack(
            self.roll,
            self.pitch,
            self.yaw,
            self.vel,
        )

    @property
    def type(self):
        return LTPACKET_TYPE.IMU



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
        return LTPACKET_TYPE.IMU_RAW



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
        return LTPACKET_TYPE.MOTOR



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
        return LTPACKET_TYPE.MOTOR_RAW



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
        return LTPACKET_TYPE.SETPOINT



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
        return LTPACKET_TYPE.SETPARAM



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
        return LTPACKET_TYPE.LED



@dataclass
class ControlDebug:
    current_d: float
    current_yaw: float
    desired_d: float
    desired_yaw: float
    current_vel: float
    current_w: float
    desired_vel: float
    desired_w: float
    u_v: float
    u_w: float
    u_r: float
    u_l: float

    @staticmethod
    def from_bytes(data):
        return ControlDebug(*control_debug_struct.unpack(data))

    def to_bytes(self):
        return control_debug_struct.pack(
            self.current_d,
            self.current_yaw,
            self.desired_d,
            self.desired_yaw,
            self.current_vel,
            self.current_w,
            self.desired_vel,
            self.desired_w,
            self.u_v,
            self.u_w,
            self.u_r,
            self.u_l,
        )

    @property
    def type(self):
        return LTPACKET_TYPE.CONTROL_DEBUG



reserved_struct = struct.Struct('B')
imu_struct = struct.Struct('ffff')
imu_raw_struct = struct.Struct('fffffffff')
motor_struct = struct.Struct('ff')
motor_raw_struct = struct.Struct('hh')
setpoint_struct = struct.Struct('ff')
setparam_struct = struct.Struct('if')
led_struct = struct.Struct('BB')
control_debug_struct = struct.Struct('ffffffffffff')

class LTPACKET_TYPE(Enum):
    RESERVED = 0
    IMU = 1
    IMU_RAW = 2
    MOTOR = 6
    MOTOR_RAW = 7
    SETPOINT = 8
    SETPARAM = 10
    LED = 11
    CONTROL_DEBUG = 12

type_map = {
    LTPACKET_TYPE.RESERVED: Reserved,
    LTPACKET_TYPE.IMU: Imu,
    LTPACKET_TYPE.IMU_RAW: ImuRaw,
    LTPACKET_TYPE.MOTOR: Motor,
    LTPACKET_TYPE.MOTOR_RAW: MotorRaw,
    LTPACKET_TYPE.SETPOINT: Setpoint,
    LTPACKET_TYPE.SETPARAM: Setparam,
    LTPACKET_TYPE.LED: Led,
    LTPACKET_TYPE.CONTROL_DEBUG: ControlDebug,
}
type_map_rev = {v: k for k, v in type_map.items()}