from .ltpacket_def import *

LTPACKET_MAGIC = 0x78
LTPACKET_MAX_LENGTH = 1024


def encode(packet, frame=True):
    # This is a hot path, so we use a dictionary instead of a function call
    # id_ = LTPacketType.from_type(type(packet)).value
    id_ = type_map_rev[type(packet)].value
    data = id_.to_bytes(1, 'little') + packet.to_bytes()
    if frame:
        crc = crc16(data).to_bytes(2, 'little')
        length = (len(data) + 5).to_bytes(2, 'little')
        magic = LTPACKET_MAGIC.to_bytes(1, 'little')
        data = magic + length + data + crc
    return data


def decode(byts, frame=True):
    if len(byts) < 1:
        return None
    if frame:
        if byts[0] != LTPACKET_MAGIC:
            raise ValueError('Invalid magic')
        length = int.from_bytes(byts[1:3], 'little')
        if len(byts) < length:
            return None
        crc = int.from_bytes(byts[-2:], 'little')
        byts = byts[3:-2]
        if crc != crc16(byts):
            return ValueError('Invalid CRC')
    # To save a function call, this is a hot path
    # typ = LTPacketType(byts[0]).to_type()
    typ = type_map[LTPacketType(byts[0])]
    return typ.from_bytes(byts[1:])


def crc16(data):
    crc = 0xFFFF
    crc = crc16_continue(crc, data)
    return crc


def crc16_continue(crc, data):
    for byte in data:
        crc ^= byte
        for i in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


if __name__ == "__main__":
    imu = Imu(1.0, 2.0, 3.0, 4.0)
    print(imu)
    byts = encode(imu)
    print(byts)
    imu2 = decode(byts)
    print(imu2)
    print(imu == imu2)
