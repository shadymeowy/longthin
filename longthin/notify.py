import enum
import numpy as np

from .ltpacket import *


class LedType(enum.Enum):
    LED_BUILTIN = 0
    LED_RED = 1
    LED_GREEN = 2
    LED_BLUE = 3
    LED_BUZZER = 4


class Notify:
    def __init__(self, node):
        self.node = node

    def pulse(self, led_type, high_time=0, low_time=0, remaining_cycles=-1, default_state=False):
        if isinstance(led_type, (list, tuple)):
            for led in led_type:
                self.pulse(led, high_time, low_time, remaining_cycles, default_state)
            return
        packet = LedControl(
            id=led_type.value,
            high_time=high_time,
            low_time=low_time,
            remaining_cycles=remaining_cycles,
            default_state=default_state
        )
        self.node.publish(packet)

    def set(self, led_type, state):
        if isinstance(led_type, (list, tuple)):
            for led in led_type:
                self.set(led, state)
            return
        packet = Led(
            index=led_type.value,
            state=state
        )
        self.node.publish(packet)

    def off(self, led_type):
        self.set(led_type, False)

    def on(self, led_type):
        self.set(led_type, True)

    def beeping(self, percentage):
        percentage = np.clip(percentage, 0, 1)
        # TODO: Move those to config
        low_time = int(350 * percentage)
        self.pulse(LedType.LED_BUZZER, 50, low_time)
        self.pulse(LedType.LED_BLUE, 50, low_time)

    def idle(self):
        self.pulse(LedType.LED_RED, 0, 0, 0, default_state=False)
        self.pulse(LedType.LED_GREEN, 0, 0, 0, default_state=True)
        self.pulse(LedType.LED_BLUE, 0, 0, 0, default_state=False)
        self.pulse(LedType.LED_BUZZER, 0, 0, 0, default_state=False)

    def success(self):
        self.pulse(LedType.LED_RED, 1200, 0, 1, default_state=False)
        self.pulse(LedType.LED_GREEN, 0, 0, 0, default_state=True)
        self.pulse(LedType.LED_BLUE, 1200, 0, 1, default_state=False)
        self.pulse(LedType.LED_BUZZER, 200, 200, 3, default_state=False)

    def info(self):
        self.pulse(LedType.LED_RED, 0, 0, 0, default_state=False)
        self.pulse(LedType.LED_GREEN, 0, 0, 0, default_state=True)
        self.pulse(LedType.LED_BLUE, 400, 0, 1, default_state=False)
        self.pulse(LedType.LED_BUZZER, 100, 100, 2, default_state=False)

    def error(self):
        self.pulse(LedType.LED_RED, 4000, 0, 1, default_state=False)
        self.pulse(LedType.LED_GREEN, 0, 4000, 1, default_state=True)
        self.pulse(LedType.LED_BLUE, 4000, 0, 1, default_state=False)
        self.pulse(LedType.LED_BUZZER, 100, 100, 20, default_state=False)
