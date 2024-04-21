import numpy as np
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister
import time


class SHMVideoWriter:
    def __init__(self, name, width, height):
        self.name = name
        self.size = width*height*3

        try:
            self.memory = shared_memory.SharedMemory(name=name, create=False)
        except FileNotFoundError:
            self.memory = shared_memory.SharedMemory(name=name, create=True, size=self.size)
        unregister(self.memory._name, 'shared_memory')

    def write(self, frame):
        frame = frame.tobytes()
        if self.size != len(frame):
            raise ValueError('frame size does not match')
        self.memory.buf[:len(frame)] = frame

    def close(self):
        self.memory.close()


class SHMVideoCapture:
    def __init__(self, name, width, height, period=0.1):
        self.name = name
        self.width = width
        self.height = height
        self.period = period
        self.last_time = 0

        try:
            self.memory = shared_memory.SharedMemory(name=name, create=False)
        except FileNotFoundError:
            self.memory = shared_memory.SharedMemory(name=name, create=True, size=width*height*3)
        unregister(self.memory._name, 'shared_memory')

    def read(self):
        frame = self.memory.buf
        size = self.width*self.height*3
        frame = frame[:size]
        frame = np.frombuffer(frame, dtype=np.uint8)
        frame = frame.reshape((self.height, self.width, 3))
        t = time.time()
        if t - self.last_time < self.period:
            time.sleep(self.period - (t - self.last_time))
        self.last_time = time.time()
        return True, frame

    def close(self):
        self.memory.close()
