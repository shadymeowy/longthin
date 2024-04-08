import time


class Rate:
    def __init__(self, period, node):
        self.period = period
        self.last_time = time.time()
        self.node = node

    def check(self):
        t = time.time()
        dt = t - self.last_time
        if dt > self.period:
            self.last_time = t
            return True
        return False

    def sleep(self):
        self.node.spin_once()
        dt = time.time() - self.last_time
        if dt > self.period:
            return
        time.sleep(self.period - dt)
        self.last_time = time.time()
