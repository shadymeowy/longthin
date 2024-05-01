import os
import argparse
import subprocess


from ..ltpacket import *
from ..node import LTNode


class Runner:
    def __init__(self, node, name="longthin"):
        self.node = node
        self.config = node.config
        self.name = name
        self.node.subscribe(Reboot, self.cb_reboot)

    def open_tasks(self):
        runners = self.config.runner

        ret = subprocess.run(['tmux', 'has-session', '-t', self.name])
        if ret.returncode == 0:
            self.kill_tasks()

        ret = subprocess.run(['tmux', 'new-session', '-d', '-s', self.name])
        if ret.returncode != 0:
            raise Exception(f"Failed to create session {self.name}")

        ret = subprocess.run(['tmux', 'rename-window', '-t', f'{self.name}:1', 'main'])
        if ret.returncode != 0:
            raise Exception(f"Failed to rename window 1")

        for run in runners:
            print("Running", run.cmd, run.args)
            ret = subprocess.run(['tmux', 'new-window', '-t', self.name, '-n', f'w{run.cmd}', run.cmd, *run.args])
            if ret.returncode != 0:
                raise Exception(f"Failed to create window {run.cmd}")

    def attach(self):
        ret = subprocess.run(['tmux', 'attach-session', '-t', self.name])
        if ret.returncode != 0:
            raise Exception(f"Failed to attach session {self.name}")

    def kill_tasks(self):
        ret = subprocess.run(['tmux', 'kill-session', '-t', self.name])
        if ret.returncode != 0:
            raise Exception(f"Failed to kill session {self.name}")

    def cb_reboot(self, packet):
        print("Rebooting")
        self.kill_tasks()
        self.open_tasks()


def main():
    parser = argparse.ArgumentParser(description='Run multiple tasks in tmux')
    parser.add_argument('--config', default=None, help='Config file')
    args = parser.parse_args()

    if args.config is not None:
        os.environ['LTCONFIG'] = args.config

    node = LTNode()
    runner = Runner(node)

    runner.open_tasks()
    print("Running tasks")
    node.spin()
    runner.kill_tasks()


if __name__ == "__main__":
    main()
