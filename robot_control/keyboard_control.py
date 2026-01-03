#!/usr/bin/env python3
import sys, select, termios, tty, time, signal
import numpy as np
import mujoco as mu
from mujoco import viewer
from pydrake.all import RigidTransform, RotationMatrix
from arm_controller import ArmController
from utils import get_model_path

DPOS = 0.02
DROT = np.deg2rad(5)

class KeyboardControl:
    def __init__(self, arm: ArmController):
        self.arm = arm
        self.R, self.t = arm.get_pose()
        self.R = self.R.matrix()
        self.t = np.array(self.t)
        self.settings = termios.tcgetattr(sys.stdin)

    def _read_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        with viewer.launch(self.arm.model, self.arm.data) as v:
            while v.is_running():
                key = self._read_key()
                if key == '\x1b' or key == 'q':
                    break
                elif key == ' ':
                    try:
                        kid = mu.mj_name2id(self.arm.model, mu.mjtObj.mjOBJ_KEY, "home")
                        mu.mj_resetDataKeyframe(self.arm.model, self.arm.data, kid)
                    except:
                        mu.mj_resetData(self.arm.model)
                    self.R, self.t = self.arm.get_pose()
                    self.R = self.R.matrix()
                    self.t = np.array(self.t)
                    continue
                if key == 'q': self.t[0] += DPOS
                elif key == 'e': self.t[0] -= DPOS
                elif key == 'a': self.t[1] += DPOS
                elif key == 'd': self.t[1] -= DPOS
                elif key == 'w': self.t[2] += DPOS
                elif key == 's': self.t[2] -= DPOS
                elif key == 'r': self.R = self.R @ RotationMatrix.MakeZRotation(DROT).matrix()
                elif key == 'f': self.R = self.R @ RotationMatrix.MakeZRotation(-DROT).matrix()
                elif key == 't': self.R = self.R @ RotationMatrix.MakeYRotation(DROT).matrix()
                elif key == 'g': self.R = self.R @ RotationMatrix.MakeYRotation(-DROT).matrix()
                elif key == 'y': self.R = self.R @ RotationMatrix.MakeXRotation(DROT).matrix()
                elif key == 'h': self.R = self.R @ RotationMatrix.MakeXRotation(-DROT).matrix()
                if key:
                    self._update_pose()
                v.sync()
                time.sleep(0.01)

    def _update_pose(self):
        target = RigidTransform(RotationMatrix(self.R), self.t)
        try:
            q = self.arm.compute_ik(target)
            self.arm.data.qpos[:] = q
            mu.mj_forward(self.arm.model, self.arm.data)
        except:
            pass

def main():
    model_path = get_model_path()
    arm = ArmController(model_path, show_viz=False)
    ctrl = KeyboardControl(arm)
    def handler(sig, frame):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, ctrl.settings)
        sys.exit(0)
    signal.signal(signal.SIGINT, handler)
    ctrl.run()

if __name__ == "__main__":
    main()

