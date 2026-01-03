#!/usr/bin/env python3
import numpy as np
from pydrake.all import RigidTransform
from arm_controller import ArmController
from utils import get_model_path

def main():
    model_path = get_model_path()
    arm = ArmController(model_path, show_viz=False)
    R, p0 = arm.get_pose()
    targets = [
        RigidTransform(R, p0 + np.array([0.1, 0.0, 0.0])),
        RigidTransform(R, p0 + np.array([0.1, 0.1, 0.0]))
    ]
    q_targets = [arm.compute_ik(t) for t in targets]
    arm.execute(q_targets, dt=2.0)

if __name__ == "__main__":
    main()

