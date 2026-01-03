#!/usr/bin/env python3
import time
import numpy as np
import mujoco as mu
from mujoco import viewer
from pydrake.all import (
    MultibodyPlant, Parser, InverseKinematics,
    RigidTransform, RotationMatrix, StartMeshcat,
    Simulator, RobotDiagramBuilder
)
from pydrake.solvers import Solve
from manipulation.make_drake_compatible_model import MakeDrakeCompatibleModel
from manipulation.remotes import AddMujocoMenagerie
from manipulation.utils import ApplyDefaultVisualization


class ArmController:
    def __init__(self, model_path: str, show_viz: bool = True):
        self.model_path = model_path
        self.show_viz = show_viz
        self._init_sim()
        self._init_renderer()
        self._init_kinematics()
        self.ctx = self.plant.CreateDefaultContext()
        if self.show_viz:
            self._init_viz()

    def _init_sim(self):
        self.model = mu.MjModel.from_xml_path(self.model_path)
        self.data = mu.MjData(self.model)

    def _init_renderer(self):
        self.h, self.w = 480, 640
        self.renderer = mu.Renderer(self.model, height=self.h, width=self.w)
        self.renderer.enable_depth_rendering()
        self.rgb_id = mu.mj_name2id(self.model, mu.mjtObj.mjOBJ_CAMERA, "wrist_rgb")
        self.depth_id = mu.mj_name2id(self.model, mu.mjtObj.mjOBJ_CAMERA, "wrist_depth")
        if self.rgb_id == -1 or self.depth_id == -1:
            raise RuntimeError("Required cameras not found in XML")

    def _init_kinematics(self):
        drake_path = self.model_path.replace(".xml", ".drake.xml")
        MakeDrakeCompatibleModel(self.model_path, drake_path, overwrite=True)
        builder = RobotDiagramBuilder()
        plant = builder.plant()
        parser = Parser(plant)
        AddMujocoMenagerie(parser.package_map())
        parser.AddModels(drake_path)
        plant.Finalize()
        self.plant = plant
        self.builder = builder
        self.ee = self._find_ee()

    def _init_viz(self):
        self.meshcat = StartMeshcat()
        ApplyDefaultVisualization(self.builder.builder(), meshcat=self.meshcat)
        self.diagram = self.builder.Build()
        self.simulator = Simulator(self.diagram)
        self.context = self.simulator.get_mutable_context()

    def _find_ee(self):
        for name in ["hand", "link7", "panda_link8", "panda_ee"]:
            try:
                return self.plant.GetFrameByName(name)
            except RuntimeError:
                continue
        raise RuntimeError("End-effector frame not found")

    def get_joints(self):
        return np.copy(self.data.qpos)

    def get_pose(self):
        q_mj = np.copy(self.data.qpos[:self.model.nq])
        n_d = self.plant.num_positions()
        q_sync = q_mj[:n_d]
        self.plant.SetPositions(self.ctx, q_sync)
        X = self.ee.CalcPoseInWorld(self.ctx)
        return X.rotation(), X.translation()

    def capture(self):
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.rgb_id)
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.depth_id)
        depth = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        return rgb, depth

    def compute_ik(self, target: RigidTransform):
        ik = InverseKinematics(self.plant)
        q = ik.q()
        ik.AddPositionConstraint(
            self.ee, [0, 0, 0],
            self.plant.world_frame(),
            target.translation() - [0.001]*3,
            target.translation() + [0.001]*3,
        )
        ik.AddOrientationConstraint(
            frameAbar=self.ee,
            R_AbarA=RotationMatrix(),
            frameBbar=self.plant.world_frame(),
            R_BbarB=target.rotation(),
            theta_bound=0.01,
        )
        result = Solve(ik.prog())
        if not result.is_success():
            raise RuntimeError("IK failed")
        return result.GetSolution(q)

    def plan_trajectory(self, q0, q1, n=150):
        q0, q1 = np.asarray(q0), np.asarray(q1)
        m = len(q0)
        v0, vf = np.zeros(m), np.zeros(m)
        T = 1.0
        t = np.linspace(0.0, T, n)
        q_traj, qd_traj = [], []
        a0, a1 = q0, v0
        a2 = 3 * (q1 - q0) - 2 * v0 - vf
        a3 = -2 * (q1 - q0) + v0 + vf
        for ti in t:
            q = a0 + a1*ti + a2*(ti**2) + a3*(ti**3)
            qd = a1 + 2*a2*ti + 3*a3*(ti**2)
            q_traj.append(q)
            qd_traj.append(qd)
        return np.array(q_traj), np.array(qd_traj)

    def execute(self, targets, dt=2.0):
        with viewer.launch_passive(self.model, self.data) as v:
            for q_goal in targets:
                q_start = self.get_joints()
                n_steps = int(dt / 0.01)
                q_traj, _ = self.plan_trajectory(q_start, q_goal, n=n_steps)
                for q in q_traj:
                    self.data.qpos[:] = q
                    mu.mj_forward(self.model, self.data)
                    v.sync()
                    time.sleep(0.01)
            while v.is_running():
                v.sync()
                time.sleep(0.01)

