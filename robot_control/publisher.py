#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import mujoco as mu
import numpy as np
import threading, sys, termios, tty
from utils import get_model_path

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        default_path = get_model_path()
        self.declare_parameter('xml_path', default_path)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('publish_joints', True)
        self.xml_path = self.get_parameter('xml_path').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.publish_joints = self.get_parameter('publish_joints').get_parameter_value().bool_value
        self.dt = 1.0 / self.fps
        self.model = mu.MjModel.from_xml_path(self.xml_path)
        self.data = mu.MjData(self.model)
        self.h, self.w = 480, 640
        self.renderer = mu.Renderer(self.model, height=self.h, width=self.w)
        self.renderer.enable_depth_rendering()
        self.bridge = CvBridge()
        self.cameras = self._find_cameras()
        if not self.cameras:
            raise RuntimeError("No camera pairs found")
        self.pubs = self._make_publishers()
        if self.publish_joints:
            self.joint_pub = self.create_publisher(JointState, '/panda/joint_states', 10)
        self.active = {k: {'rgb': True, 'depth': False} for k in self.cameras.keys()}
        self.keys = {
            '1': ('wrist', 'rgb'), '2': ('wrist', 'depth'),
            '3': ('table', 'rgb'), '4': ('table', 'depth'),
            '5': ('side', 'rgb'), '6': ('side', 'depth'),
        }
        threading.Thread(target=self._key_listener, daemon=True).start()
        self.create_timer(self.dt, self._publish)

    def _key_listener(self):
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch in self.keys:
                    prefix, stream = self.keys[ch]
                    self.active[prefix][stream] = not self.active[prefix][stream]
                    state = "ON" if self.active[prefix][stream] else "OFF"
                    self.get_logger().info(f"{prefix}/{stream} → {state}")
        except Exception as e:
            self.get_logger().error(f"Keyboard error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

    def _find_cameras(self):
        all_cams = [mu.mj_id2name(self.model, mu.mjtObj.mjOBJ_CAMERA, i) for i in range(self.model.ncam)]
        pairs = {}
        for name in all_cams:
            if name.endswith('_rgb'):
                prefix = name.replace('_rgb', '')
                if prefix not in pairs:
                    pairs[prefix] = {}
                pairs[prefix]['rgb'] = mu.mj_name2id(self.model, mu.mjtObj.mjOBJ_CAMERA, name)
            elif name.endswith('_depth'):
                prefix = name.replace('_depth', '')
                if prefix not in pairs:
                    pairs[prefix] = {}
                pairs[prefix]['depth'] = mu.mj_name2id(self.model, mu.mjtObj.mjOBJ_CAMERA, name)
        return {k: v for k, v in pairs.items() if 'rgb' in v and 'depth' in v}

    def _make_publishers(self):
        pubs = {}
        for prefix in self.cameras.keys():
            pubs[prefix] = {
                'rgb': self.create_publisher(Image, f'/{prefix}/camera/color/image_raw', 10),
                'depth': self.create_publisher(Image, f'/{prefix}/camera/depth/image_raw', 10)
            }
        return pubs

    def _render(self, cam_id, depth=False):
        if depth:
            self.renderer.enable_depth_rendering()
        else:
            self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera=cam_id)
        return self.renderer.render().copy()

    def _publish(self):
        mu.mj_step(self.model, self.data)
        if self.publish_joints:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [mu.mj_id2name(self.model, mu.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)]
            msg.position = self.data.qpos.tolist()
            self.joint_pub.publish(msg)
        for prefix, ids in self.cameras.items():
            if self.active[prefix]['rgb']:
                rgb = self._render(ids['rgb'], depth=False)
                msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
                msg.header.stamp = self.get_clock().now().to_msg()
                self.pubs[prefix]['rgb'].publish(msg)
            if self.active[prefix]['depth']:
                depth = np.nan_to_num(self._render(ids['depth'], depth=True))
                depth_mm = (depth * 1000.0).astype(np.uint16)
                msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
                msg.header.stamp = self.get_clock().now().to_msg()
                self.pubs[prefix]['depth'].publish(msg)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

