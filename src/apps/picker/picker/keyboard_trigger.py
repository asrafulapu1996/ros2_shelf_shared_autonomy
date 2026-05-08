"""
Keyboard teleop + pick trigger  (replaces both joy_node and manual arm control)

  W / S     +X / -X   (10 mm/step)
  A / D     +Y / -Y
  Q / E     +Z / -Z
  Z / X     Base joint  +/- 5.7 deg
  O         Open gripper
  C         Close gripper
  H         Home position
  P         Print current EEF pose
  ENTER     Trigger pick-and-place  (same as gamepad Start button)
  ESC       Quit
"""

import sys
import tty
import termios
import threading
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration as DurMsg
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import GripperCommand
from moveit_msgs.srv import GetCartesianPath
from tf2_ros import Buffer, TransformListener

from pymoveit2 import MoveIt2
from pymoveit2.moveit2 import MoveIt2State


ARM_JOINTS = [
    'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
    'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange',
]
STEP_M    = 0.010   # 10 mm per XYZ keypress
STEP_BASE = 0.100   # ~5.7 deg per Z/X keypress
CART_SPEED = 0.08   # m/s  (faster than picker for responsive teleop)


def _readkey() -> str:
    """Return a single keypress character without waiting for Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def _print_help():
    print('\n' + '=' * 54)
    print('  Keyboard Teleop + Pick Trigger')
    print('=' * 54)
    print('  W / S     +X / -X   (10 mm/step)')
    print('  A / D     +Y / -Y')
    print('  Q / E     +Z / -Z')
    print('  Z / X     Base joint  +/- 5.7 deg')
    print('  O         Open gripper')
    print('  C         Close gripper')
    print('  H         Home position')
    print('  P         Print current EEF pose')
    print('  ENTER     Trigger pick / confirm release')
    print('  ESC       Quit')
    print('=' * 54 + '\n')
    sys.stdout.flush()


class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('grasp_button_index', 7)
        self._btn = self.get_parameter('grasp_button_index').value

        self._joint_positions = {}
        self._joint_lock = threading.Lock()

        cb = ReentrantCallbackGroup()

        # ── TF for current EEF pose ───────────────────────────────────────
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        # ── Joint states (needed for base rotation) ───────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10,
            callback_group=cb)

        # ── MoveIt2 (joint-space: home, base rotate) ──────────────────────
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name='base_link',
            end_effector_name='link6_flange',
            group_name='arm',
            use_move_group_action=True,
            ignore_new_calls_while_executing=False,
            callback_group=cb,
        )
        self._moveit2.max_velocity          = 0.6
        self._moveit2.max_acceleration      = 0.6
        self._moveit2.num_planning_attempts = 5
        self._moveit2.allowed_planning_time = 2.0

        # ── Cartesian path service ────────────────────────────────────────
        self._cart_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=cb)

        # ── Arm trajectory publisher ──────────────────────────────────────
        self._arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # ── Gripper action ────────────────────────────────────────────────
        self._gripper_ac = ActionClient(
            self, GripperCommand,
            '/gripper_action_controller/gripper_cmd',
            callback_group=cb)

        # ── Joy publisher (pick trigger) ──────────────────────────────────
        self._joy_pub = self.create_publisher(Joy, '/joy', 10)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState):
        with self._joint_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    # ── Current EEF pose via TF ───────────────────────────────────────────

    def _current_pose(self):
        """Returns ([x,y,z], [qx,qy,qz,qw]) or (None, None)."""
        try:
            tf = self._tf.lookup_transform(
                'world', 'link6_flange',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
            t = tf.transform.translation
            r = tf.transform.rotation
            return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
        except Exception as exc:
            self.get_logger().warn(f'TF: {exc}', throttle_duration_sec=2.0)
            return None, None

    # ── Cartesian XYZ step ────────────────────────────────────────────────

    def _move_xyz(self, dx: float, dy: float, dz: float):
        pos, quat = self._current_pose()
        if pos is None:
            return

        end_pos = [pos[0] + dx, pos[1] + dy, pos[2] + dz]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        wp = Pose()
        wp.position.x    = end_pos[0]
        wp.position.y    = end_pos[1]
        wp.position.z    = end_pos[2]
        wp.orientation.x = quat[0]
        wp.orientation.y = quat[1]
        wp.orientation.z = quat[2]
        wp.orientation.w = quat[3]

        req = GetCartesianPath.Request()
        req.header.frame_id  = 'base_link'
        req.header.stamp     = self.get_clock().now().to_msg()
        req.group_name       = 'arm'
        req.link_name        = 'link6_flange'
        req.waypoints        = [wp]
        req.max_step         = 0.005
        req.jump_threshold   = 5.0
        req.avoid_collisions = False

        if not self._cart_client.wait_for_service(timeout_sec=1.0):
            return

        future = self._cart_client.call_async(req)
        deadline = time.time() + 3.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)

        if not future.done():
            return
        res = future.result()
        if res.fraction < 0.5 or not res.solution.joint_trajectory.points:
            self.get_logger().warn(f'Cartesian step only {res.fraction:.0%}')
            return

        traj = res.solution.joint_trajectory
        total_sec = max(dist / CART_SPEED, 0.25)
        n = len(traj.points)
        for i, pt in enumerate(traj.points):
            t = total_sec * (i + 1) / n
            pt.time_from_start = DurMsg(sec=int(t), nanosec=int((t % 1) * 1e9))

        self._arm_pub.publish(traj)

    # ── Base rotation ─────────────────────────────────────────────────────

    def _rotate_base(self, delta: float):
        with self._joint_lock:
            joints = {k: v for k, v in self._joint_positions.items()}

        positions = []
        for name in ARM_JOINTS:
            positions.append(joints.get(name, 0.0))

        positions[0] += delta
        self.get_logger().info(
            f'Base → {positions[0]*180/math.pi:.1f} deg')
        self._moveit2.move_to_configuration(joint_positions=positions)

    # ── Gripper ───────────────────────────────────────────────────────────

    def _gripper(self, position: float):
        if not self._gripper_ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Gripper server not available')
            return
        goal = GripperCommand.Goal()
        goal.command.position   = float(position)
        goal.command.max_effort = 50.0
        self._gripper_ac.send_goal_async(goal)
        self.get_logger().info(
            'Gripper → ' + ('OPEN' if position >= 0.0 else 'CLOSED'))

    # ── Home ──────────────────────────────────────────────────────────────

    def _home(self):
        self.get_logger().info('→ HOME')
        self._moveit2.move_to_configuration(
            joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(0.15)
        deadline = time.time() + 20.0
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            if time.time() > deadline:
                break
            time.sleep(0.05)

    # ── Pick trigger ──────────────────────────────────────────────────────

    def _trigger(self):
        """Simulate a rising-edge press of the Start button on /joy."""
        stamp = self.get_clock().now().to_msg()
        n = self._btn + 1

        press = Joy()
        press.header.stamp = stamp
        press.buttons = [0] * n
        press.buttons[self._btn] = 1
        self._joy_pub.publish(press)

        release = Joy()
        release.header.stamp = stamp
        release.buttons = [0] * n
        self._joy_pub.publish(release)

        self.get_logger().info('Pick trigger sent → /joy')

    # ── Print pose ────────────────────────────────────────────────────────

    def _print_pose(self):
        pos, quat = self._current_pose()
        if pos is None:
            return
        # quaternion → RPY
        qx, qy, qz, qw = quat
        roll  = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
        sp    = 2*(qw*qy-qz*qx)
        pitch = math.copysign(math.pi/2, sp) if abs(sp) >= 1 else math.asin(sp)
        yaw   = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
        self.get_logger().info(
            f'Pos : x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f}')
        self.get_logger().info(
            f'RPY : R={math.degrees(roll):.1f}  '
            f'P={math.degrees(pitch):.1f}  Y={math.degrees(yaw):.1f}  deg')

    # ── Main keyboard loop ────────────────────────────────────────────────

    def run(self):
        _print_help()
        try:
            while rclpy.ok():
                key = _readkey()

                if   key in ('w', 'W'): self._move_xyz(+STEP_M,  0,       0)
                elif key in ('s', 'S'): self._move_xyz(-STEP_M,  0,       0)
                elif key in ('a', 'A'): self._move_xyz( 0,      +STEP_M,  0)
                elif key in ('d', 'D'): self._move_xyz( 0,      -STEP_M,  0)
                elif key in ('q', 'Q'): self._move_xyz( 0,       0,      +STEP_M)
                elif key in ('e', 'E'): self._move_xyz( 0,       0,      -STEP_M)
                elif key in ('z', 'Z'): self._rotate_base(+STEP_BASE)
                elif key in ('x', 'X'): self._rotate_base(-STEP_BASE)
                elif key in ('o', 'O'): self._gripper(0.0)
                elif key in ('c', 'C'): self._gripper(-0.5)
                elif key in ('h', 'H'):
                    threading.Thread(target=self._home, daemon=True).start()
                elif key in ('p', 'P'): self._print_pose()
                elif key in ('\r', '\n'): self._trigger()
                elif key == '\x1b':     # ESC
                    break

        except (KeyboardInterrupt, EOFError):
            pass


def main():
    rclpy.init()
    node = KeyboardTeleopNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)   # let TF and joint states arrive

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
