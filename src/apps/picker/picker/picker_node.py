import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration as DurMsg
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import GripperCommand
from moveit_msgs.srv import GetCartesianPath

from pymoveit2 import MoveIt2
from pymoveit2.moveit2 import MoveIt2State


ARM_JOINTS = [
    'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
    'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange',
]

# Orientation: link6_flange Z-axis → world +Y (toward shelf).
# RPY(-π/2, 0, 0) → quat (-0.7071, 0, 0, 0.7071).
# Gripper jaws close in ±X (left/right around the medicine).
APPROACH_QUAT = [-0.7071, 0.0, 0.0, 0.7071]

# Offset from link6_flange to gripper-tip centre along world +Y:
#   0.034 m to gripper_base  +  0.021 m to finger tips  =  0.055 m geometric reach.
# The detector clusters the camera-visible BACK face of each medicine, so the
# published py is ~20 mm further than the true centroid.  An extra 0.035 m keeps
# the palm (at lf + 0.034) well clear of the medicine front face while still
# placing the finger tips 5 mm inside the medicine body for a secure jaw closure.
GRIPPER_REACH = 0.120   # metres

# Safe Y coordinate for staging (well in front of shelf face at Y≈0.18 m).
# All X/Z adjustments happen here; only the Cartesian moves cross Y=0.18.
STAGING_Y = 0.10        # metres

# Y coordinate used as a shelf-clear waypoint before the joint-space move to
# TABLE_DROP_POS.  Straight Cartesian -Y pull to this depth guarantees the arm
# is well away from the shelf before MoveIt plans the unconstrained drop move.
CLEAR_Y = -0.05         # metres (5 cm behind robot base — shelf cannot be hit)

# Position above the table where the medicine is held for manual release.
# Traced live from TF world→link6_flange while robot was at the desired drop pose.
TABLE_DROP_POS = [0.208, 0.091, 0.108]

# Cartesian motion speed — slow and steady to avoid knocking medicines.
CART_SPEED = 0.04       # m/s (4 cm/s)


class PickerNode(Node):
    """
    Gamepad-triggered pick-and-place.

    Motion strategy:
      1. Open gripper
      2. Joint-space move  → staging position (px, STAGING_Y, pz)
      3. Cartesian +Y      → grasp position   (px, lf_y,      pz)
      4. Close gripper
      5. Cartesian -Y      → staging position (px, STAGING_Y, pz)
      6. Cartesian +Z      → lifted position  (px, STAGING_Y, pz+0.15)
      7. Joint-space home
      8. Open gripper (release)

    Steps 3 / 5 / 6 are guaranteed straight lines so the gripper never
    sweeps sideways into adjacent medicines or shelf structure.
    """

    def __init__(self):
        super().__init__('picker_node')

        self.declare_parameter('grasp_button_index', 7)
        self._btn_idx = self.get_parameter('grasp_button_index').value

        self._target: PoseStamped = None
        self._target_lock = threading.Lock()
        self._executing       = False
        self._waiting_release = False
        self._release_event   = threading.Event()
        self._exec_lock       = threading.Lock()
        self._last_btn        = 0

        cb = ReentrantCallbackGroup()

        # ── MoveIt2 for joint-space moves (staging, home) ─────────────────
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name='base_link',
            end_effector_name='link6_flange',
            group_name='arm',
            use_move_group_action=True,
            ignore_new_calls_while_executing=True,
            callback_group=cb,
        )
        self._moveit2.max_velocity          = 0.4
        self._moveit2.max_acceleration      = 0.4
        self._moveit2.num_planning_attempts = 10
        self._moveit2.allowed_planning_time = 5.0

        # ── GetCartesianPath service for straight-line moves ──────────────
        self._cart_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=cb)

        # ── Arm-controller topic (receives computed Cartesian trajectories) ─
        self._arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # ── Gripper action client ─────────────────────────────────────────
        self._gripper_ac = ActionClient(
            self, GripperCommand,
            '/gripper_action_controller/gripper_cmd',
            callback_group=cb)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Joy, '/joy', self._joy_cb, 10,
                                 callback_group=cb)
        self.create_subscription(
            PoseStamped, '/target_medicine_pose', self._target_cb, 10,
            callback_group=cb)

        self.get_logger().info(
            'Picker ready.  Start medicine_detector then press Start.')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _target_cb(self, msg: PoseStamped):
        with self._target_lock:
            self._target = msg
        self.get_logger().info(
            f'Target: ({msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})',
            throttle_duration_sec=5.0)

    def _joy_cb(self, msg: Joy):
        try:
            val = msg.buttons[self._btn_idx]
        except IndexError:
            return
        do_start = False
        with self._exec_lock:
            rising = (val == 1 and self._last_btn == 0)
            self._last_btn = val
            if rising:
                if self._waiting_release:
                    self._release_event.set()
                elif not self._executing:
                    self._executing = True
                    do_start = True
        if do_start:
            self.get_logger().info('START – launching pick-and-place')
            threading.Thread(target=self._run, daemon=True).start()

    # ── Gripper ───────────────────────────────────────────────────────────

    def _gripper(self, position: float, timeout: float = 5.0):
        """Send GripperCommand action.  position: 0.0=open, -0.50=closed."""
        if not self._gripper_ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper action server not available')
            return
        goal = GripperCommand.Goal()
        goal.command.position   = float(position)
        goal.command.max_effort = 50.0
        future = self._gripper_ac.send_goal_async(goal)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            self.get_logger().warn('Gripper: send timed out')
            return
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('Gripper: goal rejected')
            return
        res_f = gh.get_result_async()
        deadline2 = time.time() + timeout
        while not res_f.done() and time.time() < deadline2:
            time.sleep(0.02)
        self.get_logger().info(
            'Gripper → ' + ('OPEN' if position >= 0.0 else 'CLOSED'))

    # ── Joint-space arm moves (via MoveGroup action) ───────────────────────

    def _move_arm_jspace(self, pos, timeout: float = 25.0) -> bool:
        """
        Plan and execute a joint-space IK move to `pos` with APPROACH_QUAT.
        Returns True if the move succeeded.
        """
        self.get_logger().info(
            f'Arm jspace → ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})')
        self._moveit2.move_to_pose(
            position=pos,
            quat_xyzw=APPROACH_QUAT,
            tolerance_position=0.01,
            tolerance_orientation=0.10,
        )
        time.sleep(0.15)   # allow __is_motion_requested to be set
        deadline = time.time() + timeout
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            if time.time() > deadline:
                self.get_logger().warn('Joint-space move timed out')
                return False
            time.sleep(0.05)
        ok = self._moveit2.motion_suceeded
        if not ok:
            self.get_logger().warn('Joint-space move: planning/IK failed')
        time.sleep(0.2)
        return ok

    def _home(self, timeout: float = 25.0):
        """Return arm to all-zero joint configuration."""
        self.get_logger().info('Arm → HOME')
        self._moveit2.move_to_configuration(
            joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(0.15)
        deadline = time.time() + timeout
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            if time.time() > deadline:
                return
            time.sleep(0.05)
        time.sleep(0.2)

    # ── Cartesian straight-line moves ─────────────────────────────────────

    def _move_cartesian(self, start_pos, end_pos,
                        speed: float = CART_SPEED,
                        timeout: float = 10.0) -> bool:
        """
        Move the end-effector in a STRAIGHT Cartesian line from
        start_pos to end_pos at APPROACH_QUAT orientation.

        Uses GetCartesianPath (5 mm step) then publishes the timed
        JointTrajectory directly to /arm_controller/joint_trajectory.

        start_pos is used only for distance/timing calculation;
        the robot must already be physically at start_pos.
        """
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(start_pos, end_pos)))
        if dist < 0.002:
            return True

        self.get_logger().info(
            f'Arm cartesian → ({end_pos[0]:.3f},{end_pos[1]:.3f},'
            f'{end_pos[2]:.3f})  dist={dist:.3f} m')

        # Target waypoint
        wp = Pose()
        wp.position.x = float(end_pos[0])
        wp.position.y = float(end_pos[1])
        wp.position.z = float(end_pos[2])
        wp.orientation.x = APPROACH_QUAT[0]
        wp.orientation.y = APPROACH_QUAT[1]
        wp.orientation.z = APPROACH_QUAT[2]
        wp.orientation.w = APPROACH_QUAT[3]

        req = GetCartesianPath.Request()
        req.header.frame_id  = 'base_link'
        req.header.stamp     = self.get_clock().now().to_msg()
        req.group_name       = 'arm'
        req.link_name        = 'link6_flange'
        req.waypoints        = [wp]
        req.max_step         = 0.005     # 5 mm interpolation → dense, smooth path
        req.jump_threshold   = 5.0
        req.avoid_collisions = False     # shelf/medicines not in planning scene

        if not self._cart_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/compute_cartesian_path not available')
            return False

        future = self._cart_client.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)

        if not future.done():
            self.get_logger().warn('Cartesian path: service timeout')
            return False

        res = future.result()
        if res.fraction < 0.80:
            self.get_logger().warn(
                f'Cartesian path only {res.fraction:.0%} computed — aborting')
            return False

        traj = res.solution.joint_trajectory
        if not traj.points:
            self.get_logger().warn('Cartesian path: empty trajectory')
            return False

        # Add uniform timing — GetCartesianPath returns no time_from_start
        total_sec = max(dist / speed, 0.5)
        n = len(traj.points)
        for i, pt in enumerate(traj.points):
            t = total_sec * (i + 1) / n
            pt.time_from_start = DurMsg(sec=int(t), nanosec=int((t % 1) * 1e9))

        self._arm_pub.publish(traj)
        time.sleep(total_sec + 0.4)   # wait for execution to complete
        return True

    # ── Pick-and-place sequence ────────────────────────────────────────────

    def _run(self):
        try:
            with self._target_lock:
                target = self._target
            if target is None:
                self.get_logger().error(
                    'No /target_medicine_pose yet. Is medicine_detector running?')
                return

            px = target.pose.position.x
            py = target.pose.position.y
            pz = target.pose.position.z
            self.get_logger().info(
                f'Picking medicine at ({px:.3f}, {py:.3f}, {pz:.3f})')

            # link6_flange Y at grasp: finger tips reach GRIPPER_REACH past lf
            lf_y = py - GRIPPER_REACH

            # The three Cartesian positions
            staging_pos = [px, STAGING_Y, pz       ]   # safe park, in front of shelf
            grasp_pos   = [px, lf_y,      pz       ]   # gripper tip at medicine centroid
            lift_pos    = [px, STAGING_Y, pz + 0.15]   # lifted above shelf height

            # ── 1. Open gripper ──────────────────────────────────────────
            self.get_logger().info('1/8 Open gripper')
            self._gripper(0.0)

            # ── 2. Move to staging (joint-space IK) ──────────────────────
            # From home, the arm moves to STAGING_Y=0.10 m which is safely
            # in front of the shelf face (Y≈0.18 m).  No risk of shelf contact.
            self.get_logger().info('2/8 Staging position (joint-space)')
            if not self._move_arm_jspace(staging_pos):
                self.get_logger().error('Staging move failed — aborting')
                return

            # ── 3. Straight approach in +Y (Cartesian) ───────────────────
            # Pure Y-axis translation: gripper travels directly toward medicine.
            # Constant X and Z guarantee no contact with adjacent medicines.
            self.get_logger().info('3/8 Approach (straight +Y)')
            if not self._move_cartesian(staging_pos, grasp_pos):
                self.get_logger().error('Cartesian approach failed — aborting')
                return

            # ── 4. Close gripper ─────────────────────────────────────────
            self.get_logger().info('4/8 Close gripper')
            self._gripper(-0.50)

            # ── 5. Straight retract in -Y (Cartesian) ────────────────────
            # Pure Y-axis retraction: gripper withdraws straight back out.
            self.get_logger().info('5/8 Retract (straight -Y)')
            self._move_cartesian(grasp_pos, staging_pos)

            # ── 6. Lift in +Z (Cartesian) ────────────────────────────────
            # At STAGING_Y=0.10 we are outside the shelf, so rising freely.
            self.get_logger().info('6/9 Lift (straight +Z)')
            self._move_cartesian(staging_pos, lift_pos)

            # ── 7. Pull back in -Y (Cartesian) ───────────────────────────
            # Straight-line retract to CLEAR_Y before the unconstrained
            # joint-space move, so MoveIt cannot arc back through the shelf.
            clear_pos = [px, CLEAR_Y, pz + 0.15]
            self.get_logger().info('7/9 Clear shelf (straight -Y)')
            self._move_cartesian(lift_pos, clear_pos)

            # ── 8. Move to table drop position (joint-space) ─────────────
            # Starts from CLEAR_Y — well behind the shelf, path stays clear.
            self.get_logger().info('8/9 Move to drop position')
            if not self._move_arm_jspace(TABLE_DROP_POS):
                self.get_logger().warn('Drop-position move failed — holding at clear position')

            # ── 9. Wait for manual gripper release, then home ─────────────
            # Robot holds the medicine here.  Press Start to go home.
            self.get_logger().info(
                '9/9 Holding medicine at drop position. '
                'Open gripper manually then press Start to go home.')
            self._release_event.clear()
            with self._exec_lock:
                self._waiting_release = True
            self._release_event.wait()   # blocks until Start is pressed again
            with self._exec_lock:
                self._waiting_release = False

            self.get_logger().info('Release confirmed — returning home')
            self._home()

            self.get_logger().info('Pick-and-place complete!')

        except Exception as exc:
            self.get_logger().error(f'Pick-and-place error: {exc}', exc_info=True)
        finally:
            with self._exec_lock:
                self._executing = False


def main():
    rclpy.init()
    node = PickerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
