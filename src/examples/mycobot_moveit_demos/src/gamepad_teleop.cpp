/**
 * @file gamepad_teleop.cpp
 * @brief Xbox 360 gamepad teleop for myCobot 280.
 *
 * Movement logic is identical to keyboard_teleop (proven working).
 *
 * BUTTON MAPPING:
 *   X         Move -X
 *   B         Move +X  (toward shelf)
 *   Y         Move +Y
 *   A         Move -Y
 *   D-Up      Move +Z
 *   D-Down    Move -Z
 *   D-Left    Base rotate +
 *   D-Right   Base rotate -
 *   LB        Toggle gripper open / close
 *   RB        Home position
 *   LS Click  Pre-grasp pose
 *   Start     Print current pose
 */

#include <cstdio>
#include <cmath>
#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// ── constants — identical to keyboard_teleop ─────────────────────────────────
static constexpr double STEP_M      = 0.010;   // 10 mm per step (same as keyboard)
static constexpr double STEP_BASE   = 0.10;    // ~5.7 deg per step
static constexpr double CART_STEP   = 0.005;   // 5 mm Cartesian interpolation
static constexpr double MOVE_HZ     = 4.0;     // commands per second while held
static constexpr double PLAN_TIME_S = 1.0;
static constexpr int    PLAN_TRIES  = 2;
static constexpr double VEL         = 0.8;
static constexpr double ACC         = 0.8;
static constexpr double POS_TOL     = 0.005;
static constexpr double ORI_TOL     = 0.15;

static constexpr double PREGRASP_X     = 0.10;
static constexpr double PREGRASP_Y     = 0.00;
static constexpr double PREGRASP_Z     = 0.17;
static constexpr double PREGRASP_ROLL  = 0.0;
static constexpr double PREGRASP_PITCH = M_PI / 2.0;
static constexpr double PREGRASP_YAW   = 0.0;


// ── Xbox 360 indices ──────────────────────────────────────────────────────────
static constexpr int BTN_A     = 0;
static constexpr int BTN_B     = 1;
static constexpr int BTN_X     = 2;
static constexpr int BTN_Y     = 3;
static constexpr int BTN_LB    = 4;
static constexpr int BTN_RB    = 5;
static constexpr int BTN_START = 7;
static constexpr int BTN_LS    = 9;
static constexpr int AX_DP_H   = 6;   // D-pad horiz: left=-1, right=+1
static constexpr int AX_DP_V   = 7;   // D-pad vert:  down=-1, up=+1

// ── RPY helpers ───────────────────────────────────────────────────────────────
static void quatToRPY(double qx, double qy, double qz, double qw,
                      double &roll, double &pitch, double &yaw)
{
    roll  = std::atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy));
    double sp = 2*(qw*qy-qz*qx);
    pitch = std::abs(sp)>=1 ? std::copysign(M_PI/2,sp) : std::asin(sp);
    yaw   = std::atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz));
}

static geometry_msgs::msg::Quaternion rpyToQuat(double r, double p, double y)
{
    double cr=cos(r*.5),sr=sin(r*.5);
    double cp=cos(p*.5),sp=sin(p*.5);
    double cy=cos(y*.5),sy=sin(y*.5);
    geometry_msgs::msg::Quaternion q;
    q.w = cr*cp*cy+sr*sp*sy;
    q.x = sr*cp*cy-cr*sp*sy;
    q.y = cr*sp*cy+sr*cp*sy;
    q.z = cr*cp*sy-sr*sp*cy;
    return q;
}

// ── MoveGroup helpers — copied verbatim from keyboard_teleop ─────────────────
using MG = moveit::planning_interface::MoveGroupInterface;

static bool planAndExecute(MG &mg, const rclcpp::Logger &log, const std::string &tag)
{
    MG::Plan plan;
    mg.setStartStateToCurrentState();
    for (int i=1; i<=PLAN_TRIES; ++i) {
        if (static_cast<bool>(mg.plan(plan))) { mg.execute(plan); return true; }
        RCLCPP_WARN(log, "[%s] attempt %d/%d failed", tag.c_str(), i, PLAN_TRIES);
    }
    RCLCPP_ERROR(log, "[%s] all attempts failed", tag.c_str());
    mg.stop(); return false;
}

// Cartesian move — identical to keyboard_teleop::cartesianMove
static bool cartesianMove(MG &arm, const rclcpp::Logger &log,
                           double dx, double dy, double dz)
{
    arm.stop();
    arm.setStartStateToCurrentState();

    auto cur = arm.getCurrentPose().pose;

    geometry_msgs::msg::Pose target = cur;
    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;

    RCLCPP_INFO(log, "Move: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)",
        cur.position.x, cur.position.y, cur.position.z,
        target.position.x, target.position.y, target.position.z);

    std::vector<geometry_msgs::msg::Pose> waypoints = {target};
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(waypoints, CART_STEP, trajectory);

    if (fraction < 0.5) {
        RCLCPP_WARN(log, "Cartesian path only %.0f%% – skipping", fraction*100);
        return false;
    }

    MG::Plan plan;
    plan.trajectory = trajectory;
    arm.asyncExecute(plan);
    return true;
}

static void syncOrientation(MG &arm, const rclcpp::Logger &log,
                             double &roll, double &pitch, double &yaw)
{
    rclcpp::sleep_for(std::chrono::milliseconds(400));
    auto p = arm.getCurrentPose().pose;
    double n = p.orientation.x*p.orientation.x + p.orientation.y*p.orientation.y
             + p.orientation.z*p.orientation.z + p.orientation.w*p.orientation.w;
    if (n < 0.01) { RCLCPP_WARN(log,"Bad quaternion – keeping previous"); return; }
    quatToRPY(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
              roll, pitch, yaw);
    RCLCPP_INFO(log, "Orientation synced: R=%.1f P=%.1f Y=%.1f deg",
                roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
}

// ── control thread ────────────────────────────────────────────────────────────
static void controlLoop(rclcpp::Node::SharedPtr node, std::atomic<bool> &running)
{
    auto log = rclcpp::get_logger("gamepad_teleop");

    // shared joy state
    std::vector<float>   axes;
    std::vector<int32_t> buttons;
    std::vector<int32_t> prev_buttons;
    std::mutex joy_mtx;

    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(joy_mtx);
            axes    = msg->axes;
            buttons = msg->buttons;
        });

    // MoveGroup setup — identical settings to keyboard_teleop
    MG arm(node, "arm");
    MG gripper(node, "gripper");

    arm.setEndEffectorLink("link6_flange");  // arm chain tip is gripper_base; fix getCurrentPose()
    arm.setPlanningPipelineId("ompl");
    arm.setPlannerId("RRTConnectkConfigDefault");
    arm.setPlanningTime(PLAN_TIME_S);
    arm.setNumPlanningAttempts(PLAN_TRIES);
    arm.setMaxVelocityScalingFactor(VEL);
    arm.setMaxAccelerationScalingFactor(ACC);
    arm.setGoalPositionTolerance(POS_TOL);
    arm.setGoalOrientationTolerance(ORI_TOL);
    arm.allowReplanning(true);

    gripper.setPlanningTime(3.0);
    gripper.setMaxVelocityScalingFactor(0.5);
    gripper.setMaxAccelerationScalingFactor(0.5);
    gripper.allowReplanning(true);

    RCLCPP_INFO(log, "Waiting for robot state...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    std::string frame = arm.getPlanningFrame();
    std::string eef   = arm.getEndEffectorLink();
    RCLCPP_INFO(log, "Frame: %s  EEF: '%s'", frame.c_str(), eef.c_str());

    double t_roll=0, t_pitch=0, t_yaw=0;
    syncOrientation(arm, log, t_roll, t_pitch, t_yaw);

    bool gripper_open = true;

    printf("\n=======================================================\n");
    printf("  myCobot Gamepad Teleop  (Xbox 360)\n");
    printf("=======================================================\n");
    printf("  X / B      Move -X / +X\n");
    printf("  Y / A      Move +Y / -Y\n");
    printf("  D-Up/Down  Move +Z / -Z\n");
    printf("  D-L/R      Base rotate +/-\n");
    printf("  LB         Toggle gripper open/close\n");
    printf("  RB         Home\n");
    printf("  LS Click   Pre-grasp (face shelf)\n");
    printf("  Start      Print current pose\n");
    printf("=======================================================\n\n");
    fflush(stdout);

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / MOVE_HZ));
    auto next_move    = std::chrono::steady_clock::now();

    while (running && rclcpp::ok())
    {
        std::vector<float>   cur_axes;
        std::vector<int32_t> cur_buttons;
        {
            std::lock_guard<std::mutex> lk(joy_mtx);
            cur_axes    = axes;
            cur_buttons = buttons;
        }

        if (cur_buttons.empty()) {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        if (prev_buttons.size() != cur_buttons.size())
            prev_buttons.assign(cur_buttons.size(), 0);

        auto btn = [&](int i) -> bool {
            return i < (int)cur_buttons.size() && cur_buttons[i];
        };
        auto rising = [&](int i) -> bool {
            return btn(i) && (i >= (int)prev_buttons.size() || !prev_buttons[i]);
        };
        auto axis = [&](int i) -> float {
            return i < (int)cur_axes.size() ? cur_axes[i] : 0.0f;
        };

        // ── single-fire buttons ────────────────────────────────────────

        if (rising(BTN_LB)) {
            gripper_open = !gripper_open;
            RCLCPP_INFO(log, "Gripper -> %s", gripper_open ? "open" : "closed");
            gripper.setNamedTarget(gripper_open ? "open" : "closed");
            gripper.move();
        }

        if (rising(BTN_RB)) {
            RCLCPP_INFO(log, "-> Home");
            arm.setNamedTarget("home");
            if (planAndExecute(arm, log, "Home"))
                syncOrientation(arm, log, t_roll, t_pitch, t_yaw);
        }

        if (rising(BTN_LS)) {
            RCLCPP_INFO(log, "-> Pre-grasp");
            arm.stop();
            arm.setPlanningTime(5.0);
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id  = frame;
            ps.pose.position.x  = PREGRASP_X;
            ps.pose.position.y  = PREGRASP_Y;
            ps.pose.position.z  = PREGRASP_Z;
            ps.pose.orientation = rpyToQuat(PREGRASP_ROLL, PREGRASP_PITCH, PREGRASP_YAW);
            arm.setPoseTarget(ps, eef);
            planAndExecute(arm, log, "PreGrasp");
            arm.setPlanningTime(PLAN_TIME_S);
        }

        if (rising(BTN_START)) {
            auto p = arm.getCurrentPose().pose;
            double r, pi_v, y;
            quatToRPY(p.orientation.x, p.orientation.y,
                      p.orientation.z, p.orientation.w, r, pi_v, y);
            RCLCPP_INFO(log, "Pos: x=%.4f y=%.4f z=%.4f",
                p.position.x, p.position.y, p.position.z);
            RCLCPP_INFO(log, "Ori: R=%.1f P=%.1f Y=%.1f deg",
                r*180/M_PI, pi_v*180/M_PI, y*180/M_PI);
        }

        // ── rate-limited movement ──────────────────────────────────────
        auto now = std::chrono::steady_clock::now();
        if (now >= next_move)
        {
            double dx=0, dy=0, dz=0;
            bool do_cart = false;
            bool do_base = false;
            double d_base = 0;

            if (btn(BTN_B)) { dx = +STEP_M; do_cart = true; }
            if (btn(BTN_X)) { dx = -STEP_M; do_cart = true; }
            if (btn(BTN_Y)) { dy = +STEP_M; do_cart = true; }
            if (btn(BTN_A)) { dy = -STEP_M; do_cart = true; }

            float dpv = axis(AX_DP_V);
            if (dpv >  0.5f) { dz = +STEP_M; do_cart = true; }
            if (dpv < -0.5f) { dz = -STEP_M; do_cart = true; }

            float dph = axis(AX_DP_H);
            if (dph < -0.5f) { d_base = +STEP_BASE; do_base = true; }
            if (dph >  0.5f) { d_base = -STEP_BASE; do_base = true; }

            if (do_cart) {
                cartesianMove(arm, log, dx, dy, dz);
                next_move = now + period;
            } else if (do_base) {
                auto joints = arm.getCurrentJointValues();
                if (!joints.empty()) {
                    joints[0] += d_base;
                    RCLCPP_INFO(log, "Base -> %.1f deg", joints[0]*180/M_PI);
                    arm.setJointValueTarget(joints);
                    planAndExecute(arm, log, "Base");
                    next_move = now + period;
                }
            }
        }

        prev_buttons = cur_buttons;
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
}

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "gamepad_teleop",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::atomic<bool> running{true};
    std::thread t(controlLoop, node, std::ref(running));

    executor.spin();
    running = false;
    t.join();
    return 0;
}
