/**
 * @file keyboard_teleop.cpp
 * @brief Keyboard teleop for myCobot 280.
 *
 * POSITION keys  (orientation locked):
 *   W / S   +X / -X   (10 mm/step)
 *   A / D   +Y / -Y
 *   Q / E   +Z / -Z
 *
 * BASE ROTATE:
 *   Z / X   Base joint  +/- 5.7 deg
 *
 * GRIPPER:
 *   O       Open  gripper
 *   C       Close gripper
 *
 * OTHER:
 *   G       Go to pre-grasp pose
 *   H / R   Home / Ready
 *   P       Print current pose
 *   ESC     Quit
 */

#include <cstdio>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// ── constants ─────────────────────────────────────────────────────────────────
static constexpr double STEP_M       = 0.010;   // 10 mm per XYZ key
static constexpr double STEP_ROT     = 0.0873;  // 5 deg per orientation key
static constexpr double STEP_BASE    = 0.10;    // ~5.7 deg per base key
static constexpr double CART_STEP    = 0.005;   // 5 mm Cartesian interpolation
static constexpr double PLAN_TIME_S  = 1.0;     // tight timeout – OMPL finds simple paths fast
static constexpr int    PLAN_TRIES   = 2;
static constexpr double VEL          = 0.8;     // fast execution
static constexpr double ACC          = 0.8;
static constexpr double POS_TOL      = 0.005;
static constexpr double ORI_TOL      = 0.15;

// Pre-grasp: 6 cm in front of shelf (front at x=0.16), lower shelf height
static constexpr double PREGRASP_X     = 0.10;
static constexpr double PREGRASP_Y     = 0.00;
static constexpr double PREGRASP_Z     = 0.17;
// pitch=pi/2 → EEF approach axis faces +X (toward shelf), gripper opens in ±Y
static constexpr double PREGRASP_ROLL  = 0.0;
static constexpr double PREGRASP_PITCH = M_PI / 2.0;
static constexpr double PREGRASP_YAW   = 0.0;

// ── RPY ↔ quaternion ──────────────────────────────────────────────────────────
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

// ── /dev/tty keyboard ─────────────────────────────────────────────────────────
static char readKey()
{
    int fd = open("/dev/tty", O_RDWR);
    if (fd < 0) { perror("/dev/tty"); return 0; }
    struct termios old{}, nw{};
    tcgetattr(fd, &old); nw=old;
    nw.c_lflag &= ~static_cast<unsigned>(ICANON|ECHO);
    nw.c_cc[VMIN]=1; nw.c_cc[VTIME]=0;
    tcsetattr(fd, TCSANOW, &nw);
    char ch=0; ::read(fd,&ch,1);
    tcsetattr(fd, TCSANOW, &old);
    close(fd); return ch;
}

static void printHelp()
{
    printf("\n=======================================================\n");
    printf("  myCobot Keyboard Teleop\n");
    printf("=======================================================\n");
    printf("  POSITION  (orientation locked):\n");
    printf("    W/S   +X/-X    A/D   +Y/-Y    Q/E   +Z/-Z\n");
    printf("\n");
    printf("  BASE ROTATE:\n");
    printf("    Z/X   Base joint +/-5.7 deg\n");
    printf("\n");
    printf("  GRIPPER:\n");
    printf("    O     Open gripper\n");
    printf("    C     Close gripper\n");
    printf("\n");
    printf("  OTHER:\n");
    printf("    G     Pre-grasp pose\n");
    printf("    H/R   Home / Ready\n");
    printf("    P     Print pose\n");
    printf("    ESC   Quit\n");
    printf("=======================================================\n\n");
    fflush(stdout);
}

// ── helpers ───────────────────────────────────────────────────────────────────
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

// Cartesian move: straight line keeping orientation exactly fixed.
// Stops any in-flight motion first, then fires the new trajectory
// asynchronously so readKey() is unblocked while the robot moves.
static bool cartesianMove(MG &arm, const rclcpp::Logger &log,
                           double dx, double dy, double dz)
{
    // Cancel whatever motion is currently running
    arm.stop();
    arm.setStartStateToCurrentState();

    auto cur = arm.getCurrentPose().pose;

    // Target = current position + delta, orientation UNCHANGED (copied exactly)
    geometry_msgs::msg::Pose target = cur;
    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;

    std::vector<geometry_msgs::msg::Pose> waypoints = {target};

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(waypoints, CART_STEP, trajectory);

    if (fraction < 0.5) {
        RCLCPP_WARN(log, "Cartesian path only %.0f%% – skipping", fraction*100);
        return false;
    }

    MG::Plan plan;
    plan.trajectory = trajectory;
    arm.asyncExecute(plan);   // non-blocking: return immediately for next keypress
    return true;
}

// ── sync orientation state from current arm pose ──────────────────────────────
static void syncOrientation(MG &arm, const rclcpp::Logger &log,
                             double &roll, double &pitch, double &yaw)
{
    rclcpp::sleep_for(std::chrono::milliseconds(400));
    auto p = arm.getCurrentPose().pose;
    double n = p.orientation.x*p.orientation.x + p.orientation.y*p.orientation.y
             + p.orientation.z*p.orientation.z + p.orientation.w*p.orientation.w;
    if (n < 0.01) { RCLCPP_WARN(log,"Bad quaternion – keeping previous orientation"); return; }
    quatToRPY(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
              roll, pitch, yaw);
    RCLCPP_INFO(log, "Orientation synced: R=%.1f P=%.1f Y=%.1f deg",
                roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
}

// ── control thread ────────────────────────────────────────────────────────────
static void controlLoop(rclcpp::Node::SharedPtr node, std::atomic<bool> &running)
{
    auto log = rclcpp::get_logger("keyboard_teleop");

    MG arm(node, "arm");
    MG gripper(node, "gripper");

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
    RCLCPP_INFO(log, "Frame: %s  EEF: %s", frame.c_str(), eef.c_str());

    // Go to ready, sync orientation state
    arm.setNamedTarget("ready");
    planAndExecute(arm, log, "InitReady");

    double t_roll=0, t_pitch=0, t_yaw=0;
    syncOrientation(arm, log, t_roll, t_pitch, t_yaw);
    printHelp();

    while (running && rclcpp::ok())
    {
        char key = readKey();

        // ════════════════════════════════════════════════════════════
        //  XYZ  –  Cartesian path keeps orientation physically locked
        // ════════════════════════════════════════════════════════════
        if (key=='w'||key=='W'||key=='s'||key=='S'||
            key=='a'||key=='A'||key=='d'||key=='D'||
            key=='q'||key=='Q'||key=='e'||key=='E')
        {
            double dx=0, dy=0, dz=0;
            switch(key){
                case 'w':case 'W': dx=+STEP_M; break;
                case 's':case 'S': dx=-STEP_M; break;
                case 'a':case 'A': dy=+STEP_M; break;
                case 'd':case 'D': dy=-STEP_M; break;
                case 'q':case 'Q': dz=+STEP_M; break;
                case 'e':case 'E': dz=-STEP_M; break;
            }
            auto cur = arm.getCurrentPose().pose;
            RCLCPP_INFO(log, "POS -> x=%.4f y=%.4f z=%.4f",
                cur.position.x+dx, cur.position.y+dy, cur.position.z+dz);
            cartesianMove(arm, log, dx, dy, dz);
            continue;
        }

        // ════════════════════════════════════════════════════════════
        //  BASE ROTATION
        // ════════════════════════════════════════════════════════════
        if (key=='z'||key=='Z'||key=='x'||key=='X')
        {
            auto joints = arm.getCurrentJointValues();
            if (joints.empty()){ RCLCPP_WARN(log,"No joints"); continue; }
            joints[0] += (key=='z'||key=='Z') ? +STEP_BASE : -STEP_BASE;
            RCLCPP_INFO(log, "Base -> %.1f deg", joints[0]*180/M_PI);
            arm.setJointValueTarget(joints);
            planAndExecute(arm, log, "Base");
            continue;
        }

        // ════════════════════════════════════════════════════════════
        //  PRE-GRASP
        // ════════════════════════════════════════════════════════════
        if (key=='g'||key=='G')
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id  = frame;
            ps.pose.position.x  = PREGRASP_X;
            ps.pose.position.y  = PREGRASP_Y;
            ps.pose.position.z  = PREGRASP_Z;
            ps.pose.orientation = rpyToQuat(PREGRASP_ROLL, PREGRASP_PITCH, PREGRASP_YAW);
            arm.setPoseTarget(ps, eef);
            RCLCPP_INFO(log, "-> Pre-grasp (pitch=90deg, facing shelf)");
            planAndExecute(arm, log, "PreGrasp");
            continue;
        }

        // ════════════════════════════════════════════════════════════
        //  NAMED POSES
        // ════════════════════════════════════════════════════════════
        if (key=='h'||key=='H'){
            arm.setNamedTarget("home");
            if(planAndExecute(arm,log,"Home"))
                syncOrientation(arm,log,t_roll,t_pitch,t_yaw);
            continue;
        }
        if (key=='r'||key=='R'){
            arm.setNamedTarget("ready");
            if(planAndExecute(arm,log,"Ready"))
                syncOrientation(arm,log,t_roll,t_pitch,t_yaw);
            continue;
        }

        // ════════════════════════════════════════════════════════════
        //  GRIPPER  –  move() uses GripperActionController directly
        // ════════════════════════════════════════════════════════════
        if (key=='o'||key=='O'){
            RCLCPP_INFO(log, "Gripper -> open");
            gripper.setNamedTarget("open");
            gripper.move(); continue;
        }
        if (key=='c'||key=='C'){
            RCLCPP_INFO(log, "Gripper -> closed");
            gripper.setNamedTarget("closed");
            gripper.move(); continue;
        }

        // ════════════════════════════════════════════════════════════
        //  PRINT
        // ════════════════════════════════════════════════════════════
        if (key=='p'||key=='P'){
            auto p = arm.getCurrentPose().pose;
            double r,pi,y;
            quatToRPY(p.orientation.x,p.orientation.y,
                      p.orientation.z,p.orientation.w,r,pi,y);
            RCLCPP_INFO(log,"Pos : x=%.4f y=%.4f z=%.4f",
                p.position.x,p.position.y,p.position.z);
            RCLCPP_INFO(log,"Ori : R=%.1f P=%.1f Y=%.1f deg (actual)",
                r*180/M_PI,pi*180/M_PI,y*180/M_PI);
            RCLCPP_INFO(log,"Lock: R=%.1f P=%.1f Y=%.1f deg (state)",
                t_roll*180/M_PI,t_pitch*180/M_PI,t_yaw*180/M_PI);
            continue;
        }

        // ════════════════════════════════════════════════════════════
        //  QUIT
        // ════════════════════════════════════════════════════════════
        if (key==27){ RCLCPP_INFO(log,"ESC"); running=false; rclcpp::shutdown(); return; }
    }
}

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "keyboard_teleop",
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
