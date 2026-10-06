// mobilization_trajectory.cpp
//
// Sends a fixed joint-space trajectory to the Armeo Power in Mobilization mode
// (OP_MODE 2) and replays it.
//
//   * Reads the joint range of motion (RoM) from the model and builds the
//     trajectory from it (small sine excursion around the current pose,
//     clamped well inside the RoM).
//   * Uses LOW controller gains (a fraction of the model's allowed range).
//   * Starts at a LOW replay speed. '[' / ']' slow down / speed up while running.
//   * Heartbeat (AC watchdog) is serviced the whole time, including while
//     waiting for you to confirm.
//   * 'x' stops: Replay=0, switch to FreeSpace, unload model.  E-stop is still
//     your real safety.
//
// Usage:
//   mobilization_trajectory --arm left|right [--dry-run] [--center-start]
//                           [--amp <rad>] [--seconds <s>] [--second <s>] [--abs-time] <joint numbers 1-7...>
//
//   e.g.  mobilization_trajectory --arm right --dry-run 4
//         mobilization_trajectory --arm right 4
//
// --dry-run still runs startupSeq() (it has to, to read parameters) but writes
// nothing afterwards.
//
// ASSUMPTIONS (from the parameter names only, please check with --dry-run):
//   - FixedTrajectory targets are absolute joint angles in the same units as
//     the RoM parameters (radians if the numbers look like +-3).
//   - Replay 0->1 starts the replay; ReplaySpeed is a scalar factor within
//     [InputReplaySpeedMin, InputReplaySpeedMax].

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "xpcapi.hpp"
#include "xpcdef.hpp"
#include "keyboard_input.hpp"
//#include "robot_common.hpp"  // the header you pasted: SP/GP/GS, startupSeq, stopSeq, heartBeat

using namespace xpcapi;
using Clock = std::chrono::steady_clock;

// ----------------------------------------------------------------------------
// Tunables
// ----------------------------------------------------------------------------
constexpr int    N_TARGETS        = 9;     // 1 start + 7 excursion points + back to start (max 20)
constexpr double AMP_FRACTION     = 0.25;  // excursion half-amplitude as fraction of half the RoM
constexpr double MARGIN_FRACTION  = 0.10;  // keep targets this far (fraction of RoM) inside each limit
constexpr double GAIN_FRACTION    = 0.20;  // stiffness/damping = min + fraction*(max-min)
constexpr double SPEED_FRACTION   = 0.20;  // initial ReplaySpeed = min + fraction*(max-min)
constexpr double SPEED_STEP_FRAC  = 0.05;  // '[' / ']' step

// First element of the Output/SoftwareSignals/JointAngle signal in your signal
// list (7 consecutive indices, joints 1..7, radians). -1 = unknown.
// Without it the program cannot know where the arm currently is, so it refuses
// to run unless you pass --center-start.
constexpr int JOINT_ANGLE_SIG_BASE = -1;

// Each FixedTrajectory/JointN/TargetK parameter is a [1,2] matrix, i.e. 2 doubles.
// I do not know what the second element is (time? hold? speed?). Assumption:
// element 0 is the joint angle. The other element is KEPT AT ITS CURRENT VALUE
// unless you pass --second <value>. The dry run prints the current values so you
// can see what the model ships with.
constexpr int TARGET_ANGLE_INDEX = 0;

// The model ships every target as [0, 0.001], so the second element looks like a
// per-target time (the "delta time" to reach that target, in seconds). A 1 ms
// default would mean "jump instantly", so it is NEVER left as is: it is set to
// a segment time computed so no joint moves faster than MAX_SEG_SPEED_RAD_S
// (and never shorter than MIN_SEG_TIME_S). Override with --second <seconds>.
// If it turns out to be an absolute timestamp rather than a delta, use --abs-time.
constexpr double MAX_SEG_SPEED_RAD_S = 0.10;  // ~5.7 deg/s nominal, before ReplaySpeed scaling
constexpr double MIN_SEG_TIME_S      = 1.0;

// ----------------------------------------------------------------------------
// Parameter IDs (from param_names.csv)
// ----------------------------------------------------------------------------
constexpr int MODE_FREESPACE    = 0;
constexpr int MODE_MOBILIZATION = 2;

constexpr int P_RECORD       = 529;  // Input/OperationMode/Mobilization/Record
constexpr int P_REPLAY       = 530;  //                                 /Replay
constexpr int P_REPLAY_SPEED = 531;  //                                 /ReplaySpeed
constexpr int P_ARM_SIDE     = 410;  // Input/Hardware/Orthosis/ArmSide

constexpr int P_JS_DAMPING_J1   = 60;  // Input/Controller/JointSpaceDampingJoint1..7 (60..66)
constexpr int P_JS_STIFFNESS_J1 = 67;  // Input/Controller/JointSpaceStiffnessJoint1..7 (67..73)

// Global limit parameters
constexpr int P_LIM_TARGET_MAX       = 3936, P_LIM_TARGET_MIN = 3937;
constexpr int P_LIM_REPLAYSPEED_MAX  = 3928, P_LIM_REPLAYSPEED_MIN = 3929;
constexpr int P_LIM_NUMTARGETS_MAX   = 3882;
constexpr int P_MOB_MAX_NUM_TARGETS  = 4031;
constexpr int P_LIM_STIFF_J1_MAX     = 3848;  // +2*j ; min is +1
constexpr int P_LIM_DAMP_J1_MAX      = 3834;  // +2*j ; min is +1

// RoM scalars: JointNMax = base+2*(N-1), JointNMin = base+2*(N-1)+1
constexpr int P_ROM_PATIENT_LEFT  = 568, P_ROM_PATIENT_RIGHT = 583;
constexpr int P_ROM_DEVICE_LEFT   = 538, P_ROM_DEVICE_RIGHT  = 553;

// Optional controller overrides. NaN = leave as is (values are only printed).
// Which of these Mobilization actually uses is a guess -> check the printout.
struct Override { int id; const char* name; double value; };
static const Override OVERRIDES[] = {
    {48, "Controller/ControlEndEffector",       NAN},
    {56, "Controller/IntentionDetection",       NAN},
    {47, "Controller/ActivateStickyController", NAN},
    {52, "Controller/GuidanceForce",            NAN},
    {49, "Controller/ControllerDirection",      NAN},
    {59, "Controller/JointSpaceAxis",           NAN},
};

constexpr double PI = 3.14159265358979323846;

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------
static long ms_since(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t).count();
}
static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

struct Beat {
    Clock::time_point t = Clock::now();
    void tick(int port) {
        if (ms_since(t) >= 500) { heartBeat(port); t = Clock::now(); }
    }
    void force(int port) { heartBeat(port); t = Clock::now(); }
    void wait(int port, int ms) {  // sleep but keep the heartbeat alive
        auto s = Clock::now();
        while (ms_since(s) < ms) { tick(port); sleep_ms(10); }
    }
};

// Number of doubles in a parameter (product of its dimensions, e.g. [1,2] -> 2).
static int param_count(int port, int idx) {
    int nd = xPCGetParamDimsSize(port, idx);
    if (nd <= 0 || nd > 8) throw std::runtime_error("bad dims size for param " + std::to_string(idx));
    std::vector<int> dims(nd);
    xPCGetParamDims(port, idx, dims.data());
    long n = 1;
    for (int d : dims) n *= d;
    if (n < 1 || n > 4096) throw std::runtime_error("bad element count for param " + std::to_string(idx));
    return (int)n;
}

// Whole parameter as a flat vector (sized from its dimensions, so no overflow).
static std::vector<double> getv(int port, int idx) {
    std::vector<double> v(param_count(port, idx), NAN);
    xPCGetParam(port, idx, v.data());
    return v;
}

// First element only (fine for scalar parameters).
static double getp(int port, int idx) {
    auto v = getv(port, idx);
    return v.empty() ? NAN : v[0];
}

static void setv(int port, int idx, const std::vector<double>& v, const std::string& what) {
    int n = param_count(port, idx);
    if (n != (int)v.size())
        throw std::runtime_error(what + " (param " + std::to_string(idx) + ") has " + std::to_string(n) +
                                 " elements, I was about to write " + std::to_string(v.size()));
    xPCSetParam(port, idx, v.data());
}

static void setp(int port, int idx, double v, const std::string& what) {  // scalar params only
    setv(port, idx, {v}, what);
}

static std::string fmt_vec(const std::vector<double>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); i++) s += (i ? ", " : "") + std::to_string(v[i]);
    return s + "]";
}

// FixedTrajectory/JointN/{NumOfTargets, Target1..Target20, defaultError}: IDs are
// assigned in alphabetical order, so Target10 comes before Target2.
// (Checked against param_names.csv for all 7 joints.)
static int traj_num_id(int j) { return 1659 + 22 * j; }
static int traj_target_id(int j, int k /*1..20*/) {
    static const std::array<int, 21> offset = [] {
        std::array<int, 21> off{};
        std::vector<int> ks(20);
        std::iota(ks.begin(), ks.end(), 1);
        std::sort(ks.begin(), ks.end(),
                  [](int a, int b) { return std::to_string(a) < std::to_string(b); });
        for (size_t r = 0; r < ks.size(); r++) off[ks[r]] = 1 + (int)r;  // 0 = NumOfTargets
        return off;
    }();
    return 1659 + 22 * j + offset[k];
}

struct Range { double lo, hi; };
static bool finite(Range r) { return std::isfinite(r.lo) && std::isfinite(r.hi); }

// Non-blocking confirmation that keeps the heartbeat going.
static bool confirm(int port, Beat& beat, const char* prompt) {
    std::cout << prompt << " [y/n] " << std::endl;
    while (true) {
        beat.tick(port);
        int k = read_key();
        if (k == 'y') return true;
        if (k == 'n' || k == 'x') return false;
        sleep_ms(20);
    }
}

// ----------------------------------------------------------------------------
struct Config {
    bool arm_set = false, left = false, dry = false, center_start = false;
    std::array<bool, 7> move{};
    double amp_rad = 0.15;  // hard cap on the excursion half-amplitude (~8.6 deg)
    double seconds = 30.0;
    double second = NAN;  // segment time [s] for the non-angle element; NaN = computed from the speed cap
    bool abs_time = false;  // treat the second element as cumulative time instead of delta time
};

static bool parse_args(int argc, char** argv, Config& c) {
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--arm" && i + 1 < argc) {
            std::string s = argv[++i];
            if (s != "left" && s != "right") return false;
            c.left = (s == "left"); c.arm_set = true;
        } else if (a == "--dry-run") c.dry = true;
        else if (a == "--center-start") c.center_start = true;
        else if (a == "--amp" && i + 1 < argc) c.amp_rad = std::atof(argv[++i]);
        else if (a == "--seconds" && i + 1 < argc) c.seconds = std::atof(argv[++i]);
        else if (a == "--second" && i + 1 < argc) c.second = std::atof(argv[++i]);
        else if (a == "--abs-time") c.abs_time = true;
        else {
            int j = std::atoi(a.c_str());
            if (j < 1 || j > 7) return false;
            c.move[j - 1] = true;
        }
    }
    bool any = false;
    for (bool b : c.move) any = any || b;
    return c.arm_set && any && c.amp_rad > 0 && c.seconds > 0;
}

static void usage() {
    std::cerr << "usage: mobilization_trajectory --arm left|right [--dry-run] [--center-start]\n"
                 "                               [--amp <rad>] [--seconds <s>] [--second <s>] [--abs-time] <joint 1-7> [<joint> ...]\n";
}

// ----------------------------------------------------------------------------
static void safe_stop(int port, Beat& beat) {
    try {
        SP(port, P_REPLAY, 0.0);
        beat.wait(port, 300);
        SP(port, OP_MODE, (double)MODE_FREESPACE);
        beat.wait(port, 1000);
    } catch (...) {}
}

static void run(int port, const Config& cfg, Beat& beat) {
    // ---- 1. read everything we need ---------------------------------------
    std::printf("ArmSide parameter currently = %g   (you asked for the %s arm - make sure they agree)\n",
                getp(port, P_ARM_SIDE), cfg.left ? "LEFT" : "RIGHT");

    const int pat_base = cfg.left ? P_ROM_PATIENT_LEFT : P_ROM_PATIENT_RIGHT;
    const int dev_base = cfg.left ? P_ROM_DEVICE_LEFT : P_ROM_DEVICE_RIGHT;

    std::array<Range, 7> pat{}, dev{}, rom{};
    double max_abs = 0;
    for (int j = 0; j < 7; j++) {
        pat[j] = {getp(port, pat_base + 2 * j + 1), getp(port, pat_base + 2 * j)};
        dev[j] = {getp(port, dev_base + 2 * j + 1), getp(port, dev_base + 2 * j)};
        if (!finite(dev[j]) || dev[j].hi <= dev[j].lo)
            throw std::runtime_error("device RoM for joint " + std::to_string(j + 1) + " looks invalid");
        rom[j] = dev[j];
        if (finite(pat[j]) && pat[j].hi - pat[j].lo > 1e-3) {  // patient RoM is set -> intersect
            rom[j].lo = std::max(dev[j].lo, pat[j].lo);
            rom[j].hi = std::min(dev[j].hi, pat[j].hi);
        }
        if (rom[j].hi <= rom[j].lo)
            throw std::runtime_error("empty RoM for joint " + std::to_string(j + 1));
        max_abs = std::max({max_abs, std::fabs(rom[j].lo), std::fabs(rom[j].hi)});
    }
    // Heuristic: radians never exceed ~2*pi; if they do, the model is using degrees.
    const bool deg = max_abs > 7.0;
    const double per_rad = deg ? 180.0 / PI : 1.0;     // native units per radian
    const double to_deg  = deg ? 1.0 : 180.0 / PI;     // native -> degrees (for printing)
    std::printf("RoM looks like %s (max |value| = %g)\n", deg ? "DEGREES" : "RADIANS", max_abs);

    const auto tmin_v = getv(port, P_LIM_TARGET_MIN), tmax_v = getv(port, P_LIM_TARGET_MAX);
    std::printf("InputTargetMin = %s  InputTargetMax = %s\n", fmt_vec(tmin_v).c_str(), fmt_vec(tmax_v).c_str());
    Range tlim{TARGET_ANGLE_INDEX < (int)tmin_v.size() ? tmin_v[TARGET_ANGLE_INDEX] : NAN,
               TARGET_ANGLE_INDEX < (int)tmax_v.size() ? tmax_v[TARGET_ANGLE_INDEX] : NAN};
    Range slim{getp(port, P_LIM_REPLAYSPEED_MIN), getp(port, P_LIM_REPLAYSPEED_MAX)};
    const double max_targets_a = getp(port, P_LIM_NUMTARGETS_MAX);
    const double max_targets_b = getp(port, P_MOB_MAX_NUM_TARGETS);
    std::printf("Limits: target [%g, %g]  replay speed [%g, %g]  max targets %g / %g\n",
                tlim.lo, tlim.hi, slim.lo, slim.hi, max_targets_a, max_targets_b);
    if (!finite(slim) || slim.hi <= slim.lo) throw std::runtime_error("replay speed limits invalid");
    for (double m : {max_targets_a, max_targets_b})
        if (std::isfinite(m) && N_TARGETS > m)
            throw std::runtime_error("N_TARGETS exceeds the model's max number of targets");

    // Current pose
    std::array<double, 7> q0{};
    if (JOINT_ANGLE_SIG_BASE >= 0) {
        for (int j = 0; j < 7; j++) q0[j] = GS(port, JOINT_ANGLE_SIG_BASE + j) * per_rad;
        for (int j = 0; j < 7; j++) {
            double tol = 0.05 * (rom[j].hi - rom[j].lo);
            if (!std::isfinite(q0[j]) || q0[j] < rom[j].lo - tol || q0[j] > rom[j].hi + tol)
                throw std::runtime_error("joint " + std::to_string(j + 1) +
                    " measured angle is outside its RoM - wrong signal index/units, or arm is out of range");
        }
    } else if (cfg.center_start) {
        std::cout << "!! JOINT_ANGLE_SIG_BASE not set: assuming trajectory starts at the RoM centre. "
                     "The arm will be pulled there from wherever it is.\n";
        for (int j = 0; j < 7; j++) q0[j] = 0.5 * (rom[j].lo + rom[j].hi);
    } else {
        throw std::runtime_error("current joint angles unknown: set JOINT_ANGLE_SIG_BASE, or pass --center-start");
    }

    // ---- 2. build the plan -------------------------------------------------
    std::array<std::vector<double>, 7> targets;
    std::array<double, 7> kp{}, kd{};
    for (int j = 0; j < 7; j++) {
        const double span = rom[j].hi - rom[j].lo;
        const double lo = rom[j].lo + MARGIN_FRACTION * span;
        const double hi = rom[j].hi - MARGIN_FRACTION * span;
        const double base = std::clamp(q0[j], lo, hi);
        const double amp = cfg.move[j] ? std::min(AMP_FRACTION * 0.5 * span, cfg.amp_rad * per_rad) : 0.0;
        for (int k = 0; k < N_TARGETS; k++) {
            double off = amp * std::sin(2.0 * PI * k / (N_TARGETS - 1));  // starts and ends at 0
            double t = std::clamp(base + off, lo, hi);
            if (finite(tlim) && (t < tlim.lo || t > tlim.hi))
                throw std::runtime_error("target outside InputTarget limits (units mismatch?) joint " +
                                         std::to_string(j + 1));
            targets[j].push_back(t);
        }
        Range sl{getp(port, P_LIM_STIFF_J1_MAX + 2 * j + 1), getp(port, P_LIM_STIFF_J1_MAX + 2 * j)};
        Range dl{getp(port, P_LIM_DAMP_J1_MAX + 2 * j + 1), getp(port, P_LIM_DAMP_J1_MAX + 2 * j)};
        if (!finite(sl) || !finite(dl)) throw std::runtime_error("gain limits unreadable");
        kp[j] = sl.lo + GAIN_FRACTION * (sl.hi - sl.lo);
        kd[j] = dl.lo + GAIN_FRACTION * (dl.hi - dl.lo);
    }
    double speed = slim.lo + SPEED_FRACTION * (slim.hi - slim.lo);

    // Segment time for the second target element (assumed: seconds per segment).
    double max_step = 0;
    for (int j = 0; j < 7; j++)
        for (int k = 1; k < N_TARGETS; k++)
            max_step = std::max(max_step, std::fabs(targets[j][k] - targets[j][k - 1]));
    const double seg_T = std::isnan(cfg.second)
                             ? std::max(MIN_SEG_TIME_S, max_step / (MAX_SEG_SPEED_RAD_S * per_rad))
                             : cfg.second;
    if (!(seg_T > 0) || !std::isfinite(seg_T)) throw std::runtime_error("bad segment time");

    // ---- 3. show it --------------------------------------------------------
    std::cout << "\nPlan (angles in degrees):\n";
    std::printf("%-3s %-5s %9s %9s %9s %9s %9s   %s\n", "J", "move", "RoM lo", "RoM hi", "start", "min tgt", "max tgt", "Kp / Kd");
    for (int j = 0; j < 7; j++) {
        auto mm = std::minmax_element(targets[j].begin(), targets[j].end());
        std::printf("%-3d %-5s %9.1f %9.1f %9.1f %9.1f %9.1f   %g / %g\n", j + 1, cfg.move[j] ? "YES" : "hold",
                    rom[j].lo * to_deg, rom[j].hi * to_deg, q0[j] * to_deg,
                    *mm.first * to_deg, *mm.second * to_deg, kp[j], kd[j]);
    }
    std::printf("Replay speed start = %g, run time = %g s, %d targets/joint\n", speed, cfg.seconds, N_TARGETS);
    std::printf("Second target element = %g s per segment (%s time); largest step %.1f deg  ->  one pass ~ %g s at ReplaySpeed 1\n\n",
                seg_T, cfg.abs_time ? "ABSOLUTE" : "delta", max_step * to_deg, seg_T * N_TARGETS);

    std::cout << "Existing FixedTrajectory targets in the model (element " << TARGET_ANGLE_INDEX
              << " is treated as the angle; the other element will be overwritten with the segment time):\n";
    for (int j = 0; j < 7; j++) {
        if (!cfg.move[j]) continue;
        std::printf("  joint %d: NumOfTargets=%g", j + 1, getp(port, traj_num_id(j)));
        for (int k = 1; k <= 3; k++)
            std::printf("  T%d=%s", k, fmt_vec(getv(port, traj_target_id(j, k))).c_str());
        std::printf(" ...\n");
    }
    std::cout << "\nCurrent values of controller parameters I did NOT choose a value for:\n";
    for (const auto& o : OVERRIDES)
        std::printf("  %-36s (id %d) = %g%s\n", o.name, o.id, getp(port, o.id),
                    std::isnan(o.value) ? "" : "   -> will be overwritten");
    std::cout << std::endl;

    if (cfg.dry) { std::cout << "--dry-run: nothing written.\n"; return; }

    if (!confirm(port, beat, "Arm clear, E-stop in hand, write trajectory and start?")) {
        std::cout << "Aborted by user.\n";
        return;
    }

    // ---- 4. write parameters (before entering the mode) --------------------
    setp(port, P_REPLAY, 0.0, "Replay");
    setp(port, P_RECORD, 0.0, "Record");
    for (const auto& o : OVERRIDES)
        if (!std::isnan(o.value)) setp(port, o.id, o.value, o.name);
    for (int j = 0; j < 7; j++) {
        setp(port, P_JS_STIFFNESS_J1 + j, kp[j], "JointSpaceStiffness" + std::to_string(j + 1));
        setp(port, P_JS_DAMPING_J1 + j, kd[j], "JointSpaceDamping" + std::to_string(j + 1));
        beat.tick(port);
    }
    setp(port, P_REPLAY_SPEED, speed, "ReplaySpeed");
    std::array<std::vector<std::vector<double>>, 7> written;
    for (int j = 0; j < 7; j++) {
        setp(port, traj_num_id(j), (double)N_TARGETS, "NumOfTargets");
        for (int k = 1; k <= N_TARGETS; k++) {
            auto v = getv(port, traj_target_id(j, k));  // keep the other element(s) as they are
            if (TARGET_ANGLE_INDEX >= (int)v.size())
                throw std::runtime_error("target has fewer elements than TARGET_ANGLE_INDEX");
            v[TARGET_ANGLE_INDEX] = targets[j][k - 1];
            if (v.size() == 2) v[1 - TARGET_ANGLE_INDEX] = cfg.abs_time ? k * seg_T : seg_T;
            else if (v.size() != 1) throw std::runtime_error("unexpected target size " + std::to_string(v.size()));
            written[j].push_back(v);
            setv(port, traj_target_id(j, k), v, "Target");
        }
        beat.tick(port);
    }
    // read back and verify
    for (int j = 0; j < 7; j++)
        for (int k = 1; k <= N_TARGETS; k++) {
            auto got = getv(port, traj_target_id(j, k));
            for (size_t e = 0; e < got.size(); e++)
                if (std::fabs(got[e] - written[j][k - 1][e]) > 1e-6)
                    throw std::runtime_error("readback mismatch on joint " + std::to_string(j + 1) +
                                             " target " + std::to_string(k));
        }
    std::cout << "Trajectory written and verified.\n";

    // ---- 5. enter mode and replay -----------------------------------------
    SP(port, SAFETY_ENABLE, 0.0);
    SP(port, OP_MODE, (double)MODE_MOBILIZATION);
    beat.wait(port, 500);
    SP(port, P_REPLAY, 0.0);
    SP(port, P_REPLAY, 1.0);
    std::cout << "Replaying.  '[' slower, ']' faster, 'x' stop.\n";

    auto t_start = Clock::now();
    auto t_log = Clock::now();
    const double step = SPEED_STEP_FRAC * (slim.hi - slim.lo);
    while (true) {
        beat.tick(port);

        int k = read_key();
        if (k == 'x') break;
        if (k == '[' || k == ']') {
            speed = std::clamp(speed + (k == ']' ? step : -step), slim.lo, slim.hi);
            SP(port, P_REPLAY_SPEED, speed);
            std::printf("ReplaySpeed = %g\n", speed);
        }
        if (ms_since(t_start) > (long)(cfg.seconds * 1000)) { std::cout << "Time up.\n"; break; }

        if (JOINT_ANGLE_SIG_BASE >= 0 && ms_since(t_log) > 500) {
            t_log = Clock::now();
            std::printf("q[deg]:");
            bool out = false;
            for (int j = 0; j < 7; j++) {
                double q = GS(port, JOINT_ANGLE_SIG_BASE + j) * per_rad;
                double tol = 0.05 * (rom[j].hi - rom[j].lo);
                if (q < rom[j].lo - tol || q > rom[j].hi + tol) out = true;
                std::printf(" %7.1f", q * to_deg);
            }
            std::printf("\n");
            if (out) { std::cout << "Joint outside RoM -> stopping.\n"; break; }
        }
        sleep_ms(10);
    }
    safe_stop(port, beat);
}

int main(int argc, char** argv) {
    Config cfg;
    if (!parse_args(argc, argv, cfg)) { usage(); return 1; }

    enable_raw_mode();
    int port = startupSeq();
    std::cout << "Startup Seq Done\n";

    Beat beat;
    beat.force(port);

    int rc = 0;
    try {
        run(port, cfg, beat);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        safe_stop(port, beat);
        rc = 1;
    }

    disable_raw_mode();
    stopSeq(port);
    return rc;
}