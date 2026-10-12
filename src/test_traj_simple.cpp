#include "xpcapi.hpp"
#include "xpcdef.hpp"

#include <string>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <numeric>

#include "keyboard_input.hpp"

using namespace xpcapi;

constexpr int P_RECORD       = 529;  // Input/OperationMode/Mobilization/Record
constexpr int P_REPLAY       = 530;  //                                 /Replay
constexpr int P_REPLAY_SPEED = 531;  //                                 /ReplaySpeed
constexpr int P_ARM_SIDE     = 410;  // Input/Hardware/Orthosis/ArmSide
#define NOW() std::chrono::steady_clock::now()
#define DURATION(time) std::chrono::duration_cast<std::chrono::milliseconds>(NOW() - time).count()

static std::string fmt_vec(const std::vector<double>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); i++) s += (i ? ", " : "") + std::to_string(v[i]);
    return s + "]";
}

/// @brief Gets the param "number of trajectories" for the given joint's mobilization mode
/// @param j 
/// @return 
static int traj_num_id(int j) { return 1659 + 22 * j; }
/// @brief Gets the given target for some joint and target number (mobilization mode)
/// @param j 
/// @param k 
/// @return 
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

void set_joint_target(int port, int n, int j, double angle, double second_value) {
    double buf[] = {angle, second_value};
    xpcapi::xPCSetParam(port, traj_target_id(j, n), buf);
}
void set_target(int port, int n, std::vector<double> angles, std::vector<double> second_value) {
    set_joint_target(port, n, 0, angles[0], second_value[0]);
    set_joint_target(port, n, 1, angles[1], second_value[1]);
    set_joint_target(port, n, 2, angles[2], second_value[2]);
    set_joint_target(port, n, 3, angles[3], second_value[3]);
    set_joint_target(port, n, 4, angles[4], second_value[4]);
    set_joint_target(port, n, 5, angles[5], second_value[5]);
    set_joint_target(port, n, 6, angles[6], second_value[6]);
}

int main() {
    enable_raw_mode();
    int port = startupSeq();

    std::cout << "Startup Seq Done\n"; 

    double MAX_FORCE = GP(port, 152);
    printf("MAX_FORCE: %f\n", MAX_FORCE);
    
    heartBeat(port);
    auto hb_time = NOW();

    // Create targets
    auto joint_angles = get_joint_angles(port);
    joint_angles[0] = -2.65;
    set_target(port, 1, joint_angles, std::vector<double>(7, 0.1));
    joint_angles[0] = -2.9;
    set_target(port, 2, joint_angles, std::vector<double>(7, 0.1));
    for (int i = 0; i < 7; i++) {
        SP(port, traj_num_id(i), 2.0);
    }

    SP(port, P_REPLAY, 0.0);
    SP(port, P_RECORD, 0.0);
    SP(port, P_REPLAY_SPEED, 0.05);
    
    SP(port, SAFETY_ENABLE, 0.0);
    SP(port, OP_MODE, OMMobilization);

    SP(port, P_REPLAY, 0.0);
    SP(port, P_REPLAY, 1.0);

    
    auto tq_time = NOW();



    std::array<double, 7> torques;
    double applied_torque = 1.0;
    while (true) {
        if (DURATION(hb_time) > 500) {
            heartBeat(port);
            hb_time = NOW();
        }

        if (DURATION(tq_time) > 50) {
            joint_angles = get_joint_angles(port);
            std::cout << "ja:" << fmt_vec(joint_angles) << std::endl;
            tq_time = NOW();
        }
    }

    disable_raw_mode();
    stopSeq(port);
    return 0;
}
