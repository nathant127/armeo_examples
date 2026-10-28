#include "xpcapi.hpp"
#include "xpcdef.hpp"

#include <string>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <numeric>

#include "keyboard_input.hpp"

using namespace xpcapi;
using namespace std::chrono_literals;

constexpr int P_RECORD       = 529;  // Input/OperationMode/Mobilization/Record
constexpr int P_REPLAY       = 530;  //                                 /Replay
constexpr int P_REPLAY_SPEED = 531;  //                                 /ReplaySpeed

static int get_num_targets_id(int joint_num) {
    constexpr std::array<int, 7> base_lookup = {
        1659, 1681, 1703, 1725, 1747, 1769, 1791
    };
    return base_lookup[joint_num];
}

static int get_target_id(int target_num, int joint_num) {
    constexpr std::array<int, 20> offset_lookup = {
        1,12,14,15,16,17,18,19,20,2,3,4,5,6,7,8,9,10,11,13
    };
    return get_num_targets_id(joint_num) + offset_lookup[target_num];
}


void set_joint_target(int port, int n, int j, double angle, double time) {
    double buf[] = {angle, time};
    xpcapi::xPCSetParam(port, get_target_id(n, j), buf);
}
void set_target(int port, int n, std::vector<double> angles, std::vector<double> time) {
    set_joint_target(port, n, 0, angles[0], time[0]);
    set_joint_target(port, n, 1, angles[1], time[1]);
    set_joint_target(port, n, 2, angles[2], time[2]);
    set_joint_target(port, n, 3, angles[3], time[3]);
    set_joint_target(port, n, 4, angles[4], time[4]);
    set_joint_target(port, n, 5, angles[5], time[5]);
    set_joint_target(port, n, 6, angles[6], time[6]);
}

int main() {
    enable_raw_mode();
    int port = startupSeq();

    auto ja = get_joint_angles(port);
    for (int i = 0; i < 7; i++) {
        set_joint_target(port, 0, i, ja[i], 1.0);
        SP(port, get_num_targets_id(i), 1.0);
    }


    // Create targets
    int joint = 0;
    double targets[2] = {0.0, -0.5};
    double durations[2] = {2.0, 2.0};
    set_joint_target(port, 0, joint, targets[0], durations[0]);
    set_joint_target(port, 1, joint, targets[1], durations[1]);
    SP(port, get_num_targets_id(joint), 2.0);


    SP(port, P_REPLAY, 0.0);
    SP(port, P_RECORD, 0.0);
    SP(port, P_REPLAY_SPEED, 1.0);
    
    SP(port, SAFETY_ENABLE, 0.0);
    SP(port, OP_MODE, OMMobilization);

    SP(port, P_REPLAY, 1.0);

    while (true) {
        char key = static_cast<char>(read_key());
        if (key == 'x') {
            break;
        }
    }

    disable_raw_mode();
    stopSeq(port);
    return 0;
}
