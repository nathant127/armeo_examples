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

#define NOW() std::chrono::steady_clock::now()
#define DURATION(time) std::chrono::duration_cast<std::chrono::milliseconds>(NOW() - time).count()

static std::string fmt_vec(const std::vector<double>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); i++) s += (i ? ", " : "") + std::to_string(v[i]);
    return s + "]";
}

bool check_continue() {

    int key = -1;
    while (true) {
        key = read_key();
        switch (key) {

            case 'c': return true;

            case '\0': return false;

            default: break;
        }
    }

    return false;
}

int main() {
    enable_raw_mode();
    int port = startupSeq();

    std::cout << "Startup Seq Done\n"; 
    
    heartBeat(port);
    auto hb_time = NOW();

    // Create targets
    auto joint_angles = get_joint_angles(port);

    SP(port, P_REPLAY, 0.0);
    SP(port, P_RECORD, 0.0);
    SP(port, P_REPLAY_SPEED, 1.0);
    
    SP(port, SAFETY_ENABLE, 0.0);
    SP(port, OP_MODE, OMMobilization);

    // SP(port, P_REPLAY, 1.0);
    SP(port, P_RECORD, 1.0);

    std::cout << "RECORDING" << std::endl;

    
    auto tq_time = NOW();

    while (true) {
        if (DURATION(hb_time) > 500) {
            heartBeat(port);
            hb_time = NOW();
        }

        if (DURATION(tq_time) > 50) {
            tq_time = NOW();
            joint_angles = get_joint_angles(port);
            std::cout << "ja:" << fmt_vec(joint_angles) << std::endl;

            if (check_continue()) 
                break;

        }
    }

    SP(port, P_RECORD, 0.0);
    SP(port, P_REPLAY, 1.0);

    std::cout << "REPLAYING" << std::endl;

    while (true) {
        if (DURATION(hb_time) > 500) {
            heartBeat(port);
            hb_time = NOW();
        }

        if (DURATION(tq_time) > 50) {
            tq_time = NOW();
            joint_angles = get_joint_angles(port);
            std::cout << "ja:" << fmt_vec(joint_angles) << std::endl;

            if (check_continue()) 
                break;

        }
    }

    disable_raw_mode();
    stopSeq(port);
    return 0;
}
