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

int main() {
    enable_raw_mode();
    int port = startupSeq();

    std::cout << "Startup Seq Done\n"; 
    
    heartBeat(port);
    auto hb_time = NOW();

    // Create targets
    auto joint_angles = get_joint_angles(port);

    // SP(port, P_REPLAY, 0.0);
    // SP(port, P_RECORD, 0.0);
    // SP(port, P_REPLAY_SPEED, 1.0);
    
    SP(port, SAFETY_ENABLE, 0.0);
    SP(port, OP_MODE, OMServiceMode);

    // 533,Input/OperationMode/ServiceMode/DesiredDq,Value
    // 534,Input/OperationMode/ServiceMode/DesiredTorque,Value
    // 535,Input/OperationMode/ServiceMode/ServiceOption,Value
    // 536,Input/OperationMode/ServiceMode/TestJoint,Value
    // 537,Input/OperationMode/ServiceMode/defaultError,Value

    // 3932,,InputServiceOptionMax
    // 3933,,InputServiceOptionMin

    double opt_max, opt_min;
    opt_max = GP(port, 3932);
    opt_min = GP(port, 3933);
    std::cout << opt_min << "," << opt_max << std::endl;

    SP(port, 536, 1.0);
    SP(port, 533, 0.2);
    SP(port, 535, 0.0);


    // SP(port, P_REPLAY, 1.0);

    
    auto tq_time = NOW();

    int opt = 0;
    int joint = 0;
    bool run = true;
    while (run) {
        if (DURATION(hb_time) > 500) {
            heartBeat(port);
            hb_time = NOW();
        }

        if (DURATION(tq_time) > 50) {
            switch (read_key()) {
                case 'x':
                    run = false;
                    break;
                case 'a':
                    opt--;
                    std::cout << "opt: " << opt << std::endl;
                    SP(port, 535, (double)opt);
                    break;
                case 'd':
                    opt++;
                    std::cout << "opt: " << opt << std::endl;
                    SP(port, 535, (double)opt);
                    break;

                case 'w':
                    joint++;
                    std::cout << "joint: " << joint << std::endl;
                    SP(port, 536, (double)joint);
                    break;
                case 's':
                    joint--;
                    std::cout << "joint: " << joint << std::endl;
                    SP(port, 536, (double)joint);
                    break;
            }
            
            tq_time = NOW();
            joint_angles = get_joint_angles(port);
            // std::cout << "ja:" << fmt_vec(joint_angles) << std::endl;

        }
    }

    disable_raw_mode();
    stopSeq(port);
    return 0;
}
