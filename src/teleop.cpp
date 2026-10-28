#include "xpcapi.hpp"
#include "xpcdef.hpp"

#include <string>
#include <chrono>
#include <iostream>
#include <algorithm>

#include "keyboard_input.hpp"

using namespace xpcapi;

using namespace std::chrono_literals;

void apply_torques(int port, std::array<double, 7> torques) {
    SP(port, 467, torques[0]);
    SP(port, 468, torques[1]);
    SP(port, 469, torques[2]);
    SP(port, 470, torques[3]);
    SP(port, 471, torques[4]);
    SP(port, 472, torques[5]);
    SP(port, 473, torques[6]);
}

bool handle_keys(std::array<double, 7>& torques, double &applied_torque, const double max_torque) {
    torques.fill(0.0); // Reset torques

    int key = -1;
    while (true) {
        key = read_key();
        switch (key) {
            case UP_ARROW: 
                applied_torque += 1.0; 
                applied_torque = std::clamp(applied_torque, -max_torque, max_torque);
                std::cout << "New Torque: " << applied_torque << std::endl;
                break;
            case DOWN_ARROW: 
                applied_torque -= 1.0; 
                applied_torque = std::clamp(applied_torque, -max_torque, max_torque);
                std::cout << "New Torque: " << applied_torque << std::endl;
                break;

            case 'q': torques[0] = applied_torque; break;
            case 'a': torques[0] = -applied_torque; break;
            case 'w': torques[1] = applied_torque; break;
            case 's': torques[1] = -applied_torque; break;
            case 'e': torques[2] = applied_torque; break;
            case 'd': torques[2] = -applied_torque; break;
            case 'r': torques[3] = applied_torque; break;
            case 'f': torques[3] = -applied_torque; break;
            case 't': torques[4] = applied_torque; break;
            case 'g': torques[4] = -applied_torque; break;
            case 'y': torques[5] = applied_torque; break;
            case 'h': torques[5] = -applied_torque; break;
            case 'u': torques[6] = applied_torque; break;
            case 'j': torques[6] = -applied_torque; break;

            case 'x': return false;

            case '\0': return true;

            default: break;
        }
    }

    return true;
}

int main() {
    enable_raw_mode();
    int port = startupSeq();
    
    SP(port, SAFETY_ENABLE, 0.0);
    SP(port, OP_MODE, 10.0);

    std::cout << "Controls:" << std::endl;
    std::cout << "Change Torque: up arrow  down arrow"  << std::endl;
    std::cout << "Apply Torque to Joints" << std::endl;
    std::cout << "    pos  neg" << std::endl;
    std::cout << "J1: 'q'  'a'" << std::endl;
    std::cout << "J2: 'w'  's'" << std::endl;
    std::cout << "J3: 'e'  'd'" << std::endl;
    std::cout << "J4: 'r'  'f'" << std::endl;
    std::cout << "J5: 't'  'g'" << std::endl;
    std::cout << "J6: 'y'  'h'" << std::endl;
    std::cout << "J7: 'u'  'j'" << std::endl;
    std::cout << std::endl;
    std::cout << "exit program: 'x'" << std::endl;


    std::array<double, 7> torques;
    double applied_torque = 1.0;
    while (handle_keys(torques, applied_torque, 25.0)) {
        apply_torques(port, torques);
        std::this_thread::sleep_for(50ms);
    }

    disable_raw_mode();
    stopSeq(port);
    return 0;
}
