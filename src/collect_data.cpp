#include "xpcapi.hpp"

#include <string>
#include <iostream>
#include <vector>
#include <cstring>
#include <array>

#include <sstream>

#include <fstream>

#include "keyboard_input.hpp"



using namespace xpcapi;

static void print_jacobian(std::array<double, 36> j) {
    std::cout << "Jacobian:" << std::endl;
    for (int i = 0; i < 6; i++) {
        std::cout << "[" << j[i*6 + 0] << "," << j[i*6 + 1] << "," << j[i*6 + 2]
            << "," << j[i*6 + 3] << "," << j[i*6 + 4] << "," << j[i*6 + 5] << "]" << std::endl; 
    }
}

static std::string fmt_vec(const std::vector<int>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); i++) s += (i ? ", " : "") + std::to_string(v[i]);
    return s + "]";
}

int main() {
    enable_raw_mode();
    constexpr char addr[] = "10.10.10.11";
    constexpr char net_port[] = "22222";

    // Open a connection to the Armeo Power
    int port = startupSeq();

    std::ofstream file("Samples.csv");
    file << "q1,q2,q3,q4,q5,q6,q7,x,y,z,j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11,j12,j13,j14,j15,j16,j17,j18,j19,j20,j21,j22,j23,j24,j25,j26,j27,j28,j29,j30,j31,j32,j33,j34,j35,j36" << std::endl;
    
    while (true) {

        std::array<double, 7+3+36> samples;
        static const std::array<int, 7 + 3 + 36> sig_ids = [port]() {
            std::array<int, 7 + 3 + 36> arr;
            int idx = 0;

            int base_id = xPCGetSignalIdx(port, "Output/SoftwareSignals/JointAngle/s1");
            for (int i = 0; i < 7; i++) {
                arr[idx] = base_id + i;
                idx++;
            }
            base_id = xPCGetSignalIdx(port, "Output/SoftwareSignals/EndEffector/s1");
            for (int i = 0; i < 3; i++) {
                arr[idx] = base_id + i;
                idx++;
            }
            base_id = xPCGetSignalIdx(port, "Kinematics/Jacobian EE/s1");
            for (int i = 0; i < 36; i++) {
                arr[idx] = base_id + i;
                idx++;
            }
            return arr;
        }();
        xPCGetSignals(port, 7+3+36, sig_ids.data(), samples.data());
        std::stringstream ss;
        for (int i = 0; i < 7+3+36; i++) {
            ss << samples[i] << ",";
        }
        auto line = ss.str();
        line.pop_back();
        file << line << std::endl;        

        if (read_key() == 'x') {
            break;
        }
    }

    file.close();
    
    if (xPCIsAppRunning(port)) {
        xPCStopApp(port);
    }
    xPCUnloadApp(port);
    return 0;
}
