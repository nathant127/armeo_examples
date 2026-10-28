#include "xpcapi.hpp"

#include <string>
#include <iostream>
#include <vector>
#include <cstring>
#include <array>

#include <sstream>

#include <fstream>

#include "keyboard_input.hpp"
#include "fkine.hpp"

#include "xpcdef.hpp"



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

    std::ofstream file("Comparison.csv");
    file << "q1,q2,q3,q4,q5,q6,q7,x,y,z,x_est,y_est,z_est,error" << std::endl;

    double min = 999.0;
    double max = 0.0;
    double avg = 0.0;
    int count = 0;
    
    while (true) {

        std::array<double, 7+3> samples;
        static const std::array<int, 7 + 3> sig_ids = [port]() {
            std::array<int, 7 + 3> arr;
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
            return arr;
        }();
        xPCGetSignals(port, 7+3, sig_ids.data(), samples.data());
        std::stringstream ss;
        for (int i = 0; i < 7+3; i++) {
            ss << samples[i] << ",";
        }
        auto line = ss.str();
        line.pop_back();
        auto ee_est = estimate_ee_updated_params(&samples[0]);
        double error = sqrtf64(powf64(ee_est[0] - samples[7], 2) + powf64(ee_est[1] - samples[8], 2) + powf64(ee_est[2] - samples[9], 2));

        file << line << "," << ee_est[0] << "," << ee_est[1] << "," << ee_est[2] << "," << error << std::endl;   
        
        if (error < min) min = error;
        if (error > max) max = error;
        avg += error;
        count++;
        

        if (read_key() == 'x') {
            break;
        }
    }
    
    file.close();
    avg = avg / count;
    std::cout << "avg: " << avg << std::endl;
    std::cout << "min: " << min << std::endl;
    std::cout << "max: " << max << std::endl;

    
    if (xPCIsAppRunning(port)) {
        xPCStopApp(port);
    }
    xPCUnloadApp(port);
    return 0;
}
