#include "xpcapi.hpp"

#include <string>
#include <iostream>
#include <vector>
#include <cstring>
#include <array>

#include <fstream>

#include "keyboard_input.hpp"
#include "fkine.hpp"



using namespace xpcapi;

static void print_jacobian(std::array<double, 36> j) {
    std::cout << "Jacobian:" << std::endl;
    for (int i = 0; i < 6; i++) {
        std::cout << "[" << j[i*6 + 0] << "," << j[i*6 + 1] << "," << j[i*6 + 2]
            << "," << j[i*6 + 3] << "," << j[i*6 + 4] << "," << j[i*6 + 5] << "]" << std::endl; 
    }
}

int main() {
    enable_raw_mode();
    constexpr char addr[] = "10.10.10.11";
    constexpr char net_port[] = "22222";

    // Open a connection to the Armeo Power
    int port = xPCOpenTcpIpPort(addr, net_port);

    
    char name[100];
    // Check if an app is loaded
    // If no app is loaded, the app is called "loader"
    xPCGetAppName(port, name);
    if (std::strcmp("loader", name) != 0) {
        // Stop app if it is running
        if (xPCIsAppRunning(port)) {
            xPCStopApp(port);
        }
        xPCUnloadApp(port);
    }
    
    constexpr char target_model[] = "AM2_3_1_01HWfullV2_PCIe";
    constexpr char path[] = "../models";

    // Load the model for the Armeo Power
    xPCLoadApp(port, path, target_model);
    // Start the app
    xPCStartApp(port);


    struct {
        double x;
        double y;
        double z;
    } end_effector;
    struct {
        double base_joint;
        double shoulder_joint;
        double shoulder_twise_joint;
        double elbow_joint;
        double arm_twist_joint;
        double wrist_joint;
        double gripper_joint;
    } joint_angles, joint_velocities;
    // std::ofstream file("JointAngleVsEndEffector.csv");
    // file << "j1,j2,j3,j4,j5,j6,j7,x,y,z,x_est,y_est,z_est,error" << std::endl;
    
    // 1345,Output/SoftwareSignals/EndEffector/s1
    // 1346,Output/SoftwareSignals/EndEffector/s2
    // 1347,Output/SoftwareSignals/EndEffector/s3
    while (true) {
        // Get end effector position
        // You can use either xPCGetSignals to get all 3, or 3 individual xPCGetSignal() calls to get each individually
        int sig_ids[3] = {1345, 1346, 1347};
        xPCGetSignals(port, 3, sig_ids, reinterpret_cast<double*>(&end_effector));
    
        // 1349,Output/SoftwareSignals/JointAngle/s1
        // 1350,Output/SoftwareSignals/JointAngle/s2
        // 1351,Output/SoftwareSignals/JointAngle/s3
        // 1352,Output/SoftwareSignals/JointAngle/s4
        // 1353,Output/SoftwareSignals/JointAngle/s5
        // 1354,Output/SoftwareSignals/JointAngle/s6
        // 1355,Output/SoftwareSignals/JointAngle/s7

        // Get joint angles using indiviudal method
        joint_angles.base_joint = xPCGetSignal(port, 1349);
        joint_angles.shoulder_joint = xPCGetSignal(port, 1350);
        joint_angles.shoulder_twise_joint = xPCGetSignal(port, 1351);
        joint_angles.elbow_joint = xPCGetSignal(port, 1352);
        joint_angles.arm_twist_joint = xPCGetSignal(port, 1353);
        joint_angles.wrist_joint = xPCGetSignal(port, 1354);
        joint_angles.gripper_joint = xPCGetSignal(port, 1355);
    
    
    
        // 1356,Output/SoftwareSignals/JointVelocity/s1
        // 1357,Output/SoftwareSignals/JointVelocity/s2
        // 1358,Output/SoftwareSignals/JointVelocity/s3
        // 1359,Output/SoftwareSignals/JointVelocity/s4
        // 1360,Output/SoftwareSignals/JointVelocity/s5
        // 1361,Output/SoftwareSignals/JointVelocity/s6
        // 1362,Output/SoftwareSignals/JointVelocity/s7


        // Rather than hardcoding the id we can retrieve it from the name of the parameter
        int joint_vel_id_0 = xPCGetSignalIdx(port, "Output/SoftwareSignals/JointVelocity/s1");
        // Individual method, but using a for loop to iterate through each;
        double* joint_vel_map_arr = reinterpret_cast<double*>(&joint_velocities);
        for (int i = 0; i < 7; i++) {
            joint_vel_map_arr[i] = xPCGetSignal(port, i + joint_vel_id_0);
        }

        double jacobian_ee[36];
        static const std::array<int, 36> sig_ids_jacobian = [port]() {
            int jacobian_base_id = xPCGetSignalIdx(port, "Kinematics/Jacobian EE/s1");
            std::array<int, 36> arr;
            for (int i = 0; i < 36; i++) {
                arr[i] = jacobian_base_id + i;
            }
            return arr;
        }();
        xPCGetSignals(port, 36, sig_ids_jacobian.data(), jacobian_ee);

        const std::array<double, 36> jacobian_row_major = [jacobian_ee]() {
            std::array<double, 36> dst;
           for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < 6; ++c) {
                    // Source (column-major): row + col * total_rows
                    // Destination (row-major): row * total_cols + col
                    dst[r * 6 + c] = jacobian_ee[r + c * 6];
                }
            }
            return dst;
        }();

        print_jacobian(jacobian_row_major);



        auto ee_est = estimate_ee(reinterpret_cast<double*>(&joint_angles));

        double error = sqrtf64(powf64(ee_est[0] - end_effector.x, 2) + powf64(ee_est[1] - end_effector.y, 2) + powf64(ee_est[2] - end_effector.z, 2));

        // file << joint_angles.shoulder_joint << "," << joint_angles.j1 << "," << joint_angles.j2 <<
        //     "," << joint_angles.j3 << "," << joint_angles.j4 << "," << joint_angles.j5 << 
        //     "," << joint_angles.j6 << "," << end_effector.x << "," << end_effector.y << "," << end_effector.z <<
        //     "," << ee_est[0] << "," << ee_est[1] << "," << ee_est[2] <<
        //     "," << error << std::endl;

        
        
        std::cout << "End Effector (x,y,z)  : " << end_effector.x << " " << end_effector.y << " " << end_effector.z << std::endl;
        std::cout << "End Effector Est (xyz): " << ee_est[0] << " " << ee_est[1] << " " << ee_est[2] << std::endl;
        std::cout << "Joint Angles          : " << joint_angles.base_joint << " " << 
            joint_angles.shoulder_joint << " " << joint_angles.shoulder_twise_joint << " " << joint_angles.elbow_joint << " " << 
            joint_angles.arm_twist_joint << " " << joint_angles.wrist_joint << " " << joint_angles.gripper_joint << " " << std::endl;
        std::cout << "Joint Velocities      : " << joint_velocities.base_joint << " " << 
            joint_velocities.shoulder_joint << " " << joint_velocities.shoulder_twise_joint << " " << joint_velocities.elbow_joint << " " << 
            joint_velocities.arm_twist_joint << " " << joint_velocities.wrist_joint << " " << joint_velocities.gripper_joint << " " << std::endl;

        

        if (read_key() == 'x') {
            break;
        }
    }

    // file.close();
    
    if (xPCIsAppRunning(port)) {
        xPCStopApp(port);
    }
    xPCUnloadApp(port);
    return 0;
}
