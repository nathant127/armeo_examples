#include "xpcapi.hpp"

#include <string>
#include <iostream>
#include <vector>
#include <cstring>



using namespace xpcapi;

int main() {
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
        double shoulder_joint;
        double j1;
        double j2;
        double j3;
        double j4;
        double j5;
        double j6;
    } joint_angles, joint_velocities;

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
        joint_angles.shoulder_joint = xPCGetSignal(port, 1349);
        joint_angles.j1 = xPCGetSignal(port, 1350);
        joint_angles.j2 = xPCGetSignal(port, 1351);
        joint_angles.j3 = xPCGetSignal(port, 1352);
        joint_angles.j4 = xPCGetSignal(port, 1353);
        joint_angles.j5 = xPCGetSignal(port, 1354);
        joint_angles.j6 = xPCGetSignal(port, 1355);
    
    
    
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

        std::cout << "End Effector (x,y,z) : " << end_effector.x << " " << end_effector.y << " " << end_effector.z << std::endl;
        std::cout << "Joint Angles         : " << joint_angles.shoulder_joint << " " << 
            joint_angles.j1 << " " << joint_angles.j2 << " " << joint_angles.j3 << " " << 
            joint_angles.j4 << " " << joint_angles.j5 << " " << joint_angles.j6 << " " << std::endl;
        std::cout << "Joint Velocities     : " << joint_velocities.shoulder_joint << " " << 
            joint_velocities.j1 << " " << joint_velocities.j2 << " " << joint_velocities.j3 << " " << 
            joint_velocities.j4 << " " << joint_velocities.j5 << " " << joint_velocities.j6 << " " << std::endl;

    }
    
    if (xPCIsAppRunning(port)) {
        xPCStopApp(port);
    }
    xPCUnloadApp(port);
    return 0;
}
