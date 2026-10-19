#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

// Structure to represent a single row in the DH parameter table
struct DHParam {
    double a;     // Link length
    double d;     // Link offset - Variable for Prismatic joints
    double alpha; // Link twist (radians)
    double theta; // Joint angle (radians) - Variable for Revolute joints
    bool isPrismatic; // true = Prismatic, false = Revolute
};

// 4x4 Matrix representation
using Matrix4x4 = std::vector<std::vector<double>>;

// Helper function to initialize an identity matrix
Matrix4x4 getIdentity() {
    return {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
}

// Function to multiply two 4x4 matrices
Matrix4x4 multiply(const Matrix4x4& A, const Matrix4x4& B) {
    Matrix4x4 C = { {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

// Computes the individual transformation matrix for a single link
Matrix4x4 computeLinkTransform(const DHParam& param, double joint_input) {
    // Determine active parameters depending on joint type
    double theta = param.isPrismatic ? param.theta : param.theta + joint_input;
    double d     = param.isPrismatic ? param.d + joint_input : param.d;
    double a     = param.a;
    double alpha = param.alpha;

    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);

    // Standard DH formulation matrix
    return {
        {ct, -st * ca,  st * sa, a * ct},
        {st,  ct * ca, -ct * sa, a * st},
        {0.0,    sa,       ca,      d  },
        {0.0,   0.0,      0.0,     1.0 }
    };
}

// Forward Kinematics function (fkine)
Matrix4x4 fkine(const std::vector<DHParam>& robot_dh, const std::vector<double>& joint_states) {
    Matrix4x4 T_total = getIdentity();

    for (size_t i = 0; i < robot_dh.size(); ++i) {
        Matrix4x4 T_link = computeLinkTransform(robot_dh[i], joint_states[i]);
        T_total = multiply(T_total, T_link);
    }

    return T_total;
}

void printMatrix(const Matrix4x4& M) {
    for (const auto& row : M) {
        for (double val : row) {
            // Clean up tiny scientific notation noise close to 0
            if (std::abs(val) < 1e-9) val = 0.0; 
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << val << " ";
        }
        std::cout << "\n";
    }
}

std::array<double, 3> estimate_ee(double* joint_angles) {
    // Example: Defining a simple 3-Axis Revolute Robot (3R Planar Arm)
    // parameters: {theta, d, a, alpha, isPrismatic}
    std::vector<DHParam> robot_dh = {
        {-0.0000,    -0.0001,     1.5703,    -0.0016, false},
        {-0.0008,    -0.0027,     1.5677,     0.0018, false},
        {-0.0004,     0.2522,     1.5634,    -3.1419, false},
        { 0.0003,     0.0011,    -1.5665,    -0.0007, false},
        {-0.0000,     0.2487,     1.5641,     0.0041, false},
        { 0.0604,    -0.1167,     3.6046,     2.3425, false},
        {-0.0001,    -0.1311,     4.1266,     0.5968, false}
    };

    std::vector<double> joint_angles_vec(7);
    for (int i = 0; i < 7; i++) {
        joint_angles_vec[i] = joint_angles[i];
    }
    
    Matrix4x4 end_effector_pose = fkine(robot_dh, joint_angles_vec);
    // printMatrix(end_effector_pose);
    
    return {end_effector_pose[0][3], end_effector_pose[1][3], end_effector_pose[2][3]};

}

std::array<double, 3> estimate_ee_updated_params(double* joint_angles) {
    // Example: Defining a simple 3-Axis Revolute Robot (3R Planar Arm)
    // parameters: {theta, d, a, alpha, isPrismatic}

    std::vector<DHParam> robot_dh = {
        {-0.0000,  0.0000,  1.5708, -0.0000, false},
        { 0.0000,  0.0000, -1.5708,  3.1416, false},
        {-0.0000,  0.2490,  1.5708, -0.0000, false},
        {-0.0000, -0.0000, -1.5708, -0.0000, false},
        { 0.0000,  0.2520,  1.5708,  0.0000, false},
        { 0.0840, -0.0000,  0.0000,  1.5708, false}
    };

    std::vector<double> joint_angles_vec(6);
    for (int i = 0; i < 6; i++) {
        joint_angles_vec[i] = joint_angles[i];
    }
    
    Matrix4x4 end_effector_pose = fkine(robot_dh, joint_angles_vec);
    // printMatrix(end_effector_pose);
    
    return {end_effector_pose[0][3], end_effector_pose[1][3], end_effector_pose[2][3]};

}