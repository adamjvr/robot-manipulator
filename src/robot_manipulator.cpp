/*
MIT License

Copyright (c) 2023 Adam Vadala-Roth

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "robot_manipulator.h"
#include <Arduino.h>
#include <Eigen.h>

// Constructor implementation
RobotManipulator::RobotManipulator(double a1, double alpha1, double d1,
                                   double a2, double alpha2, double d2,
                                   double a3, double alpha3, double d3)
    : a1_(a1), alpha1_(alpha1), d1_(d1),
      a2_(a2), alpha2_(alpha2), d2_(d2),
      a3_(a3), alpha3_(alpha3), d3_(d3) {}

// Forward kinematics implementation
void RobotManipulator::forwardKinematics(double theta1, double theta2, double theta3) {
    Eigen::Matrix4d T01, T12, T23, T03;

    // Compute transformation matrices
    computeTransform(T01, a1_, alpha1_, d1_, theta1);
    computeTransform(T12, a2_, alpha2_, d2_, theta2);
    computeTransform(T23, a3_, alpha3_, d3_, theta3);

    // Combine transformations to get end-effector pose
    T03 = T01 * T12 * T23;

    // Extract position and orientation from the resulting matrix
    Eigen::Vector3d position = T03.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = T03.block<3, 3>(0, 0);

    // Display the results
    Serial.print("Forward Kinematics - End-Effector Position: ");
    Serial.print("X: "); Serial.print(position[0]); Serial.print(", ");
    Serial.print("Y: "); Serial.print(position[1]); Serial.print(", ");
    Serial.print("Z: "); Serial.println(position[2]);
}

// Inverse kinematics implementation
bool RobotManipulator::inverseKinematics(double x, double y, double z, double& theta1, double& theta2, double& theta3) {
    // Compute joint angles for a simple 3-DOF robot
    Eigen::Matrix4d T01, T12, T23, T03;

    // Joint 1 angle (theta1)
    theta1 = atan2(y, x);

    // Joint 3 angle (theta3)
    double c3 = (x*x + y*y + (z - d1_)*(z - d1_) - a2_*a2_ - a3_*a3_) / (2*a2_*a3_);
    double s3 = sqrt(1 - c3*c3);

    // Check for the existence of a solution
    if (isnan(s3)) {
        Serial.println("Inverse Kinematics - No solution found!");
        return false;
    }

    theta3 = atan2(s3, c3);

    // Joint 2 angle (theta2)
    double beta = atan2((z - d1_), sqrt(x*x + y*y));
    double alpha = atan2((a3_*s3), (a2_ + a3_*c3));
    theta2 = M_PI_2 - alpha - beta;

    // Display the results
    Serial.print("Inverse Kinematics - Joint Angles: ");
    Serial.print("Theta1: "); Serial.print(theta1); Serial.print(", ");
    Serial.print("Theta2: "); Serial.print(theta2); Serial.print(", ");
    Serial.print("Theta3: "); Serial.println(theta3);

    return true;
}

// Compute transformation matrix implementation
void RobotManipulator::computeTransform(Eigen::Matrix4d& transform, double a, double alpha, double d, double theta) {
    double cTheta = cos(theta);
    double sTheta = sin(theta);
    double cAlpha = cos(alpha);
    double sAlpha = sin(alpha);

    transform << cTheta, -sTheta * cAlpha, sTheta * sAlpha, a * cTheta,
                 sTheta, cTheta * cAlpha, -cTheta * sAlpha, a * sTheta,
                 0, sAlpha, cAlpha, d,
                 0, 0, 0, 1;
}
