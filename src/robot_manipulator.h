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

#ifndef ROBOT_MANIPULATOR_H
#define ROBOT_MANIPULATOR_H

#include <ArduinoEigen.h>

class RobotManipulator {
public:
    // Constructor
    RobotManipulator(double a1, double alpha1, double d1,
                     double a2, double alpha2, double d2,
                     double a3, double alpha3, double d3);

    // Forward kinematics
    void forwardKinematics(double theta1, double theta2, double theta3);

    // Inverse kinematics
    bool inverseKinematics(double x, double y, double z, double& theta1, double& theta2, double& theta3);

private:
    // Denavit-Hartenberg parameters
    double a1_, alpha1_, d1_;
    double a2_, alpha2_, d2_;
    double a3_, alpha3_, d3_;

    // Function to compute a transformation matrix for a given set of DH parameters
    void computeTransform(Eigen::Matrix4d& transform, double a, double alpha, double d, double theta);
};

#endif // ROBOT_MANIPULATOR_H
