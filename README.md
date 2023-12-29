# robot-manipulator
Define Robot Manipulators or Industrial Robots in the form of DH Parameters for Arduino sketches 
Requires Arduino Eigen library as dependency found here:
https://github.com/hideakitai/ArduinoEigen

## RobotManipulator Class

The `RobotManipulator` class is a C++ class designed for defining and manipulating the kinematics of industrial robots. It includes forward kinematics and inverse kinematics methods, supporting robots with up to 6 degrees of freedom (DOF). Requires Arduino Eigen library as dependency found here: https://github.com/hideakitai/ArduinoEigen

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [API Documentation](#api-documentation)
  - [Constructor](#constructor)
  - [Forward Kinematics](#forward-kinematics)
  - [Inverse Kinematics](#inverse-kinematics)
  - [Private Methods](#private-methods)
- [Usage Example](#usage-example)
- [Contributing](#contributing)
- [License](#license)

## Introduction

The `RobotManipulator` class is designed to handle the kinematics of industrial robots, allowing users to perform forward and inverse kinematics calculations. It supports robots with up to 6 degrees of freedom.

## Installation

1. Copy the `robot_manipulator.h` and `robot_manipulator.cpp` files into your project.
2. Make sure you have the ArduinoEigen library installed (https://github.com/hideakitai/ArduinoEigen).

## API Documentation

### Constructor

```cpp
RobotManipulator(const JointParameters joints[], int numJoints);
```

The constructor initializes a RobotManipulator object with the specified joint parameters.

    joints: An array of JointParameters structs defining the joint parameters of the robot.
    numJoints: The number of joints in the robot.

## Forward Kinematics

```cpp
void forwardKinematics(const double jointAngles[]);
```

Computes and displays the end-effector position using the given joint angles.

    jointAngles: An array of joint angles in radians.

## Inverse Kinematics

```cpp
bool inverseKinematics(const Eigen::Vector3d& targetPosition, const Eigen::Matrix3d& targetOrientation,
                        double jointAngles[], double tolerance = 1e-5, int maxIterations = 100);
```

Computes the joint angles required to reach the specified end-effector pose using the Jacobian Transpose method.

    - targetPosition: The target position of the end-effector.
    - targetOrientation: The target orientation of the end-effector.
    - jointAngles: An array to store the computed joint angles.
    - tolerance: The convergence tolerance for the inverse kinematics algorithm (default is 1e-5).
    - maxIterations: The maximum number of iterations for convergence (default is 100).

## Private Methods

These methods are used internally for computation and are not intended for direct use.

```cpp
void computeTransform(Eigen::Matrix4d& transform, double a, double alpha, double d, double theta);
```
    Computes a transformation matrix for a given set of Denavit-Hartenberg (DH) parameters.

```cpp
void computeJacobian(const Eigen::Vector3d& endEffectorPosition, const Eigen::Matrix3d& endEffectorOrientation, Eigen::MatrixXd& jacobian);
```

    Computes the Jacobian matrix for a given end-effector position and orientation.

```cpp
bool isConverged(const Eigen::VectorXd& error, double tolerance);
```

    Checks if the difference between two vectors is below a specified tolerance, indicating convergence.

## Usage Example

```cpp
#include "robot_manipulator.h"

// Define joint parameters for a 3-DOF robot
JointParameters joints[] = {
    {1.0, 0.0, 0.0, 0.0},
    {2.0, 0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0, 0.0}
};

// Instantiate RobotManipulator
RobotManipulator robot(joints, 3);

// Perform forward kinematics
double jointAngles[] = {0.1, 0.2, 0.3};
robot.forwardKinematics(jointAngles);

// Perform inverse kinematics
Eigen::Vector3d targetPosition(2.0, 1.0, 0.5);
Eigen::Matrix3d targetOrientation = Eigen::Matrix3d::Identity();
robot.inverseKinematics(targetPosition, targetOrientation, jointAngles);
```

## Contributing 
Feel free to contribute by opening issues or pull requests. Your feedback and contributions are welcome!

## License
This project is licensed under the MIT License

MIT License

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