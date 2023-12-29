# robot-manipulator
Define Robot Manipulators or Industrial Robots in the form of DH Parameters for Arduino sketches 
Requires Arduino Eigen library as dependency found here:
https://github.com/hideakitai/ArduinoEigen

# RobotManipulator Class

## Overview
The `RobotManipulator` class is designed for defining robot manipulators or industrial robots using Denavit-Hartenberg (DH) parameters. It provides functionality for forward kinematics and inverse kinematics using the Jacobian Transpose method. The class is designed to be compatible with the Arduino platform.

## Table of Contents
- [RobotManipulator Class](#robotmanipulator-class)
  - [Overview](#overview)
  - [Table of Contents](#table-of-contents)
  - [Structs](#structs)
  - [Constructor](#constructor)
  - [Forward Kinematics](#forward-kinematics)
  - [Inverse Kinematics](#inverse-kinematics)
  - [DH Parameters](#dh-parameters)
  - [Private Methods](#private-methods)
  - [Installation](#installation)
  - [Example Arduino Sketch](#example-arduino-sketch)

## Structs
### JointParameters
- `double a`: Link length
- `double alpha`: Twist angle in radians
- `double d`: Offset along the z-axis
- `double theta`: Joint angle in radians

### ForwardKinematicsResult
- `Eigen::Vector3d position`: End-effector position
- `Eigen::Matrix3d orientation`: End-effector orientation

### InverseKinematicsResult
- `double jointAngles[6]`: Array of joint angles (assuming a maximum of 6 joints)
- `bool success`: Indicates if the inverse kinematics computation was successful

## Constructor
```cpp
RobotManipulator(const JointParameters joints[], int numJoints);
```
### Description:
Constructs an instance of the RobotManipulator class with the specified joint parameters for the robot.
### Parameters:
- `joints`: An array of JointParameters representing the robot's joints.
- `numJoints`: Number of joints in the robot.

## Forward Kinematics

```cpp
ForwardKinematicsResult forwardKinematics(const double jointAngles[]);
```
###  Parameters:
- `jointAngles`: Array of joint angles for the robot.
### Returns:
- `ForwardKinematicsResult`: a struct containing the end-effector position and orientation.

## Inverse Kinematics

```cpp
InverseKinematicsResult inverseKinematics(const Eigen::Vector3d& targetPosition,
                                          const Eigen::Matrix3d& targetOrientation,
                                          double tolerance = 1e-5, int maxIterations = 100);
```

### Parameters:
- `targetPosition`: Target end-effector position.
- `targetOrientation`: Target end-effector orientation.
- `tolerance`: Convergence tolerance for the inverse kinematics algorithm (default: 1e-5).
- `maxIterations`: Maximum iterations for the inverse kinematics algorithm (default: 100).
### Returns:
- `InverseKinematicsResult`: a struct containing the computed joint angles and a success flag.

## DH Parameters

The class uses the Denavit-Hartenberg (DH) convention for defining robot kinematics. The JointParameters struct contains the DH parameters for each joint.

## Private Methods

These methods are used internally for computation and are not intended for direct use.

```cpp
void computeTransform(Eigen::Matrix4d& transform, double a, double alpha, double d, double theta);
```
### Description:
Computes a transformation matrix for a given set of DH parameters.
### Parameters:
- `transform`: Reference to the matrix to store the computed transformation.
- `a`: Link length.
- `alpha`: Twist angle in radians.
- `d`: Offset along the z-axis.
- `theta`: Joint angle in radians.

```cpp
void computeJacobian(const Eigen::Vector3d& endEffectorPosition, const Eigen::Matrix3d& endEffectorOrientation, Eigen::MatrixXd& jacobian);
```

### Description:
Computes the Jacobian matrix.
### Parameters:
- `endEffectorPosition`: End-effector position.
- `endEffectorOrientation`: End-effector orientation.
- `jacobian`: Reference to the matrix to store the computed Jacobian.

```cpp
bool isConverged(const Eigen::VectorXd& error, double tolerance);
```

### Description:
Checks if the difference between two vectors is below a certain tolerance.
### Parameters:
- `error`: Vector representing the difference between two vectors.
- `tolerance`: Tolerance threshold for convergence.

## Installation

   1. Include the robot_manipulator.h and robot_manipulator.cpp files in your Arduino project.
   2. Ensure that the required libraries (Eigen and ArduinoEigen) are properly installed.


## Example Arduino Sketch

```cpp
#include "robot_manipulator.h"

// Define joint parameters for a 3-DOF robot
JointParameters robotJoints[] = {
    {1.0, 0.0, 0.0, 0.0},
    {2.0, 0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0, 0.0}
};

// Instantiate RobotManipulator for the robot
RobotManipulator robot(robotJoints, 3);

void setup() {
    Serial.begin(9600);
    delay(1000);

    // Example: Forward Kinematics
    double jointAngles[] = {0.1, 0.2, 0.3};
    ForwardKinematicsResult fkResult = robot.forwardKinematics(jointAngles);
    
    // Print results
    Serial.println("Forward Kinematics Result:");
    Serial.print("Position: ");
    Serial.print(fkResult.position.transpose());
    Serial.println();
    Serial.println("Orientation:");
    Serial.print(fkResult.orientation);
    Serial.println();
    
    // Example: Inverse Kinematics
    Eigen::Vector3d targetPosition(2.0, 1.0, 0.5);
    Eigen::Matrix3d targetOrientation = Eigen::Matrix3d::Identity();
    InverseKinematicsResult ikResult = robot.inverseKinematics(targetPosition, targetOrientation);

    // Print results
    Serial.println("Inverse Kinematics Result:");
    if (ikResult.success) {
        Serial.print("Computed Joint Angles: ");
        for (int i = 0; i < 3; ++i) {
            Serial.print(ikResult.jointAngles[i], 4);
            if (i < 2) Serial.print(", ");
        }
        Serial.println();
    } else {
        Serial.println("Inverse Kinematics did not converge.");
    }
}

void loop() {
    // Nothing to do in the loop
}

```
This example demonstrates the usage of the RobotManipulator class with a 3-DOF robot, performing both forward kinematics and inverse kinematics.


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