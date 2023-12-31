# RobotManipulator Class

The `RobotManipulator` class is a C++ library designed for defining and manipulating robot manipulators or industrial robots using Denavit-Hartenberg (DH) parameters. It provides functionality for forward kinematics, inverse kinematics, and other relevant calculations for robotic systems.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
  - [Defining Robot Structure](#defining-robot-structure)
  - [Creating an Instance](#creating-an-instance)
  - [Forward Kinematics](#forward-kinematics)
  - [Inverse Kinematics](#inverse-kinematics)
  - [Structs for Storing Results](#structs-for-storing-results)
  - [Private Methods](#private-methods)
- [Dependencies](#dependencies)
- [Example Arduino Sketch](#example-arduino-sketch)
- [Contributing](#contributing)
- [License](#license)

## Installation

1. **Install the Arduino IDE:** If you haven't already, install the Arduino IDE from [arduino.cc](https://www.arduino.cc/).

2. **Install the ArduinoEigen Library:** The `RobotManipulator` class depends on the ArduinoEigen library, which provides Eigen matrix support for Arduino projects. Install it using the Library Manager in the Arduino IDE.

3. **Include the `RobotManipulator` Class Files:**
   - Download the `robot_manipulator.h` and `robot_manipulator.cpp` files from this repository.
   - Place these files in the same directory as your Arduino sketch or in a separate library folder.

4. **Configure Arduino IDE:** Make sure the Arduino IDE recognizes the `RobotManipulator` class files. This can be done by including them in your Arduino sketch or adding the library folder to the Arduino IDE's library path.

**Note:**

This library does NOT support following boards because they don't have standard libraries.
- AVR (Uno, Nano, Mega, etc.)
- MEGAAVR (Uno WiFi, Nano Every, etc.)
- SAM (Due)


## Usage

### Defining Robot Structure

Before using the `RobotManipulator` class, you need to define the structure of your robot using DH parameters. Create a struct to represent the DH parameters for each joint in your robot. For example:

```cpp
struct JointParameters {
  double a;     // Link length
  double alpha; // Link twist
  double d;     // Link offset
  double theta; // Joint angle
};
```

### Creating an Instance
Create an instance of the RobotManipulator class by providing the array of joint parameters and the number of joints. For example:

```cpp
JointParameters joints[] = {
  {1.0, 0.0, 0.0, 0.0},  // Joint 1
  {1.0, 0.0, 0.0, 0.0},  // Joint 2
  {1.0, 0.0, 0.0, 0.0}   // Joint 3
};

RobotManipulator robot(joints, 3);
```

### Forward Kinematics
Compute forward kinematics to get the end-effector position and orientation given joint angles:

```cpp
double jointAngles[] = {0.1, 0.2, 0.3};
ForwardKinematicsResult fkResult = robot.forwardKinematics(jointAngles);
```

**Parameters:**
- `jointAngles` (array of doubles): Array containing joint angles in radians.

**Returns (ForwardKinematicsResult struct):**
- `position` (Eigen::Vector3d): End-effector position (x, y, z).
- `orientation` (Eigen::Matrix3d): End-effector orientation matrix.

### Inverse Kinematics
Compute inverse kinematics to find joint angles for a desired end-effector pose:

```cpp
Eigen::Vector3d targetPosition(1.5, 0.5, 1.0);
Eigen::Matrix3d targetOrientation;  // Set your desired orientation matrix
InverseKinematicsResult ikResult = robot.inverseKinematics(targetPosition, targetOrientation);
```

**Parameters:**

- `targetPosition` (Eigen::Vector3d): Desired end-effector position.
- `targetOrientation` (Eigen::Matrix3d): Desired end-effector orientation matrix.

**Returns (InverseKinematicsResult struct):**

- `success` (bool): Indicates whether inverse kinematics converged.
- `jointAngles` (array of doubles): Array of joint angles if successful.

### Structs for Storing Results

The library provides structs to store the results of forward and inverse kinematics computations:

**ForwardKinematicsResult:**
```cpp
struct ForwardKinematicsResult {
    Eigen::Vector3d position;     // End-effector position
    Eigen::Matrix3d orientation;  // End-effector orientation
};
```
**Struct Members:**
- `position` (Eigen::Vector3d): End-effector position (x, y, z).
- `orientation` (Eigen::Matrix3d): End-effector orientation matrix.

**InverseKinematicsResult:**
```cpp
struct InverseKinematicsResult {
    double jointAngles[6];  // Joint angles (assuming a maximum of 6 joints)
    bool success;           // Indicates if the inverse kinematics computation was successful
};
```

**Struct Members:**
- `success` (bool): Boolean indicating whether inverse kinematics converged.
- `jointAngles` (array of doubles): Array of joint angles if successful.

### Private Methods

The library includes private methods for internal calculations, including `computeTransform` and `computeJacobian`. These methods are not intended for direct use but contribute to the functionality of the public methods.

## Dependencies

    ArduinoEigen library: Provides Eigen matrix support for Arduino projects.

## Example Arduino Sketch

An example Arduino sketch (example_robot_manipulator.ino) is provided to demonstrate the usage of the RobotManipulator class. Adjust the joint parameters, angles, and target position/orientation as needed for your specific robot configuration.

```cpp
#include <ArduinoEigen.h>
#include "robot_manipulator.h"

void printMatrix(const Eigen::Matrix3d& mat) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Serial.print(mat(i, j));
      Serial.print("\t");
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(9600);

  // Define joint parameters for a simple 3-DOF robot
  JointParameters joints[] = {
    {1.0, 0.0, 0.0, 0.0},  // Joint 1
    {1.0, 0.0, 0.0, 0.0},  // Joint 2
    {1.0, 0.0, 0.0, 0.0}   // Joint 3
  };

  // Create a RobotManipulator instance
  RobotManipulator robot(joints, 3);

  // Define joint angles for a specific robot configuration
  double jointAngles[] = {0.1, 0.2, 0.3};

  // Forward Kinematics
  ForwardKinematicsResult fkResult = robot.forwardKinematics(jointAngles);

  // Print Forward Kinematics results
  Serial.println("Forward Kinematics Result:");
  Serial.print("End-Effector Position: ");
  Serial.print(fkResult.position[0]);
  Serial.print(", ");
  Serial.print(fkResult.position[1]);
  Serial.print(", ");
  Serial.println(fkResult.position[2]);
  Serial.println("End-Effector Orientation:");
  printMatrix(fkResult.orientation);

  // Inverse Kinematics target position and orientation
  Eigen::Vector3d targetPosition(1.5, 0.5, 1.0);
  Eigen::Matrix3d targetOrientation;
  targetOrientation << 1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;

  // Inverse Kinematics
  InverseKinematicsResult ikResult = robot.inverseKinematics(targetPosition, targetOrientation);

  // Print Inverse Kinematics results
  Serial.println("\nInverse Kinematics Result:");
  if (ikResult.success) {
    Serial.print("Joint Angles: ");
    for (int i = 0; i < 3; ++i) {
      Serial.print(ikResult.jointAngles[i]);
      Serial.print(", ");
    }
    Serial.println();
  } else {
    Serial.println("Inverse Kinematics failed to converge.");
  }
}

void loop() {
  // Nothing to do in the loop for this example
}
```

## Contributing

Feel free to contribute to the development of this library. If you find any issues or have suggestions for improvements, please create an issue or submit a pull request.

## License

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