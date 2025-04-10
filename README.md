# Robotic Gadget

A basic robotic arm kinematics library for ESP-IDF platform, providing fundamental solutions for robotic arm modeling and control.

## Features

- **DH Modeling**: Basic implementation of Denavit-Hartenberg (DH) parameters
- **Forward Kinematics**: Simple forward kinematics calculation
- **Inverse Kinematics**: Basic inverse kinematics solver
- **ESP-IDF Integration**: Works with ESP-IDF framework

## Add component to your project

Please use the component manager command `add-dependency` to add the `fox_robotic_gadget` to your project's dependency, during the `CMake` step the component will be downloaded automatically.

```
idf.py add-dependency "yanke01/fox_robotic_gadget=*"
```

## Usage

```cpp
#include "robotbox.h"

Eigen::MatrixXd RODH(6, 4);

//DH d,a,alpha,offset,theta
RODH <<  131.22, 0.0, (PI / 2), 0.0,
        0.0, -110.4, 0.0, (-PI / 2),
        0.0, -96, 0.0, 0.0,
        63.4, 0.0, (PI / 2), (-PI / 2),
        75.05, 0.0, (-PI / 2), (PI / 2),
        45.6, 0.0, 0.0, 0.0;
std::cout << "set DH d,a,alpha,offset:\n" << RODH << std::endl;

robotbox robot(RODH);
Eigen::MatrixXd forwardf;
std::vector<double> theta = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
robot.fkine(forwardf, theta);
std::cout << "forward kinematics:\n" << forwardf << std::endl;

theta = {0, -PI / 3, 0, -PI / 6, 0, 0};
robot.fkine(forwardf, theta);
std::cout << "forward kinematics:\n" << forwardf << std::endl;

std::vector<double> theikine;
bool result = robot.ikine(theikine, forwardf, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, 500, 100, 1);

if (result) {
    std::cout << "The inverse kinematics: ";
    for (size_t i = 0; i < theikine.size(); i++) {
        std::cout << theikine[i] << " ";
    }
    std::cout << std::endl;
}
```
