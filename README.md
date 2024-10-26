#  TRAJECTORY

                                            +---------------+
                                            |   TRAJECTORY  |
                                            +---------------+
## Overview

**trajectory** A large number of points that make a chain of the path together, which is called a trajectory.In other words, a trajectory is made by connecting many discrete points together.   

---

## Features

- ⚡ **Interpolation mode**: It is better to use this movement mode to command the robot to make a continuous path.
- 🕹️ **Custom control**: With this code, the robot can be directed to any desired path.
- 🔍 **Matrix form**: The accepted form of micro is given in datatest.If your trajectory is not in this form, you can use this code(https://drive.google.com/file/d/1L1dOT8l50HNOU7fc3vAxus3UddGh42wJ/view?usp=drive_link)
- 📌 you don't need *inverse function*.
---

## Getting Started

To set up the project locally, follow these steps.

### Prerequisites
- **STM32CubeIDE**: Ensure you have [STM32CubeIDE version 1.13.0(recommended)]
- **CANopen stack**: you need this stack for your communication 
i have used this stack in the code, you can also download and including it from [github website](https://github.com/CANopenNode/CANopenNode)

### Run code
1. **open new project**: 
    go to file --> new --> STM32 project
2. **add the stack**: 
    Go to project --> properties --> paths and symbols --> includs --> add --> file systrm
    Then choose CANopenNode and core/CANopenNode_STM32
    Ensure there is CANopenNode in the source location(if this file does not exist in the source location, first apply and close and build the project, then come to the source location again.)
3. **datatest.h**: you can change the trajectory in datatest.h as you want.The trajectory is made in MATLAB. My motors have been 6 and that's why I have 6 columns.
4. **run**:
    now, you can use this mode by upload the code to micro.

5. **This code can be written with interrupt timer in a more optimal way**

---

## License
- you can download
 *CANopenNode* document from this link(https://drive.google.com/file/d/1Lg_B5r14P_CGMcFEsO6PIyp6nOz_elqh/view?usp=drive_link)

---

## Contact
- **Email**: bestbs.1382@gmail.com
- **Github**: Amir-SB82