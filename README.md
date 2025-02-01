
# AL5B Robotic Arm Project

This project involves controlling an **AL5B 5 DOF robotic arm** using **MATLAB** and the **SSC-32 servo controller**. The purpose of the project is to study the arm's joint movements and implement various tasks like moving to absolute and relative positions, as well as pseudo pick-and-place operations.

---

## **Files Overview**

### **1. Lab4 MATLAB Code Files** *(MATLAB code)*
- **Purpose:** 
  - Provides MATLAB scripts for controlling the AL5B robotic arm.
  - Implements tasks such as absolute and relative joint movement, frame transformations, and pseudo pick-and-place.

- **Key Tasks:** 
  - Move Absolute: Moves the arm to a defined position using joint angles.
  - Move Relative: Moves the arm to a position relative to its current position.
  - Pseudo Pick and Place: Simulates picking up and placing an object from one position to another.

---

## **Project Details**

### **Robot Information**
- **Robot:** AL5B robotic arm with 5 degrees of freedom (DOF).
- **Components:** 
  - Base, shoulder, elbow, wrist, wrist twist, and gripper joints.
  - Powered by 7 servo motors controlled by PWM signals.
- **Controller:** SSC-32 servo controller for precise and smooth servo transitions.

---

### **Control Mechanism**
- The robotic arm accepts **PWM** values for servo motor control.
- MATLAB functions such as `deg2val()` and `robot.moveRelative()` are used to translate joint angles to PWM values.
- The **SSC-32 controller** ensures precise movement with options for speed and timed transitions.

---

### **Lab Tasks Performed**
1. **Measurements:**  
   - Calculated lengths and angles for each joint.
   
2. **Repeatability:**  
   - Programmed the robot to repeatedly mark the same position on paper to test repeatability.

3. **Move Absolute:**  
   - Moved the robot from the home position to a defined position using absolute joint angles.

4. **Move Relative:**  
   - Moved the robot relative to its current position to test smooth joint transitions.

5. **Pseudo Pick and Place:**  
   - Simulated moving an object (e.g., an eraser) from one location to another.

---

## **How to Use**
1. Connect the **SSC-32 controller** to the AL5B robotic arm.
2. Load the MATLAB scripts and functions.
3. Execute the tasks (e.g., move absolute, move relative) to control the robotic arm.
4. Adjust parameters like joint angles and PWM values to achieve the desired movement.

---

## **Sample Code Snippet**
Here's a sample MATLAB function to convert joint angles to PWM values:

```matlab
function pwmValue = deg2val(degree, joint)
    % Define PWM limits for each joint
    pwmLimits = [500, 2400]; % Example limits
    maxAngle = 90; % Maximum angle for joint movement
    
    % Calculate PWM value based on degree
    pwmValue = ((degree + maxAngle) / (2 * maxAngle)) * (pwmLimits(2) - pwmLimits(1)) + pwmLimits(1);
end
```

---

## **Advantages and Disadvantages of Robotic Arms**
| Feature             | Articulated Joint (AL5B)                | Prismatic Joint                      |
|---------------------|------------------------------------------|---------------------------------------|
| Flexibility         | High                                    | Limited                              |
| Workspace Coverage  | Large relative to robot volume           | Smaller                              |
| Kinematics          | Complex                                 | Simple                               |
| Visualization       | Difficult                               | Easy                                 |

---

## **References**
- [Lynxmotion SSC-32 Servo Controller](http://www.robotshop.com/en/lynxmotion-ssc-32-servo-controller.html)
- [Robotic Arm Wikipedia](https://en.wikipedia.org/wiki/Robotic_arm)

---
