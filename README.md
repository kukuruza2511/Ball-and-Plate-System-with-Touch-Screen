# Ball and Plate System (2-DOF)

This project develops a 2-DOF Ball and Plate System (BPS) using a 5-wire resistive touchscreen and an STM32 microcontroller. The project consists of the following components:

- Mechanical design of the Ball and Plate structure
- System modeling and control algorithm development in MATLAB Simulink
- Firmware development for communication between sensors and the STM32

---

## Control Simulation and Algorithm Development

To identify the most suitable control strategy for the BPS, three control methods were analyzed and compared:

- PID Control
- LQR (Linear Quadratic Regulator)
- Fuzzy Logic Control

The system modeling and simulation results can be found in the `Simulink_MATLAB` folder.

---

## Sensor and Microcontroller Integration

Custom firmware has been developed for interfacing the STM32 with:

- A 5-wire resistive touchscreen
- An MPU6500 IMU sensor

You can find the libraries in the uploaded files or download the custom-developed libraries from the following links:

- **Touchscreen driver**: 
- **MPU6500 driver**: 

---

More updates and documentation will be added as the project progresses.
