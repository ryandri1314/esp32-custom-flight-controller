# Custom UAV Flight Controller (FC) Development

## 📌 Overview
This project is part of my ongoing research into **Aerospace Control and Automation**. The goal is to develop a lightweight, high-performance Flight Controller (FC) from scratch, bridging theoretical **Classical Mechanics** with practical embedded system implementation.

The system is designed to handle real-time attitude estimation and stabilization for small-scale quadcopters.

## 🚀 Key Features
- **Sensor Fusion:** Implemented **Mahony AHRS** algorithm to fuse data from MPU6500 (Gyroscope & Accelerometer) for precise Roll, Pitch, and Yaw estimation.
- **Control System:** Custom-tuned **PID Control** loops for attitude stabilization and motor speed management.
- **Communication:** Integrated **SBUS protocol** for low-latency RC signal decoding.
- **Embedded Performance:** Optimized control loop running at **1kHz (1ms)** for real-time responsiveness.
- **Hardware Integration:** Custom PCB design featuring **MOSFET-based** motor drivers and power management.

## 🛠 Tech Stack
- **Language:** C/C++ (Object-Oriented Programming)
- **Frameworks/Tools:** ArduinoIDE.
- **Hardware:** ESP32-C3, MPU6500 IMU, RC Receiver (SBUS), Motor 8520.
- **Design:** Autodesk Fusion 360 (Frame & Mechanical joints).

## 📈 Current Status: [In-Progress / Researching]
- [x] Hardware Schematic & PCB Design.
- [x] Basic IMU Driver & Mahony Filter integration.
- [x] SBUS Signal Decoding.
- [ ] PID Tuning and Flight Testing (Currently in **Hardware-in-the-Loop** stage).
- [ ] Integration of Barometer and ToF sensors for altitude hold.

## 🤝 Acknowledgments
This project is inspired by my passion for UAV technology and developed during my preparation for the **Viettel Digital Talent 2026** program.
