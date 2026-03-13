# Self-Stabilising Drone

This project implements a **custom quadcopter flight controller** running inside the **Gazebo simulator**.
The goal is to build and understand a **self-stabilising drone from scratch**, including the complete control pipeline:

* IMU processing
* Sensor filtering
* PID control
* Motor mixing
* Gazebo actuator interface

---

## Features

* Custom **PID rate controller**
* **1 kHz control loop**
* **D-term filtering**
* **Dynamic D-min**
* **I-term relax**
* **Throttle PID attenuation (TPA)**
* **Feedforward control**
* **Motor mixer with desaturation**
* **Gyro filtering (Low-pass + Notch)**
* **Joystick control input**

The controller subscribes to the Gazebo **IMU topic** and publishes **motor velocities** to the actuator interface.

---

## Motor Layout

```
       Front

    M3      M1

    M2      M4

       Rear
```

---

## Dependencies

* Gazebo / Ignition Gazebo
* C++
* Gazebo Transport
* Linux Input API (joystick)

---

## Running

From the **build directory**:

```bash
export GZ_SIM_RESOURCE_PATH=../models
export GZ_SIM_SYSTEM_PLUGIN_PATH=./

make -j
gz sim ../world.sdf
```

---

## Purpose

This project is intended for **learning and experimentation with flight control systems**, including:

* Multirotor control theory
* PID tuning
* Real-time control loops
* Simulation-based testing

---

## Future Improvements

Currently the firmware implements **Acro (rate) mode only**.

Planned additions:

* **Angle mode (self-leveling)**
* Improved flight dynamics and controller tuning
* Additional flight modes and navigation features

---
