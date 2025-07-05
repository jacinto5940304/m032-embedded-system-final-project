# Wearable Activity Classification System using M032

This project presents a wearable embedded system built on the Nuvoton M032 microcontroller to classify human activities such as sitting, walking, running, and jumping using machine learning and dual ADXL345 accelerometers.

## System Overview

The system consists of two ADXL345 sensors mounted on each thigh to capture motion data in 3D (X, Y, Z). These signals are processed to extract features like peak acceleration, which are used to train and infer activity using a lightweight ML model.


## Hardware Setup

- MCU: Nuvoton M032KG
- Sensors: 2 × ADXL345 via SPI
- Connection: Each sensor wired to SPI interface
- Sampling Frequency: ~50 Hz per sensor


## Firmware

The firmware is written in Arduino C++ (in `CAR.ino`) and handles:

- SPI communication with both ADXL345 sensors
- Collecting 3-axis acceleration data
- Feature extraction (peak value per axis)
- Serial output for ML model integration

## Machine Learning

We use scikit-learn to train a simple classifier based on extracted features. Features include:

- `x0, y0, z0`: Peak values from the left thigh
- `x1, y1, z1`: Peak values from the right thigh

The classifier distinguishes between:

- Sitting
- Walking
- Running
- Vertical Jump
- Jumping Jacks

## Data Collection & Training

Collected datasets are processed using a Python notebook under `/model/model_training.ipynb`.

## Future Work

- On-device classification using lightweight model (e.g., Decision Tree)
- Integration with Bluetooth for real-time display
- Battery-powered wearable prototype

## data plot examples for Vertical Jump
![image](https://github.com/user-attachments/assets/407907bf-ba2f-4791-b527-fef7d3664583)
![image](https://github.com/user-attachments/assets/980501ba-19ea-4ca6-b277-e09220fc310d)

## DEMO
[Demo Video](https://www.youtube.com/watch?v=aHFhqFQXUIc&ab_channel=%E6%AD%A6%E9%99%B5%E5%A4%A7%E7%AC%A8%E9%B3%A5)

