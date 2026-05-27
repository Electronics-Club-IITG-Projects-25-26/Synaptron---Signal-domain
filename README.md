![prodthetic hand](prosthetic_3d_printed.jpeg)
# Synaptron---Signal-domain
Synaptron (Signal Domain) ​Overview: An EMG-based gesture recognition system that bridges the gap between human muscle signals and machine interaction.   ​Key Features: Acquisition of tiny electrical signals (EMG) from muscle contractions, followed by amplification, filtering, and feature extraction to classify hand gestures in real-time.  

Tech Stack: Surface EMG sensors and microprocessors like Arduino/Esp32.

## 🔷 Project Overview

Human muscles generate small electrical signals during contraction, known as Electromyography (EMG) signals.

This project captures EMG signals from three forearm muscle groups, processes them in MATLAB, and uses the extracted features to control a 5-finger prosthetic hand.

Instead of controlling each finger independently (which is difficult with surface EMG), the system leverages **muscle synergies** to recognize intuitive gestures:

- Open Hand  
- Close Fist  
- Pinch Grip  

These gestures are mapped to coordinated finger movements of the prosthetic hand.

---

## 🔷 Hardware Components

### Sensors
- 3 × Surface EMG Sensors (Muscle Sensor v3)

### Microcontroller
- Arduino Uno / Raspberry Pi Pico / ESP32

### Actuators
- 5 × Servo Motors

### Other Components
- EMG electrodes  
- External power supply  
- 3D printed prosthetic hand structure  

---

## 🔷 Sensor Placement

Three sensors are placed on distinct forearm muscle groups:

| Sensor   | Muscle Group         | Function             |
|----------|---------------------|----------------------|
| Sensor 1 | Flexor compartment   | Close fingers (fist) |
| Sensor 2 | Extensor compartment | Open hand            |
| Sensor 3 | Thumb muscles        | Pinch grip           |

---

## 🔷 Methodology

### 1. Signal Acquisition  
EMG signals are acquired as analog voltages from muscle activity and sampled at **1000 Hz** using a microcontroller ADC. This satisfies the Nyquist requirement for EMG signals (20–450 Hz).

---

### 2. Signal Preprocessing  
Raw EMG signals are conditioned through:

- **DC Offset Removal** → Eliminates baseline drift  
- **Bandpass Filtering (20–200 Hz)** → Removes motion artifacts and high-frequency noise  
- **Notch Filtering (50 Hz, 100 Hz)** → Suppresses power-line interference  
- **Rectification** → Converts bipolar signal to unipolar  
- **RMS Envelope Extraction (~50 ms)** → Smooth representation of muscle activation  

Output: Three processed EMG envelopes corresponding to three muscle groups.

---

### 3. Normalization  
Signals are scaled to a **0–1 range** to reduce variability across users and sessions:

- 0 → Relaxed  
- 1 → Maximum contraction  

---

### 4. Feature Extraction  
Signals are segmented using sliding windows (200 ms with overlap). Features extracted:

- Mean Absolute Value (MAV)  
- Root Mean Square (RMS)  

These represent muscle activation intensity.

---

### 5. Gesture Classification  
A lightweight rule-based classifier determines the gesture based on dominant muscle activation:

- Flexor → Close Fist  
- Extensor → Open Hand  
- Thumb → Pinch Grip  

This method is computationally efficient and suitable for real-time systems.

---

### 6. Gesture-to-Motor Mapping  

Each gesture is mapped to predefined servo configurations:

| Gesture | Thumb | Index | Middle | Ring | Pinky |
|--------|------|------|--------|------|-------|
| Open   | 0    | 0    | 0      | 0    | 0     |
| Fist   | 1    | 1    | 1      | 1    | 1     |
| Pinch  | 1    | 1    | 0      | 0    | 0     |

(0 → Open, 1 → Closed)

---

### 7. Communication  
MATLAB streams motor commands to the microcontroller via serial communication for real-time control.

---

### 8. Actuation  
The microcontroller converts incoming commands into servo angles (0°–180°), actuating the prosthetic fingers to perform the desired gesture.

---

## 🔷 How to Run

1. Upload Arduino code from the `/arduino` folder  
2. Connect EMG sensors to the microcontroller  
3. Close Arduino IDE (to free the serial port)  
4. Open MATLAB  
5. Run:
   ```matlab
   final_emg_run

