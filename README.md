# MaxArm Color Sorting System

## Overview
The MaxArm Color Sorting System is an automated robotic arm designed to identify, pick, and place objects based on color detection. Utilizing the Hiwonder MaxArm and a WonderCam for color recognition, this system precisely manipulates objects in industrial or educational settings, making it ideal for demonstrations or practical applications requiring sorting by color.

## Features
- **Color Detection**: Utilizes WonderCam to detect and differentiate between multiple colors.
- **PID Control**: Implements PID control for precise movement along the X and Y axes.
- **Automatic Handling**: Picks up and places objects at designated locations based on their color.
- **Alert System**: Features a buzzer for auditory signals during operations.

## Hardware Requirements
- Hiwonder MaxArm
- WonderCam
- ESP32
- Suction nozzle setup
- Buzzer

## Software Dependencies
- Arduino IDE for programming ESP32
- Libraries:
  - `PID.h` for PID control
  - `ESPMax.h` specific functionalities for Hiwonder MaxArm
  - `Buzzer.h` for controlling the buzzer
  - `WonderCam.h` for camera functionalities
  - `SuctionNozzle.h` for controlling the suction nozzle
  - `ESP32PWMServo.h` for PWM servo control

## Installation
1. **Set up the Arduino IDE**:
   - Download and install the Arduino IDE from [Arduino Website](https://www.arduino.cc/en/software).
   - Configure the IDE to support the ESP32 module.

2. **Install Required Libraries**:
   - Install the libraries mentioned in the Software Dependencies section. These can be added through the Arduino Library Manager or manually installed into your Arduino libraries folder.

3. **Hardware Assembly**:
   - Assemble the Hiwonder MaxArm with the WonderCam and all other necessary hardware components as per the product manuals.

4. **Upload the Code**:
   - Open the provided Arduino sketch file.
   - Connect your ESP32 to your computer via USB.
   - Select the correct board and port in the Arduino IDE.
   - Upload the sketch to the ESP32.

## Usage
Once the setup is complete and the sketch is uploaded:
1. Power on the Hiwonder MaxArm.
2. Place colored objects within the operational area of the robotic arm.
3. The system will automatically detect colors, pick up objects, and place them in designated locations based on their colors.
