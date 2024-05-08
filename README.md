Project Name: MAE 162 Robot

-> Description
This project is an Arduino-based line-following robot that can navigate paths autonomously. It uses an Elegoo ATMEGA2560 board and is equipped with infrared sensors for line tracking and a DRV8833 motor driver for precise motor control.

-> Features
Line Tracking: Uses multiple infrared sensors to detect and follow a line on the ground.
Adjustable Speed: Includes settings to adjust the speed of the robot.
Error Correction: Implements PID control to ensure smooth and accurate path following.

-> Hardware Requirements
Elegoo ATMEGA2560
DRV8833 Motor Driver
Infrared Sensors
Motors and Wheels
Power Supply (Battery)
Breadboard and Connecting Wires
Software Requirements
Arduino IDE or PlatformIO with Visual Studio Code

-> Necessary Libraries:
DRV8833.h for motor control (included in this repository)

-> Installation
- Setting Up the Hardware:
Connect the DRV8833 motor driver to the Elegoo ATMEGA2560 according to the circuit diagram (add a link to the circuit diagram here).
Attach the infrared sensors to the front of the robot base.
Connect the motors to the motor driver.
Ensure all components are securely mounted on the robot platform and connected to the power supply.

- Software Setup:
Clone this repository to your local machine using:
bash
Copy code
git clone https://github.com/ajdcquisto/MAE_162_Robot.git
Open the project with your Arduino IDE or PlatformIO.
Install any required libraries through the Library Manager in Arduino IDE or declare them in the platformio.ini for PlatformIO.

-> Usage
Ensure the robot is on a suitable track with a clearly marked line.
Power on the robot.
Upload the code from the src/main.cpp file to the Elegoo ATMEGA2560.
Adjust the PID settings and speed settings as necessary based on testing results.
Contributing

We welcome contributions to this project! If you have suggestions or improvements, please fork the repository and submit a pull request, or open an issue with the tags "enhancement".

License
This project is released under the MIT License.
