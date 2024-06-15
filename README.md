# Code of the Raspberry Pi Pico W

repository of all the code destined to the Raspberry Pi Pico W. This could should work for any micro-controller (ESP32, Arduino, etc.) but performances may vary. Historically, an ESP32 NodeMCU-32S was used, but it was not well suited for the application and posed many problems. Now, a Raspberry Pi Pico W is used. The name of the repository was not changed to avoid any kind of git problems.

This repository is composed of the following folders:
  - EncoderTest: contains a very basic code to read the encoders. This allows to easily check if they work, if there is bouncing, etc.
  - EncoderVerification: upgraded version of the aforementioned code: it computes the velocity of a wheel and can receive I2C messages.
  - I2C_GRiSP: contains the basic code to try out I2C communication between a GRiSP2 and a micro-controller (or between any two devices, really, with some adaptations).
  - Old_MotorControl: contains an old version of the final micro-controller code. This one has no continuous velocity profile and can be used to do some first hand tests with commands going from 0 to 100 instantly.
  - MotorControl: contains the final version of the code, that can also be found in the appendix of the [final thesis file](https://github.com/Neackow/movement_detection/blob/main/ISENGUERRE_50041800_2024.pdf).

Regarding the other files:
  - FilterCoefficients.py: code used to find the coefficients of the first order low-pass velocity filter.
  - code i2c: text file containing lines to be typed in a GRiSP2 shell. Used to try out stuff and debug. The file was left as it could still be useful to debug.
  - serialRead.py: attempt to record the content of the serial monitor of the Arduino IDE. It barely worked and was abandoned. It could probably be improved.
