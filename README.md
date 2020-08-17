# myJoystic
## simple usb-serial joystick made with ADXL345 accelerometer and STC microcontroller

This is a course design of *Design of Intelligent Test and Control Instrument*
It simply sets the accelerometer to use IIC to communicate with the MCU, and the MCU uses USB emulated serial port to communicate with the python program running on computer whitch manages the serial port, decode its data stream and convert them into simulated keyboard inputs in "real time". So that it can be used to contol the movements of an object in a game such as *Balance*.
Unfortunately, due to the limitations of **pyautogui**,currently, most PC games cannot be controled (looks like it has something to do with DirectX) by this joystick. However, **FLASH games are OK** XD.
The complete setup needs simple mechanical structure to limit the degree of freedom of the accelerometer to only 2 spinning angle dimentions. You can make it simply with a wheel of your swivel chair. However, they can be unnecessary. But I haven't gone this way yet. It can be much more difficult.

For more details and functions, just find them in these files.

Acknowledgements for STC program examples.
