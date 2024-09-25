Frankenstein the 3D Inkjet Bioprinter combines a modified Creality Ender 3 3D printer with an inkjet printer's print cartridges, X-axis, control board, and paper feed mechanism to enable 3D inkjet printing.

Frankenstein is the first prototype on my journey to create an open source 3D inkjet printer and software stack. I envision creating a company to sell the hardware, develop the software, and pay the bills for the open source project.
I believe the project needs to be open source in order to facilitate the biological and medical research that could be done on such a platform.

The system uses an Arduino-based control system for movement and timing, supporting color and monochrome printing modes with Y-axis control for paper feed simulation.

Required hardware includes a modified Creality Ender 3 3D printer, inkjet printer components, Arduino Uno, A4988 driver for driving the Y-axis stepper motor, servo motor, rotary encoder, and a limit switch for the Y-axis.

The software requires the Servo.h library.

The system is controlled via serial commands: 'c' for color mode, 'm' for monochrome mode, and 'x' for idle mode. It also has a push button mode toggle.

