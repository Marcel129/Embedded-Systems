# About
The program was developed as part of an embedded systems laboratory course to control a line-following robot. The goal of the course was to become familiar with programming STM32 microcontrollers, particularly in managing peripherals and external sensors/drivers.
Event triggering is based on the microprocessor's runtime measurement, ensuring that the software is non-blocking.

The firmware allows for:

- controlling the robot's wheels using PWM signals
- controlling an RGB LED
- reading temperature and humidity from a dedicated sensor
- measuring distance using an ultrasonic rangefinder
