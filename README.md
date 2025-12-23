AGV - OMRON PROJECT (Mechatronics Eng. 4th semester FJ24)
===================================
STM32f0Discovery C code and MIT APP Inventor App files designed for the construction of a simple AGV-type differential robot with an elevated platform.
-----------------------------------
Main aspects of the code include:
- Key usage of registers, clock trees, and memory maps for STM32 pin configuration.
- Timer preescaling for precise PWM outputs, ultrasonic sensors and interrupt-based non-blocking task control.
- DMA configuration for WS2812b precise timing setup.
- Implementation of simple PID and differential-drive model equations for intiutive user control.
-----------------------------------
This repository also includes several Solidworks pieces that illustrates the core structure of the chassis utilized, though it is not the final design since this rendition features exclusively the profiles.
