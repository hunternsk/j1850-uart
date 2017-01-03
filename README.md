# j1850-uart
J1850PWM to CDC/UART adapter based on STM32

Simple tool for communicating with J1850 PWM bus (MSM6636/MSM6636B) using interrupts.

This bus commonly used in Nissan and Infinity cars for Xanavi/Clarion multimedia and wheel buttons.

Used STM32F103C8T6 chip's two hardware timers:
* TIM2 PWM Capture - JNet SOF & Bits reading
* TIM3 overrun interrupt - read timeouts and w/o interrupt - generation.

DONE:
* JNET
 * Detect SOF, EOF.
 * Read from JNet (CRC Fast checked).
 * Make IFR after EOD if enabled (config.h).
 * Write to JNet.
* CDC
 * Send 11 byte right filled with 0x00 packets to CDC
 * Read packets from CDC (CR is terminator)

WIP: 
 * Documentation (Build, Schematic, PCB)
 * JNet from/to UART
 
WONTFIX:
 Collision detection
 
Adapter schematic prototype v0.1:
![J1850PWM Adapter](https://github.com/hunternsk/j1850-uart/blob/master/Doc/sch-1.png "J1850PWM Adapter Prototype v0.1")
