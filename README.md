# j1850-uart
J1850PWM to CDC/UART adapter based on STM32

Simple tool for communicating with J1850 PWM bus (MSM6636/MSM6636B) using interrupts.

This bus commonly used in Nissan and Infinity cars for Xanavi/Clarion multimedia, steering wheel buttons and dash monitor control.

Used STM32F103C8T6 chip's two hardware timers:
* TIM2 PWM Capture - JNet SOF & Bits reading
* TIM3 overrun interrupt - read timeouts and w/o interrupt - generation.

DONE:
* JNET
 * Detect SOF, EOF.
 * Read from JNet (CRC Fast checked w/pre generated table).
 * Make IFR after EOD if enabled (AUTOIFR in config.h).
 * Write to JNet.
* CDC
 * Send up to 11 bytes right filled with 0x00 packets to CDC
 * Read packets from CDC (CR is terminator)

WIP: 
 * Documentation (Build, Schematic, PCB)
 * JNet from/to UART

BUILD:
 * Need STM32CUBEF1 library (tested with v1.6.0) - place Drivers and Middlewares in project dir.
 
USAGE:
 * Nissan Gloria ENY34: https://www.drive2.ru/l/463617875018515146/

WONTFIX:
 Collision detection
 
Adapter schematic prototype v0.1:
![J1850PWM Adapter](https://github.com/hunternsk/j1850-uart/blob/master/Doc/sch-1.png "J1850PWM Adapter Prototype v0.1")
