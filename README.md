# j1850-uart
J1850PWM to UART breakout based on STM32

Simple tool for communicating with J1850PWM bus (MSM6636/MSM6636B) using interrupts.

Code uses stm32f103 chip's two hardware timers:
Read based on PWM Capture of TIM2 Inputs
TIM3 overrun interrupt is for read timeouts and no interrupt mode for bus generation.

DONE:
 Read from net to circular buffer. Check target address and CRC. Make IFR.
 Generate packet and write to net from circular buffer. 
 USB_CDC from/to circular buffer.

WIP: 
 J_NET from/to hardware UART
 
WONTFIX:
 Collision detection
