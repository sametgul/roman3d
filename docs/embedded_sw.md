# Roman3D: Embedded Software

## Overview

We used STM32CubeIDE to develop the embedded software. Here is Pinout View of the MCU.

![template](/figs/pinoutstm32.png)

In pinout & configuration window, make sure you chose the following settings.

System Core -> GPIO

![gpio](/figs/gpiocube.png)

System Core -> RCC

![rcc](/figs/rcccube.png)

System Core -> SYS

![sys](/figs/syscube.png)

We use Timers for Encoder reading. Timers -> TIM2, TIM3, TIM4

![tim234](/figs/tim234cube.png)

The SPI settings (Data Size and First Bit) chosen according to DAC chips datasheet. 

Connectivity -> SPI2

![spi2](/figs/spi2cube.png)

Connectivity -> USB

![USB](/figs/usbcube.png)

Middleware -> USB_DEVICE

![USB Dev](/figs/usbdevicecube.png)

Here is Clock Configurtion

![Clock Config](/figs/clockcube.png)

