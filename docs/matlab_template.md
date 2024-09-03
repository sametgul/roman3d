# Roman3D: Matlab/Simulink Template 

## Overview

In this experimental setup, the control algorithms run on Simulink. The data acquisition device communicates with MATLAB through a serial COM port. The Real-Time Desktop package for MATLAB enables real-time serial port communication. To test new control algorithms, simply replace the controller block in the provided template.

![template](/figs/template.png)

## Contents
To use or understand how this template works, review the following sections:

1. [Real-Time Desktop Kernel Setup](#real-time-desktop-kernel-setup)
2. [Simulink Configuration](#simulink-configuration) 
3. [Encoder Input Configuration](#encoder-input-configuration) 
4. [DAC Output Configuration](#dac-output-configurations)

## Real-Time Desktop Kernel Setup

Install the Real-Time Desktop Kernel on the host computer by executing the following command in the MATLAB command window:
 ```
 sldrtkernel -install
 ```
To verify the installation, use the command:
```
rtwho
```
Example output:

![rtwho command output](/figs/rtwho.png)

## Simulink configuration
After creating a Simulink project, configure it for the real-time kernel by following these steps:

1. Go to Model settings.
2. Navigate to Code Generation.
3. Set the System target file to `sldrt.tlc`.

![code generation](/figs/code_gen.png)

After this configuration, a new tab named ***Desktop Real-Time*** will appear in Simulink. Start simulations from this tab using the ***Run In Real Time*** button:

![run button](/figs/run_real.png)

## Encoder Input Configuration

To communicate with the device, add a ***Packet Input*** block from the ***Simulink Desktop Real-Time Library***:

![packet input](/figs/pac_in.png)

Install a new serial port board as shown:

![input config](/figs/in_config.png) 

![input config 2](/figs/in_config2.png) 

Select the serial port according to the device manager:

![Device manager](/figs/dev_man.png) 

Configure the device to send encoder input in 16-bit integer format at ***Block output data types***:

![input config 3](/figs/in_config3.png) 

Now, convert the encoder input to radians. First, convert int16 to double using a convert block:

![input block](/figs/input.png) 

## DAC Output Configurations

To send DAC values to the device, use the ***Packet Output*** block from the ***Simulink Desktop Real-Time*** library. Calculate the real DAC value by converting +/- 10V to a 0-4096 range in MATLAB. Also, include a ***Rate Transition*** block before outputting DAC values:

![output block](/figs/out_block.png) 