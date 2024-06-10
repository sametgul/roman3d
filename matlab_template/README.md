# Roman3D: Matlab/Simulink Template 

## Overview

In this experimental setup, the conrol algorithms run on Simulink. The data acquisation device can communicate with MATLAB through serial com port. Real-Time Desktop package for MATLAB implements serial port communication in a real-time manner. 

## Contents

- `CAD/`: Contains the CAD model files for the robot.
- `EmbeddedCode/`: Contains the embedded code for the data acquisition card.
- `CircuitDesign/`: Contains the circuit design files for the data acquisition card.
- `MATLABProject/`: Contains the MATLAB project template for running the experiment.

## Real-Time Desktop Kernel Setup

The Real-Time Desktop Kernel can be installed to the host computer via issuing this MATLAB command at the MATLAB command window: `sldrtkernel -install` 

Whether the kernel installed correctly or not can be checked via this command: `rtwho`

The output of the command is shown below:

![rtwho](/figs/rtwho.png)

## Simulink configuration
After creating a Simulink project the steps shown below should be done to simulate the real-time kernel.
Model settings opened: