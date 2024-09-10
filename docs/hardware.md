# Roman3D: Hardware Overview

## Overview

The hardware consists of a development board (STM32F103 Bluepill), two DAC chips (MCP4921 and MCP4922), and two OpAmp chips (LM358). The microcontroller unit (MCU) communicates with the Control Unit (host computer) using the USB port, presenting itself as a virtual COM port device. This configuration allows the host computer to recognize the DAQ device as a COM port, simplifying its use in MATLAB, as it eliminates the need for special communication blocks or drivers.

![template](/figs/schematics.png)

The STM32F103 "Bluepill" board serves as the main controller, interfacing with the host computer via the USB port. It uses its SPI2 bus to communicate with the MCP4922 12-bit, 2-channel DAC, and the MCP4921 12-bit, 1-channel DAC. The TIMER peripherals of the board capture encoder inputs, enabling precise measurement and control tasks.

### 1.2 How Op-Amps Work in This Project

The LM358 operational amplifier (op-amp) is used to convert the 0-5V output range of the DACs to a -10V to +10V range. The op-amp circuit achieves this by performing a linear transformation on the DAC output.

#### Op-Amp Configuration

- **Non-Inverting Input (+):** A constant 2V reference voltage is applied to the non-inverting input of the op-amp. 
- **Inverting Input (-):** The inverting input is connected to the DAC output through a resistor $R_i = 10k\Omega$.
- **Feedback Resistor (Rf):** A feedback resistor $R_f = 40k\Omega$ connects the op-amp output to the inverting input.

The output voltage $V_{\text{out}}$ of the op-amp is determined by the following equation:

$$
V_{\text{out}} = V_+ \left( 1 + \frac{R_f}{R_i} \right) - V_{\text{in}} \left( \frac{R_f}{R_i} \right)
$$

where:
- $V_+ = 2V$ (constant voltage at the non-inverting input),
- $V_{\text{in}}$ is the DAC output voltage (ranging from 0 to 5V),
- $R_f = 40k\Omega$,
- $R_i = 10k\Omega$.

#### Explanation of Op-Amp Behavior

Substituting the values into the equation:

$$
V_{\text{out}} = 2 \left( 1 + \frac{40k}{10k} \right) - V_{\text{in}} \left( \frac{40k}{10k} \right)
$$

$$
V_{\text{out}} = 2 \times 5 - 4V_{\text{in}}
$$

$$
V_{\text{out}} = 10 - 4V_{\text{in}}
$$

This equation shows that the op-amp performs a linear transformation on the DAC output, mapping the 0-5V input range to a corresponding +10V to -10V output range:

- **When the DAC outputs 0V:**  
  $$
  V_{\text{out}} = 10 - 4 \times 0 = 10V
  $$
  The op-amp outputs +10V.

- **When the DAC outputs 2.5V:**  
  $$
  V_{\text{out}} = 10 - 4 \times 2.5 = 10 - 10 = 0V
  $$
  The op-amp outputs 0V.

- **When the DAC outputs 5V:**  
  $$
  V_{\text{out}} = 10 - 4 \times 5 = 10 - 20 = -10V
  $$
  The op-amp outputs -10V.

#### How the Op-Amp Achieves This Conversion

1. **Scaling and Inversion:** The ratio of $R_f$ to $R_i$ determines the gain of the circuit. Here, the gain is -4 (negative due to the inverting configuration). The input voltage $V_{\text{in}}$ from the DAC is scaled by this factor, inverting the signal and amplifying it by four times.
   
2. **Offset Adjustment:** The constant 2V applied to the non-inverting input introduces an offset. After amplification by a factor of 5, this offset becomes 10V at the output, establishing the upper limit of the output voltage when the DAC outputs 0V.

3. **Linear Mapping:** The combination of gain and offset creates a linear transformation that maps the DAC's unipolar 0-5V range to the bipolar -10V to +10V range needed for the application. This approach is useful for applications requiring both positive and negative voltage levels, such as driving certain actuators or interfacing with analog devices that need bipolar signals.

We will add CAD drawings here as well.