# VCU-IO

# Analog Inputs

There are 2 analog inputs on the VCU-IO PCB. These are currently soldered with a resistor divider consisting of 10kΩ and 18kΩ resistors. This translates the input:
$
V_{out} = V_{in} \frac{18}{10 + 18}
$

For a 5V input this results in a 3.21V output voltage that can be read by the ADC on the MCU. The MCU is 3.3V and therefore cannot read any voltages higher than this. A 3.3V TVS diode is included on the MCU pin to protect it from ESD. To measure higher voltage levels the ratio of the resistive divider network must be changed.

**Note:** Applying voltages higher than 3.3V to the MCU inputs will damage it.

| PCB Label<br>Analog-In[i] | Designator Class e.g. R11[A] | MCU Label | MCU Pin |
| ------------------------- | --------------------------- | --------- | ------- |
| 9                         | A                           | IN10      | 14      |
| 10                        | B                           | IN9       | 15      |

# Digital Inputs

There are 8 digital inputs on the VCU-IO PCB. These are configurable on the PCB using a jumper to accept a 5V or 24V logic level. The jumper used determines the resistor ratio used for logic translation from the PCB input to the 3.3V logic MCU.


| PCB Label<br>Digital-In[i] | Designator Class e.g. R3[A] | MCU Label | MCU Pin |
| -------------------------- | ---------------------------- | --------- | ------- |
| 1                          | H                            | IN1       | 11      |
| 2                          | G                            | IN2       | 12      |
| 3                          | F                            | IN3       | 21      |
| 4                          | E                            | IN4       | 20      |
| 5                          | A                            | IN5       | 19      |
| 6                          | B                            | IN6       | 18      |
| 7                          | C                            | IN7       | 17      |
| 8                          | D                            | IN8       | 16      |

# Digital Outputs 5V

There are 8 5V-logic level digital outputs on the VCU-IO PCB. These use individual (per output) discrete level shifter IC's to translate the 3.3V MCU output to 5V outputs. The MCU outputs should be initialised and set low to prevent any issues arising from floating inputs.

| PCB Label<br>5V-DIGO[i] | Designator Class e.g. IC2[A] | MCU Label | MCU Pin |
| --------------------- | ----------------------------- | --------- | ------- |
| 1                     | E                             | OUT1      | 0       |
| 2                     | F                             | OUT2      | 1       |
| 3                     | G                             | OUT3      | 2       |
| 4                     | H                             | OUT4      | 3       |
| 5                     | D                             | OUT5      | 4       |
| 6                     | C                             | OUT6      | 5       |
| 7                     | B                             | OUT7      | 7       |
| 8                     | A                             | OUT8      | 8       |

# Digital Outputs 24V

There are 2 24V-logic level digital outputs on the VCU-IO PCB. These are implemented using an opto-coupler with strong passive pull-down. These do not have very strong drive strength and likely require additional external buffer circuity for driving e.g. relays... The optocoupler requires the LED to be powered for a logic output to be observed. Not-initialising the MCU outputs will result in in-active outputs and should not results in any issues. For best results initialise the MCU outputs and set low.

| PCB Label<br>24-DIGO[i] | MCU Label | MCU Pin |
| --------------------- | --------- | ------- |
| 9                     | OUT9      | 9       |
| 10                    | OUT10     | 10      |

