# rp16a_psoc4
Holds the PSoC Creator Application for the RP16A Shield card

This project was created with PSoC4 Creator version 4.4.

The code receives the I2C commands from the Red Pitaya (custom modified 
code for Red Pitaya!) and translates it into SPI commands compatible with the
ALEX board set. The PSoC code ALSO controls two Antenna signals to an 
external 3 way T/R relay.

Three signals - Ant1, Ant2, and Ant3.
