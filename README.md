# 27MHz-Transmitter
27.12MHz wiresless transmitter

## Basic flow
* There is a external oscillator that wakes up the MCU a few times every second (allowing MCU to sleep 99.9% of the time) if this is the Nth time the MCU has been woken up it sends a packet
* The MCU powers up the RF chain
* The MCU reads from the sensors then puts this information into a packet
* This packet has ECC added with 255, 154 BCH code. There is an additional framing section added that is not protected by error correction
* The packet is sent out by toggling a gpio pin with 20us bit period
* This waveform is pulse shapped by low pass filters before being sent to the modulator
* The modulated waveform is sent out over the airwaves
* The MCU powers off the board and goes back to sleep



