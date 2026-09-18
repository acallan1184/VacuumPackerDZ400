# VacuumPackerDZ400
This is a project that resulted from a faulty vacuum packer - a PCB that showed 8.8. on the two-digit 7-segment display.

Debug resulted in no obvious component faults. The PIC was running code that appeared to beep during boot up, and recognize each button was being pressed (by beeping when a button was pressed, and chirping when any button was held).
A logic analyzer proved that the device beeped on boot up, and during long button presses. The logic analyzer also proved that the eeprom was being accessed during boot-up at 4 addresses (0x50, 0x51, 0x52, 0x53).
It was assumed that these were for the temperature setting (configurable by user), the vacuum timer setting (configurable by user), the seal timer setting (configurable by user), and the cool timer setting (not configurable but loaded from EEPROM).
It was assumed that the EEPROM got corrupted during some kind of transient event. The readbacks were 0xFF, 0xC, 0x10, 0x19.
Eventually, a colleague held the EEPROM SDA low briefly and got back some functionality. The PIC now started as "--" on the display, reacted to button presses, and allowed configuration of the vacuum and seal time. The readbacks were 0x00, 0xB, 0x16, 0x00.
However, another value in the EEPROM (the cool time) appeared to be set to zero. I attempted to write a value to the EEPROM while on the board, but appeared to lose functionality (the lid is now always considered to always be closed, regardless of input state of pin).
The I2C was originally done in a non-conventional way, where the SCL was bit-banged (with no pull-up resistor) and the SDA was using an open-drain pin (with pull-up resistor). 
The guess is that this in some way resulted in the PIC16F72 RA3 pin getting damaged, making it impossible to keep using the PIC with the program it had. The program was code-protected, so could not be read back and reprogrammed to another chip.
Another theory is that some bit in the EEPROM was changed in this messing, that resulted in the program ignoring the input pin. However, the PIC was never identified to read this value, so I'm not sure how it could even know :)
The EEPROM was unsoldered, socketed, and a lot of trials of different values in the bytes that it does read were tried at boot-up, to no avail (no change in behavior.
The PIC was also unsoldered (poorly, admittedly, by hand pump). Some traces were pulled up as a result and had to be reworked. There is no doubt that the input to the RA3 pin worked flawlessly on the PCB.
With all hope lost, a new PIC was purchased with the idea of re-writing the code from scratch, on the understanding of what was already known about the PCB functionality.

This is that code.......

I would love to finish this project by ultimately programming the damaged PIC to prove if the pin was indeed damaged or not.....
