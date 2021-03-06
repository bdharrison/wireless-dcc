# wireless-dcc
Wireless DCC transmitter &amp; receiver code

This code is for the TI Launchpad with MSP430G2553 with Anaren Radio
See Garden Railways magazine issues April - August 2014 for details on the hardware
This source code is intended to allow customization of the receiver and transmitter.
For those who want to download a built version of the code, 
each project contains a 'hex' text file that can be loaded into the target hardware.  

The [Receiver](https://github.com/bdharrison/wireless-dcc/wiki/Receiver) project is 
designed for connection to a DCC decoder & sound card, 
rather than directly to a motor and lights.  This is intended for wireless & battery 
conversion of an existing DCC-enabled locomotive.  

The [Transmitter](https://github.com/bdharrison/wireless-dcc/wiki/Transmitter) project 
differs from the magazine article in that up to six pushbuttons
are supported, giving access to all 13 DCC functions (F0-F12).  The extra pushbuttons
are connected to pins 8, 9 and 10 instead of the radio channel jumper described in the 
article.  (Radio channel selection is performed by modifying the constant in the code
file at location 1042.)  

The [Decoder](https://github.com/bdharrison/wireless-dcc/wiki/Decoder) project is a full wireless receiver and DCC decoder, supporting direct
connection to a DC motor and up to 7 outputs controlled by the DCC functions.  

The [WebController](https://github.com/bdharrison/wireless-dcc/wiki/WebController) project has a modified version of the Transmitter project that accepts commands over the serial port instead of using the input buttons and potentiometer. This can run on the unmodified AIR booster pack. A simple example file allows the serial port to be connected to a Photon board from Particle, enabling DCC commands to be sent via web requests. An example HTML file is provided to demonstrate the commands; note that this file must be edited to provide your device ID and access token.


See the [Wiki](https://github.com/bdharrison/wireless-dcc/wiki) for more information.  
