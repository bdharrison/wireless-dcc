# wireless-dcc
Wireless DCC transmitter &amp; reciever code

This code is for the TI Launchpad with MSP430G2553 with Anaren Radio
See Garden Railways magazine issues April - August 2014 for details on the hardware
This source code is intended to allow customization of the receiver and transmitter.

The reciever code is designed for connection to a DCC decoder & sound card, 
rather than directly to a motor and lights.  This is intended for wirelss & battery 
conversion of an existing DCC-enabled locomotive.  

The transmitter code differs from the magazine article in that up to six pushbuttons
are supported, giving access to all 13 DCC functions (F0-F12).  The extra pushbuttons
are connected to pins 8, 9 and 10 instead of the radio channel jumper described in the 
article.  (Radio channel selection is performed by modifying the constant in the code
file at location 1042.)  
