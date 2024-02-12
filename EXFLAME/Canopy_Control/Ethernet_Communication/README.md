# Ethernet Communication 
Author: Hayden Moxsom   
Date: 12th February 2024

### Description 
This is intended to used on the canopy, to control the shaking. The control box code is for uploading onto the Mduino within the control box, and the GUI code is for running on a computer to communicate with the Mduino.  


# Setup  
To ensure it is possible to connect to the Mduino, connect both the control box and the computer to an unmanaged switch. Enter the network settings in the computer and configure the switch connection to set a static IP address, as per the images below. The identity and security are not important, so long as the MAC address of the computer is not `DE:AD:BE:EF:FE:ED`. If it is, change it to any unique identifier of the form `XX:XX:XX:XX:XX:XX`, where `X` is replaced with any of `0-9` and `A-F`.

The IPv4 must be set to manual, as the unmanaged switch can not automatically assign it. Set the address to be of the form `192.168.1.XXX`, where `XXX` is any number from `2-255` apart from `102`. Set the netmask to be `255.255.255.0`, the gateway as `192.168.1.1`, and the DNS as `192.168.1.1`.   
![Set IPv4 method to manual, with the IP: 192.168.1.52, the netmask: 255.255.255.0, the gateway: 192.168.1.1, and the DNS: 192.168.1.1](ipv4_settings.png)  

Set the IPv6 to disabled, as it is not needed for connecting to the Mduino, and can not be automatically assigned by an unmanaged switch. 
![Disable IPv6 for connection settings](ipv6_settings.png)  


# Ethernet Canopy Control GUI
A GUI application that is designed to communicate with an arduino/Mduino running the `Ethernet_canopy_control_box.ino` program. The program connects to the arduino and then opens a GUI that controls the motion of the canopy. The control is achieved by sending command messages to the arduino across ethernet.

### Prerequisites
list of prereqs and how to install here.


# Ethernet Canopy Control Box
This is in a working state, simply upload to the Mduino, if not already loaded onto it. This can be used in conjuction with the GUI program or simply connect through telnet.  
 
Arduino code which controls two motors based on messages sent through ethernet. This is intended to be used in conjunction with the Ethernet_canopy_control_GUI.py program, but can also be used with telnet connection. This code is intended to be uploaded onto the Mduino, inside the canopy control box. Some options can be changed before uploading, but is not necessary [(see SETUP OPTIONS on line 48)](Ethernet_canopy_control_box/Ethernet_canopy_control_box.ino).  

### Prerequisites (For uploading only)
To upload this code onto the control box ensure the board is set to 'M-Duino family' (industrial shields) and the model is set to 'M-Duino 58+'.  

See the industrial shields website for instructions on how to add this board option to the arduino IDE. (https://www.industrialshields.com/blog/arduino-industrial-1/how-to-install-industrial-shields-boards-in-the-arduino-ide-63) 

### User Instructions
This code is used to control the motors on the canopy. This is achieved by sending messages through ethernet to the arduino.
The arduino can be connected to by running the `Ethernet_canopy_control_GUI.py` program or by using telnet.

`Ethernet_canopy_control_GUI.py` program is the intended way to control the canopy motors. To use it simply run the python program. Note: more details on this are in the GUI section above.

Alternatively, it is possible to control it using telnet (Note: if not installed already use `sudo apt install telnet`) by running the terminal command:  
```bash
telnet 192.168.1.102 1050
``` 

This establishes the ethernet connection between the devices. After running the command, press enter once to connect. Note: `192.168.1.102` is the IP address and `1050` is the port, both of which are set in the Mduino's program.  

Once connected through telnet, the following commands are available to control the canopy:
 - Disconnect from the arduino --> 'dc' or	'disconnect'	
 - Stop the motors --> 'x' or 'stop'
 - Set motor 1 frequency N (Hz) --> 'm1 N' (where N is any number 0.0 <= N <= 2.0)
 - Set motor 2 frequency N (Hz) --> 'm2 N' (where N is any number 0.0 <= N <= 2.0)
 - Set both motors frequency N (Hz) --> 'm3 N' (where N is any number 0.0 <= N <= 2.0)
 - Set Phase Offset N (Degrees) --> 'po N' (where N is any number 0.0 <= N <= 360.0)
 - Toggle repeated print output --> 'tp' or 'toggle print'
 - Zero the cranks --> 'zc' or 'zero cranks'
 - Print output once --> 'p' or 'print'
 - Simulate communication from gui --> 'gui,[MOTOR 1 FREQUENCY],[MOTOR 2 FREQUENCY],[PHASE OFFSET]' 


# Webserver Canopy Control Box
This is incomplete. It is a program for uploading onto the Mduino. It was intended to allow the connection to be done through a webpage, which would mean the user does not need the GUI application or any of its prerequisites. It is currently an exmaple of how to turn the led on and off through the webpage, but has not been progressed any further due to being a low priority. Much of the code from the `Ethernet_canopy_control_box` would be usable here.
