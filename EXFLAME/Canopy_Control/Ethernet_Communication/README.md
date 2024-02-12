# Ethernet Communication 
Author: Hayden Moxsom   
Date: 12th February 2024

### Description 
This is intended to used on the canopy, to control the shaking. The control box code is for uploading onto the Mduino within the control box, and the GUI code is for running on a computer to communicate with the Mduino.  


# Ethernet Canopy Control Box
This is in a working state, simply upload to the Mduino, if not already loaded onto it. This can be used in conjuction with the GUI program or simply connect through telnet.  
Improve this description  
Arduino code which controls two motors based on messages sent through ethernet. This is intended to be used in conjunction with the Ethernet_canopy_control_GUI.py program, but can also be used with telnet connection. See USER INTRUCTIONS (line 11). This code is intended to be uploaded onto the Mduino, inside the canopy control box. Some options can be adjusted before uploading (see SETUP OPTIONS on line 39).
 

### Use
copy description from code file
/*///////////// USER INSTRUCTIONS //////////////////////////////////////
 * This code is used to control the motors on the canopy. This is achieved by sending messages through ethernet to the arduino.
 * The arduino can be connected to by running the "Ethernet_canopy_control_GUI.py" program or by using telnet.
 * 
 * "Ethernet_canopy_control_GUI.py" program is the intended way to control the canopy motors. To use it simply run the python program.
 *
 * Alternatively, it is possible to control it using telnet by running the command "telnet 192.168.1.102 1050". This establishes the ethernet connection between the devices.
 * After running the command "telnet 192.168.1.102 1050", press enter once to connect. Note: 192.168.1.102 is the IP address and 1050 is the port, both of which are set in this program.
 * Once connected, the following commands are available to control the canopy:
 * Disconnect 							'dc' 	or 	'disconnect'	
 * Stop motors							'x'		or 	'stop'
 * Set motor 1 frequency N (Hz)			'm1 N'	(where N is any number 0.0 <= N <= 2.0)
 * Set motor 2 frequency N (Hz)			'm2 N' 	(where N is any number 0.0 <= N <= 2.0)
 * Set both motors frequency N (Hz) 	'm3 N' 	(where N is any number 0.0 <= N <= 2.0)
 * Set Phase Offset N (Degrees)			'po N' 	(where N is any number 0.0 <= N <= 360.0)
 * Toggle print							'tp'	or 	'toggle print'
 * Zero cranks							'zc'	or 	'zero cranks'
 * Print once 							'p' 	or 	'print'
 * Simulate communication from gui 		'gui,[MOTOR 1 FREQUENCY],[MOTOR 2 FREQUENCY],[PHASE OFFSET]' 
 * 
 * Note on uploading:
 * To upload this code onto the control box ensure the board is set to 'M-Duino family' (industrial shields) and the model is set to 'M-Duino 58+'.
 * See the industrial shields website for instructions on how to add this option to the arduino IDE. (https://www.industrialshields.com/blog/arduino-industrial-1/how-to-install-industrial-shields-boards-in-the-arduino-ide-63) 
 * 
 * Some options can be changed before uploading if desired. See the SETUP OPTIONS section below.
/*


# Ethernet Canopy Control GUI
useful description here.

### Prerequisites
list of prereqs and how to install here.


# Webserver Canopy Control Box
This is incomplete. It is a program for uploading onto the Mduino. It was intended to allow the connection to be done through a webpage, which would mean the user does not need the GUI application or any of its prerequisites. It is currently an exmaple of how to turn the led on and off through the webpage, but has not been progressed any further due to being a low priority. Much of the code from the `Ethernet_canopy_control_box` would be usable here.
