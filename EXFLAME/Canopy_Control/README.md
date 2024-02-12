# Canopy Control 
These codes are intended to be used with the canopy, to control the shaking of it. The ethernet communication is the prefered method due to the higher reliability and better implementation. However, the serail communication version has been kept as a backup, due to lots chnaging between the two versions.

# Ethernet Communication
See the readme within the ethernet_communication folder for details.

# Serial Communication
This as an obselete method which communicates to the Mduino using the serial port, which is highly discouraged. It also polls the encoders for ticks, which is an unreliable solution.  

It has been kept here as a reference, as some functionality was not ported over to the ethernet communication version. The main reasons for this is that some functionality was unclear on its purpose and/or did not seem to achieve the intedned functionality. The functionality not ported was:
 - An absolute encoder offset global variable as well as some calculations involving it. It was unclear on its exact purpose, but it was suspected to be the difference in ticks between the home position of the two motors. However, it was poorly implemented and did not serve this purpose properly. 
 - Encoder reading averaging. The implementation of this was poor and relied on polling to work. The readings in the ethernet version were accurate enough not to need this.