// USER INSTRUCTIONS
// This code is used to control the frequency and phase of each disturbance generator
// This program is intended to communicate with Ethernet_canopy_control_GUI.py but can also be run independently

// To upload this code onto the control box ensure the board is set to 'M-Duino family' (industrial shields) and the model is set to 'M-Duino 58+'
// Some options can be changed before uploading. See the SETUP OPTIONS section below.

// To control the canopy using only this program, connect to the arduino using telnet by running the command "telnet 192.168.1.102 1050".
// To connect, press enter. Once connected, the following commands can be used to control the canopy:
// Disconnect 						'dc' 	or 	'disconnect'	
// Stop motors                      'x'		or 	'stop'
// Set motor 1 frequency N (Hz)     'm1 N' (where N is any number 0.0 <= N <= 2.0)
// Set motor 2 frequency N (Hz)     'm2 N' (where N is any number 0.0 <= N <= 2.0)
// Set both motors frequency N (Hz) 'm3 N' (where N is any number 0.0 <= N <= 2.0)
// Set Phase Offset N (Degrees)     'po N' (where N is any number 0.0 <= N <= 360.0)
// Toggle print                     'tp'	or 	'toggle print'
// Zero cranks                      'zc'	or 	'zero cranks'
// Print once 						'p' 	or 	'print'
// Simulate communication from gui  'gui,[MOTOR 1 FREQUENCY],[MOTOR 2 FREQUENCY],[PHASE OFFSET]'



//////////////////// LIBRARY IMPORTS ///////////////////////////////////
#include <SPI.h>
#include <Ethernet.h>
////////////////////////////////////////////////////////////////////////



//////////////////// SETUP OPTIONS /////////////////////////////////////
/* MAC, IP, and PORT number for setting up ethernet communication */
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; 	// MAC address of arduino to use (must be unique on network)
byte ip[] = {192, 168, 1, 102};					   	// IP address that the arduino will use (format: 192.168.1.XXX)
EthernetServer server(1050);					 	// Port to create the communication server on

/* Number of pulses per revoltion to measure of the encoders */
#define PPR 	192 	// Encoder pulses per revolution (options: 48, 96, 192)

/* Motor 1 PID tuning parameters */
#define P1  	80 		// Motor 1 proportional gain
#define I1  	300		// Motor 1 intergral gain
#define D1  	5 		// Motor 1 derivative gain

/* Motor 2 PID tuning parameters */
#define P2  	80 		// Motor 2 proportional gain
#define I2  	300		// Motor 2 intergral gain
#define D2      5 		// Motor 2 derivative gain

#define PHASE_CORRECTION 	2.88 	// Factor to adjust the motor 2 frequency by, to reach target phase offset

#define PRINT_TIME			50 		// Time interval bewteen prints (milliseconds)

#define MOTOR_INPUT_CAP 	127 	// Analog input limit for the motors
////////////////////////////////////////////////////////////////////////



/////////////////// PIN DEFINITIONS ////////////////////////////////////
/* Encoder 1 pins */
#define ENC_1A 		I0_6 	// Encoder 1 channel A pin
#define ENC_1B 		I2_6 	// Encoder 1 channel B pin
#define ENC_1X 		I0_5	// Encoder 1 channel X pin (index)

/* Encoder 2 pins */
#define ENC_2A 		I1_6 	// Encoder 2 channel A pin
#define ENC_2B 		I1_5	// Encoder 2 channel B pin
#define ENC_2X 		I2_5 	// Encoder 2 channel X pin (index)

/* Motor potentiometer pins */
#define MOTOR_1 	A0_5 	// Motor 1 potentiometer pin. 5V is stopped, 0V is full power
#define MOTOR_2 	A0_6	// Motor 2 potentiometer pin. 5V is stopped, 0V is full power

/* Led and relay pins */
#define LED 		Q1_2	// LED pin
#define RELAY 		Q1_0	// Relay pin
////////////////////////////////////////////////////////////////////////



/////////////////// ENCODER INTERRUPT SERVICE ROUTINES /////////////////
/* Stores the tick and rev count of the encoders */
volatile long enc1_count = 0, enc2_count = 0; // For tracking encoder 1 and 2's pulse count (channels A and B)
volatile long enc1_revs = 0, enc2_revs = 0; // For tracking encoder 1 and 2's index channel (number of revs)

/* For handling encoder 1's A channel interrupts */
void isr_enc1A() {
	if (digitalRead(ENC_1A) != digitalRead(ENC_1B)) ++enc1_count; // Clockwise
	else --enc1_count; // Anti-clockwise
}

#if PPR == 192
/* For handling encoder 1's B channel interrupts (only needed for 192ppr) */
void isr_enc1B() {
	if (digitalRead(ENC_1B) == digitalRead(ENC_1A)) ++enc1_count; // Clockwise
	else --enc1_count; // Anti-clockwise
}
#endif

/* For handling encoder 1's index channel interrupts */
void isr_enc1X() {
	++enc1_revs; // Homing position
}

/* For handling encoder 2's A channel interrupts */
void isr_enc2A() {
	if (digitalRead(ENC_2A) != digitalRead(ENC_2B)) ++enc2_count; // Clockwise
	else --enc2_count; // Anti-clockwise
}

#if PPR == 192
/* For handling encoder 2's B channel interrupts (only needed for 192ppr) */
void isr_enc2B() {
	if (digitalRead(ENC_2B) == digitalRead(ENC_2A)) ++enc2_count; // Clockwise
	else --enc2_count; // Anti-clockwise
}
#endif

/* For handling encoder 2's index channel interrupts */
void isr_enc2X() {
	++enc2_revs; // Homing position
}
////////////////////////////////////////////////////////////////////////



////////////////////// PROTOTYPES //////////////////////////////////////
/* setup */
void setup_encoders();

/* calculate */
void calc_motor_freq(float* motor_freq, long enc_count, long* enc_prev_count, unsigned long* enc_prev_time);

/* PID */
int calculate_PID(int motor, float target_freq, float actual_freq, float* cum_error, float* prev_error, unsigned long* prev_time);

/* msg */
bool handle_msg(const EthernetClient& client, String& msg_str, bool* power_on, float* motor_1_target_freq, float* motor_2_target_freq, float* target_phase_offset, int* print_flag, bool* new_input_flag);

/* print */
void print_outputs(const EthernetClient& client, int print_flag, float m1_targ_freq, float m1_freq, long enc1_safe_count, float m2_targ_freq, float m2_adj_freq, float m2_freq, long enc2_safe_count, float targ_phase_offset);
////////////////////////////////////////////////////////////////////////



//////////////// SETUP /////////////////////////////////////////////////
void setup() {
	// Set relay pin as output and turn off
	pinMode(RELAY, OUTPUT);
	digitalWrite(RELAY, LOW);

	// Set LED pin as output and turn on
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	// Set motor potentiometer pins as outputs and turn off
	// Note: analog output is between 0V to 10V. Motors are stationary at 5V (analog write 127) and full speed at 0V (analog write 0)
	pinMode(MOTOR_1, OUTPUT);
	pinMode(MOTOR_2, OUTPUT);
	analogWrite(MOTOR_1, 127);
	analogWrite(MOTOR_2, 127);
	
	// Setup the encoder pins and attach interrupts
	setup_encoders();

	// Start ethernet and communication server
	Ethernet.begin(mac, ip);
	server.begin();
}
////////////////////////////////////////////////////////////////////////



//////////////// LOOP //////////////////////////////////////////////////
void loop() {
	// Create a client connection, if someone is trying to connect
	EthernetClient client = server.available();

	// If a client has connected
	if (client) {
		// Send a message to the client to let them know they have connected
		client.print("Connected\r\n");
		
		// Clears the input buffer
		int clr = 0;
		while (clr != -1) clr = client.read();

		// Flag to track if the relay is on or not
		bool power_on = false;

		// For tracking encoder 1 and 2's pulse count (channels A and B)
		long enc1_safe_count, enc2_safe_count;			// Encoder ticks
		long enc1_prev_count = 0, enc2_prev_count = 0;	// Encoder previous ticks

		// For tracking encoder 1 and 2's index channel (number of revs)
		long enc1_safe_revs, enc2_safe_revs; 			// Encoder revolutions
		long enc1_prev_revs = 0, enc2_prev_revs = 0;	// Encoder previous revolutions

		// String for ethernet communication
		String msg_str;

		// For storing the previous time, between loops
		unsigned long prev_1_time = 0, prev_2_time = 0;			// Previous PID calculation time
		unsigned long enc1_prev_time = 0, enc2_prev_time = 0; 	// Previous encoder reading time
		unsigned long prev_print_time = 0;						// Previous print time
      
	  	// Measured motor frequencies
		float motor_1_freq = 0, motor_2_freq = 0; 

		// Stores the motor target points, that are entered by the user
		float motor_1_target_freq = 0, motor_2_target_freq = 0, motor_2_adj_freq; 	// Target frequency
		float target_phase_offset = 0;		// Taget phase offset

		// Stores the previous and cumulative errors, between loops
		float motor_1_prev_error = 0, motor_2_prev_error = 0; 	// Previous error
		float motor_1_cum_error = 0, motor_2_cum_error = 0; 	// Cumulative error

		// Flag to indicate if theres new input values
		bool new_input_flag = false;

		// Controls the print output. 0 = no print, 1 = print once, 2 = continuously print, 3 = print for gui.
		int print_flag = 0;

		// Do the main program loop as long as the client is connected
		while (client.connected()) {
			// Safely gets the encoder tick counts (interrupts must be prevented, as it has to do a multi-byte fetch on variables changed by the isr)
			noInterrupts();
			enc1_safe_count = enc1_count;
			enc2_safe_count = enc2_count;
			interrupts();

			// Safely gets the encoder revs (seprated from the first block to allow any interrupts to quickly be handled in between)
			noInterrupts();
			enc1_safe_revs = enc1_revs;
			enc2_safe_revs = enc2_revs;
			interrupts();            

			// Calculate the motor frequencies
			calc_motor_freq(&motor_1_freq, enc1_safe_count, &enc1_prev_count, &enc1_prev_time);
			calc_motor_freq(&motor_2_freq, enc2_safe_count, &enc2_prev_count, &enc2_prev_time);

			// Non-blocking method of getting input from the client
			if (client.available()) { 
				char c = client.read(); // Reads a character from the client

				// Limit the string length to 100 
				if (msg_str.length() < 100) {
					msg_str += c; // Store the character to the string
				}

				// If a new line character was input, process the command
				if (c == '\n') {
					int exit_loop = handle_msg(client, msg_str, &power_on, &motor_1_target_freq, &motor_2_target_freq, &target_phase_offset, &print_flag, &new_input_flag);
					if (exit_loop) break; // If a disconnect command was given, immediately exit the loop
				}
			}

			if (new_input_flag) {
				noInterrupts();
				enc1_count = 0, enc2_count = 0;
				enc1_revs = 0, enc2_revs = 0;
				interrupts();

				enc1_safe_count = 0, enc2_safe_count = 0;
				enc1_prev_count = 0, enc2_prev_count = 0;
				enc1_safe_revs = 0, enc2_safe_revs = 0;
				enc1_prev_revs = 0, enc2_prev_revs = 0;

                prev_1_time = micros();
				prev_2_time = micros();

                if (motor_1_target_freq == 0) motor_1_cum_error = 0;
                if (motor_2_target_freq == 0) motor_2_cum_error = 0;

				// If both targets are 0 and the power is on, turn it off
				if ((motor_1_target_freq == 0) && (motor_2_target_freq == 0) && power_on) {
					digitalWrite(RELAY, LOW); // Turn power off to the motors
					power_on = false; 
				}
			}

			// If the power is off and the motors are trying to go, turn the power on
			if (!power_on && (motor_1_target_freq > 0) || (motor_2_target_freq > 0)) {
				digitalWrite(RELAY, HIGH); // Turn power on
				power_on = true;
			} 
			
			// If the power is on, calculate the required motor input values
			if (power_on) {
				// Determine if the motor 2 target needs to be adjusted, to align to the chosen phase
				if ((motor_1_target_freq == 0) || (motor_2_target_freq == 0) || new_input_flag) {
					motor_2_adj_freq = motor_2_target_freq; // Motor 2 target does not need adjusting
				}
				else { // Needs adjusting, apply some calculations to determine the adjusted target
					// 
					float correction_factor = (1 + (PHASE_CORRECTION * ((enc1_safe_count - ((enc2_safe_count - target_phase_offset) * (motor_1_target_freq / motor_2_target_freq))) / float(PPR))));
					
					// Limit the correction factor to be within 0.5 and 1.5
					if (correction_factor > 1.5) correction_factor = 1.5; 
					else if (correction_factor < 0.5) correction_factor = 0.5;

					// Apply the correction factor to the chosen target 
					motor_2_adj_freq = motor_2_target_freq * correction_factor;
				}

				// Apply PID to determine motor input values
				int motor_1_input = calculate_PID(1, motor_1_target_freq, motor_1_freq, &motor_1_cum_error, &motor_1_prev_error, &prev_1_time);
				int motor_2_input = calculate_PID(2, motor_2_target_freq, motor_2_freq, &motor_2_cum_error, &motor_2_prev_error, &prev_2_time);

				// Write the input to the motors
				analogWrite(MOTOR_1, motor_1_input);
				analogWrite(MOTOR_2, motor_2_input);
			}

			// Clear the input flag, now that its been dealt with
			new_input_flag = false;
			
			// Checks if a print should be done, and if so prints the corresponding output
			print_outputs(client, print_flag, motor_1_target_freq, motor_1_freq, enc1_safe_count, motor_2_target_freq, motor_2_adj_freq, motor_2_freq, enc2_safe_count, target_phase_offset);
		}

		// When client disconnects, turn off motors
		analogWrite(MOTOR_1, 127);
		analogWrite(MOTOR_2, 127);
		digitalWrite(RELAY, LOW);
		power_on = false;

		// Reset enoder counters to 0
		noInterrupts();
		enc1_count = 0, enc2_count = 0;
		enc1_revs = 0, enc2_revs = 0;
		interrupts();

		enc1_safe_count = 0, enc2_safe_count = 0;
		enc1_prev_count = 0, enc2_prev_count = 0;
		enc1_safe_revs = 0, enc2_safe_revs = 0;
		enc1_prev_revs = 0, enc2_prev_revs = 0;

		msg_str = "";
	}
}
////////////////////////////////////////////////////////////////////////



////////////////// ENCODER SETUP ///////////////////////////////////////
/* Setups the encoder pins and attaches interrupts */
void setup_encoders() { 
	// Set encoder 1 pins as inputs
	pinMode(ENC_1A, INPUT_PULLUP);
	pinMode(ENC_1B, INPUT_PULLUP);
	pinMode(ENC_1X, INPUT_PULLUP);

	// Set encoder 2 pins as inputs
	pinMode(ENC_2A, INPUT_PULLUP);
	pinMode(ENC_2B, INPUT_PULLUP);
	pinMode(ENC_1X, INPUT_PULLUP);

#if PPR == 48
	// Sets up encoder 1 to count only rising pulses on the A channel (48 ppr)
	attachInterrupt(digitalPinToInterrupt(ENC_1A), isr_enc1A, RISING);
#elif PPR == 96
	// Sets up encoder 1 to count all pulses on the A channel (96 ppr)
	attachInterrupt(digitalPinToInterrupt(ENC_1A), isr_enc1A, CHANGE);
#elif PPR == 192
	// Sets up encoder 1 to count every pulse (192 ppr)
	attachInterrupt(digitalPinToInterrupt(ENC_1A), isr_enc1A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENC_1B), isr_enc1B, CHANGE);
#endif

#if PPR == 48
	// Sets up encoder 2 to count only rising pulses on the A channel (48 ppr)
	attachInterrupt(digitalPinToInterrupt(ENC_2A), isr_enc2A, RISING);
#elif PPR == 96
	// Sets up encoder 2 to count all pulses on the A channel (96 ppr)
	attachInterrupt(digitalPinToInterrupt(ENC_2A), isr_enc2A, CHANGE);
#elif PPR == 192
	// Sets up encoder 2 to count every pulse (192 ppr)
	attachInterrupt(digitalPinToInterrupt(ENC_2A), isr_enc2A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENC_2B), isr_enc2B, CHANGE);
#endif

	// Sets up encoder 1 and 2 to count the rising pulse on the index channel (revs)
	attachInterrupt(digitalPinToInterrupt(ENC_1X), isr_enc1X, RISING);
	attachInterrupt(digitalPinToInterrupt(ENC_2X), isr_enc2X, RISING);
}
////////////////////////////////////////////////////////////////////////




void calc_motor_freq(float* motor_freq, long enc_count, long* enc_prev_count, unsigned long* enc_prev_time) {
	// If the tick count is different to the last time, calculate the motor frequency
	if (enc_count != *enc_prev_count) { 
		// Get the current time (us)
		unsigned long enc_cur_time = micros(); 
		unsigned long enc_elapsed_time = enc_cur_time - *enc_prev_time; // Calculate the elapsed time (us)
		*enc_prev_time = enc_cur_time; // Save the current time for the next loop (us)

		// If the elapsed time is 0 us, set it to 1 us, to prevent a divide by 0 error
		if (enc_elapsed_time == 0) enc_elapsed_time = 1; // micro seconds (us)

		// Calculate the motor frequency in Hz
		*motor_freq = float(enc_count - *enc_prev_count) / float(PPR) / float(enc_elapsed_time) * 1000000.0; // Hz
		*enc_prev_count = enc_count; // Save the current encoder count for next loop
	}
	// Else if no ticks have been detected for a while, the frequency must be 0
	else if ((micros() - *enc_prev_time) > 500000) {
		*motor_freq = 0; // Motor frequency must be 0
	}
}

int calculate_PID(int motor, float target_freq, float actual_freq, float* cum_error, float* prev_error, unsigned long* prev_time) {
	// Get change in time (elapsed_time)
	unsigned long cur_time = micros();
	unsigned long elapsed_time = cur_time - *prev_time;
	if (elapsed_time == 0) elapsed_time = 1;
	*prev_time = cur_time;
	double elapsed_secs = float(elapsed_time) / 1000000.0;

	// Get Errors for PID
	float error = target_freq - actual_freq;
	*cum_error += error * elapsed_secs;
	float grad_error = (error - *prev_error) / float(elapsed_secs);

	// Calculate PID components
	float motor_p, motor_i, motor_d;
	if (motor == 1) {
		motor_p = error * P1;
		motor_i = *cum_error * I1;
		motor_d = grad_error * D1;
	}
	else if (motor == 2) {
		motor_p = error * P2;
		motor_i = *cum_error * I2;
		motor_d = grad_error * D2;
	}	

	// Calculate motor input
	int motor_speed = motor_p + motor_i + motor_d;
	*prev_error = error;

	// Apply constraints to motor 1
	if (motor_speed < 0) motor_speed = 0;
	else if (motor_speed > MOTOR_INPUT_CAP) motor_speed = MOTOR_INPUT_CAP;

	// Update motor power
	int motor_input = 127 - motor_speed;
	if (target_freq == 0) motor_input = 127;

	return motor_input;
}

bool handle_msg(const EthernetClient& client, String& msg_str, bool* power_on, float* motor_1_target_freq, float* motor_2_target_freq, float* target_phase_offset, int* print_flag, bool* new_input_flag) {
	// Convert message to lower case
	for (auto &x : msg_str) { x = tolower(x); }

	// If input is "dc" or "disconnect" then disconnect the client 
	if ((msg_str.substring(0, 2) == "dc") || (msg_str.substring(0, 10) == "disconnect")) {
		client.print("Disconnecting\r\n");
		delay(1);
		client.stop(); // Disconnect
		return true; // Exit the loop immediately
	}
	// If input beings with "x" or is "stop" then stop the motors
	else if ((msg_str.substring(0, 1) == "x") || (msg_str.substring(0,4) == "stop")) {
		client.print("Stop command\r\n");

		// Turn motor power off
		analogWrite(MOTOR_1, 127);
		analogWrite(MOTOR_2, 127);
		digitalWrite(RELAY, LOW); 
		*power_on = false;
		
		// Set motor targets to 0
		*motor_1_target_freq = 0;
		*motor_2_target_freq = 0;
		*target_phase_offset = 0;
		
		// Disable prints
		*print_flag = 0;
		
		// Signal that there is new input values
		*new_input_flag = true;
	}
	// If the input starts with "gui" then the data comes from the gui program in an expected form
	else if (msg_str.substring(0, 3) == "gui") {
		client.print("GUI command\r\n");

		// Split the string into the expected values and store them
		char *token = strtok(msg_str.c_str(), ",");
		int c = 0;
		while (token != NULL) {
			if (c == 1) *motor_1_target_freq = strtod(token, NULL);
			else if (c == 2) *motor_2_target_freq = strtod(token, NULL);
			else if (c == 3) *target_phase_offset = strtod(token, NULL) * (float(PPR) / 360.0);

			token = strtok(NULL, ",");
			++c;
		}

		// Enable gui print output
		*print_flag = 3;

		// Signal that there is new input values
		*new_input_flag = true;
	}
	// If input is in the form "m1 N", set motor 1 to frequency N (Hz)
	else if (msg_str.substring(0, 3) == "m1 ") {
		client.print("Motor 1 command\r\n");
		String str_seg = msg_str.substring(3, '\n');
		if (str_seg.toFloat() >= 0) {
			*motor_1_target_freq = str_seg.toFloat();
		}

		// Signal that there is new input values
		*new_input_flag = true;
	}
	// If input is in the form "m2 N", set motor 2 to frequency N (Hz)
	else if (msg_str.substring(0, 3) == "m2 ") {
		client.print("Motor 2 command\r\n");
		String str_seg = msg_str.substring(3, '\n');
		if (str_seg.toFloat() >= 0) {
			*motor_2_target_freq = str_seg.toFloat();
		}

		// Signal that there is new input values
		*new_input_flag = true;
	}
	// If input is in the form "m3 N", set both motors to frequency N (Hz)
	else if (msg_str.substring(0, 3) == "m3 ") {
		client.print("Both motors command\r\n");
		String str_seg = msg_str.substring(3, '\n');
		if (str_seg.toFloat() >= 0) {
			*motor_1_target_freq = str_seg.toFloat();
			*motor_2_target_freq = str_seg.toFloat();
		}

		// Signal that there is new input values
		*new_input_flag = true;
	}
	// If input is in the form "po N", set target phase offset to N (Degrees)
	else if (msg_str.substring(0, 3) == "po ") {
		client.print("Phase offset command\r\n");
		String str_seg = msg_str.substring(3, '\n');
		if (str_seg.toFloat() >= 0) {
			*target_phase_offset = (str_seg.toFloat()) * (float(PPR) / 360.0);
		}

		// Signal that there is new input values
		*new_input_flag = true;
	}
	// If input is "tp" or "toggle_print" print until entered again
	else if ((msg_str.substring(0, 2) == "tp") || (msg_str.substring(0, 12) == "toggle print")) {
		client.print("Toggle print command\r\n");
		if (*print_flag != 2) *print_flag = 2;
		else *print_flag = 0;
	}
	// If input is "p" or "print" print output once
	else if (msg_str.substring(0, 1) == "p" || (msg_str.substring(0, 5) == "print")) { 
		client.print("Print command\r\n");
		*print_flag = 1;
	}
	// If input is in the form of "zc" or "zero crank" zeros the encoder readings?? (not sure why this exists, kept for the sake of it)
	else if ((msg_str.substring(0, 2) == "zc") || (msg_str.substring(0, 10) == "zero crank")) {
		client.print("Zero crank command\r\n");

		// Signal that there is new input values
		*new_input_flag = true;
	}
	// If input is none of the above, return an error message to the client
	else {
		client.print("Unrecognised command\r\n");
	}

	// Clearing string for next read
	msg_str = "";

	return false;
}

void print_outputs(const EthernetClient& client, int print_flag, float m1_targ_freq, float m1_freq, long enc1_safe_count, float m2_targ_freq, float m2_adj_freq, float m2_freq, long enc2_safe_count, float targ_phase_offset) {
	static unsigned long prev_print_time = millis();

	if (millis() - prev_print_time >= PRINT_TIME) {
		switch (print_flag) {
			case 1: // A single print output (shares its print output with case 2)
				print_flag = 0; // Set the print flag back to 0, to print once
				// Intentionally continue to case 2
			case 2: // A repeating print output (shares its print output with case 1)
				client.print("m1 targ freq: ");
				client.print(m1_targ_freq);
				client.print(" | m1 freq: ");
				client.println(m1_freq);
				client.print("m2 targ freq: ");
				client.print(m2_adj_freq);
				client.print(" | m2 freq: ");
				client.println(m2_freq);
				client.print("phase offset targ: ");
				client.print(targ_phase_offset);
				client.print(" | enc1 ticks: ");
				client.print(enc1_safe_count);
				client.print(" | enc2 ticks: ");
				client.print(enc2_safe_count);
				client.print(" | phase offset: ");
				client.print((float(enc1_safe_count) - (float(enc2_safe_count) * (m1_targ_freq / m2_targ_freq))) * (360.0 / float(PPR)));
				client.print(" | not sure: ");
				client.println(float(enc1_safe_count) / (float(enc2_safe_count) + targ_phase_offset));
				break;

			case 3: // The print output for the gui 
				client.print(m1_freq);
				client.print(", ");
				client.print(m2_freq);
				client.print(", ");
				if (m2_targ_freq > 0) {
					client.print((float(enc1_safe_count) - ((float(enc2_safe_count) - targ_phase_offset) * (m1_targ_freq / m2_targ_freq))) * (360.0 / float(PPR)));
				} else client.print('-');
				client.print(", ");
				client.print(enc1_safe_count % PPR);
				client.print(", ");
				client.print(enc2_safe_count % PPR);
				client.print(", ");
				client.print(PPR);
				client.print("\r\n");
				break;

			default:
				break;
		}
		prev_print_time = millis();
	}
}
