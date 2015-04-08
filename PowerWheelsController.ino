
//Options
 
boolean enableTimeout = false;  //This allows overriding the automatic timeout function
unsigned int intTimeout = 30000;  //This is the length of the timeout, regardless of the method of detecting if there is a valid connection to a parent device
								  // Once that connection is considered lost, This sets how long the parent has to reconnect before little Billy is left 'dead in the water'
								  // Only has any effect if enableTimeout is set to TRUE.


//Pin definitions
int statusControlPin = 13; // name the output pin for 'Manual Control is' - Currently does double duty, it is an output pin for an LED as a visual indicator, as well as used as a status variable.
int motorpin = A0; // name the output pin for 'PowerWheels is ' - This pin is the output to a relay normally located under the throttle pedal.
int steeringPin = 3; // steering Servo signal output, Highly recommend using a metal gear, high torque servo, POWERED FROM A SEPERATE SOURCE! NOT POWERED THROUGH THE ARDUINO!
int pedal_pin = A1; // Accelerator wired to  - This pin is an input FROM the throttle pin, vcc should be connected to the pressed contact, and gnd to the released, with this pin going to the center. That will pull the pin one way rather than leave it floating.
int signal_pin = 2; //If using an LED from the radio as a connection indicator, this is the pin it is wired to.
int signal_led = 12; //visual indicator of the connection state(assumes signal_pin is defined and being used)

//static definitions for somethings that won't change during execution normally

unsigned int steeringLimits[3] = {2000,1500,800}; //This will be used as the limits for the steering servo, as well as the values for parental commands, (left, center, right)


//Status Variables - Initialize some variables so we read everything once per loop which prevents unexpected results due to mid-loop changes. Also eliminates? or significantly reduces any possible switch bouncing due to poor connections from age.

unsigned long heartbeat = millis(); //This is where we store the current time whenever we determine that the parent connection is working properly, either using a heartbeat response, or even using an onboard 'connection status' LED wired into an input pin.
unsigned long lastControlCheck = millis(); //Because we are relying on relays, we need to slow down any manual control changes, default is to check and apply them at 10hz (about every 100ms)
unsigned long autostoptime=millis(); //If you happen to be using a straight serial connection for control, this holds when the power to the wheels will be cut when it is moving due to parent commands. ie, you send the GO command, if nothing else is received it should stop moving after 30 seconds. You can always send additional GO commands, which will push the timer back out to 30 seconds from when the command was received.
boolean statusTimedOut = false; //Status variable to more easily determine if control was disabled by command, or by timeout. allows for altering what is required to re-arm the system and allow manual control
int pedal_last_state = LOW; //Status variable to allow rider to interrupt parental throttle commands(providing control is enabled) by pressing and releasing the throttle.
int value_signal = digitalRead(signal_led);
int value_control = digitalRead(statusControlPin);
boolean value_pedal = digitalRead(pedal_pin);

#include <Servo.h>
Servo steeringServo;



void setup() {
	Serial.begin(115200);
	steeringServo.attach(steeringPin);
//	steeringServo.attach(steeringPin,min(steeringLimits[0],steeringLimits[2]),max(steeringLimits[0],steeringLimits[2]));
	steeringServo.writeMicroseconds(steeringLimits[1]);
	pinMode(statusControlPin, OUTPUT); 
	pinMode(motorpin, OUTPUT); 
	pinMode(signal_led, OUTPUT);

	pinMode(pedal_pin, INPUT); // Throttle Manual control pin
	pinMode(signal_pin, INPUT); // Parental connection status pin
	digitalWrite(statusControlPin,HIGH); //By using a separate command rather than using input_pullup allows us to use this to know when the main loop has started executing.
}

void loop() {
	value_signal = digitalRead(signal_pin);
	value_control = digitalRead(statusControlPin);
	value_pedal = digitalRead(pedal_pin);
	
	if (value_signal == HIGH) {
		digitalWrite(signal_led,HIGH);
	}
	else {
	digitalWrite(signal_led,LOW);
	}
	if (statusTimedOut == false && value_control == HIGH && value_signal == HIGH && millis()-heartbeat>intTimeout/2) {
		heartbeat=millis();
		Serial.println("Connection active, Timer reset");
	}
	if (enableTimeout == true && millis()-heartbeat>intTimeout && value_control==HIGH) {
		Serial.println("ERROR - NO Heartbeat received, disabling control");
		statusTimedOut=true;
		digitalWrite(statusControlPin,LOW);
		digitalWrite(motorpin,LOW);
	}
	if (millis()>autostoptime && digitalRead(motorpin)==HIGH && value_pedal == LOW) {
		digitalWrite(motorpin,LOW);
	}
	 if(value_control == HIGH && millis()-lastControlCheck >=100) {
	  lastControlCheck=millis();
	  if(value_pedal != pedal_last_state) {
		  Serial.print("Pedal changed - ");
		  Serial.println(value_pedal);
		  digitalWrite(motorpin,value_pedal);
		  pedal_last_state = value_pedal;
		}
  }
}

void serialEvent() {
	if (Serial.available()) {
		//if(statusTimedOut==true) {
		//	statusTimedOut=false;
		//	digitalWrite(statusControlPin,HIGH);
		//	heartbeat=millis();
		//}
		
		heartbeat = millis();
		
		char inChr=(char)Serial.read();
		Serial.println(inChr);
		if (inChr=='c' or inChr=='C') {
			Serial.println("Toggling Control");
			digitalWrite(statusControlPin,!value_control);
		}	
		if (inChr=='w' or inChr=='W') {
			Serial.println("Running for 2s");
			digitalWrite(motorpin,HIGH);
			autostoptime=millis()+2000;
		}
		if (inChr=='g' or inChr=='G') {
			Serial.println("Running motor");
			digitalWrite(motorpin,HIGH);
			autostoptime=millis()+120000;
		}
		if (inChr=='h' or inChr=='H') {
			Serial.println("Forcing Halt");
			digitalWrite(motorpin,LOW);
		}
		if (inChr=='a' or inChr=='A') {
			Serial.println("Turning Left");
			steeringServo.writeMicroseconds(steeringLimits[0]);
		}
		if (inChr=='d' or inChr=='D') {
			Serial.println("Turning Right");
			steeringServo.writeMicroseconds(steeringLimits[2]);
		}
		if (inChr=='s' or inChr=='S') {
			Serial.println("Centering");
			steeringServo.writeMicroseconds(steeringLimits[1]);
		}
		if (inChr=='<') {
			steeringLimits[1]--;
			steeringServo.writeMicroseconds(steeringLimits[1]);
		}
		if (inChr=='>') {
			steeringLimits[1]++;
			steeringServo.writeMicroseconds(steeringLimits[1]);
		}
		
//		int sigstatus = analogRead(signal_pin);
//		int sigstatus2 = value_signal;
//		Serial.println(sigstatus);
//		Serial.println(sigstatus2);
		
			Serial.print("Control is ");
			if (value_control == HIGH) Serial.println("Enabled");
			else Serial.println("Disabled");
			Serial.print("Motor is ");
			if (digitalRead(motorpin) == HIGH) Serial.println("ON");
			else Serial.println("OFF");
		}
	}
	