unsigned long heartbeat = millis();
unsigned int intTimeout = 30000;
unsigned long lastControlCheck = millis();
unsigned long autostoptime=millis();
boolean statusTimedOut = false;
boolean enableTimeout = false;
// We will set the heartbeat to transmit every 3 seconds, expect it every 10 seconds, and require it every 30 seconds.

int ledpin = 13; // name the output pin for 'Manual Control is'
int motorpin = A0; // name the output pin for 'PowerWheels is '

int pedal_pin = A1; // Accelerator wired to
int pedal_last_state = LOW;
int signal_pin = 2;
int signal_led = 12;


	int value_signal = digitalRead(signal_led);
	int value_control = digitalRead(ledpin);
	boolean value_pedal = digitalRead(pedal_pin);

// the setup routine runs once on reset:
void setup() {
  Serial.begin(115200);
  
  //pinMode(ledpin, INPUT_PULLUP); // make sure pin goes high on being made an output
  pinMode(ledpin, OUTPUT); // output for 'Manual Control is' is initially HIGH, comment out the line above if you want it initially LOW
   //pinMode(cmd_G_pin, INPUT_PULLUP); 
  pinMode(motorpin, OUTPUT); // output for 'PowerWheels is ' is initially LOW, uncomment line above if you want it initially HIGH
 pinMode(signal_led, OUTPUT);
// <<<<<<<<< Your extra setup code goes here
pinMode(pedal_pin, INPUT); // Manual control pin
pinMode(signal_pin, INPUT); // Parental connection pin
digitalWrite(ledpin,HIGH);
}

// the loop routine runs over and over again forever:
void loop() {
	value_signal = digitalRead(signal_pin);
	value_control = digitalRead(ledpin);
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
		digitalWrite(ledpin,LOW);
		digitalWrite(motorpin,LOW);
	}
//	if (millis()>autostoptime && digitalRead(motorpin)==HIGH && value_pedal == LOW) {
//		digitalWrite(motorpin,LOW);
//	}
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
//digitalWrite(motorpin,pfodLongRtn); // set output

void serialEvent() {
	if (Serial.available()) {
		//if(statusTimedOut==true) {
		//	statusTimedOut=false;
		//	digitalWrite(ledpin,HIGH);
		//	heartbeat=millis();
		//}
		
		heartbeat = millis();
		
		char inChr=(char)Serial.read();
		Serial.println(inChr);
		if (inChr=='c' or inChr=='C') {
			Serial.println("Toggling Control");
			digitalWrite(ledpin,!value_control);
		}	
		if (inChr=='g' or inChr=='G') {
			Serial.println("Running for 30s");
			digitalWrite(motorpin,HIGH);
			autostoptime=millis()+30000;
		}
		if (inChr=='h' or inChr=='H') {
			Serial.println("Forcing Halt");
			digitalWrite(motorpin,LOW);
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
	