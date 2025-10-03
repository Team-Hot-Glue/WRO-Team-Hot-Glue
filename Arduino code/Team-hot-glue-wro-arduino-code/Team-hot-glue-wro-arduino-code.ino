//L298N motor driver pins
int motor1pin1 = 2;
int motor1pin2 = 3;

int motor2pin1 = 4;
int motor2pin2 = 5;

void setup() {
	Serial.begin(115200);
	Serial.setTimeout(1);
	pinMode(13, OUTPUT); // LED pin

	pinMode(motor1pin1, OUTPUT);
  	pinMode(motor1pin2, OUTPUT);
  	pinMode(motor2pin1, OUTPUT);
  	pinMode(motor2pin2, OUTPUT);

  	pinMode(9, OUTPUT); 
  	pinMode(10, OUTPUT);
}

void loop() {
	if (Serial.available()) {
		String msg = Serial.readStringUntil('\n');
		int commaIdx = msg.indexOf(',');
		if (commaIdx > 0) {
			String color = msg.substring(0, commaIdx);
			// String angle = msg.substring(commaIdx + 1); // not used here
			if (color == "RED") {
				digitalWrite(13, HIGH);
				delay(200); // blink duration
				digitalWrite(13, LOW);
			} else {
				digitalWrite(13, LOW);
			}
		}
	}

	//Using L298N motor driver

	//Controlling speed (0 = off and 255 = max speed):
  	analogWrite(9, 100); //ENA pin
  	analogWrite(10, 200); //ENB pin

	//Controlling spin direction of motors:
  	digitalWrite(motor1pin1, HIGH);
 	digitalWrite(motor1pin2, LOW);

  	digitalWrite(motor2pin1, HIGH);
  	digitalWrite(motor2pin2, LOW);
  	delay(1000);

  	digitalWrite(motor1pin1, LOW);
  	digitalWrite(motor1pin2, HIGH);

  	digitalWrite(motor2pin1, LOW);
  	digitalWrite(motor2pin2, HIGH);
  	delay(1000);

}
