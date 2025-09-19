//Pinout
//RPM pins are for pwm, Hall Pins are for Reading back to the controller (CURRENTLY UNUSED), Relay Pins are for direction
#define rpm1 1   // Motor1 PWM pin
#define h1 2   // Motor1 hall pin
#define r1 3   // Motor1 relay pin

#define rpm2 4  // Motor2 PWM pin
#define h2 5   // Motor2 hall pin
#define r2 6   // Motor2 relay pin

#define rpm3 7  // Motor3 PWM pin
#define h3 8   // Motor3 hall pin
#define r3 9   // Motor3 relay pin

#define rpm4 10  // Motor4 PWM pin
#define h4 11   // Motor4 hall pin
#define r4 12   // Motor4 relay pin

float rpmValue1 = 0, rpmValue2 = 0, rpmValue3 = 0, rpmValue4 = 0;
float rpmValue1_f,rpmValue2_f,rpmValue3_f,rpmValue4_f;
int dirValue1 = 0, dirValue2 = 0, dirValue3 = 0, dirValue4 = 0;
float feedbackCurrent1 = 0, feedbackCurrent2 = 0, feedbackCurrent3 = 0, feedbackCurrent4 = 0;

void setup() {
  // Set pin modes for motor control
  pinMode(rpm1, OUTPUT);
  pinMode(h1, OUTPUT);
  pinMode(r1, OUTPUT);
  pinMode(rpm2, OUTPUT);
  pinMode(h2, OUTPUT);
  pinMode(r2, OUTPUT);
  pinMode(rpm3, OUTPUT);
  pinMode(h3, OUTPUT);
  pinMode(r3, OUTPUT);
  pinMode(rpm4, OUTPUT);
  pinMode(h4, OUTPUT);
  pinMode(r4, OUTPUT);

  // Set initial motor state
  analogWrite(rpm1, 0);
  analogWrite(rpm2, 0);
  analogWrite(rpm3, 0);
  analogWrite(rpm4, 0);

  // Initialize serial communication
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    // Read incoming data until newline
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      // Parse the incoming data
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);
      int thirdSpace = input.indexOf(' ', secondSpace + 1);
      int forthSpace = input.indexOf(' ', thirdSpace + 1);
      int fifthSpace = input.indexOf(' ', forthSpace + 1);

      if (firstSpace != -1 && secondSpace != -1 && thirdSpace != -1 && forthSpace != -1 && fifthSpace != -1) {
        rpmValue1 = input.substring(0, firstSpace).toFloat();
        rpmValue1_f = (rpmValue1);
        dirValue1 = input.substring(firstSpace + 1, secondSpace).toInt();
        
        rpmValue2 = input.substring(secondSpace + 1, thirdSpace).toFloat();
        rpmValue2_f = (rpmValue2);
        dirValue2 = input.substring(thirdSpace + 1).toInt();

        rpmValue3 = input.substring(thirdSpace + 1, forthSpace).toFloat();
        rpmValue3_f = (rpmValue3);
        dirValue3 = input.substring(forthSpace + 1).toInt();

        rpmValue4 = input.substring(forthSpace + 1, fifthSpace).toFloat();
        rpmValue4_f = (rpmValue4);
        dirValue4 = input.substring(fifthSpace + 1).toInt();

        // Control Motor 1
        analogWrite(rpm1, rpmValue1_f);
        // Forward
        if (dirValue1 == 0) {
          digitalWrite(r1, HIGH);
        } else {
          //Reverse
          digitalWrite(r1, LOW);
        }

        // Control Motor 2
        analogWrite(rpm2, rpmValue2_f);
        // Forward
        if (dirValue2 == 0) {
          digitalWrite(r2, HIGH);
        } else {
          //Reverse
          digitalWrite(r2, LOW);
        }

        // Control Motor 3
        analogWrite(rpm3, rpmValue3_f);
        // Forward
        if (dirValue3 == 0) {
          digitalWrite(r3, HIGH);
        } else {
          //Reverse
          digitalWrite(r3, LOW);
        }

        // Control Motor 4
        analogWrite(rpm4, rpmValue4_f);
        // Forward
        if (dirValue4 == 0) {
          digitalWrite(r4, HIGH);
        } else {
          //Reverse
          digitalWrite(r4, LOW);
        }


        //Motor Shutoff
        if (rpmValue1 == 0 && rpmValue2 == 0 && rpmValue3 == 0 && rpmValue4 == 0) {
          // Set all digital pins to LOW if both RPM values are zero
          digitalWrite(r1, LOW);
          digitalWrite(r2, LOW);
          digitalWrite(r3, LOW);
          digitalWrite(r4, LOW);
          analogWrite(rpm1, 0);
          analogWrite(rpm2, 0);
          analogWrite(rpm3, 0);
          analogWrite(rpm4, 0);
        }

//        // Read current feedback from sensors (analog pins A0, A1)
//        feedbackCurrent1 = analogRead(A0)* 0.0196 * 0.5;
//        feedbackCurrent2 = analogRead(A1); //* 0.0196 * 0.5;

        Serial.print("Motor1 RPM: ");
        Serial.print(rpmValue1);
        Serial.print(", Direction: ");
        Serial.println(dirValue1);

        Serial.print("Motor2 RPM: ");
        Serial.print(rpmValue2);
        Serial.print(", Direction: ");
        Serial.println(dirValue2);

        Serial.print("Motor3 RPM: ");
        Serial.print(rpmValue3);
        Serial.print(", Direction: ");
        Serial.println(dirValue3);

        Serial.print("Motor4 RPM: ");
        Serial.print(rpmValue4);
        Serial.print(", Direction: ");
        Serial.println(dirValue4);

//        Serial.print("Motor1 Feedback Current: ");
//        Serial.println(feedbackCurrent1);
//
//        Serial.print("Motor2 Feedback Current: ");
//        Serial.println(feedbackCurrent2);

        // Send current feedback back to the ROS2 node using Serial.write
//        Serial.write((byte *)&feedbackCurrent1, sizeof(feedbackCurrent1));
//        Serial.write((byte *)&feedbackCurrent2, sizeof(feedbackCurrent2));
      }
    }
  }
}