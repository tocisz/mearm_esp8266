#include <Servo.h>

const uint8_t sn = 4;
Servo s[sn];
const int s_pin[sn] = {13, 12, 14, 16};

/*
 * Position range is -50..50 = -90 deg..90 deg
 * Max speed is 3ms per position.
 * Position can be updated every 20 ms
 * So max speed is 7 positions per 20 ms.
 * Min speed is 1 position per 20 ms (we want to do some move after all?)
 */

String inputString = "";
bool stringComplete = false;

// why isn't it available from Servo.h ?
#define DEFAULT_NEUTRAL_PULSE_WIDTH 1500

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);         // Start the Serial communication to send messages to the computer
  delay(10);
  Serial.println('\n');
  inputString.reserve(100);

  for (size_t i = 0; i < sn; ++i) {
    s[i].attach(s_pin[i]);
  }
}

void loop() {
  if (Serial.available() > 0) {
    serialEvent();
  }
  if (stringComplete) {
    byte servo_no = inputString[0] - 'A';
    long pos = inputString.substring(1).toInt();

    Serial.print((char)('A'+servo_no));
    Serial.print(':');
    Serial.println(pos);
    s[servo_no].writeMicroseconds(DEFAULT_NEUTRAL_PULSE_WIDTH+10*pos);

    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      // add it to the inputString:
      inputString += inChar;
    }
  }
}
