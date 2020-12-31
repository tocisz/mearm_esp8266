#include <Servo.h>
#include <Ticker.h>

const uint8_t sn = 4;
Servo s[sn];
const int s_pin[sn] = {13, 12, 14, 16};

Ticker ticker;

/*
 * Position range is -100..100 = -90 deg..90 deg
 * Max speed is 1.5ms per position.
 * Position can be updated every 20 ms
 * So max speed is 13.3 positions per 20 ms.
 * 
 * position is stored in us (sent to servo)
 * speed is position difference per 20ms
 */
const int max_speed = 140;
int current_position[sn];
int target_position[sn];
int speed_limit[sn] = {max_speed,max_speed,max_speed,max_speed};

void update_servos() {
  for (int i = 0; i < sn; ++i) {
    int diff = target_position[i]-current_position[i];
    if (diff != 0) {
      if (diff > 0) {
        if (diff > speed_limit[i]) {
          diff = speed_limit[i];
        }
      } else {
        if (diff < -speed_limit[i]) {
          diff = -speed_limit[i];
        }
      }
      current_position[i] += diff;
      s[i].writeMicroseconds(DEFAULT_PULSE_WIDTH + current_position[i]);
    }
  }
}

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
  Serial.println('\n');
  inputString.reserve(100);

  for (size_t i = 0; i < sn; ++i) {
    s[i].attach(s_pin[i]);
  }

  ticker.attach_ms(20, update_servos);
}

void loop() {
  if (Serial.available() > 0) {
    serialEvent();
  }
  if (stringComplete) {
    byte servo_no = inputString[0] - 'A';
    int separator = inputString.indexOf(',');

    long pos = inputString.substring(1).toInt();
    target_position[servo_no] = 10*pos; //position is stored in us (sent to servo)
    
    long sp;
    if (separator > 0) {
      sp = inputString.substring(separator+1).toInt();
    } else {
      sp = max_speed;
    }
    speed_limit[servo_no] = sp;

    Serial.print((char)('A'+servo_no));
    Serial.print(',');
    Serial.print(pos);
    Serial.print(',');
    Serial.println(sp);

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
