#include <Servo.h>
#include <core_esp8266_waveform.h>
#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <PubSubClient.h>
#include <Math.h>
#include <Esp.h>

#include "fk.h"
#include "ik.h"
#include "meArm.h"

// secrets.ino
extern const char* ssid;     // The SSID (name) of the Wi-Fi network you want to connect to
extern const char* password; // The password of the Wi-Fi network
extern const char* mqttServer;
extern const int   mqttPort;
extern const char* mqttUser;
extern const char* mqttPassword;

WiFiClient espClient;
PubSubClient client(espClient);

const uint8_t sn = 4;
Servo s[sn];

// base: 98 -> -PI/2, 102 -> PI/2
// shoulder: 100 -> 0, -70 -> 170/200*PI (15 -> PI/2)
// elbow: -54 -> ?, 98 -> ? (-46 -> PI/2)
// gripper: 16 -> 0, -40 -> PI
const int s_pin[sn] = {12, 13, 16, 14};
int s_values[sn];

enum CommandType {
  READ = 0, // read current position
  MOVE, // move to position with constant speed
  GRIP, // change gripper position
  MOVE_SEQ // TODO sequence of positions (every 20 ms) [what about gripper?]
};

ServoInfo base, shoulder, elbow, gripper;

/*
 * Position range is -100..100 = -90 deg..90 deg
 * Max speed is 1.5ms per position.
 * Position can be updated every 20 ms
 * So max speed is 13.3 positions per 20 ms.
 * 
 * position is stored in us (sent to servo)
 * speed is position difference per 20ms
 */
float current_position[3] = {0, 100, 50}; // x, y, z
float target_position[3] = {0, 100, 50};
float speed_limit = 0.0f;

// separate command for gripper
short current_gripper = 90;
short target_gripper = 90;
short gripper_speed_limit = 180;

bool vec_eq(float a[], float b[]) {
  for (int i = 0; i < 3; ++i) {
    if (a[i] != b[i]) // up to delta?
      return false;
  }
  return true;
}

void vec_mul_scalar(float a[], float s) {
  for (int i = 0; i < 3; ++i) {
    a[i] *= s;
  }
}

float vec_len(float a[]) {
  float d = 0.0f;
  for (int i = 0; i < 3; ++i) {
    d += a[i]*a[i];
  }
  return sqrt(d);
}

float vec_sub(float a[], float b[], float c[]) {
  for (int i = 0; i < 3; ++i) {
    a[i] = b[i]-c[i];
  }
}

void vec_assign(float a[], float b[]) {
  for (int i = 0; i < 3; ++i) {
    a[i] = b[i];
  }
}

float vec_add(float a[], float b[], float c[]) {
  for (int i = 0; i < 3; ++i) {
    a[i] = b[i]+c[i];
  }
}

void set_position(float pos[3]) {
  float a[3];
  int us[3];

  if (solve(pos, a)) {
    us[0] = angle2pwm(base,     a[0]);
    us[1] = angle2pwm(shoulder, a[1]);
    us[2] = angle2pwm(elbow,    a[2]);
    s_values[0] = DEFAULT_PULSE_WIDTH + us[0];
    s_values[1] = DEFAULT_PULSE_WIDTH + us[1];
    s_values[2] = DEFAULT_PULSE_WIDTH + us[2];
  } else {
    Serial.println("CS");
  }
}

void set_grip(float current_gripper) {
  float pwm = angle2pwm(gripper, current_gripper*PI/180.0f); // deg2rad
  s_values[3] = DEFAULT_PULSE_WIDTH + pwm;
}

uint32_t last_updated;
bool update_needed = false;

const uint32_t UPDATE_INTERNAL_US = 20000;
const uint32_t UPDATE_INTERNAL = microsecondsToClockCycles(20000);
uint32_t update_servos_irq() {
  uint32_t now = ESP.getCycleCount();
  uint32_t us = clockCyclesToMicroseconds(now - last_updated);
  if (us >= UPDATE_INTERNAL_US) {
    // just set servo positions
    s[0].writeMicroseconds(s_values[0]);
    s[1].writeMicroseconds(s_values[1]);
    s[2].writeMicroseconds(s_values[2]);
    s[3].writeMicroseconds(s_values[3]);
    last_updated = now;
    update_needed = true;
  }
  return UPDATE_INTERNAL - (now - last_updated);
}

bool in_move = false;
bool publish_position = false;

void update_servos() {
  // calculate servo positions for the next frame
  if (update_needed) {
    float delta[3]; // base, shoulder, elbow

    bool moving = false;
    bool gripping = false;
  
    if (!vec_eq(current_position, target_position)) {
      vec_sub(delta, target_position, current_position); // delta = target_position - current_position
      float dist = vec_len(delta);
      if (dist > speed_limit) {
        vec_mul_scalar(delta, speed_limit/dist);
      }
      vec_add(current_position, current_position, delta);
      set_position(current_position);
      in_move = true;
      moving = true;
    }
  
    if (current_gripper != target_gripper) {
      float dist = target_gripper - current_gripper;
      if (abs(dist) > gripper_speed_limit) {
        dist *= speed_limit/abs(dist);
      }
      current_gripper += dist;
      set_grip(current_gripper);
      in_move = true;
      gripping = true;
    }

    if (in_move && !moving && !gripping) {
      // finished moving
      publish_position = true;
      in_move = false;
    }

    update_needed = false;
  }
}

String inputString = "";
bool stringComplete = false;

void setup() {
  // INIT serial communication
  Serial.begin(115200);
  Serial.println('\n');
  inputString.reserve(100);

  // INIT WiFi
  WiFi.softAPdisconnect(true); // no AP
  WiFi.begin(ssid, password);  // Connect to the network
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++i); Serial.print(", ");
  }
  Serial.println('\n');
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());              // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());           // Send the IP address of the ESP8266 to the computer

  // INIT MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqtt_callback);

  // INIT servos
  for (size_t i = 0; i < sn; ++i) {
    s[i].attach(s_pin[i]);
  }

  // Servo callibration data (see servo.ods)
  setup_servo(base,      980, -1020, -1.5707963268f, 1.5707963268f);
  setup_servo(shoulder, 1000,  -700,  0.0f,          2.6703537556f);
  setup_servo(elbow,    -540,   980, -1.6964600329f, 0.6911503838f);
  setup_servo(gripper,   160,  -400,  PI,         0);

  set_position(current_position);
  set_grip(current_gripper);
  
  //ticker.attach_ms(20, update_servos);
  setTimer1Callback(update_servos_irq);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("robot_arm", mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Subscribe interesting topics
      client.subscribe("robot_arm/i");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

short pub_data[4];
void loop() {
  // check serial port events
  if (Serial.available() > 0) {
    serialEvent();
  }

  // serial port commands handling
  if (stringComplete) {
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
  }

  // check MQTT events
  while (!client.connected()) {
    reconnect();
  }
  client.loop();
  if (publish_position) {
    pub_data[0] = (short)current_position[0];
    pub_data[1] = (short)current_position[1];
    pub_data[2] = (short)current_position[2];
    pub_data[3] = (short)current_gripper;
    client.publish("robot_arm/o", (uint8_t *)pub_data, 8);
    publish_position = false;
  }

  update_servos();
}

short data[4];
void mqtt_callback(char* topic, unsigned char* payload, unsigned int length) {
  if (payload[0] == (unsigned char) CommandType::READ) {
    publish_position = true;
  } else if (payload[0] == (unsigned char) CommandType::MOVE) {
    memcpy((void *)data, (void *)(payload+1), length-1);
    for (int i = 0; i < 3; ++i) {
      target_position[i] = data[i];
      Serial.print(data[i]);
      Serial.print(',');
    }
    speed_limit = data[3]*0.1f;
    Serial.println(data[3]);
  } else if (payload[0] == (unsigned char) CommandType::GRIP) {
    memcpy((void *)data, (void *)(payload+1), length-1);
    target_gripper = data[0];
    gripper_speed_limit = data[1];
    Serial.print(data[0]);
    Serial.print(',');
    Serial.println(data[1]);
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
