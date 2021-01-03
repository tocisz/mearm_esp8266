#include <Servo.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <PubSubClient.h>

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
const int s_pin[sn] = {13, 12, 14, 16};

Ticker ticker;

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
float max_speed = 10.0f;
float current_position[3];
float target_position[3];
float speed_limit = max_speed;

bool vec_eq(float a[], float b[]) {
  for (int i = 0; i < 3; ++i) {
    if (a[i] != b[i])
      return false;
  }
  return true;
}

void vec_assign(float a[], float b[]) {
  for (int i = 0; i < 3; ++i) {
    a[i] = b[i];
  }
}

void scale(float a[], float s) {
  for (int i = 0; i < 3; ++i) {
    a[i] *= s;
  }
}

/*
 *     
    float x, y, z;
    solve(0.0f,228.0f,0.0f,x,y,z); // fully forward
    printf("%f %f %f\n", x*180/PI, y*180/PI, z*180/PI);
    
    solve(-228.0f,0.0f,0.0f,x,y,z); // fully left
    printf("%f %f %f\n", x*180/PI, y*180/PI, z*180/PI);

    solve(228.0f,0.0f,0.0f,x,y,z); // fully right
    printf("%f %f %f\n", x*180/PI, y*180/PI, z*180/PI);

    solve(0.0f,68.0f,160.0f,x,y,z); // fully up
    printf("%f %f %f\n", x*180/PI, y*180/PI, z*180/PI);
 */

void update_servos() {
  if (!vec_eq(current_position, target_position)) {
    float a[3]; // base, shoulder, elbow
    if (solve(target_position, a)) {
      scale(a, 200/PI);
      Serial.print(a[0]);
      Serial.print(',');
      Serial.print(a[1]);
      Serial.print(',');
      Serial.print(a[2]);
      Serial.println();
    } else {
      Serial.println("can't solve");
    }
    vec_assign(current_position, target_position);
  }
  
//  for (int i = 0; i < sn; ++i) {
//    int diff = target_position[i]-current_position[i];
//    if (diff != 0) {
//      if (diff > 0) {
//        if (diff > speed_limit[i]) {
//          diff = speed_limit[i];
//        }
//      } else {
//        if (diff < -speed_limit[i]) {
//          diff = -speed_limit[i];
//        }
//      }
//      current_position[i] += diff;
//      s[i].writeMicroseconds(DEFAULT_PULSE_WIDTH + current_position[i]);
//    }
//  }
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

  // Servo callibration data
  setup_servo(base, 145, 49, -PI/4, PI/4);
  setup_servo(shoulder, 118, 22, -PI/4, 3*PI/4);
  setup_servo(elbow, 144, 36, PI/4, -PI/4);
  setup_servo(gripper, 75, 49, PI/2, 0);

  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqtt_callback);

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("robot_arm", mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Subscribe interesting topics
      client.subscribe("robot_arm");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (Serial.available() > 0) {
    serialEvent();
  }

  while (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  if (stringComplete) {
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
  }
}

short data[4];
void mqtt_callback(char* topic, unsigned char* payload, unsigned int length) {
  if (length == 4*sizeof(short)) {
    memcpy((void *)data, (void *)payload, 4*sizeof(short));
    for (int i = 0; i < 3; ++i) {
      target_position[i] = data[i]*0.1f;
      Serial.print(data[i], DEC);
      Serial.print(',');
    }
    speed_limit = data[3]*0.1f;
    Serial.println(data[3], DEC);
  }
}

void log_servo_request(int servo_no, int pos, int sp) {
  Serial.print((char)('A'+servo_no));
  Serial.print(':');
  Serial.print(pos);
  Serial.print(',');
  Serial.println(sp);
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
