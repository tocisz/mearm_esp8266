#include <Servo.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <PubSubClient.h>

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
    int servo_no = inputString[0] - 'A';
    int separator = inputString.indexOf(',');

    int pos = inputString.substring(1).toInt();
    target_position[servo_no] = 10*pos; //position is stored in us (sent to servo)
    
    int sp;
    if (separator > 0) {
      sp = inputString.substring(separator+1).toInt();
    } else {
      sp = max_speed;
    }
    speed_limit[servo_no] = sp;

    log_servo_request(servo_no, pos, sp);

    inputString = "";
    stringComplete = false;
  }
}


void mqtt_callback(char* topic, unsigned char* payload, unsigned int length) {
  if (length == 2*sn) {
    for (int i = 0; i < sn; ++i) {
      unsigned char *data = payload + 2*i;
      int pos = (signed char)data[0]; // force signed
      int sp = data[1];

      target_position[i] = 10*pos;
      speed_limit[i] = sp;

      log_servo_request(i, pos, sp);
    }
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
