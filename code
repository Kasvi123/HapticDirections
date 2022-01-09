#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include "SSD1306Wire.h"

// Heltec OLED Internal Wiring	
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RESET 16

SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);

// WiFi Setup Variables
WiFiClient espClient;
const char* ssid = "Jupiter40";//Jupiter40Ashley's iPhone
const char* password = "planet40";//planet40passw0rd!
const int wifiTimeout = 10;
boolean wifiEnabled = true;

// Address of the MQTT Server/Node-RED/Raspberry Pi/FRED
char* mqtt_server = "10.158.42.203";//192.168.137.93
PubSubClient mqttClient(espClient);

long lastPublished = 0;
int publishInterval = 5000;
char msg[50];

String latString;
String lngString;
String courseString;
String mpsString;
String mphString;
String cardinalString;
String distanceString;
String courseToString;

// GPS
//Wiring: RX to TX, TX to RX
// Using Serial2 for GPS
HardwareSerial SerialGPS(2);
#define GPS_RX 13
#define GPS_TX -1

// The TinyGPS++ object
TinyGPSPlus gps;

// Default Destination
// Oracle Conference Center: 37.532340, -122.264024
double destinationLat = 37.532340;
double destinationLng = -122.264024;

// LED Pin
const int ledPin = LED_BUILTIN;

//Vibrating Motor Pins
const int leftPin = 27;
const int rightPin = 26;

#define BUZZ_OFF         0
#define BUZZ_START       1
#define BUZZING          2
#define BUZZ_PAUSE       3
#define BUZZ_DELAY_START 4

int leftState = BUZZ_OFF;
int rightState = BUZZ_OFF;
int leftCount = 0;
int rightCount = 0;

unsigned long leftTimestamp = 0;
unsigned long rightTimestamp = 0;

void setup() {
  // Setup Serial Monitor
  Serial.begin(9600);
  delay(1000);
  Serial.println();
  Serial.println("Serial Monitor enabled...");

  // Setup GPS on SerialGPS
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(1000);

  // Initialising the UI will init the display too.
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high

  display.init();
  display.clear();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  dualPrint(0, 0, "SSD1306 Working");
  display.display();
  delay(1000);

  //Setup WiFi and MQTT
  if (wifiEnabled) {
    setup_wifi();
    delay(2000);
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(receiveMQTT);
  }

  // Setup GPIO pins
  pinMode(ledPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);

  Serial.println("Setup Complete");
}

// WARNING: Do NOT use delays in loop.
// If you use delays in loop, MQTT messages will be dropped.
void loop() {
  if (wifiEnabled) {
    if (!mqttClient.connected()) {
      connectMQTT();
    }
    mqttClient.loop();
  }

  buzzState(&leftState, &leftCount, &leftTimestamp, leftPin);
  buzzState(&rightState, &rightCount, &rightTimestamp, rightPin);

  while (SerialGPS.available() > 0) {
    byte gpsData = SerialGPS.read();
    //Serial.write(gpsData); //Print debug
    gps.encode(gpsData);

    if (gps.location.isUpdated()) {
      // Get GPS coords as strings rounded to 6 decimals
      latString = String(gps.location.lat(), 6);
      lngString = String(gps.location.lng(), 6);
      courseString = String(gps.course.deg(), 2);
      mpsString = String(gps.speed.mps(), 2);
      mphString = String(gps.speed.mph(), 1);
      cardinalString = TinyGPSPlus::cardinal(gps.course.deg());

      calculateDestination();

      // Refresh OLED Display
      display.clear();
      if (!wifiEnabled) {
        display.drawString(115, 0, "X");
      }
      display.drawString(0, 0, "Current Speed");
      display.drawString(0, 16,  mphString + "mph");
      display.drawString(75, 16, cardinalString);

      display.drawString(0, 32, "Destination");
      display.drawString(0, 48, distanceString);
      display.drawString(75, 48, courseToString);
      display.display();
      break;
    }
  }

  // Check last publish time
  long now = millis();
  if (now - lastPublished > publishInterval) {

    //Print to coordinates to serial monitor
    Serial.print("Lat= " + latString);
    Serial.println(" Lng= " + lngString);
    Serial.print("Course= " + courseString + " deg " + cardinalString);
    Serial.println(" Speed= " + mpsString + "m/s " + mphString + "mph");

    if (wifiEnabled) {
      //Send MQTT GPS Message to NodeRED
      publishGPS();
    }
    lastPublished = now;
  }
}

void buzzState(int *state, int *count, unsigned long *timestamp, int pin) {
  switch (*state) {
    case BUZZ_START:
      *timestamp = millis();
      *state = BUZZING;
      digitalWrite(pin, HIGH);
      break;
    case BUZZING:
      if (millis() - *timestamp > 500) {
        *count = *count - 1;
        if (*count <= 0) {
          *state = BUZZ_OFF;
        }
        else {
          *timestamp = millis();
          *state = BUZZ_PAUSE;
        }
        digitalWrite(pin, LOW);
      }
      break;
    case BUZZ_PAUSE:
      if (millis() - *timestamp > 500) {
        *timestamp = millis();
        *state = BUZZING;
        digitalWrite(pin, HIGH);
      }
      break;
    case BUZZ_DELAY_START:
      *timestamp = millis();
      *state = BUZZ_PAUSE;
      digitalWrite(pin, LOW);
      break;
    default:
      return;
  }
}

void calculateDestination() {
  double metersToDestination  =
    TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      destinationLat,
      destinationLng);
  double milesToDestination = metersToDestination / 1609.344;
  if (milesToDestination < .1) {
    distanceString = String(milesToDestination * 5280.0, 0) + " ft";
  }
  else {
    distanceString = String(milesToDestination, 1) + " mi";
  }

  double courseToDestination =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      destinationLat,
      destinationLng);
  courseToString = TinyGPSPlus::cardinal(courseToDestination);
}

//Publish MQTT GPS messages to Node-RED/RaspPi/FRED
void publishGPS() {
  if (mqttClient.connected()) {
    //String json = "{\"lat\":" + latString + ", \"lng\":" + lngString + "}";
    String s = latString + "," + lngString;
    //Convert String type to char array
    char tempString[s.length() + 1];
    s.toCharArray(tempString, s.length() + 1);
    mqttClient.publish("esp32/gps", tempString);
  }
}

//Callback function that interrupts loop when a message is received
//Do NOT use delays in this function
void receiveMQTT(char* topic, byte * message, unsigned int length) {
  // Debug print message topic
  Serial.print("Message topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageString;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageString += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT
  if (String(topic) == "esp32/led") {
    Serial.print("Changing output to ");
    if (messageString == "on") {
      Serial.println("on");
      digitalWrite(ledPin, HIGH);

    }
    else if (messageString == "off") {
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
  else if (String(topic) == "esp32/destination") {
    // Parse message for longitude and latitude
    int index = messageString.indexOf(',');
    String s1 = messageString.substring(0, index);
    String s2 = messageString.substring(index + 1, messageString.length());
    // Convert type string to double
    destinationLat = s1.toDouble();
    destinationLng = s2.toDouble();

    Serial.print("Destination received: ");
    Serial.print(destinationLat, 6);
    Serial.print(", ");
    Serial.println(destinationLng, 6);
  }
  else if (String(topic) == "esp32/signal") {
    Serial.println("signal received " + messageString);
    if (messageString == "left") {
      leftState = BUZZ_START;
      leftCount = 1;
    }
    else if (messageString == "left2") {
      leftState = BUZZ_START;
      leftCount = 2;
    }
    else if (messageString == "right") {
      rightState = BUZZ_START;
      rightCount = 1;
    }
    else if (messageString == "right2") {
      rightState = BUZZ_START;
      rightCount = 2;
    }
    else if (messageString == "both") {
      leftState = BUZZ_START;
      leftCount = 1;
      rightState = BUZZ_START;
      rightCount = 1;
    }
    else if (messageString == "alternating") {
      leftState = BUZZ_START;
      leftCount = 2;
      rightState = BUZZ_DELAY_START;
      rightCount = 2;
    }
  }
}

// Setup WiFi to SSID with password
// WiFi will timeout after set number of retries
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  display.clear();
  dualPrint(0, 0, "Connecting to Wifi");
  dualPrint(0, 16, "SSID: " + String(ssid));
  display.display();

  WiFi.begin(ssid, password);

  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED) {

    Serial.print(".");
    delay(1000);
    retryCount++;

    if (retryCount > wifiTimeout) {
      wifiEnabled = false;
      dualPrint("WiFi Failed.");
      return;
    }
  }
  // Display WiFi Stats
  display.clear();
  dualPrint(0, 0, "WiFi connected");
  dualPrint(0, 16, "IP address: ");
  dualPrint(0, 32, WiFi.localIP().toString());
  display.display();
}

// Connects to MQTT server (RasPi/Node-RED/FRED)
void connectMQTT() {
  // Loop until we're reconnected
  int retryCount = 0; //TODO finish retry count later
  while (!mqttClient.connected()) {
    dualPrint("MQTT Connect...");
    dualPrint(0, 16, String(mqtt_server));
    // Attempt to connect
    if (mqttClient.connect("ESP8266Client")) {
      dualPrint(0, 32, "Success");
      // Subscribe
      mqttClient.subscribe("esp32/led");
      mqttClient.subscribe("esp32/destination");
      mqttClient.subscribe("esp32/signal");
    } else {
      dualPrint(0, 32, "Failed, rc=" + String(mqttClient.state()));
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  display.display();
  delay(1000);
}

// dualPrint is a helper function that will
// print text in Serial Monitor and the OLED Screen
void dualPrint(String text) {
  display.clear();
  dualPrint(0, 0, text);
  display.display();
}

void dualPrint(int x, int y, String text) {
  Serial.println(text);
  display.drawString(x, y, text);
}

