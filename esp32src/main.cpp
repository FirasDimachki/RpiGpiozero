#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <WiFi.h>
#include <secrets.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
// #define LED_BUILTIN 2
// put function declarations here:
// int myFunction(int, int);
int LED_BUILTIN = 2;

// note moist sensor using 3V3, but can be used with 5V
//uncomment below
int MOIST_SENS = 35; 
double moist_volt; // init value
double  MAX_MOISTURE = 4100;

// #define DHTPIN 4     // Digital pin connected to the DHT sensor
// #define DHTTYPE DHT11   // DHT 11
// // Initialize DHT sensor.
// // Note that older versions of this library took an optional third parameter to
// // tweak the timings for faster processors.  This parameter is no longer needed
// // as the current DHT reading algorithm adjusts itself to work on faster procs.
// DHT dht(DHTPIN, DHTTYPE);

int plant_id = PLANT_ID;
const char broker[] = BROKER;
int port = BROKER_PORT;
// const char topic[]  = MOIST_TOPIC;
String topic = "/plant/2/moisture";

const char regist_topic[] = REGISTER_TOPIC;
String regist_feedback_topic = REGISTER_FEEDBACK_TOPIC;
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

unsigned long previousMillis = 0;
unsigned long interval = 10000;


WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

int count = 0;
void onMqttMessage(int messageSize);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Hello Setup!");


  // -------------------- SETUP WIFI -------------------------
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  int tries = 10;
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED && tries > 0) {
    // failed, retry
    Serial.print(".");
    tries--;
    delay(5000);
  }
  if (tries <= 0){
    Serial.println("Failed to connect to the network");
    
  } else {
    Serial.println("You're connected to the network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
  }
 

 //--------------------------------- MQTT ------------------------------- 
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // subscribe to response topic
  mqttClient.subscribe(regist_feedback_topic, 2);
  mqttClient.onMessage(onMqttMessage);

  // publish register request
  mqttClient.beginMessage(regist_topic);
  mqttClient.print(plant_id);
  mqttClient.endMessage();

  // ------------------------------- SENSOR ---------------------------
  pinMode(MOIST_SENS, INPUT);

  // set board led as output pin
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("Hello loop! Blink!");

  // Blink led on board
  // delay(1000);
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(1000);
  // digitalWrite(LED_BUILTIN, LOW);

// --------------------------------------------------

  Serial.println("Looped again - ");
  mqttClient.poll();

  
  unsigned long currentMillis = millis();

  // Read sensor
  moist_volt = analogRead(MOIST_SENS);
  Serial.print("Moisture: ");
  moist_volt = (1-moist_volt/MAX_MOISTURE)*100;
  Serial.println(moist_volt);
  
  // this if is for future code scalability (in case we need to add more logic below it)
  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    Serial.print("Sending message to topic: ");
    Serial.println(topic);


    // Send Json message on topic
    JsonDocument doc;
    // Add key-value pairs to the document
    doc["id"] = plant_id;
    doc["moisture"] = moist_volt;
    // send message, the Print interface can be used to set the message contents
    String message_to_send;
    serializeJson(doc, message_to_send);
    mqttClient.beginMessage(topic);
    mqttClient.print(message_to_send);
    mqttClient.endMessage();

    Serial.println();

    count++;
  }
  
  // Blink led on board
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  // delay(1000);

  // wait interval before reading and publishing again
  delay((interval>1000)?interval-1000: interval);
  digitalWrite(LED_BUILTIN, HIGH);
  

}

// put function definitions here:

void onMqttMessage(int messageSize) {
  String mes_topic = mqttClient.messageTopic();
  Serial.println("Received a message with topic: '" + mes_topic + "'");

  if (mes_topic == regist_feedback_topic) {
    // Handle message received on the registration topic
    String message;
    while (mqttClient.available()) {
      message += char(mqttClient.read());
    }

    topic = message;
    Serial.println("Message content (registration): " + message);
    mqttClient.unsubscribe(regist_feedback_topic);
    // Parse the registration message (if needed)
    // Update variables or perform actions based on the registration message
  }
}