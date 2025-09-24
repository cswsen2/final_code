#include <ArduinoJson.h>

#define NORTH_RED_PIN     2
#define NORTH_YELLOW_PIN  3
#define NORTH_GREEN_PIN   4

#define EAST_RED_PIN      5
#define EAST_YELLOW_PIN   6
#define EAST_GREEN_PIN    7

#define SOUTH_RED_PIN     8
#define SOUTH_YELLOW_PIN  9
#define SOUTH_GREEN_PIN   10

#define WEST_RED_PIN      11
#define WEST_YELLOW_PIN   12
#define WEST_GREEN_PIN    13

#define pedestrian0 22
#define pedestrian1 23
#define pedestrian2 24
#define pedestrian3 25

#define STATUS_LED_PIN    A0
#define BUZZER_PIN        A1

#define RAIN_SENSOR_PIN   A2
#define RAIN_STATUS_LED   A3

#define SERIAL_BAUDRATE   9600
#define JSON_BUFFER_SIZE  1024
#define PEDESTRIAN_CROSSING_TIME 6000 
#define RAINY_PEDESTRIAN_TIME 10000    

#define RAIN_THRESHOLD    700     
#define RAIN_CHECK_INTERVAL 2000  

int currentPriorityLane = 0;
int previousPriorityLane = -1;  
String currentPriorityType = "normal";
bool pedestrianActive = false;
bool emergencyActive = false;

bool rainDetected = false;
unsigned long lastRainCheck = 0;

unsigned long lastUpdate = 0;
unsigned long pedestrianStartTime = 0;

int redPins[4] = {NORTH_RED_PIN, EAST_RED_PIN, SOUTH_RED_PIN, WEST_RED_PIN};
int yellowPins[4] = {NORTH_YELLOW_PIN, EAST_YELLOW_PIN, SOUTH_YELLOW_PIN, WEST_YELLOW_PIN};
int greenPins[4] = {NORTH_GREEN_PIN, EAST_GREEN_PIN, SOUTH_GREEN_PIN, WEST_GREEN_PIN};
int pedestrianLights[4] = {pedestrian0,pedestrian1,pedestrian2,pedestrian3};

String laneNames[4] = {"North", "East", "South", "West"};

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  
  pinMode(pedestrian0, OUTPUT);
  pinMode(pedestrian1, OUTPUT);
  pinMode(pedestrian2, OUTPUT);
  pinMode(pedestrian3, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(redPins[i], OUTPUT);
    pinMode(yellowPins[i], OUTPUT);
    pinMode(greenPins[i], OUTPUT);
  }
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RAIN_STATUS_LED, OUTPUT);
  
  pinMode(RAIN_SENSOR_PIN, INPUT);
  
  setAllLanesRed();
  
  digitalWrite(STATUS_LED_PIN, HIGH);
  digitalWrite(RAIN_STATUS_LED, LOW);
  
  Serial.println("=================================");
  Serial.println(" Arduino Traffic Light Controller");
  Serial.println("Features: Emergency Priority, Pedestrian Safety, Rain Detection");
  Serial.println("Rain sensor initialized on pin A2");
  Serial.println(" Full Python control - Arduino only executes commands");
  Serial.println(" Waiting for Python data...");
  Serial.println("=================================");
  
  lastUpdate = millis();
  lastRainCheck = millis();
}

void loop() {
  checkRainSensor();
  
  if (Serial.available()) {
    processSerialData();
  }
  
  managePedestrianCrossing();
  
  if (millis() % 1000 < 100) {
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    digitalWrite(STATUS_LED_PIN, LOW);
  }
}

void checkRainSensor() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastRainCheck >= RAIN_CHECK_INTERVAL) {
    int rainValue = analogRead(RAIN_SENSOR_PIN);
    bool previousRainState = rainDetected;
    
    rainDetected = (rainValue < RAIN_THRESHOLD);
    
    digitalWrite(RAIN_STATUS_LED, rainDetected ? HIGH : LOW);
    
    if (rainDetected != previousRainState) {
      if (rainDetected) {
        Serial.println("🌧️ RAIN DETECTED! Extended pedestrian crossing time available");
        Serial.println("   Rain sensor value: " + String(rainValue));
      } else {
        Serial.println("☀️ Rain stopped. Normal pedestrian crossing time");
        Serial.println("   Rain sensor value: " + String(rainValue));
      }
    }
    
    lastRainCheck = currentTime;
  }
}

unsigned long getCurrentPedestrianTime() {
  return rainDetected ? RAINY_PEDESTRIAN_TIME : PEDESTRIAN_CROSSING_TIME;
}

void processSerialData() {
  String jsonString = Serial.readStringUntil('\n');
  jsonString.trim();
  
  if (jsonString.length() == 0) return;
  
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  
  if (error) {
    Serial.println(" JSON parsing failed: " + String(error.c_str()));
    return;
  }
  
  int priorityLane = doc["priority_lane"];
  int prevPriorityLane = doc["previous_priority_lane"];
  String priorityType = doc["priority_type"].as<String>();

  if (pedestrianActive && priorityType != "emergency") {
    Serial.println(" Pedestrian crossing active — ignoring normal/other commands until finished");
    return;
  }
  
  Serial.println("\n Received data from Python:");
  Serial.println("  Priority Lane: " + String(priorityLane));
  Serial.println("  Previous Priority Lane: " + String(prevPriorityLane));
  Serial.println("  Priority Type: " + priorityType);
  if (rainDetected) {
    Serial.println("   Rain Mode: Extended pedestrian time if needed");
  }
  
  currentPriorityLane = priorityLane;
  previousPriorityLane = prevPriorityLane;
  currentPriorityType = priorityType;
  
  if (priorityType == "emergency") {
    handleEmergencyVehicle(priorityLane, prevPriorityLane);
    
  } else if (priorityType == "pedestrian_safety") {
    handlePedestrianSafety(priorityLane, prevPriorityLane);
  } else {
    handleNormalTraffic(priorityLane, prevPriorityLane);
  }
  
  lastUpdate = millis();
}

void managePedestrianCrossing() {
  if (!pedestrianActive) return;
  
  unsigned long currentTime = millis();
  unsigned long pedestrianElapsed = currentTime - pedestrianStartTime;
  
  if (pedestrianElapsed >= getCurrentPedestrianTime()) {
    pedestrianActive = false;

    for (int i = 0; i < 4; i++) {
      digitalWrite(pedestrianLights[i], LOW);
    }
    Serial.println(" Pedestrian crossing time completed");
    Serial.println(" Ready for next Python command");
  }
}

void handlePedestrianSafety(int lane, int prevLane) {
  Serial.println(" PEDESTRIAN SAFETY MODE ACTIVATED");
  Serial.println(" All lanes set to RED for pedestrian crossing");
  if (rainDetected) {
    Serial.println("Extended pedestrian crossing time due to rain: " + String(getCurrentPedestrianTime()/1000) + " seconds");
  } else {
    Serial.println(" Pedestrian crossing time: " + String(getCurrentPedestrianTime()/1000) + " seconds");
  }
  
  pedestrianActive = true;
  emergencyActive = false;
  
  if (prevLane >= 0 && prevLane <= 3) {
    Serial.println(" Setting " + laneNames[prevLane] + " to YELLOW (transition)");
    digitalWrite(yellowPins[prevLane], HIGH);
    digitalWrite(greenPins[prevLane], LOW);
    delay(2000);
  }
  
  setAllLanesRed();

  for (int i = 0; i < 4; i++) {
    digitalWrite(pedestrianLights[i], LOW);
  }

  digitalWrite(pedestrianLights[lane], HIGH);
  
  // activatePedestrianSignal();
  
  pedestrianStartTime = millis();
}

void handleEmergencyVehicle(int lane, int prevLane) {

  for (int i = 0; i < 4; i++) {
      digitalWrite(pedestrianLights[i], LOW);
    }
  Serial.println("🚨 EMERGENCY VEHICLE DETECTED!");
  Serial.println("🟢 Immediate priority given to " + laneNames[lane] + " lane");
  Serial.println("⚠️  Emergency overrides pedestrian safety");
  
  emergencyActive = true;
  pedestrianActive = false;

   if (previousPriorityLane == lane && emergencyActive) {
    Serial.println("⏱️ Continuing EMERGENCY GREEN for " + laneNames[lane]);
    digitalWrite(redPins[lane], LOW);
    digitalWrite(greenPins[lane], HIGH);
    return;
  }
  
  if (prevLane >= 0 && prevLane <= 3 ) {
    Serial.println("🟡 Setting " + laneNames[prevLane] + " to YELLOW (transition)");
    digitalWrite(yellowPins[prevLane], HIGH);
    digitalWrite(greenPins[prevLane], LOW);
    delay(1500);
  }
  
  setAllLanesRed();
  delay(5000);
  
  digitalWrite(redPins[lane], LOW);
  digitalWrite(greenPins[lane], HIGH);

  delay(2000);
  Serial.println("🟢 " + laneNames[lane] + " lane activated for EMERGENCY");
  
  // activateEmergencyAlert();
  
  Serial.println("🤖 Python controls emergency duration");
}

void handleNormalTraffic(int lane, int prevLane) {

  for (int i = 0; i < 4; i++) {
      digitalWrite(pedestrianLights[i], LOW);
    }
  Serial.println("🚦 Normal traffic command received");
  Serial.println("🟢 Setting " + laneNames[lane] + " lane to GREEN");
  
  emergencyActive = false;
  pedestrianActive = false;

  if (previousPriorityLane == lane && digitalRead(greenPins[lane]) == HIGH) {
    Serial.println("⏱️ Continuing EMERGENCY GREEN for " + laneNames[lane]);
    return;
  }
  
  if (prevLane >= 0 && prevLane <= 3 && prevLane != lane) {
    Serial.println("🟡 Setting " + laneNames[prevLane] + " to YELLOW (transition)");
    digitalWrite(yellowPins[prevLane], HIGH);
    digitalWrite(greenPins[prevLane], LOW);
    delay(2000);
  }
  
  setAllLanesRed();
  delay(500);

  digitalWrite(yellowPins[lane], HIGH);
  delay(500);
  digitalWrite(yellowPins[lane], LOW);
  
  digitalWrite(redPins[lane], LOW);
  digitalWrite(greenPins[lane], HIGH);

  Serial.println("🟢 " + laneNames[lane] + " lane activated");
  
  Serial.println("✅ Lane activated - Python controls duration");
}

void setAllLanesRed() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(redPins[i], HIGH);
    digitalWrite(yellowPins[i], LOW);
    digitalWrite(greenPins[i], LOW);
  }
}

void setAllLanesOff() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(redPins[i], LOW);
    digitalWrite(yellowPins[i], LOW);
    digitalWrite(greenPins[i], LOW);
  }
}



