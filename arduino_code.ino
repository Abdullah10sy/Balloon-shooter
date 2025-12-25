/*
Arduino Code for Balloon Targeting System
Receives serial commands from Python and controls servos and laser.

Hardware Connections:
- Pan Servo: Pin 9
- Tilt Servo: Pin 10  
- Trigger Servo: Pin 11
- Laser Module: Pin 12 (via relay or transistor)

Serial Commands:
- PAN:90.0,TILT:90.0  -> Move servos to specified angles
- FIRE:0.5            -> Fire trigger for specified duration
- LASER:ON            -> Turn laser on
- LASER:OFF           -> Turn laser off
*/

#include <Servo.h>

// Servo objects
Servo panServo;
Servo tiltServo;
Servo triggerServo;

// Pin definitions
const int PAN_PIN = 9;
const int TILT_PIN = 10;
const int TRIGGER_PIN = 11;
const int LASER_PIN = 12;

// Servo positions
float panAngle = 90.0;
float tiltAngle = 90.0;
float triggerRestAngle = 0.0;
float triggerFireAngle = 90.0;

// Laser state
bool laserState = false;

// Serial communication
String inputString = "";
bool stringComplete = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Balloon Targeting System Arduino Controller");
  Serial.println("Ready for commands...");
  
  // Attach servos
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  triggerServo.attach(TRIGGER_PIN);
  
  // Initialize laser pin
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  
  // Move servos to center position
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  triggerServo.write(triggerRestAngle);
  
  delay(1000);  // Allow servos to reach position
  
  Serial.println("Initialization complete");
}

void loop() {
  // Check for serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processCommand(String command) {
  command.trim();
  Serial.println("Received: " + command);
  
  if (command.startsWith("PAN:") && command.indexOf("TILT:") > 0) {
    // Parse pan and tilt angles
    int panIndex = command.indexOf("PAN:") + 4;
    int commaIndex = command.indexOf(",");
    int tiltIndex = command.indexOf("TILT:") + 5;
    
    if (panIndex > 3 && commaIndex > panIndex && tiltIndex > commaIndex) {
      float newPanAngle = command.substring(panIndex, commaIndex).toFloat();
      float newTiltAngle = command.substring(tiltIndex).toFloat();
      
      moveServos(newPanAngle, newTiltAngle);
    }
  }
  else if (command.startsWith("FIRE:")) {
    // Parse fire duration
    float duration = command.substring(5).toFloat();
    fireWeapon(duration);
  }
  else if (command == "LASER:ON") {
    setLaser(true);
  }
  else if (command == "LASER:OFF") {
    setLaser(false);
  }
  else {
    Serial.println("Unknown command: " + command);
  }
}

void moveServos(float newPanAngle, float newTiltAngle) {
  // Constrain angles to safe limits
  newPanAngle = constrain(newPanAngle, 0, 180);
  newTiltAngle = constrain(newTiltAngle, 45, 135);  // Limited tilt range for safety
  
  panAngle = newPanAngle;
  tiltAngle = newTiltAngle;
  
  // Move servos smoothly
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  
  Serial.println("Moved to Pan:" + String(panAngle) + " Tilt:" + String(tiltAngle));
  
  delay(50);  // Small delay for servo movement
}

void fireWeapon(float duration) {
  Serial.println("FIRING for " + String(duration) + " seconds");
  
  // Move trigger servo to fire position
  triggerServo.write(triggerFireAngle);
  delay(duration * 1000);  // Convert to milliseconds
  
  // Return trigger to rest position
  triggerServo.write(triggerRestAngle);
  
  Serial.println("Fire sequence complete");
}

void setLaser(bool state) {
  laserState = state;
  digitalWrite(LASER_PIN, state ? HIGH : LOW);
  
  Serial.println("Laser " + String(state ? "ON" : "OFF"));
}

// Emergency stop function (can be called via serial or button)
void emergencyStop() {
  // Turn off laser
  setLaser(false);
  
  // Center servos
  moveServos(90, 90);
  
  // Ensure trigger is at rest
  triggerServo.write(triggerRestAngle);
  
  Serial.println("EMERGENCY STOP ACTIVATED");
}