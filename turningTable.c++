#include <ArduinoBLE.h>

// Pin definitions
#define ELECTROMAGNET_PIN 2  // Pin für Elektromagnet Relais
#define MOTOR_ENABLE_PIN 3   // Pin für Motor Enable
#define MOTOR_DIR_PIN 4      // Pin für Motor Direction
#define MOTOR_STEP_PIN 5     // Pin für Motor Step (falls Schrittmotor)

// Commands from Raspberry Pi
#define CMD_ACTIVATE_MAGNET 0x10    // Elektromagnet aktivieren
#define CMD_DEACTIVATE_MAGNET 0x11  // Elektromagnet deaktivieren
#define CMD_START_ROTATION 0x12     // Rotation starten
#define CMD_STOP_ROTATION 0x13      // Rotation stoppen
#define CMD_SET_SPEED 0x14          // Rotationsgeschwindigkeit setzen

// Status commands to send
#define STATUS_MAGNET_ON 0x20
#define STATUS_MAGNET_OFF 0x21
#define STATUS_ROTATING 0x22
#define STATUS_STOPPED 0x23
#define STATUS_READY 0x24

enum TableState {
  DISCONNECTED,
  READY,
  MAGNET_ACTIVE,
  ROTATING
};

TableState currentState = DISCONNECTED;

// BLE Service
BLEService tableService("19B20000-E8F2-537E-4F6C-D104768A1215");

// BLE Characteristics
BLECharacteristic statusCharacteristic("19B20001-E8F2-537E-4F6C-D104768A1215",
                                       BLERead | BLENotify, 1);

BLECharacteristic commandCharacteristic("19B20002-E8F2-537E-4F6C-D104768A1215",
                                        BLEWrite, 1);

// Rotation parameters
bool magnetActive = false;
bool isRotating = false;
unsigned long rotationStartTime = 0;
int rotationSpeed = 50; // Default speed (0-100)
unsigned long lastStepTime = 0;
int stepDelay = 1000; // Microseconds between steps

void setup() {
  Serial.begin(115200);
  // while (!Serial);

  // Configure pins
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Initial states
  digitalWrite(ELECTROMAGNET_PIN, LOW);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  digitalWrite(MOTOR_DIR_PIN, HIGH);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setConnectionInterval(6, 6);
  BLE.setAdvertisingInterval(160);
  BLE.setLocalName("TurningTable");
  BLE.setDeviceName("TurningTable");

  BLE.setAdvertisedService(tableService);

  tableService.addCharacteristic(statusCharacteristic);
  tableService.addCharacteristic(commandCharacteristic);

  BLE.addService(tableService);

  // Set event handler
  commandCharacteristic.setEventHandler(BLEWritten, onCommandReceived);

  // Start advertising
  BLE.advertise();

  Serial.println("Turning Table ready, waiting for connections...");
}

void onCommandReceived(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t cmd = commandCharacteristic.value()[0];
  
  if(Serial) {
    Serial.print("Received command: 0x");
    Serial.println(cmd, HEX);
  }

  switch(cmd) {
    case CMD_ACTIVATE_MAGNET:
      activateMagnet();
      break;
      
    case CMD_DEACTIVATE_MAGNET:
      deactivateMagnet();
      break;
      
    case CMD_START_ROTATION:
      startRotation();
      break;
      
    case CMD_STOP_ROTATION:
      stopRotation();
      break;
      
    case CMD_SET_SPEED:
      // Speed value should be sent in next write
      // For now, use default
      break;
  }
}

void activateMagnet() {
  if(!magnetActive) {
    digitalWrite(ELECTROMAGNET_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    magnetActive = true;
    currentState = MAGNET_ACTIVE;
    
    uint8_t status = STATUS_MAGNET_ON;
    statusCharacteristic.writeValue(&status, 1);
    
    if(Serial) Serial.println("Electromagnet activated");
  }
}

void deactivateMagnet() {
  if(magnetActive) {
    digitalWrite(ELECTROMAGNET_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    magnetActive = false;
    
    if(!isRotating) {
      currentState = READY;
    }
    
    uint8_t status = STATUS_MAGNET_OFF;
    statusCharacteristic.writeValue(&status, 1);
    
    if(Serial) Serial.println("Electromagnet deactivated");
  }
}

void startRotation() {
  if(!isRotating) {
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    isRotating = true;
    rotationStartTime = millis();
    lastStepTime = micros();
    currentState = ROTATING;
    
    // Calculate step delay based on speed (0-100)
    stepDelay = map(rotationSpeed, 0, 100, 5000, 500);
    
    uint8_t status = STATUS_ROTATING;
    statusCharacteristic.writeValue(&status, 1);
    
    if(Serial) {
      Serial.println("Rotation started");
      Serial.print("Speed: ");
      Serial.print(rotationSpeed);
      Serial.print("%, Step delay: ");
      Serial.print(stepDelay);
      Serial.println(" us");
    }
  }
}

void stopRotation() {
  if(isRotating) {
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    digitalWrite(MOTOR_STEP_PIN, LOW);
    isRotating = false;
    
    if(!magnetActive) {
      currentState = READY;
    } else {
      currentState = MAGNET_ACTIVE;
    }
    
    uint8_t status = STATUS_STOPPED;
    statusCharacteristic.writeValue(&status, 1);
    
    if(Serial) Serial.println("Rotation stopped");
  }
}

void updateRotation() {
  if(!isRotating) return;
  
  unsigned long currentMicros = micros();
  
  if(currentMicros - lastStepTime >= stepDelay) {
    // Toggle step pin
    digitalWrite(MOTOR_STEP_PIN, !digitalRead(MOTOR_STEP_PIN));
    lastStepTime = currentMicros;
  }
}

void loop() {
  BLEDevice central = BLE.central();
  
  BLE.poll();

  switch(currentState) {
    case DISCONNECTED:
      if(central && central.connected()) {
        if(Serial) Serial.print("Connected to central: ");
        if(Serial) Serial.println(central.address());
        currentState = READY;
        
        uint8_t status = STATUS_READY;
        statusCharacteristic.writeValue(&status, 1);
      }
      break;
      
    case READY:
    case MAGNET_ACTIVE:
    case ROTATING:
      if(!central.connected()) {
        if(Serial) Serial.println("Disconnected from central");
        
        // Safety: Stop everything on disconnect
        deactivateMagnet();
        stopRotation();
        currentState = DISCONNECTED;
      }
      
      // Update motor if rotating
      updateRotation();
      break;
  }
  
  // Small delay to prevent overwhelming the loop
  delay(1);
}