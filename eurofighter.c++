#include <ArduinoBLE.h>
#include "RotationManager.h"
#include "SmoothData4D.h"

#define FREQUENCY 25.0f // Hz       Frequenz mit der Sensrodaten gemessen/gesendet werden
#define MAG_FREQUENCY 19.0f // Hz   Frequenz für Magnetometer Kalibrierung

#define CMD_RUNNING 0x00          // Jet bewegt sich und sendet Daten
#define CMD_IDLE 0x01             // Jet geht in den IDLE Mode
#define CMD_PLATFORM_DETECTED 0x02  // Jet war im IDLE und hat detected, dass er auf der Plattform steht
#define CMD_CALIBRATE_MAG 0x03    // Jet geht in den Kalibrierungsmodus

#define CALIB_START_MAG_CALIBRATION 0x30  // Start sending mag data for calibration
#define CALIB_STOP_MAG_CALIBRATION 0x31   // Stop sending mag data for calibration
#define CALIB_SEND_CALIBRATION_MATRIX 0x32 // Receive calibration matrix

uint8_t cmd;

enum State {
  RUNNING,
  CALIBRATING,
  DISCONNECTED,
  IDLE,
  PLATFORM
};

State currentState = DISCONNECTED;

// Single Sensor Sample (32 Bytes)
struct SensorValues {
  uint32_t timestamp_ms;        // in ms
  float accelX, accelY, accelZ; // in m/s
  float q0, q1, q2, q3;         // in Grad
  float magX, magY, magZ;       // in Tesla    
  float tempC;                  // in Celsius
} __attribute__((packed));

// Sendet nur Magnetometer Daten für Kalibrierung
struct MagnoValues {
  float mx, my, mz;             // in Tesla    
} __attribute__((packed));

// === Globals === 
SensorValues sensorData;
MagnoValues magValues;
unsigned long lastSendTime;
float tempC;
int idlecount = 0;
static const int idleCountMax = 100;
bool idle = false;
bool calibrating = false;

// Calibration matrix (3x3, stored as 9 floats)
float calibrationMatrix[3][3] = {
  {1.0, 0.0, 0.0},
  {0.0, 1.0, 0.0},
  {0.0, 0.0, 1.0}
};
 
// BLE Service erstellen
BLEService myService("19B10000-E8F2-537E-4F6C-D104768A1214");

// === SensorManager ===
RotationManager sensor;
Quaternion _Quaternion;
float ax = 0, ay = 0, az = 0;
float mx = 0, my = 0, mz = 0;

// === SmoothData4D ===
SmoothQuaternionData smoothing;  

// === BLE Characteristics ===
BLECharacteristic dataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLEWrite | BLENotify, sizeof(SensorValues));

BLECharacteristic magCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLENotify, sizeof(MagnoValues));

BLECharacteristic statusCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLEWrite | BLENotify, 1);

BLECharacteristic commandCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214",
                                   BLEWrite, 1);
 
void setup() {
  Serial.begin(115200);
  // while (!Serial);
 
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
 
  BLE.setConnectionInterval(6, 6);
  BLE.setAdvertisingInterval(160);
  BLE.setLocalName("Eurofighter");
  BLE.setDeviceName("Eurofighter");
 
  BLE.setAdvertisedService(myService);
 
  myService.addCharacteristic(dataCharacteristic);
  myService.addCharacteristic(magCharacteristic);
  myService.addCharacteristic(statusCharacteristic);
  myService.addCharacteristic(commandCharacteristic);
 
  BLE.addService(myService);
 
  dataCharacteristic.writeValue("Ready");
  
  // Set event handler for incoming commands/calibration data
  commandCharacteristic.setEventHandler(BLEWritten, commandCharacteristicReceived);
 
  BLE.advertise();
 
  Serial.println("Bluetooth device active, waiting for connections...");

  sensor.init(FREQUENCY);
  smoothing.initSmoothing(FREQUENCY, 0.6f);
}

void commandCharacteristicReceived(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t* data = (uint8_t*)commandCharacteristic.value();
  uint8_t dataLen = commandCharacteristic.valueLength();
  
  if(dataLen < 1) return;
  
  uint8_t receivedCmd = data[0];
  
  if(Serial) {
    Serial.print("Received command: 0x");
    Serial.println(receivedCmd, HEX);
  }
  
  switch(receivedCmd) {
    // === Calibration Commands ===
    case CALIB_START_MAG_CALIBRATION:
      if(Serial) Serial.println("→ Starting magnetometer calibration mode");
      calibrating = true;
      break;
      
    case CALIB_STOP_MAG_CALIBRATION:
      if(Serial) Serial.println("→ Stopping magnetometer calibration mode");
      calibrating = false;
      break;
      
    case CALIB_SEND_CALIBRATION_MATRIX:
      // Receive calibration matrix (3x3 = 9 floats = 36 bytes)
      if(dataLen >= 37) { // 1 byte cmd + 36 bytes matrix
        if(Serial) Serial.println("→ Receiving calibration matrix...");
        
        // Extract calibration matrix from data
        float* matrixPtr = (float*)&data[1];
        for(int i = 0; i < 3; i++) {
          for(int j = 0; j < 3; j++) {
            calibrationMatrix[i][j] = matrixPtr[i * 3 + j];
          }
        }
        
        if(Serial) {
          Serial.println("✓ Calibration matrix received and stored:");
          for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
              Serial.print(calibrationMatrix[i][j]);
              Serial.print("\t");
            }
            Serial.println();
          }
        }
      }
      break;
      
    // === Table Feedback Commands ===
    case 0x10: // Magnet activated
      if(Serial) Serial.println("Turning table magnet activated");
      break;
    case 0x11: // Magnet deactivated
      if(Serial) Serial.println("Turning table magnet deactivated");
      break;
    case 0x12: // Rotation started
      if(Serial) Serial.println("Turning table rotation started");
      break;
    case 0x13: // Rotation stopped
      if(Serial) Serial.println("Turning table rotation stopped");
      break;
  }
}
 
void loop() {
  BLEDevice central = BLE.central();
  BLE.poll();

  switch(currentState) {
    // ==== RECONNECTING STATE ====
    case DISCONNECTED:
      if (central && central.connected()) {
        if(Serial) Serial.print("Connected to central: ");
        if(Serial) Serial.println(central.address());
        currentState = RUNNING;
        lastSendTime = millis();
      }
      break;
    
    // ==== MAGNETOMETER KALIBRIERUNG ====
    case CALIBRATING:
      if(!(millis() - lastSendTime >= 1000 / MAG_FREQUENCY)) {
        break;
      }
      if(central.connected() == false) {
        break;
      } 
      if(!IMU.magneticFieldAvailable()) {
        break;
      }
      lastSendTime = millis();
      IMU.readMagneticField(mx, my, mz);
      magValues = {mx, my, mz};
      // Send magnetometer data
      magCharacteristic.writeValue((byte*)&magValues, sizeof(float)*3);
      break;

    // =====================
    // ===== MAIN LOOP =====
    // =====================
    case RUNNING:
      if(central.connected() == false) {
        if(Serial) Serial.print("Disconnected from central: ");
        if(Serial) Serial.println(central.address());
        currentState = DISCONNECTED;
        idlecount = 0;
        calibrating = false;
        break;
      }
      
      // Check if calibration was requested
      if(calibrating) {
        currentState = CALIBRATING;
        lastSendTime = millis();
        if(Serial) Serial.println("State: CALIBRATING");
        break;
      }
      
      if(!(millis() - lastSendTime >= 1000 / FREQUENCY)) {
        break;
      }
      
      float gyroMag;
      lastSendTime = millis();
      sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
      sensor.getTemperature(tempC);

      // SensorData Struct mit aktuellen Werten füllen
      sensorData.timestamp_ms = millis();
      sensorData.accelX = ax;
      sensorData.accelY = ay;
      sensorData.accelZ = az;
      sensorData.q0 = _Quaternion.q0;
      sensorData.q1 = _Quaternion.q1;
      sensorData.q2 = _Quaternion.q2;
      sensorData.q3 = _Quaternion.q3;
      sensorData.magX = mx;
      sensorData.magY = my;
      sensorData.magZ = mz;
      sensorData.tempC = tempC;
      
      dataCharacteristic.writeValue((byte*)&sensorData, sizeof(sensorData));

      // IDLE Detection
      float accelMag = sqrt(ax*ax + ay*ay + (az-1)*(az-1));
      if(accelMag < 0.03f && gyroMag < 10.0f) {
        idlecount++;
        if(idlecount > idleCountMax){
          currentState = IDLE;
          if(Serial) Serial.println("State: IDLE");
        }
      } else {
        idlecount = 0;
      }
      break;

    case IDLE:
      if(!idle){ 
        statusCharacteristic.writeValue(&CMD_IDLE, 1);
        idle = true;
        if(Serial) Serial.println("Sent: CMD_IDLE");
      }

      sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
      float accelMag = sqrt(ax*ax + ay*ay + az*az);
      float magnetMag = sqrt(mx*mx + my*my + mz*mz);

      if((accelMag - 1) > 0.15f || gyroMag > 10.0f){
        cmd = CMD_RUNNING;
        statusCharacteristic.writeValue(&cmd, 1);
        idle = false;
        idlecount = 0;
        currentState = RUNNING;
        if(Serial) Serial.println("State: RUNNING (movement detected)");
      }

      if((magnetMag - 48) > 25){
        cmd = CMD_PLATFORM_DETECTED;
        statusCharacteristic.writeValue(&cmd, 1);
        currentState = PLATFORM;
        if(Serial) Serial.println("State: PLATFORM (magnet detected)");
        if(Serial) {
          Serial.print("Magnetic field magnitude: ");
          Serial.println(magnetMag);
        }
      }
      break;

    case PLATFORM:
      sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
      accelMag = sqrt(ax*ax + ay*ay + az*az);
      
      if((accelMag - 1) > 0.2f || gyroMag > 15.0f){
        cmd = CMD_RUNNING;
        statusCharacteristic.writeValue(&cmd, 1);
        idle = false;
        idlecount = 0;
        currentState = RUNNING;
        if(Serial) Serial.println("State: RUNNING (jet removed from platform)");
      }
      break;
  }
}
