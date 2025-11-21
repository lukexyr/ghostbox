#include <ArduinoBLE.h>
#include "RotationManager.h"
#include "SmoothData4D.h"

#define FREQUENCY 25.0f
#define MAG_FREQUENCY 19.0f

// BLE Command Bytes
const uint8_t CMD_RUNNING = 0x00;
const uint8_t CMD_IDLE = 0x01;
const uint8_t CMD_PLATFORM_DETECTED = 0x02;
const uint8_t CMD_CALIBRATE_MAG = 0x03;

// Thresholds for idle detection
const float ACCEL_IDLE_THRESHOLD = 0.03f;
const float GYRO_IDLE_THRESHOLD = 10.0f;
const int IDLE_COUNT_MAX = 100;

// Thresholds for platform detection (magnetometer)
const float MAG_BASELINE = 48.0f;
const float MAG_DETECT_THRESHOLD = 25.0f;

// Threshold for movement detection from platform
const float PLATFORM_ACCEL_THRESHOLD = 0.05f;
const float PLATFORM_GYRO_THRESHOLD = 200.0f;

enum State {
  RUNNING,
  CALIBRATING,
  DISCONNECTED,
  IDLE,
  PLATFORM
};

State currentState = DISCONNECTED;
State previousState = DISCONNECTED;

// Sensor data structure
struct SensorValues {
  uint32_t timestamp_ms;
  float accelX, accelY, accelZ;
  float q0, q1, q2, q3;
  float magX, magY, magZ;
  float tempC;
} __attribute__((packed));

struct MagnoValues {
  float mx, my, mz;
} __attribute__((packed));

// Globals
SensorValues sensorData;
MagnoValues magValues;
unsigned long lastSendTime = 0;
unsigned long lastStateChangeTime = 0;
float tempC;
int idlecount = 0;
float accelMag = 0.0f;
float magnetMag = 0.0f;
float gyroMag = 0.0f;

// BLE Service
BLEService myService("19B10000-E8F2-537E-4F6C-D104768A1214");

// Sensor objects
RotationManager sensor;
Quaternion _Quaternion;
float ax = 0, ay = 0, az = 0;
float mx = 0, my = 0, mz = 0;

// Smoothing
SmoothQuaternionData smoothing;

// BLE Characteristics
BLECharacteristic dataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLENotify, sizeof(SensorValues));

BLECharacteristic magCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLENotify, sizeof(MagnoValues));

BLECharacteristic controlCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLENotify, 1);

void setup() {
  Serial.begin(115200);

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
  myService.addCharacteristic(controlCharacteristic);

  BLE.addService(myService);

  // Start advertising
  BLE.advertise();

  Serial.println("Eurofighter ready, waiting for connections...");

  sensor.init(FREQUENCY);
  smoothing.initSmoothing(FREQUENCY, 0.7f);

  lastStateChangeTime = millis();
}

void changeState(State newState) {
  if(currentState != newState) {
    previousState = currentState;
    currentState = newState;
    lastStateChangeTime = millis();
    
    if(Serial) {
      Serial.print("State: ");
      switch(newState) {
        case RUNNING: Serial.println("RUNNING"); break;
        case CALIBRATING: Serial.println("CALIBRATING"); break;
        case DISCONNECTED: Serial.println("DISCONNECTED"); break;
        case IDLE: Serial.println("IDLE"); break;
        case PLATFORM: Serial.println("PLATFORM"); break;
      }
    }
  }
}

void sendControlCommand(uint8_t cmd) {
  controlCharacteristic.writeValue(&cmd, 1);
  if(Serial) {
    Serial.print("→ Sent control: 0x");
    Serial.println(cmd, HEX);
  }
}

void loop() {
  BLEDevice central = BLE.central();
  BLE.poll();

  // Connection state management
  if(currentState != DISCONNECTED && (!central || !central.connected())) {
    changeState(DISCONNECTED);
    idlecount = 0;
  }

  switch(currentState) {
    // ===== DISCONNECTED =====
    case DISCONNECTED:
      if (central && central.connected()) {
        if(Serial) {
          Serial.print("✓ Connected to central: ");
          Serial.println(central.address());
        }
        changeState(RUNNING);
        lastSendTime = millis();
        idlecount = 0;
      }
      break;

    // ===== MAGNETOMETER CALIBRATION =====
    case CALIBRATING:
      if(millis() - lastSendTime < 1000 / MAG_FREQUENCY) break;
      if(!central.connected()) break;
      if(!IMU.magneticFieldAvailable()) break;

      lastSendTime = millis();
      IMU.readMagneticField(mx, my, mz);
      magValues = {mx, my, mz};
      magCharacteristic.writeValue((byte*)&magValues, sizeof(float)*3);
      break;

    // ===== RUNNING (MAIN LOOP) =====
    case RUNNING:
      if(!central.connected()) break;
      if(millis() - lastSendTime < 1000 / FREQUENCY) break;

      lastSendTime = millis();
      
      // Get sensor data
      sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
      sensor.getTemperature(tempC);
      smoothing.smoothQuaternion(_Quaternion, millis());

      // Fill and send sensor data
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

      // IDLE detection
      accelMag = sqrt(ax*ax + ay*ay + (az-1)*(az-1));
      if(accelMag < ACCEL_IDLE_THRESHOLD && gyroMag < GYRO_IDLE_THRESHOLD) {
        idlecount++;
        if(idlecount > IDLE_COUNT_MAX) {
          changeState(IDLE);
          sendControlCommand(CMD_IDLE);
          idlecount = 0;
        }
      } else {
        idlecount = 0;
      }
      break;

    // ===== IDLE =====
    case IDLE:
      if(!central.connected()) break;

      // Get current sensor readings
      sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
      accelMag = sqrt(ax*ax + ay*ay + az*az);
      magnetMag = sqrt(mx*mx + my*my + mz*mz);

      // Movement detection -> back to RUNNING
      if(abs(accelMag - 1.0f) > ACCEL_IDLE_THRESHOLD || gyroMag > GYRO_IDLE_THRESHOLD) {
        changeState(RUNNING);
        sendControlCommand(CMD_RUNNING);
        idlecount = 0;
        break;
      }

      // Platform detection (strong magnetic field)
      if(abs(magnetMag - MAG_BASELINE) > MAG_DETECT_THRESHOLD) {
        changeState(PLATFORM);
        sendControlCommand(CMD_PLATFORM_DETECTED);
        if(Serial) {
          Serial.print("Platform detected! Mag: ");
          Serial.println(magnetMag);
        }
      }
      break;

    // ===== PLATFORM =====
    case PLATFORM:
      if(!central.connected()) break;

      // Monitor for pickup/movement
      sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
      accelMag = sqrt(ax*ax + ay*ay + az*az);

      // Stronger thresholds for platform detection
      if(abs(accelMag - 1.0f) > PLATFORM_ACCEL_THRESHOLD || gyroMag > PLATFORM_GYRO_THRESHOLD) {
        changeState(RUNNING);
        sendControlCommand(CMD_RUNNING);
        idlecount = 0;
        if(Serial) Serial.println("Movement detected - leaving platform");
      }
      break;
  }
}
