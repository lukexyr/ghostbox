#include <ArduinoBLE.h>
#include "RotationManager.h"
#include "SmoothData4D.h"

#define FREQUENCY 25.0f // Hz       Frequenz mit der Sensrodaten gemessen/gesendet werden
#define MAG_FREQUENCY 19.0f // Hz   Frequenz für Magnetometer Kalibrierung

#define CMD_RUNNING 0x00          // Jet bewegt sich und sendet Daten
#define CMD_IDLE 0x01             // Jet geht in den IDLE Mode
#define CMD_PLATFORM_DETECTED 0x02  // Jet war im IDLE und hat detected, dass er auf der Plattform steht
#define CMD_CALIBRATE_MAG 0x03    // Jet geht in den Kalibrierungsmodus
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
// Sendet alle Sensordaten in einem struct via BLE
struct SensorValues {
  uint32_t timestamp_ms;        // in ms
  float accelX, accelY, accelZ; // in m/s
  float q0, q1, q2, q3;         // in Grad
  float magX, magY, magZ;       // in Tesla    
  float tempC;                  // in Celsius
} __attribute__((packed));      // Verhindert Padding

// Sendet nur Magnetometer Daten für Kalibrierung
struct MagnoValues {
  float mx, my, mz;             // in Tesla    
} __attribute__((packed));      // Verhindert Padding

// === Globals === 
SensorValues sensorData;
MagnoValues magValues;
unsigned long lastSendTime;
float tempC;
int idlecount = 0;
static const int idleCountMax = 100;
bool idle = false;
 
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
                                  // Sensorwerte 
BLECharacteristic dataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLEWrite | BLENotify, sizeof(SensorValues));

                                   // Magnetometer Werte
BLECharacteristic magCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLENotify, sizeof(MagnoValues));

                                   // Commands (Idle, Platform, Kalibrierung)
BLECharacteristic controlCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLEWrite | BLENotify, 1); // Steuerungs-Charakteristik

                                   // Receive commands from Raspberry Pi
BLECharacteristic rxCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214",
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
  // Set advertised local name and service
  BLE.setLocalName("Eurofighter");
  BLE.setDeviceName("Eurofighter");
 
  BLE.setAdvertisedService(myService);
 
  // Add characteristic to service
  myService.addCharacteristic(dataCharacteristic);
  myService.addCharacteristic(magCharacteristic);
  myService.addCharacteristic(controlCharacteristic);
  myService.addCharacteristic(rxCharacteristic);
 
  // Add service
  BLE.addService(myService);
 
  // Set initial value
  dataCharacteristic.writeValue("Ready");

  // Set event handler for incoming commands
  rxCharacteristic.setEventHandler(BLEWritten, onRxCharacteristicWritten);
 
  // Start advertising
  BLE.advertise();
 
  Serial.println("Bluetooth device active, waiting for connections...");

  sensor.init(FREQUENCY);

  // Initialisiere Smoothing mit Target-Frequenz und Alpha-Wert
  smoothing.initSmoothing(FREQUENCY, 0.6f);
}

void onRxCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t receivedCmd = rxCharacteristic.value()[0];
  
  if(Serial) {
    Serial.print("Received command: 0x");
    Serial.println(receivedCmd, HEX);
  }
  
  // Handle incoming commands from Raspberry Pi / Turning Table
  switch(receivedCmd) {
    case 0x10: // Magnet activated confirmation
      if(Serial) Serial.println("Turning table magnet activated");
      break;
    case 0x11: // Rotation started confirmation
      if(Serial) Serial.println("Turning table rotation started");
      break;
    case 0x12: // Rotation stopped confirmation
      if(Serial) Serial.println("Turning table rotation stopped");
      break;
  }
}
 
void loop() {
  // Central (Raspberry) verbindet sich dann mit dem Nano
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
      // Sende nur Magnetometer Daten
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
        break;
      }
      if(!(millis() - lastSendTime >= 1000 / FREQUENCY)) {
        break;
      }
      float gyroMag;
      lastSendTime = millis();
      sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
      sensor.getTemperature(tempC);

      //smoothing.smooth(_Quaternion);

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
          // Go IDLE
          currentState = IDLE;
          if(Serial) Serial.println("State: IDLE");
        }
      } else {
        idlecount = 0;
      }
      break;

    case IDLE:
      if(!idle){ 
        controlCharacteristic.writeValue(&CMD_IDLE, 1);
        idle = true;
        if(Serial) Serial.println("Sent: CMD_IDLE");
      }

      sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
      float accelMag = sqrt(ax*ax + ay*ay + az*az);
      float magnetMag = sqrt(mx*mx + my*my + mz*mz);

      if((accelMag - 1) > 0.15f || gyroMag > 10.0f){
        // Bewegung erkannt -> gehe zu state RUNNING
        cmd = CMD_RUNNING;
        controlCharacteristic.writeValue(&cmd, 1);
        idle = false;
        idlecount = 0;
        currentState = RUNNING;
        if(Serial) Serial.println("State: RUNNING (movement detected)");
      }

      if((magnetMag - 48) > 25){
        // Elektromagnet auf der Platform erkannt -> gehe zu state PLATFORM
        cmd = CMD_PLATFORM_DETECTED;
        controlCharacteristic.writeValue(&cmd, 1);
        currentState = PLATFORM;
        if(Serial) Serial.println("State: PLATFORM (magnet detected)");
        if(Serial) {
          Serial.print("Magnetic field magnitude: ");
          Serial.println(magnetMag);
        }
      }

      break;

    case PLATFORM:
      // Jet is on platform, wait for commands or movement
      sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
      accelMag = sqrt(ax*ax + ay*ay + az*az);
      
      if((accelMag - 1) > 0.2f || gyroMag > 15.0f){
        // Jet removed from platform
        cmd = CMD_RUNNING;
        controlCharacteristic.writeValue(&cmd, 1);
        idle = false;
        idlecount = 0;
        currentState = RUNNING;
        if(Serial) Serial.println("State: RUNNING (jet removed from platform)");
      }
      break;
  }
}