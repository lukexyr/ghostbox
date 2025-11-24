# BLE Central Control System - Complete Documentation

## 1. Project Overview

This project implements a Bluetooth Low Energy (BLE) communication system that coordinates control between two Arduino devices (a jet and a turning table) via a Raspberry Pi central controller. The system performs sensor data collection, device state management, auto-reconnection, and magnetometer calibration workflows.

### Key Components
- **Jet Arduino** (Eurofighter): Collects acceleration, gyroscope, and magnetic field data, detects motion/idle states, and performs magnetometer calibration
- **Turning Table Arduino**: Controls an electromagnet and stepper motor, manages rotation, and triggers calibration workflows
- **Raspberry Pi (Central Controller)**: Orchestrates communication between devices, manages system workflows, and sends data to an local server

---

## 2. System Architecture

### 2.1 Communication Protocol

The system uses BLE GATT (Generic Attribute Profile) with services and characteristics. Each device advertises a service containing multiple characteristics for bidirectional communication.

**Jet Device Service** (`19B10000-E8F2-537E-4F6C-D104768A1214`)
- `19B10001...` - Data Characteristic: Sends 48-byte sensor packets (timestamp, acceleration, rotation quaternion, magnetometer, temperature)
- `19B10002...` - Magnetometer Characteristic: Sends raw magnetometer readings during calibration
- `19B10003...` - Status Characteristic: Sends status updates (IDLE, RUNNING, PLATFORM_DETECTED)
- `19B10004...` - Command Characteristic: Receives commands (calibration start/stop, calibration matrix)

**Turning Table Service** (`19B20000-E8F2-537E-4F6C-D104768A1215`)
- `19B20001...` - Status Characteristic: Sends table status (magnet on/off, rotating/stopped)
- `19B20002...` - Command Characteristic: Receives motor/magnet control commands
- `19B20003...` - Button Characteristic: Sends button press notifications (triggers calibration)

### 2.2 Data Structures

**Jet Sensor Data Packet (48 bytes, sent at 25 Hz)**
Contains all sensor readings in a single efficient package:
```cpp
struct SensorValues {
  uint32_t timestamp_ms;        // Milliseconds since boot (4 bytes)
  float accelX, accelY, accelZ; // Linear acceleration in m/s² (12 bytes)
  float q0, q1, q2, q3;         // Quaternion rotation (16 bytes)
  float magX, magY, magZ;       // Magnetic field in Tesla (12 bytes)
  float tempC;                  // Temperature in Celsius (4 bytes)
} __attribute__((packed));
```
Using `__attribute__((packed))` ensures no padding is added between fields, keeping it exactly 48 bytes for transmission.

**Magnetometer Calibration Data (12 bytes, sent at 19 Hz during calibration)**
```cpp
struct MagnoValues {
  float mx, my, mz;  // Raw magnetic field readings
} __attribute__((packed));
```

**Calibration Matrix (36 bytes)**
```cpp
float calibrationMatrix[3][3] = {
  {1.0, 0.0, 0.0},
  {0.0, 1.0, 0.0},
  {0.0, 0.0, 1.0}
};
// Packed as 9 floats for transmission
```

---

## 3. Jet Device (Eurofighter Arduino)

### 3.1 Responsibilities

The Jet Arduino is the sensor hub of the system. It continuously reads from an IMU (Inertial Measurement Unit) that provides acceleration, rotation, and magnetic field data. Based on this sensor data, it performs calculations, makes intelligent decisions about its state, and communicates these state changes to the central controller via BLE notifications.

### 3.2 Sensor Data Flow

The sensor data collection happens at a fixed frequency with automatic state detection:

#### 1. **Sensor Reading at 25 Hz**: The RotationManager library fetches raw data from the IMU sensors
   ```cpp
   #define FREQUENCY 25.0f // Hz - how often we read sensors
   
   // In main loop:
   if(!(millis() - lastSendTime >= 1000 / FREQUENCY)) {
     break;  // Not time to send yet
   }
   lastSendTime = millis();
   
   // Read all sensor data at once
   sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
   sensor.getTemperature(tempC);
   ```

#### 2. **Data Transmission**: Every 40ms, sensor packets are broadcast via the Data Characteristic
   ```cpp
   // Pack all sensor data into struct
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
   
   // Send as binary packet over BLE
   dataCharacteristic.writeValue((byte*)&sensorData, sizeof(sensorData));
   ```

3. **State Detection**: The Arduino analyzes acceleration and gyroscope data to determine state:
   - **RUNNING**: Device is moving (high acceleration or rotation)
   - **IDLE**: Device is stationary (low acceleration < 0.03 m/s² AND low rotation < 10°/s)
   - **PLATFORM**: Magnet is nearby (magnetic field magnitude increases significantly)

### 3.3 State Machine

```
DISCONNECTED → RUNNING → IDLE → PLATFORM → RUNNING
```

Each state has specific behavior and transition conditions:

**DISCONNECTED State** - Waits for BLE connection
```cpp
case DISCONNECTED:
  if (central && central.connected()) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    currentState = RUNNING;  // Move to RUNNING once connected
    lastSendTime = millis();
  }
  break;
```

**RUNNING State** - Collects and sends sensor data continuously.
This is the normal operating mode. The device reads sensors and broadcasts data. It also monitors for the idle condition (stationary device).
```cpp
case RUNNING:
  if(central.connected() == false) {
    currentState = DISCONNECTED;  // Disconnect detected
    idlecount = 0;
    calibrating = false;
    break;
  }
  
  // Send data at fixed frequency
  if(!(millis() - lastSendTime >= 1000 / FREQUENCY)) {
    break;
  }
  lastSendTime = millis();
  
  // Get and send sensor data (code shown above)
  
  // IDLE Detection - accumulate samples meeting idle criteria
  float accelMag = sqrt(ax*ax + ay*ay + (az-1)*(az-1));
  if(accelMag < 0.03f && gyroMag < 10.0f) {
    idlecount++;
    if(idlecount > idleCountMax){  // 100 samples of idle = ~4 seconds at 25 Hz
      currentState = IDLE;
      Serial.println("State: IDLE");
    }
  } else {
    idlecount = 0;  // Reset counter on motion
  }
  break;
```

**IDLE State** - Device is stationary, notifies central and waits for input
```cpp
case IDLE:
  // Send IDLE status once (not repeatedly)
  if(!idle) {
    controlCharacteristic.writeValue(&CMD_IDLE, 1);
    idle = true;
    Serial.println("Sent: CMD_IDLE");
  }
  
  // Still read sensors to detect motion or magnet
  sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
  
  // Exit IDLE if motion detected (return to RUNNING)
  float accelMag = sqrt(ax*ax + ay*ay + az*az);
  if((accelMag - 1) > 0.15f || gyroMag > 10.0f) {
    cmd = CMD_RUNNING;
    controlCharacteristic.writeValue(&cmd, 1);
    idle = false;
    idlecount = 0;
    currentState = RUNNING;
    Serial.println("State: RUNNING (movement detected)");
  }
  
  // Detect if placed on magnet (transition to PLATFORM)
  float magnetMag = sqrt(mx*mx + my*my + mz*mz);
  if((magnetMag - 48) > 25) {  // Magnet field threshold
    cmd = CMD_PLATFORM_DETECTED;
    controlCharacteristic.writeValue(&cmd, 1);
    currentState = PLATFORM;
    Serial.println("State: PLATFORM (magnet detected)");
  }
  break;
```

**PLATFORM State** - Magnet detected while idle, monitor for removal
```cpp
case PLATFORM:
  sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz, gyroMag);
  accelMag = sqrt(ax*ax + ay*ay + az*az);
  
  // If moved significantly, exit platform
  if((accelMag - 1) > 0.2f || gyroMag > 15.0f) {
    cmd = CMD_RUNNING;
    controlCharacteristic.writeValue(&cmd, 1);
    idle = false;
    idlecount = 0;
    currentState = RUNNING;
    Serial.println("State: RUNNING (jet removed from platform)");
  }
  break;
```

### 3.4 Command Handling

The Jet Arduino receives commands via the command characteristic. When the central controller needs calibration data or to update the calibration matrix, it sends commands that change the device behavior:

```cpp
void commandCharacteristicReceive(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t* data = (uint8_t*)commandCharacteristic.value();
  uint8_t dataLen = commandCharacteristic.valueLength();
  
  if(dataLen < 1) return;
  
  uint8_t receivedCmd = data[0];
  Serial.print("Received command: 0x");
  Serial.println(receivedCmd, HEX);
  
  switch(receivedCmd) {
    case CALIB_START_MAG_CALIBRATION:
      Serial.println("→ Starting magnetometer calibration mode");
      calibrating = true;  // This flag triggers CALIBRATING state in main loop
      break;
      
    case CALIB_STOP_MAG_CALIBRATION:
      Serial.println("→ Stopping magnetometer calibration mode");
      calibrating = false;
      break;
      
    case CALIB_SEND_CALIBRATION_MATRIX:
      // Receive 3x3 matrix = 9 floats = 36 bytes
      if(dataLen >= 37) {  // 1 byte cmd + 36 bytes matrix
        Serial.println("→ Receiving calibration matrix...");
        
        // Extract calibration matrix from data
        float* matrixPtr = (float*)&data[1];
        for(int i = 0; i < 3; i++) {
          for(int j = 0; j < 3; j++) {
            calibrationMatrix[i][j] = matrixPtr[i * 3 + j];
          }
        }
        
        Serial.println("✓ Calibration matrix stored");
      }
      break;
  }
}
```

| Command | Value | Action | Purpose |
|---------|-------|--------|---------|
| START_MAG_CALIBRATION | 0x30 | Enter CALIBRATING state, send mag data at 19 Hz | Collect magnetic field samples |
| STOP_MAG_CALIBRATION | 0x31 | Exit CALIBRATING state, return to RUNNING | End data collection |
| SEND_CALIBRATION_MATRIX | 0x32 | Receive 3x3 matrix, store for future use | Apply corrections to future readings |

### 3.5 Calibration Mode

When calibration starts, the device switches to a special mode that only sends magnetometer data at a different frequency:

```cpp
#define MAG_FREQUENCY 19.0f // Hz - slower rate for stable calibration
```

**CALIBRATING State** - Special mode during calibration
```cpp
case CALIBRATING:
  if(!(millis() - lastSendTime >= 1000 / MAG_FREQUENCY)) {
    break;  // Not time to send yet
  }
  if(central.connected() == false) {
    break;  // Connection lost
  }
  if(!IMU.magneticFieldAvailable()) {
    break;  // Sensor not ready
  }
  lastSendTime = millis();
  
  // Read ONLY magnetometer data (not accelerometer or gyroscope)
  IMU.readMagneticField(mx, my, mz);
  magValues = {mx, my, mz};
  
  // Send magnetometer data at 19 Hz
  magCharacteristic.writeValue((byte*)&magValues, sizeof(float)*3);
  break;
```

The central controller collects these samples for 30 seconds, then calculates corrections and sends them back.

---

## 4. Turning Table Device (TurningTable Arduino)

### 4.1 Responsibilities

The Turning Table Arduino is the hardware controller. It manages two physical devices: an electromagnet and a stepper motor. It also monitors a calibration button that can be pressed to trigger the calibration workflow. All commands come from the central controller via BLE.

### 4.2 Hardware Control

The device uses specific pins to control the electromagnet and stepper motor:

```cpp
#define ELECTROMAGNET_PIN 2       // Pin 2: Relay control (HIGH = ON)
#define MOTOR_ENABLE_PIN 3        // Pin 3: Enable motor
#define MOTOR_DIR_PIN 4           // Pin 4: Direction control
#define MOTOR_STEP_PIN 5          // Pin 5: Step signal (toggle = 1 step)
#define CALIBRATION_BUTTON_PIN 6  // Pin 6: Button input (LOW when pressed)

void setup() {
  Serial.begin(115200);
  
  // Configure pins as outputs or inputs
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);  // Internal pullup - LOW when pressed
  pinMode(LED_BUILTIN, OUTPUT);

  // Initial pin states - everything OFF
  digitalWrite(ELECTROMAGNET_PIN, LOW);      // Magnet OFF
  digitalWrite(MOTOR_ENABLE_PIN, LOW);       // Motor disabled (LOW = disabled on this design)
  digitalWrite(MOTOR_DIR_PIN, HIGH);         // Direction set to forward
}
```

### 4.3 Operating States

The turning table manages its own state independently and communicates it back to the central controller:

```cpp
enum TableState {
  DISCONNECTED,   // No BLE connection - waiting for central
  READY,          // Connected, idle, ready for commands
  MAGNET_ACTIVE,  // Electromagnet is activated
  ROTATING        // Motor is running
};

TableState currentState = DISCONNECTED;
```

| State | Description | Hardware Status | Next States |
|-------|-------------|-----------------|-------------|
| DISCONNECTED | No BLE connection | Everything OFF | → READY (on connect) |
| READY | Connected and idle | Everything OFF | → MAGNET_ACTIVE (cmd) |
| MAGNET_ACTIVE | Magnet energized | Magnet ON, Motor OFF | → ROTATING (cmd) or → READY (cmd) |
| ROTATING | Motor stepping | Magnet OFF, Motor ON | → READY (stop all) |

### 4.4 Magnet Control

When a command arrives, the system activates or deactivates it and notifies the central controller:

```cpp
void activateMagnet() {
  if(!magnetActive) {
    digitalWrite(ELECTROMAGNET_PIN, HIGH);  // Set pin HIGH to activate relay
    digitalWrite(LED_BUILTIN, HIGH);         // Visual feedback on board LED
    magnetActive = true;
    currentState = MAGNET_ACTIVE;
    
    // Send status update to central controller
    uint8_t status = STATUS_MAGNET_ON;
    statusCharacteristic.writeValue(&status, 1);
    
    Serial.println("Electromagnet activated");
  }
}

void deactivateMagnet() {
  if(magnetActive) {
    digitalWrite(ELECTROMAGNET_PIN, LOW);   // Set pin LOW to deactivate relay
    digitalWrite(LED_BUILTIN, LOW);          // Turn off LED
    magnetActive = false;
    
    // Update state - if motor not running, go to READY
    if(!isRotating) {
      currentState = READY;
    }
    
    // Send status update to central controller
    uint8_t status = STATUS_MAGNET_OFF;
    statusCharacteristic.writeValue(&status, 1);
    
    Serial.println("Electromagnet deactivated");
  }
}
```

### 4.5 Motor Control

The stepper motor is controlled by toggling a step pin at precise intervals. Speed is controlled by adjusting the delay between step toggles:

```cpp
void startRotation() {
  if(!isRotating) {
    digitalWrite(MOTOR_ENABLE_PIN, LOW);  // Enable motor (LOW = enabled)
    isRotating = true;
    rotationStartTime = millis();
    lastStepTime = micros();  // Use microseconds for precise timing
    currentState = ROTATING;
    
    // Map speed 0-100 to step delay
    // Higher speed = shorter delay = more steps per second = faster rotation
    stepDelay = map(rotationSpeed, 0, 100, 5000, 500);  // 5000µs at 0%, 500µs at 100%
    
    uint8_t status = STATUS_ROTATING;
    statusCharacteristic.writeValue(&status, 1);
    
    Serial.println("Rotation started");
  }
}

void stopRotation() {
  if(isRotating) {
    digitalWrite(MOTOR_ENABLE_PIN, LOW);   // Disable motor
    digitalWrite(MOTOR_STEP_PIN, LOW);     // Stop stepping
    isRotating = false;
    
    if(!magnetActive) {
      currentState = READY;
    } else {
      currentState = MAGNET_ACTIVE;
    }
    
    uint8_t status = STATUS_STOPPED;
    statusCharacteristic.writeValue(&status, 1);
    
    Serial.println("Rotation stopped");
  }
}
```

**Motor Stepping Mechanism**

Each step of the stepper motor occurs when the step pin is toggled (goes from LOW to HIGH or HIGH to LOW). By toggling it repeatedly at precise intervals, we control the speed:

```cpp
void updateRotation() {
  if(!isRotating) return;  // Only if motor is running
  
  unsigned long currentMicros = micros();
  
  // Check if enough time has passed for the next step
  if(currentMicros - lastStepTime >= stepDelay) {
    // Toggle the step pin - one toggle = one step
    // This is called every loop() iteration
    digitalWrite(MOTOR_STEP_PIN, !digitalRead(MOTOR_STEP_PIN));
    lastStepTime = currentMicros;
  }
}
```

Speed calculation:
- At 100% (stepDelay = 500 µs): Toggle every 500 µs = 2000 toggles/second = 1000 steps/second
- At 50% (stepDelay = 2750 µs): Toggle every 2.75 ms = 364 toggles/second = 182 steps/second
- At 0% (stepDelay = 5000 µs): Toggle every 5 ms = 200 toggles/second = 100 steps/second

### 4.6 Command Reception

When the central controller sends commands, they arrive via the commandCharacteristic:

```cpp
void onCommandReceived(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t cmd = commandCharacteristic.value()[0];
  
  Serial.print("Received command: 0x");
  Serial.println(cmd, HEX);

  switch(cmd) {
    case CMD_ACTIVATE_MAGNET:      // 0x10
      activateMagnet();
      break;
      
    case CMD_DEACTIVATE_MAGNET:    // 0x11
      deactivateMagnet();
      break;
      
    case CMD_START_ROTATION:       // 0x12
      startRotation();
      break;
      
    case CMD_STOP_ROTATION:        // 0x13
      stopRotation();
      break;
  }
}
```

### 4.7 Calibration Button Workflow

The calibration button is a physical input that triggers the magnetometer calibration process. The button must be held for 2 seconds to prevent accidental triggers:

```cpp
#define BUTTON_HOLD_TIME 2000   // 2 seconds

unsigned long buttonPressStartTime = 0;
bool buttonPressed = false;
bool calibrationTriggered = false;

void handleButtonPress() {
  // READ: Button is LOW when pressed (INPUT_PULLUP configuration)
  bool currentButtonState = digitalRead(CALIBRATION_BUTTON_PIN) == LOW;
  
  if(currentButtonState && !buttonPressed) {
    // Button just pressed (transition from released to pressed)
    buttonPressed = true;
    buttonPressStartTime = millis();
    calibrationTriggered = false;
    Serial.println("Calibration button pressed");
    
  } else if(!currentButtonState && buttonPressed) {
    // Button released (transition from pressed to released)
    buttonPressed = false;
    calibrationTriggered = false;
    Serial.println("Calibration button released");
    
  } else if(currentButtonState && buttonPressed && !calibrationTriggered) {
    // Button held down - check if hold time exceeded
    unsigned long holdTime = millis() - buttonPressStartTime;
    
    if(holdTime >= BUTTON_HOLD_TIME) {
      // 2 seconds have passed - trigger calibration
      calibrationTriggered = true;  // Only trigger once per press
      Serial.println("Calibration triggered!");
      
      // Notify central controller by sending notification
      uint8_t buttonNotification = BUTTON_PRESSED;
      buttonCharacteristic.writeValue(&buttonNotification, 1);
    }
  }
}
```

The button handler is called in every loop iteration to continuously monitor the button state.

---

## 5. Raspberry Pi Central Controller

### 5.1 System Architecture

The Raspberry Pi runs Python with the `asyncio` library to manage concurrent BLE connections to both Arduino devices. This asynchronous approach allows the controller to:
- Send commands to one device while receiving notifications from another
- Monitor connections in the background while processing workflows
- Handle reconnection attempts without blocking other operations

```python
import asyncio
import struct
import logging
from bleak import BleakClient, BleakScanner
import numpy as np
import requests
```

### 5.2 Device Classes

**JetDevice Class** - Manages the sensor Arduino

This class encapsulates all communication with the Jet Arduino. It handles connection, subscribes to sensor data notifications, sends calibration commands, and buffers magnetometer data:

```python
class JetDevice:
    def __init__(self, device, system_controller=None):
        self.name = JET_NAME
        self.device = device
        self.address = device.address
        self.client = None
        self.connected = False
        self.system = system_controller
        self._write_lock = asyncio.Lock()  # Prevent simultaneous BLE writes
        self.last_sensor_data = None
        self.mag_data_buffer = []  # Stores calibration samples

    # Callback wrappers - convert sync callbacks to async tasks
    def _notify_data_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_data(sender, data))

    def _notify_control_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_control(sender, data))

    def _notify_mag_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_mag(sender, data))
```

The callback wrappers convert BLE notification callbacks (which are synchronous) into async tasks that don't block the event loop.

**Processing Sensor Data**

When sensor data arrives, it's unpacked and forwarded to the local server:

```python
async def notification_handler_data(self, sender, data):
    """Handle 32-byte sensor data packets from Jet"""
    try:
        unpacked = struct.unpack('<Ifffffffffff', data)
        
        (
            timestamp_ms,
            accelX, accelY, accelZ,
            q0, q1, q2, q3,
            magX, magY, magZ,
            temp
        ) = unpacked

        # Store latest reading
        self.last_sensor_data = unpacked

        # Prepare JSON payload for server
        payload = {
            "device": self.name,
            "timestamp_ms": timestamp_ms,
            "accelX": accelX, "accelY": accelY, "accelZ": accelZ,
            "q0": q0, "q1": q1, "q2": q2, "q3": q3,
            "magX": magX, "magY": magY, "magZ": magZ,
            "temp": temp,
        }

        # Send to external server (non-blocking, ignore failures)
        try:
            requests.post(SERVER_URL, json=payload, timeout=0.01)
        except:
            logger.warning(f"[{self.name}] Server offline")

    except struct.error as e:
        logger.error(f"[{self.name}] Struct unpack error: {e}")
```

The unpacking uses `<Ifffffffffff` format string:
- `<` = little-endian byte order
- `I` = unsigned 32-bit int (timestamp)
- `f` × 11 = eleven 32-bit floats (accel, quaternion, mag, temp)

**Processing State Notifications**

When the Jet changes state, it sends a notification that triggers workflow actions:

```python
async def notification_handler_control(self, sender, data):
    """Handle state change notifications from Jet"""
    if not data:
        return
    cmd = data[0]
    
    if cmd == JetCmd.IDLE:
        logger.info("JET STATUS: IDLE detected")
        if self.system:
            await self.system.on_jet_idle()
            
    elif cmd == JetCmd.PLATFORM_DETECTED:
        logger.info("JET STATUS: Platform detected")
        if self.system:
            await self.system.on_platform_detected()
            
    elif cmd == JetCmd.RUNNING:
        logger.info("JET STATUS: Running/Moving")
        if self.system:
            await self.system.on_jet_running()
    else:
        logger.debug(f"{self.name}: Unknown control 0x{cmd:02X}")
```

These notifications trigger callback methods on the SystemController to coordinate the workflow.

**Collecting Calibration Data**

During calibration, magnetometer samples are collected into a buffer:

```python
async def notification_handler_mag(self, sender, data):
    """Handle magnetometer data during calibration"""
    try:
        if len(data) < 12:  # 3 floats = 12 bytes
            return
        values = struct.unpack('<3f', data)
        mag_data = {'x': values[0], 'y': values[1], 'z': values[2]}
        self.mag_data_buffer.append(mag_data)
        logger.debug(f"Received mag data: {mag_data}")
    except Exception:
        logger.exception(f"{self.name}: Error parsing magnetometer data")
```

Each sample is stored as a dictionary with x, y, z components. After 30 seconds, these samples are used to calculate the calibration matrix.

**Sending Commands**

Commands are sent via the command characteristic using a write lock to ensure only one write happens at a time:

```python
async def send_command(self, command):
    """Send single-byte command to Jet"""
    if not self.is_connected():
        logger.warning(f"{self.name}: Cannot send command - not connected")
        return False
    
    async with self._write_lock:  # Ensure exclusive access
        try:
            await self.client.write_gatt_char(JET_CHARACTERISTIC_COMMAND, bytes([command]))
            await asyncio.sleep(0.1)
            return True
        except Exception:
            logger.exception(f"{self.name}: Failed to send command 0x{command:02X}")
            return False

async def start_mag_calibration(self):
    """Command Jet to start sending magnetometer data"""
    logger.info("→ Commanding Jet to start magnetometer calibration")
    return await self.send_command(CalibrationCmd.START_MAG_CALIBRATION)

async def stop_mag_calibration(self):
    """Command Jet to stop sending magnetometer data"""
    logger.info("→ Commanding Jet to stop magnetometer calibration")
    return await self.send_command(CalibrationCmd.STOP_MAG_CALIBRATION)

async def send_calibration_matrix(self, matrix):
    """Send 3x3 calibration matrix to Jet (9 floats = 36 bytes)"""
    if not self.is_connected():
        logger.warning(f"{self.name}: Cannot send calibration matrix - not connected")
        return False
    
    async with self._write_lock:
        try:
            # Pack 3x3 matrix as 9 floats
            matrix_bytes = struct.pack('<9f', *matrix.flatten())
            await self.client.write_gatt_char(JET_CHARACTERISTIC_COMMAND, matrix_bytes)
            await asyncio.sleep(0.2)
            logger.info("✓ Calibration matrix sent to Jet")
            return True
        except Exception:
            logger.exception(f"{self.name}: Failed to send calibration matrix")
            return False
```

### 5.3 TableDevice Class

Similar to JetDevice, but manages the turning table:

```python
class TableDevice:
    def __init__(self, device, system_controller=None):
        self.name = TABLE_NAME
        self.device = device
        self.address = device.address
        self.client = None
        self.connected = False
        self.system = system_controller
        self._write_lock = asyncio.Lock()

    # Handle status updates from table
    async def notification_handler_status(self, sender, data):
        """Log status changes from turning table"""
        if not data:
            return
        status = data[0]
        
        if status == TableStatus.MAGNET_ON:
            logger.info("TABLE STATUS: Magnet ON")
        elif status == TableStatus.ROTATING:
            logger.info("TABLE STATUS: Rotating")
        # ... etc for other statuses
    
    # Handle button press notification
    async def notification_handler_button(self, sender, data):
        """Handle calibration button press from Turning Table"""
        if not data:
            return
        button_state = data[0]
        if button_state == 0x01:
            logger.info("TABLE: Button pressed - initiating magnetometer calibration")
            if self.system:
                await self.system.on_calibration_button_pressed()

    async def send_command(self, command):
        """Send command to table motor/magnet"""
        if not self.is_connected():
            logger.warning(f"{self.name}: Cannot send command - not connected")
            return False
        
        async with self._write_lock:
            try:
                await self.client.write_gatt_char(TABLE_CHARACTERISTIC_COMMAND, bytes([command]))
                await asyncio.sleep(0.1)
                return True
            except Exception:
                logger.exception(f"{self.name}: Failed to send 0x{command:02X}")
                return False

    async def activate_magnet(self):
        logger.info("→ Activating magnet")
        return await self.send_command(TableCmd.ACTIVATE_MAGNET)

    async def deactivate_magnet(self):
        logger.info("→ Deactivating magnet")
        return await self.send_command(TableCmd.DEACTIVATE_MAGNET)

    async def start_rotation(self):
        logger.info("→ Starting rotation")
        return await self.send_command(TableCmd.START_ROTATION)

    async def stop_rotation(self):
        logger.info("→ Stopping rotation")
        return await self.send_command(TableCmd.STOP_ROTATION)
```

### 5.4 SystemController Class - Workflow Orchestration

The SystemController is the brain of the system. It receives notifications from both devices and orchestrates the workflows. It uses asyncio locks to prevent race conditions where two workflows might run simultaneously:

```python
class SystemController:
    def __init__(self, jet, table):
        self.jet = jet
        self.table = table
        self.state = SystemState.DISCONNECTED
        self._state_lock = asyncio.Lock()  # Protects state and flags
        self._workflow_active = False  # Prevents overlapping workflows
        self._calibration_active = False

    async def on_jet_idle(self):
        """Called when Jet Arduino detects idle state and sends IDLE notification"""
        async with self._state_lock:
            # Only execute if we're in normal running state and no other workflow active
            if self.state != SystemState.JET_RUNNING or self._workflow_active:
                return
            
            logger.info("\n" + "="*60)
            logger.info("WORKFLOW: Jet IDLE → Activating magnet")
            logger.info("="*60 + "\n")
            
            self._workflow_active = True  # Mark workflow as active
            self.state = SystemState.JET_IDLE
            
            # Step 1: Command table to activate electromagnet
            success = await self.table.activate_magnet()
            if not success:
                logger.error("Failed to activate magnet - resetting workflow")
                self._workflow_active = False
                self.state = SystemState.JET_RUNNING
                return
            
            await asyncio.sleep(0.5)  # Let magnet stabilize
            self.state = SystemState.WAITING_FOR_PLATFORM
            logger.info("Waiting for platform detection...")
            # Now we wait for on_platform_detected() to be called
```

This function is triggered when the Jet sends an IDLE notification. The workflow:
1. Checks that we're in the right state (JET_RUNNING) and no other workflow is active
2. Sets workflow_active flag to prevent concurrent workflows
3. Commands the table to activate the magnet
4. Waits for the Jet to detect the magnet (will trigger on_platform_detected())

```python
    async def on_platform_detected(self):
        """Called when Jet detects magnet and sends PLATFORM_DETECTED notification"""
        async with self._state_lock:
            # Must be in correct state to proceed
            if self.state != SystemState.WAITING_FOR_PLATFORM:
                logger.debug(f"Platform detected but wrong state: {self.state}")
                return
            
            logger.info("\n" + "="*60)
            logger.info("WORKFLOW: Platform detected → Starting rotation")
            logger.info("="*60 + "\n")
            
            self.state = SystemState.PLATFORM_DETECTED
            await asyncio.sleep(1.0)  # Brief delay before starting rotation
            
            # Step 2: Command table to start motor rotation
            success = await self.table.start_rotation()
            if not success:
                logger.error("Failed to start rotation - aborting workflow")
                await self._abort_workflow()
                return
            
            self.state = SystemState.ROTATING
            # Workflow continues until jet moves (triggers on_jet_running())
```

This continues the workflow when the magnetic field is detected. The acknowledgment of the magnetic pulse confirms that the jet is positioned on top of the table, so we can start rotation.

```python
    async def on_jet_running(self):
        """Called when Jet detects motion (leaves platform)"""
        async with self._state_lock:
            # If we're in the middle of a workflow, abort it
            if self.state in [SystemState.ROTATING, SystemState.PLATFORM_DETECTED, 
                             SystemState.WAITING_FOR_PLATFORM, SystemState.JET_IDLE]:
                logger.info("Jet moved → stopping workflow")
                await self._abort_workflow()

    async def _abort_workflow(self):
        """Emergency stop - halt all hardware operations"""
        await self.table.stop_rotation()
        await asyncio.sleep(0.2)
        await self.table.deactivate_magnet()
        
        logger.info("Workflow aborted - returning to normal operation")
        self.state = SystemState.JET_RUNNING
        self._workflow_active = False
```

This handles the case where the Jet moves while a workflow is active. It immediately stops the motor and disactivates the magnet for safety.

### 5.5 Calibration Workflow

When the button is pressed for 2 seconds on the turning table, it triggers calibration:

```python
    async def on_calibration_button_pressed(self):
        """Called when calibration button held for 2+ seconds"""
        async with self._state_lock:
            if self._calibration_active:
                logger.warning("Calibration already in progress")
                return
            
            logger.info("\n" + "="*60)
            logger.info("CALIBRATION: Starting magnetometer calibration")
            logger.info("="*60 + "\n")
            
            self._calibration_active = True
            self.state = SystemState.CALIBRATING
            self.jet.mag_data_buffer = []  # Clear any previous data
            
            # Step 1: Command Jet to start sending magnetometer data
            success = await self.jet.start_mag_calibration()
            if not success:
                logger.error("Failed to start calibration on Jet")
                self._calibration_active = False
                self.state = SystemState.JET_RUNNING
                return
            
            # Step 2: Collect magnetometer data for 30 seconds
            logger.info("Collecting magnetometer data for 30 seconds...")
            await asyncio.sleep(30.0)  # Wait while data is collected
            
            # Step 3: Stop data collection on Jet
            await self.jet.stop_mag_calibration()
            
            # Step 4: Check if we got data
            if len(self.jet.mag_data_buffer) == 0:
                logger.error("No magnetometer data collected")
                self._calibration_active = False
                self.state = SystemState.JET_RUNNING
                return
            
            logger.info(f"Collected {len(self.jet.mag_data_buffer)} samples")
            
            # Step 5: Calculate calibration matrix from collected samples
            calibration_matrix = calculate_calibration_matrix(self.jet.mag_data_buffer)
            
            # Step 6: Send calibration matrix back to Jet
            logger.info("Sending calibration matrix to Jet...")
            success = await self.jet.send_calibration_matrix(calibration_matrix)
            
            if success:
                logger.info("\n" + "="*60)
                logger.info("CALIBRATION: Complete")
                logger.info("="*60 + "\n")
            else:
                logger.error("Failed to send calibration matrix")
            
            # Clean up
            self._calibration_active = False
            self.state = SystemState.JET_RUNNING
            self.jet.mag_data_buffer = []
```

The calibration workflow:
1. Checks that calibration isn't already running
2. Commands Jet to enter CALIBRATING mode (sends mag data at 19 Hz)
3. Waits 30 seconds for data collection (central receives notifications during this time)
4. Commands Jet to stop calibration mode
5. Processes the collected samples into a 3x3 calibration matrix
6. Sends the matrix back to Jet
7. Returns to normal operation

**Calibration Matrix Calculation**

Currently a placeholder, but this is where the actual hard-iron and soft-iron corrections would be computed:

```python
def calculate_calibration_matrix(mag_data_buffer):
    """
    Calculate magnetometer calibration corrections from collected data.
    
    During calibration, the table rotates the Jet in a known pattern while
    collecting magnetometer readings. These samples are used to compute:
    - Hard-iron correction: offset due to permanent magnetic fields nearby
    - Soft-iron correction: distortion by conductive materials (metal, wiring)
    
    Args:
        mag_data_buffer: List of dicts with 'x', 'y', 'z' magnetometer readings
    
    Returns:
        np.array: 3x3 calibration matrix to correct future measurements
    """
    logger.info("Calculating calibration matrix from collected data...")
    logger.info(f"Collected {len(mag_data_buffer)} magnetometer samples")
    
    # TODO: Implement actual calibration algorithm
    # For now, return identity matrix (no correction applied)
    calibration_matrix = np.eye(3, dtype=np.float32)
    
    logger.info("✓ Calibration matrix calculated")
    return calibration_matrix
```

### 5.6 Device Connection & Scanning

Before the system can operate, it must find and connect to both Arduino devices:

```python
async def scan_for_devices(timeout=15.0):
    """Scan for BLE devices and find our target devices by name"""
    logger.info(f"Scanning for devices ({timeout}s)...")
    
    scanner = BleakScanner()
    devices = await scanner.discover(timeout=timeout)
    
    found = {}
    for d in devices:
        name = d.name or ""
        if name == JET_NAME:
            found[JET_NAME] = d
            logger.info(f"✓ Found {JET_NAME} at {d.address}")
        elif name == TABLE_NAME:
            found[TABLE_NAME] = d
            logger.info(f"✓ Found {TABLE_NAME} at {d.address}")
    
    return found
```

The scanner uses the BLE device names to identify our devices. Both Arduinos advertise their names in their BLE setup.

```python
async def main():
    """Main entry point - initialize and run system"""
    logger.info("\n" + "="*60)
    logger.info("Raspberry Pi BLE Central Control System")
    logger.info("="*60 + "\n")
    
    # Phase 1: Scan for devices
    found = await scan_for_devices()
    
    if JET_NAME not in found:
        logger.error(f"{JET_NAME} not found!")
        return
    if TABLE_NAME not in found:
        logger.error(f"{TABLE_NAME} not found!")
        return

    # Phase 2: Create device objects
    jet = JetDevice(found[JET_NAME])
    table = TableDevice(found[TABLE_NAME])
    system = SystemController(jet, table)
    
    # Cross-reference - devices need access to system controller for callbacks
    jet.system = system
    table.system = system

    try:
        logger.info("\nConnecting to devices...\n")
        
        # Phase 3: Connect to both devices
        try:
            await jet.connect()
        except Exception as e:
            logger.error(f"Failed to connect to Jet: {e}")
            return
        
        await asyncio.sleep(2.0)  # Let Jet stabilize
        
        try:
            await table.connect()
        except Exception as e:
            logger.error(f"Failed to connect to Table: {e}")
            await jet.disconnect()
            return
        
        if not jet.connected or not table.connected:
            logger.error("Failed to establish all connections")
            return

        # Phase 4: System is ready - set initial state
        system.state = SystemState.JET_RUNNING
        
        logger.info("\n" + "="*60)
        logger.info("System Ready")
        logger.info("="*60)
        logger.info("Workflow: IDLE → Magnet → Platform → Rotation")
        logger.info("Calibration: Button press on Turning Table → Mag calibration")
        logger.info("Auto-reconnection enabled")
        logger.info("Press Ctrl+C to stop\n")

        # Phase 5: Start background connection monitor
        monitor_task = asyncio.create_task(connection_monitor(jet, table, system))
        
        # Phase 6: Main loop - keep running
        while True:
            await asyncio.sleep(1.0)
                
    except KeyboardInterrupt:
        logger.info("\nShutting down...")
        await table.stop_rotation()
        await table.deactivate_magnet()
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        await jet.disconnect()
        await table.disconnect()
        logger.info("✓ Shutdown complete")

if __name__ == "__main__":
    asyncio.run(main())
```

---

## 6. Connection Management & Auto-Reconnection

### 6.1 Monitoring Connections

A background task continuously monitors device connection status. If a device disconnects, it automatically attempts to reconnect:

```python
async def connection_monitor(jet, table, system):
    """Background task - continuously monitor device connections"""
    both_connected = True
    
    while True:
        try:
            jet_ok = jet.is_connected()
            table_ok = table.is_connected()
            
            # Check if any device disconnected
            if not jet_ok or not table_ok:
                if both_connected:
                    # First time we detected disconnection
                    logger.warning("Connection lost - initiating reconnection sequence")
                    both_connected = False
                    
                    # Safety: abort any active workflows
                    async with system._state_lock:
                        if system._workflow_active:
                            await system._abort_workflow()
                
                # Attempt to reconnect disconnected devices
                if not jet_ok:
                    await reconnect_device(jet)
                if not table_ok:
                    await reconnect_device(table)
                
                # Check if all devices reconnected successfully
                if jet.is_connected() and table.is_connected():
                    logger.info("All devices reconnected - resuming normal operation")
                    system.state = SystemState.JET_RUNNING
                    both_connected = True
            
            await asyncio.sleep(1.0)  # Check every 1 second
            
        except Exception as e:
            logger.exception(f"Connection monitor error: {e}")
            await asyncio.sleep(2.0)
```

This background task:
1. Checks connection status every 1 second
2. If a disconnect is detected, logs a warning
3. Aborts any active workflows (safety)
4. Attempts reconnection with exponential backoff
5. Resumes operation when reconnected

### 6.2 Reconnection Strategy

When a device disconnects, the system attempts to reconnect with increasing delays to avoid overwhelming the BLE radio:

```python
async def reconnect_device(device, max_retries=RECONNECT_MAX_RETRIES):
    """Attempt to reconnect device with exponential backoff"""
    delay = RECONNECT_DELAY  # Start at 2.0 seconds
    
    for attempt in range(max_retries):
        try:
            logger.warning(f"Reconnecting {device.name} (attempt {attempt + 1}/{max_retries})...")
            await device.connect()
            logger.info(f"Successfully reconnected to {device.name}")
            return True
        except Exception as e:
            logger.warning(f"Reconnection failed: {e}")
            if attempt < max_retries - 1:
                # Wait before retry, with exponential backoff
                await asyncio.sleep(delay)
                delay *= RECONNECT_BACKOFF  # Multiply by 1.5 each time
    
    logger.error(f"Failed to reconnect {device.name} after {max_retries} attempts")
    return False
```

Backoff sequence:
- Attempt 1: immediate
- Attempt 2: wait 2.0s
- Attempt 3: wait 3.0s (2.0 × 1.5)
- Attempt 4: wait 4.5s (3.0 × 1.5)
- Attempt 5: wait 6.75s (4.5 × 1.5)

This prevents the system from spamming connection attempts and gives the BLE hardware time to recover.

---

## 7. Data Flow & Communication Patterns

### 7.1 Notification-Driven Architecture

The system uses BLE notifications (asynchronous messages from the devices) to trigger actions on the central controller. This is an event-driven architecture:

```
Jet Arduino                     Raspberry Pi Central
    ↓ (detects idle)
    [sends IDLE notification]
         ↓
        (notification callback triggered)
            ↓
        (async task created)
            ↓
        (on_jet_idle() executes)
            ↓
            [activates magnet]
            ↓
    ← (activation confirmed)
    
    ↓ (detects magnet)
    [sends PLATFORM_DETECTED]
         ↓
        (on_platform_detected() executes)
            ↓
            [starts rotation]
            ↓
    ← (rotation confirmed)
```

This event-driven approach means the central controller never needs to poll devices - it immediately reacts to changes.

### 7.2 Asynchronous Lock Protection

To prevent race conditions, critical sections use locks:

```python
# Example: Two events trying to modify state simultaneously

# Thread 1: on_jet_idle() called
async with self._state_lock:  # Wait for lock
    if self.state != SystemState.JET_RUNNING:
        return
    self._workflow_active = True  # Now safe to modify
    # ... rest of workflow

# Thread 2: on_calibration_button_pressed() called (while Thread 1 has lock)
async with self._state_lock:  # Wait here until Thread 1 releases lock
    if self._calibration_active:
        return
    # ... now it's safe to modify
```

The lock ensures that only one async task can modify shared state at a time.

---

## 8. State Machines in Action

### 8.1 Jet State Machine Flowchart

```
DISCONNECTED
    ↓ (BLE connected)
RUNNING (sensors active, 25 Hz)
    ↓ (idle detected: accel < 0.03, gyro < 10° for 100+ samples)
IDLE (notify central, wait for magnet)
    ├→ RUNNING (motion detected: accel > 0.15 or gyro > 10°)
    └→ PLATFORM (magnet detected: mag > 73 units)
        ↓
        RUNNING (motion detected: accel > 0.2 or gyro > 15°)
```

Each state:
- **DISCONNECTED**: Waits for BLE connection, no sensor reading
- **RUNNING**: Sends sensor data at 25 Hz, accumulates idle counter
- **IDLE**: Sends only state updates, monitors for exit conditions
- **PLATFORM**: Minimal data sending, waits for motion to exit

### 8.2 Turning Table State Machine Flowchart

```
DISCONNECTED
    ↓ (BLE connected)
READY (all hardware OFF)
    ├→ MAGNET_ACTIVE (electromagnet ON, motor OFF)
    │   ├→ READY (magnet off command)
    │   └→ ROTATING (rotation start command, motor ON)
    │       ├→ READY (rotation stop command)
    └→ DISCONNECTED (connection lost → safety shutdown)
```

State transitions happen via commands from the central controller.

### 8.3 Central Controller State Machine Flowchart

```
DISCONNECTED
    ↓ (both devices connected)
JET_RUNNING
    ├→ JET_IDLE (idle notification from Jet)
    │   ├→ WAITING_FOR_PLATFORM (magnet activated)
    │   │   └→ PLATFORM_DETECTED (platform detected notification)
    │   │       └→ ROTATING (rotation started)
    │   │           └→ JET_RUNNING (jet moves)
    │   └→ JET_RUNNING (timeout or error)
    │
    └→ CALIBRATING (calibration button pressed)
        └→ JET_RUNNING (calibration complete)
```

---

## 9. Command Reference

### 9.1 All Command Codes

| Command | Hex | Source | Target | Purpose |
|---------|-----|--------|--------|---------|
| RUNNING | 0x00 | Jet | Central | Jet is moving |
| IDLE | 0x01 | Jet | Central | Jet is stationary |
| PLATFORM_DETECTED | 0x02 | Jet | Central | Magnet detected by Jet |
| START_MAG_CALIBRATION | 0x30 | Central | Jet | Begin collecting mag data |
| STOP_MAG_CALIBRATION | 0x31 | Central | Jet | Stop collecting mag data |
| SEND_CALIBRATION_MATRIX | 0x32 | Central | Jet | Send 3x3 matrix (36 bytes) |
| ACTIVATE_MAGNET | 0x10 | Central | Table | Turn electromagnet ON |
| DEACTIVATE_MAGNET | 0x11 | Central | Table | Turn electromagnet OFF |
| START_ROTATION | 0x12 | Central | Table | Start motor stepping |
| STOP_ROTATION | 0x13 | Central | Table | Stop motor stepping |

### 9.2 Status Codes (Jet sends)

- `0x00` = RUNNING: Device moving, sending full sensor data
- `0x01` = IDLE: Device stationary, ready for workflow
- `0x02` = PLATFORM_DETECTED: Magnet field detected

### 9.3 Status Codes (Table sends)

- `0x20` = MAGNET_ON: Electromagnet activated
- `0x21` = MAGNET_OFF: Electromagnet deactivated
- `0x22` = ROTATING: Motor is stepping
- `0x23` = STOPPED: Motor is idle
- `0x24` = READY: System initialized and ready

---

## 10. Performance Characteristics

### 10.1 Timing & Frequencies

```
Jet Sensor Sampling:           25 Hz (40 ms intervals)
Jet Sensor Broadcast:          25 Hz via dataCharacteristic
Jet Magnetometer (normal):     25 Hz (mixed with accel data)
Jet Magnetometer (calibration): 19 Hz (dedicated characteristic)
Table Status Updates:          Event-driven (on state change)
BLE Connection Interval:       7.5 ms
BLE Advertising Interval:      100 ms (when not connected)
Idle Detection Threshold:      100 consecutive samples (~4 seconds)
Calibration Button:            2 seconds
