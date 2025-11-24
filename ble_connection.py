#!/usr/bin/env python3
"""
BLE Central Control System for Raspberry Pi
Manages communication between Jet Arduino and Turning Table Arduino
Features: Auto-reconnection, Magnetometer calibration workflow
"""

import asyncio
import struct
import logging
import numpy as np
import requests
from bleak import BleakClient, BleakScanner

# ----------------------------
# Logging
# ----------------------------
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ----------------------------
# BLE Device Configuration
# ----------------------------

# Jet Arduino
JET_DEVICE_NAME = "Eurofighter"
JET_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
JET_CHARACTERISTIC_SENSOR_DATA = "19B10001-E8F2-537E-4F6C-D104768A1214"
JET_CHARACTERISTIC_MAGNETOMETER = "19B10002-E8F2-537E-4F6C-D104768A1214"
JET_CHARACTERISTIC_STATUS = "19B10003-E8F2-537E-4F6C-D104768A1214"
JET_CHARACTERISTIC_COMMAND = "19B10004-E8F2-537E-4F6C-D104768A1214"

# Turning Table Arduino
TABLE_DEVICE_NAME = "TurningTable"
TABLE_SERVICE_UUID = "19B20000-E8F2-537E-4F6C-D104768A1215"
TABLE_CHARACTERISTIC_STATUS = "19B20001-E8F2-537E-4F6C-D104768A1215"
TABLE_CHARACTERISTIC_COMMAND = "19B20002-E8F2-537E-4F6C-D104768A1215"
TABLE_CHARACTERISTIC_BUTTON = "19B20003-E8F2-537E-4F6C-D104768A1215"

SERVER_URL = "http://localhost:8000/update-data"

# Reconnection settings
RECONNECT_DELAY = 2.0
RECONNECT_MAX_RETRIES = 5
RECONNECT_BACKOFF = 1.5

# ----------------------------
# Commands
# ----------------------------
class JetCmd:
    RUNNING = 0x00
    IDLE = 0x01
    PLATFORM_DETECTED = 0x02
    CALIBRATE_MAG = 0x03

class TableCmd:
    ACTIVATE_MAGNET = 0x10
    DEACTIVATE_MAGNET = 0x11
    START_ROTATION = 0x12
    STOP_ROTATION = 0x13

class TableStatus:
    MAGNET_ON = 0x20
    MAGNET_OFF = 0x21
    ROTATING = 0x22
    STOPPED = 0x23
    READY = 0x24

class CalibrationCmd:
    START_MAG_CALIBRATION = 0x30
    STOP_MAG_CALIBRATION = 0x31
    SEND_CALIBRATION_MATRIX = 0x32

class SystemState:
    DISCONNECTED = "DISCONNECTED"
    JET_RUNNING = "JET_RUNNING"
    JET_IDLE = "JET_IDLE"
    WAITING_FOR_PLATFORM = "WAITING_FOR_PLATFORM"
    PLATFORM_DETECTED = "PLATFORM_DETECTED"
    ROTATING = "ROTATING"
    CALIBRATING = "CALIBRATING"

# ----------------------------
# Jet Device
# ----------------------------
class JetDevice:
    def __init__(self, device, system_controller=None):
        self.name = JET_DEVICE_NAME
        self.device = device
        self.address = device.address
        self.client = None
        self.connected = False
        self.system = system_controller
        self._write_lock = asyncio.Lock()
        self.data_records = []
        self.last_sensor_data = None
        self.mag_data_buffer = []

    def _notify_data_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_data(sender, data))

    def _notify_control_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_control(sender, data))

    def _notify_mag_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_mag(sender, data))

    async def notification_handler_data(self, sender, data):
        try:
            unpacked = struct.unpack('<Ifffffffffff', data)

            (
                timestamp_ms,
                accelX, accelY, accelZ,
                q0, q1, q2, q3,
                magX, magY, magZ,
                temp
            ) = unpacked

            # Store raw packet
            self.data_records.append(unpacked)

            payload = {
                "timestamp_ms": timestamp_ms,
                "accelX": accelX, "accelY": accelY, "accelZ": accelZ,
                "q0": q0, "q1": q1, "q2": q2, "q3": q3,
                "magX": magX, "magY": magY, "magZ": magZ,
                "temp": temp,
            }

            try:
                requests.post(SERVER_URL, json=payload, timeout=0.01)
            except:
                logger.warning(f"[{self.name}] Server offline")

        except struct.error as e:
            logger.error(f"[{self.name}] Struct unpack error: {e}")
            return

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

    async def notification_handler_control(self, sender, data):
        if not data:
            return
        cmd = data[0]
        if cmd == JetCmd.IDLE:
            logger.info("🛑 JET STATUS: IDLE detected")
            if self.system:
                await self.system.on_jet_idle()
        elif cmd == JetCmd.PLATFORM_DETECTED:
            logger.info("🧲 JET STATUS: Platform detected")
            if self.system:
                await self.system.on_platform_detected()
        elif cmd == JetCmd.RUNNING:
            logger.info("✈️  JET STATUS: Running/Moving")
            if self.system:
                await self.system.on_jet_running()
        else:
            logger.debug(f"{self.name}: Unknown control 0x{cmd:02X}")

    async def connect(self):
        try:
            logger.info(f"Connecting to {self.name} at {self.address}...")
            self.client = BleakClient(self.device, timeout=30.0)
            await self.client.connect()
            
            if not self.client.is_connected:
                raise Exception("Failed to establish connection")
            
            logger.info(f"✓ Connected to {self.name}")
            await asyncio.sleep(0.5)
            
            try:
                await self.client.start_notify(JET_CHARACTERISTIC_SENSOR_DATA, self._notify_data_callback)
                logger.info("✓ Subscribed to sensor data")
            except Exception as e:
                logger.warning(f"Could not subscribe to sensor data: {e}")
            
            try:
                await self.client.start_notify(JET_CHARACTERISTIC_MAGNETOMETER, self._notify_mag_callback)
                logger.info("✓ Subscribed to magnetometer data")
            except Exception as e:
                logger.warning(f"Could not subscribe to mag data: {e}")
            
            try:
                await self.client.start_notify(JET_CHARACTERISTIC_STATUS, self._notify_control_callback)
                logger.info("✓ Subscribed to control/status")
            except Exception as e:
                logger.warning(f"Could not subscribe to control: {e}")
            
            self.connected = True
            
        except Exception as e:
            self.connected = False
            logger.error(f"{self.name}: Failed to connect - {e}")
            raise

    async def disconnect(self):
        if self.client and self.connected:
            try:
                await self.client.disconnect()
                self.connected = False
                logger.info(f"Disconnected from {self.name}")
            except Exception:
                logger.exception(f"{self.name}: Failed to disconnect")

    def is_connected(self):
        try:
            return bool(self.client and self.client.is_connected)
        except Exception:
            return False

    async def send_command(self, command):
        if not self.is_connected():
            logger.warning(f"{self.name}: Cannot send command - not connected")
            return False
        async with self._write_lock:
            try:
                await self.client.write_gatt_char(JET_CHARACTERISTIC_COMMAND, bytes([command]))
                await asyncio.sleep(0.1)
                return True
            except Exception:
                logger.exception(f"{self.name}: Failed to send command 0x{command:02X}")
                return False

    async def start_mag_calibration(self):
        logger.info("→ Commanding Jet to start magnetometer calibration")
        return await self.send_command(CalibrationCmd.START_MAG_CALIBRATION)

    async def stop_mag_calibration(self):
        logger.info("→ Commanding Jet to stop magnetometer calibration")
        return await self.send_command(CalibrationCmd.STOP_MAG_CALIBRATION)

    async def send_calibration_matrix(self, matrix):
        """Send calibration matrix to Jet (3x3 matrix = 9 floats)"""
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

# ----------------------------
# Table Device
# ----------------------------
class TableDevice:
    def __init__(self, device, system_controller=None):
        self.name = TABLE_DEVICE_NAME
        self.device = device
        self.address = device.address
        self.client = None
        self.connected = False
        self.system = system_controller
        self._write_lock = asyncio.Lock()

    def _notify_status_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_status(sender, data))

    def _notify_button_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_button(sender, data))

    async def notification_handler_status(self, sender, data):
        if not data:
            return
        status = data[0]
        if status == TableStatus.MAGNET_ON:
            logger.info("🧲 TABLE STATUS: Magnet ON")
        elif status == TableStatus.MAGNET_OFF:
            logger.info("🧲 TABLE STATUS: Magnet OFF")
        elif status == TableStatus.ROTATING:
            logger.info("🔄 TABLE STATUS: Rotating")
        elif status == TableStatus.STOPPED:
            logger.info("⏹️  TABLE STATUS: Stopped")
        elif status == TableStatus.READY:
            logger.info("✓ TABLE STATUS: Ready")
        else:
            logger.debug(f"{self.name}: Unknown status 0x{status:02X}")

    async def notification_handler_button(self, sender, data):
        """Handle button press notification from Turning Table"""
        if not data:
            return
        button_state = data[0]
        if button_state == 0x01:
            logger.info("🔘 TABLE: Button pressed - initiating magnetometer calibration")
            if self.system:
                await self.system.on_calibration_button_pressed()

    async def connect(self):
        try:
            logger.info(f"Connecting to {self.name} at {self.address}...")
            self.client = BleakClient(self.device, timeout=30.0)
            await self.client.connect()
            
            if not self.client.is_connected:
                raise Exception("Failed to establish connection")
            
            logger.info(f"✓ Connected to {self.name}")
            await asyncio.sleep(0.5)
            
            try:
                await self.client.start_notify(TABLE_CHARACTERISTIC_STATUS, self._notify_status_callback)
                logger.info("✓ Subscribed to table status")
            except Exception as e:
                logger.warning(f"Could not subscribe to status: {e}")

            try:
                await self.client.start_notify(TABLE_CHARACTERISTIC_BUTTON, self._notify_button_callback)
                logger.info("✓ Subscribed to table button")
            except Exception as e:
                logger.warning(f"Could not subscribe to button: {e}")
            
            self.connected = True
            
        except Exception as e:
            self.connected = False
            logger.error(f"{self.name}: Failed to connect - {e}")
            raise

    async def disconnect(self):
        if self.client and self.connected:
            try:
                await self.client.disconnect()
                self.connected = False
                logger.info(f"Disconnected from {self.name}")
            except Exception:
                logger.exception(f"{self.name}: Failed to disconnect")

    def is_connected(self):
        try:
            return bool(self.client and self.client.is_connected)
        except Exception:
            return False

    async def send_command(self, command):
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

# ----------------------------
# Magnetometer Calibration
# ----------------------------
def calculate_calibration_matrix(mag_data_buffer):
    """
    Placeholder for magnetometer calibration calculation.
    In reality, this would compute hard-iron and soft-iron corrections.
    
    Args:
        mag_data_buffer: List of dicts with 'x', 'y', 'z' magnetometer readings
    
    Returns:
        np.array: 3x3 calibration matrix
    """
    logger.info("🔧 Calculating calibration matrix from collected data...")
    logger.info(f"   Collected {len(mag_data_buffer)} magnetometer samples")
    
    # TODO: Implement actual calibration algorithm
    # This is a placeholder - should calculate hard-iron and soft-iron corrections
    # For now, return identity matrix as placeholder
    calibration_matrix = np.eye(3, dtype=np.float32)
    
    logger.info("✓ Calibration matrix calculated")
    return calibration_matrix

# ----------------------------
# System Controller
# ----------------------------
class SystemController:
    def __init__(self, jet, table):
        self.jet = jet
        self.table = table
        self.state = SystemState.DISCONNECTED
        self._state_lock = asyncio.Lock()
        self._workflow_active = False
        self._calibration_active = False

    async def on_jet_idle(self):
        async with self._state_lock:
            if self.state != SystemState.JET_RUNNING or self._workflow_active:
                return
            
            logger.info("\n" + "="*60)
            logger.info("🔄 WORKFLOW: Jet IDLE → Activating magnet")
            logger.info("="*60 + "\n")
            
            self._workflow_active = True
            self.state = SystemState.JET_IDLE
            
            success = await self.table.activate_magnet()
            if not success:
                logger.error("Failed to activate magnet - resetting workflow")
                self._workflow_active = False
                self.state = SystemState.JET_RUNNING
                return
            
            await asyncio.sleep(0.5)
            self.state = SystemState.WAITING_FOR_PLATFORM
            logger.info("⏳ Waiting for platform detection...")

            await asyncio.sleep(0.5)
            await self.table.deactivate_magnet()

    async def on_platform_detected(self):
        async with self._state_lock:
            if self.state != SystemState.WAITING_FOR_PLATFORM:
                logger.debug(f"Platform detected but wrong state: {self.state}")
                return
            
            logger.info("\n" + "="*60)
            logger.info("🎯 WORKFLOW: Platform detected → Starting rotation")
            logger.info("="*60 + "\n")
            
            self.state = SystemState.PLATFORM_DETECTED
            await asyncio.sleep(1.0)
            
            success = await self.table.start_rotation()
            if not success:
                logger.error("Failed to start rotation - aborting workflow")
                await self._abort_workflow()
                return
            
            self.state = SystemState.ROTATING

    async def on_jet_running(self):
        async with self._state_lock:
            if self.state in [SystemState.ROTATING, SystemState.PLATFORM_DETECTED, 
                             SystemState.WAITING_FOR_PLATFORM, SystemState.JET_IDLE]:
                logger.info("✈️  Jet moved → stopping workflow")
                await self._abort_workflow()

    async def on_calibration_button_pressed(self):
        """Handle calibration button press from Turning Table"""
        async with self._state_lock:
            if self._calibration_active:
                logger.warning("⚠️  Calibration already in progress")
                return
            
            logger.info("\n" + "="*60)
            logger.info("🔧 CALIBRATION: Starting magnetometer calibration")
            logger.info("="*60 + "\n")
            
            self._calibration_active = True
            self.state = SystemState.CALIBRATING
            self.jet.mag_data_buffer = []  # Clear buffer
            
            # Command Jet to start sending mag data
            success = await self.jet.start_mag_calibration()
            if not success:
                logger.error("Failed to start calibration on Jet")
                self._calibration_active = False
                self.state = SystemState.JET_RUNNING
                return
            
            # Collect magnetometer data for 30 seconds
            logger.info("📊 Collecting magnetometer data for 30 seconds...")
            await asyncio.sleep(30.0)
            
            # Stop calibration on Jet
            await self.jet.stop_mag_calibration()
            
            # Calculate calibration matrix
            if len(self.jet.mag_data_buffer) == 0:
                logger.error("❌ No magnetometer data collected")
                self._calibration_active = False
                self.state = SystemState.JET_RUNNING
                return
            
            calibration_matrix = calculate_calibration_matrix(self.jet.mag_data_buffer)
            
            # Send calibration matrix back to Jet
            logger.info("📤 Sending calibration matrix to Jet...")
            success = await self.jet.send_calibration_matrix(calibration_matrix)
            
            if success:
                logger.info("\n" + "="*60)
                logger.info("✅ CALIBRATION: Complete")
                logger.info("="*60 + "\n")
            else:
                logger.error("❌ Failed to send calibration matrix")
            
            self._calibration_active = False
            self.state = SystemState.JET_RUNNING
            self.jet.mag_data_buffer = []

    async def _abort_workflow(self):
        """Abort workflow and return to safe state"""
        await self.table.stop_rotation()
        await asyncio.sleep(0.2)
        await self.table.deactivate_magnet()
        
        logger.info("⚠️  Workflow aborted - returning to normal operation")
        self.state = SystemState.JET_RUNNING
        self._workflow_active = False

# ----------------------------
# BLE Scan & Connection Management
# ----------------------------
async def scan_for_devices(timeout=15.0):
    logger.info(f"🔍 Scanning for devices ({timeout}s)...")
    
    scanner = BleakScanner()
    devices = await scanner.discover(timeout=timeout)
    
    found = {}
    for d in devices:
        name = d.name or ""
        if name == JET_DEVICE_NAME:
            found[JET_DEVICE_NAME] = d
            logger.info(f"✓ Found {JET_DEVICE_NAME} at {d.address}")
        elif name == TABLE_DEVICE_NAME:
            found[TABLE_DEVICE_NAME] = d
            logger.info(f"✓ Found {TABLE_DEVICE_NAME} at {d.address}")
    
    return found

async def reconnect_device(device, max_retries=RECONNECT_MAX_RETRIES):
    delay = RECONNECT_DELAY
    
    for attempt in range(max_retries):
        try:
            logger.warning(f"🔄 Reconnecting {device.name} (attempt {attempt + 1}/{max_retries})...")
            await device.connect()
            logger.info(f"✅ Successfully reconnected to {device.name}")
            return True
        except Exception as e:
            logger.warning(f"Reconnection failed: {e}")
            if attempt < max_retries - 1:
                await asyncio.sleep(delay)
                delay *= RECONNECT_BACKOFF
    
    logger.error(f"❌ Failed to reconnect {device.name} after {max_retries} attempts")
    return False

async def connection_monitor(jet, table, system):
    both_connected = True
    
    while True:
        try:
            jet_ok = jet.is_connected()
            table_ok = table.is_connected()
            
            if not jet_ok or not table_ok:
                if both_connected:
                    logger.warning("⚠️  Connection lost - initiating reconnection sequence")
                    both_connected = False
                    async with system._state_lock:
                        if system._workflow_active:
                            await system._abort_workflow()
                
                if not jet_ok:
                    await reconnect_device(jet)
                if not table_ok:
                    await reconnect_device(table)
                
                if jet.is_connected() and table.is_connected():
                    logger.info("✅ All devices reconnected - resuming normal operation")
                    system.state = SystemState.JET_RUNNING
                    both_connected = True
            
            await asyncio.sleep(1.0)
            
        except Exception as e:
            logger.exception(f"Connection monitor error: {e}")
            await asyncio.sleep(2.0)

async def main():
    logger.info("\n" + "="*60)
    logger.info("Raspberry Pi BLE Central Control System")
    logger.info("="*60 + "\n")
    
    found = await scan_for_devices()
    
    if JET_DEVICE_NAME not in found:
        logger.error(f"❌ {JET_DEVICE_NAME} not found!")
        return
    if TABLE_DEVICE_NAME not in found:
        logger.error(f"❌ {TABLE_DEVICE_NAME} not found!")
        return

    jet = JetDevice(found[JET_DEVICE_NAME])
    table = TableDevice(found[TABLE_DEVICE_NAME])
    system = SystemController(jet, table)
    jet.system = system
    table.system = system

    try:
        logger.info("\n📡 Connecting to devices...\n")
        
        try:
            await jet.connect()
        except Exception as e:
            logger.error(f"Failed to connect to Jet: {e}")
            return
        
        await asyncio.sleep(2.0)
        
        try:
            await table.connect()
        except Exception as e:
            logger.error(f"Failed to connect to Table: {e}")
            await jet.disconnect()
            return
        
        if not jet.connected or not table.connected:
            logger.error("❌ Failed to establish all connections")
            return

        system.state = SystemState.JET_RUNNING
        
        logger.info("\n" + "="*60)
        logger.info("✅ System Ready")
        logger.info("="*60)
        logger.info("Workflow: IDLE → Magnet → Platform → Rotation")
        logger.info("Calibration: Button press on Turning Table → Mag calibration")
        logger.info("Auto-reconnection enabled")
        logger.info("Press Ctrl+C to stop\n")

        monitor_task = asyncio.create_task(connection_monitor(jet, table, system))
        
        while True:
            await asyncio.sleep(1.0)
                
    except KeyboardInterrupt:
        logger.info("\n🛑 Shutting down...")
        await table.stop_rotation()
        await table.deactivate_magnet()
    except Exception as e:
        logger.error(f"❌ Error: {e}", exc_info=True)
    finally:
        await jet.disconnect()
        await table.disconnect()
        logger.info("✓ Shutdown complete")

if __name__ == "__main__":
    asyncio.run(main())
