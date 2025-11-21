#!/usr/bin/env python3
"""
BLE Central Control System for Raspberry Pi 5
Manages communication between Jet Arduino and Turning Table Arduino
Handles the complete workflow: Idle detection -> Magnet activation -> Platform detection -> Rotation
"""

import asyncio
import struct
from bleak import BleakClient, BleakScanner
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ===== JET DEVICE CONFIGURATION =====
JET_NAME = "Eurofighter"
JET_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
JET_DATA_CHAR = "19B10001-E8F2-537E-4F6C-D104768A1214"      # Sensor data (notify)
JET_CONTROL_CHAR = "19B10003-E8F2-537E-4F6C-D104768A1214"   # Control/status (notify)
JET_RX_CHAR = "19B10004-E8F2-537E-4F6C-D104768A1214"        # Commands to Jet (write)

# ===== TABLE DEVICE CONFIGURATION =====
TABLE_NAME = "TurningTable"
TABLE_SERVICE_UUID = "19B20000-E8F2-537E-4F6C-D104768A1215"
TABLE_STATUS_CHAR = "19B20001-E8F2-537E-4F6C-D104768A1215"  # Table status (notify)
TABLE_COMMAND_CHAR = "19B20002-E8F2-537E-4F6C-D104768A1215" # Commands to Table (write)

# ===== JET COMMANDS =====
CMD_RUNNING = 0x00
CMD_IDLE = 0x01
CMD_PLATFORM_DETECTED = 0x02
CMD_CALIBRATE_MAG = 0x03

# ===== TABLE COMMANDS =====
CMD_ACTIVATE_MAGNET = 0x10
CMD_DEACTIVATE_MAGNET = 0x11
CMD_START_ROTATION = 0x12
CMD_STOP_ROTATION = 0x13
CMD_SET_SPEED = 0x14

# ===== TABLE STATUS =====
STATUS_MAGNET_ON = 0x20
STATUS_MAGNET_OFF = 0x21
STATUS_ROTATING = 0x22
STATUS_STOPPED = 0x23
STATUS_READY = 0x24

# ===== SYSTEM STATE =====
class SystemState:
    DISCONNECTED = "DISCONNECTED"
    WAITING_FOR_JET = "WAITING_FOR_JET"
    JET_RUNNING = "JET_RUNNING"
    JET_IDLE = "JET_IDLE"
    WAITING_FOR_PLATFORM = "WAITING_FOR_PLATFORM"
    PLATFORM_DETECTED = "PLATFORM_DETECTED"
    ROTATING = "ROTATING"
    ROTATION_COMPLETE = "ROTATION_COMPLETE"


class JetDevice:
    """Manages connection and communication with the Jet Arduino"""
    
    def __init__(self, address, system_controller):
        self.name = JET_NAME
        self.address = address
        self.client = None
        self.connected = False
        self.system = system_controller
        self.last_sensor_data = None
        
    async def notification_handler_data(self, sender, data):
        """Handle sensor data notifications"""
        try:
            # Unpack SensorValues struct (32 bytes)
            # uint32_t timestamp_ms, 3x float accel, 4x float quaternion, 3x float mag, float temp
            values = struct.unpack('<I10f', data)
            
            self.last_sensor_data = {
                'timestamp': values[0],
                'accel': {'x': values[1], 'y': values[2], 'z': values[3]},
                'quaternion': {'q0': values[4], 'q1': values[5], 'q2': values[6], 'q3': values[7]},
                'mag': {'x': values[8], 'y': values[9], 'z': values[10]},
                'temp': values[11]
            }
            
        except Exception as e:
            logger.error(f"Error parsing sensor data: {e}")
    
    async def notification_handler_control(self, sender, data):
        """Handle control/status notifications from Jet"""
        cmd = data[0]
        
        if cmd == CMD_IDLE:
            logger.info("🛑 JET STATUS: IDLE detected")
            await self.system.on_jet_idle()
            
        elif cmd == CMD_PLATFORM_DETECTED:
            logger.info("🧲 JET STATUS: Platform detected (magnet sensed)")
            await self.system.on_platform_detected()
            
        elif cmd == CMD_RUNNING:
            logger.info("✈️  JET STATUS: Running/Moving")
            await self.system.on_jet_running()
    
    async def connect(self):
        """Connect to the Jet Arduino"""
        try:
            logger.info(f"Connecting to {self.name} at {self.address}...")
            self.client = BleakClient(self.address)
            await self.client.connect()
            self.connected = True
            logger.info(f"✓ Connected to {self.name}")
            
            # Start notifications for sensor data
            await self.client.start_notify(JET_DATA_CHAR, self.notification_handler_data)
            logger.info("✓ Subscribed to sensor data")
            
            # Start notifications for control/status
            await self.client.start_notify(JET_CONTROL_CHAR, self.notification_handler_control)
            logger.info("✓ Subscribed to control/status")
            
        except Exception as e:
            logger.error(f"Failed to connect to {self.name}: {e}")
            self.connected = False
            
    async def disconnect(self):
        """Disconnect from the Jet Arduino"""
        if self.client and self.connected:
            try:
                await self.client.disconnect()
                self.connected = False
                logger.info(f"Disconnected from {self.name}")
            except Exception as e:
                logger.error(f"Error disconnecting from {self.name}: {e}")
    
    async def send_command(self, command):
        """Send a command to the Jet"""
        if self.client and self.connected:
            try:
                await self.client.write_gatt_char(JET_RX_CHAR, bytes([command]))
                logger.debug(f"Sent command to Jet: 0x{command:02X}")
            except Exception as e:
                logger.error(f"Failed to send command to Jet: {e}")


class TableDevice:
    """Manages connection and communication with the Turning Table Arduino"""
    
    def __init__(self, address, system_controller):
        self.name = TABLE_NAME
        self.address = address
        self.client = None
        self.connected = False
        self.system = system_controller
        
    async def notification_handler_status(self, sender, data):
        """Handle status notifications from Table"""
        status = data[0]
        
        if status == STATUS_MAGNET_ON:
            logger.info("🧲 TABLE STATUS: Electromagnet activated")
            
        elif status == STATUS_MAGNET_OFF:
            logger.info("🧲 TABLE STATUS: Electromagnet deactivated")
            
        elif status == STATUS_ROTATING:
            logger.info("🔄 TABLE STATUS: Rotation started")
            
        elif status == STATUS_STOPPED:
            logger.info("⏹️  TABLE STATUS: Rotation stopped")
            
        elif status == STATUS_READY:
            logger.info("✓ TABLE STATUS: Ready")
    
    async def connect(self):
        """Connect to the Turning Table Arduino"""
        try:
            logger.info(f"Connecting to {self.name} at {self.address}...")
            self.client = BleakClient(self.address)
            await self.client.connect()
            self.connected = True
            logger.info(f"✓ Connected to {self.name}")
            
            # Start notifications for status
            await self.client.start_notify(TABLE_STATUS_CHAR, self.notification_handler_status)
            logger.info("✓ Subscribed to table status")
            
        except Exception as e:
            logger.error(f"Failed to connect to {self.name}: {e}")
            self.connected = False
            
    async def disconnect(self):
        """Disconnect from the Turning Table Arduino"""
        if self.client and self.connected:
            try:
                await self.client.disconnect()
                self.connected = False
                logger.info(f"Disconnected from {self.name}")
            except Exception as e:
                logger.error(f"Error disconnecting from {self.name}: {e}")
    
    async def send_command(self, command):
        """Send a command to the Table"""
        if self.client and self.connected:
            try:
                await self.client.write_gatt_char(TABLE_COMMAND_CHAR, bytes([command]))
                logger.debug(f"Sent command to Table: 0x{command:02X}")
            except Exception as e:
                logger.error(f"Failed to send command to Table: {e}")
    
    async def activate_magnet(self):
        """Activate the electromagnet"""
        logger.info("→ Activating electromagnet on turning table...")
        await self.send_command(CMD_ACTIVATE_MAGNET)
    
    async def deactivate_magnet(self):
        """Deactivate the electromagnet"""
        logger.info("→ Deactivating electromagnet on turning table...")
        await self.send_command(CMD_DEACTIVATE_MAGNET)
    
    async def start_rotation(self):
        """Start rotation"""
        logger.info("→ Starting table rotation...")
        await self.send_command(CMD_START_ROTATION)
    
    async def stop_rotation(self):
        """Stop rotation"""
        logger.info("→ Stopping table rotation...")
        await self.send_command(CMD_STOP_ROTATION)


class SystemController:
    """Main system controller managing the workflow between Jet and Table"""
    
    def __init__(self, jet, table):
        self.jet = jet
        self.table = table
        self.state = SystemState.DISCONNECTED
        self.rotation_duration = 10.0  # seconds for one complete scan
        
    async def on_jet_idle(self):
        """Handle Jet entering IDLE state"""
        if self.state == SystemState.JET_RUNNING:
            logger.info("\n" + "="*60)
            logger.info("🔄 WORKFLOW: Jet is IDLE - Activating magnet on table")
            logger.info("="*60 + "\n")
            
            self.state = SystemState.JET_IDLE
            
            # Activate electromagnet on turning table
            await self.table.activate_magnet()
            self.state = SystemState.MAGNET_ACTIVATING
            
            # Wait for magnet to fully activate
            await asyncio.sleep(0.5)

            # Deactivate magnet
            await self.table.deactivate_magnet()
            self.state = SystemState.MAGNET_DEACTIVATING
            
            self.state = SystemState.WAITING_FOR_PLATFORM
            logger.info("⏳ Waiting for jet to detect platform magnet...")
    
    async def on_platform_detected(self):
        """Handle Jet detecting the platform"""
        if self.state == SystemState.WAITING_FOR_PLATFORM:
            logger.info("\n" + "="*60)
            logger.info("🎯 WORKFLOW: Platform detected - Starting rotation sequence")
            logger.info("="*60 + "\n")
            
            self.state = SystemState.PLATFORM_DETECTED
            
            # Confirm to Jet that table received the signal
            await self.jet.send_command(0x10)
            
            # Small delay for stability
            await asyncio.sleep(1.0)
            
            # Start rotation
            await self.table.start_rotation()
            self.state = SystemState.ROTATING
            
            # logger.info(f"🔄 Rotating for {self.rotation_duration} seconds...")
            
            # Rotate for specified duration
            # await asyncio.sleep(self.rotation_duration)
            
            # Stop rotation
            # await self.table.stop_rotation()
            # await asyncio.sleep(0.5)
            
            
            # self.state = SystemState.ROTATION_COMPLETE
            
            # logger.info("\n" + "="*60)
            # logger.info("✅ WORKFLOW COMPLETE: Rotation finished, magnet off")
            # logger.info("="*60 + "\n")
            
            # Return to waiting state
            # self.state = SystemState.JET_RUNNING
    
    async def on_jet_running(self):
        """Handle Jet returning to RUNNING state"""
        if self.state in [SystemState.ROTATION_COMPLETE, SystemState.PLATFORM_DETECTED]:
            logger.info("✈️  Jet removed from platform or moving again")
            
            # Safety: ensure magnet is off and rotation stopped
            await self.table.deactivate_magnet()
            await self.table.stop_rotation()
            
            self.state = SystemState.JET_RUNNING


async def scan_for_devices():
    """Scan for both Arduino devices"""
    logger.info("🔍 Scanning for devices...")
    devices = await BleakScanner.discover(timeout=10.0)
    
    found_devices = {}
    for device in devices:
        if device.name == JET_NAME:
            found_devices[JET_NAME] = device.address
            logger.info(f"✓ Found {JET_NAME} at {device.address}")
        elif device.name == TABLE_NAME:
            found_devices[TABLE_NAME] = device.address
            logger.info(f"✓ Found {TABLE_NAME} at {device.address}")
    
    return found_devices


async def main():
    """Main function"""
    
    logger.info("\n" + "="*60)
    logger.info("Raspberry Pi BLE Central Control System")
    logger.info("Jet ↔ Raspberry Pi ↔ Turning Table")
    logger.info("="*60 + "\n")
    
    # Scan for devices
    found_devices = await scan_for_devices()
    
    if JET_NAME not in found_devices:
        logger.error(f"❌ Could not find {JET_NAME}!")
        return
    
    if TABLE_NAME not in found_devices:
        logger.error(f"❌ Could not find {TABLE_NAME}!")
        return
    
    # Create system controller first (needed for device initialization)
    system = None
    
    # Create device objects
    jet = JetDevice(found_devices[JET_NAME], system)
    table = TableDevice(found_devices[TABLE_NAME], system)
    
    # Now create system controller with devices
    system = SystemController(jet, table)
    jet.system = system
    table.system = system
    
    try:
        # Connect to both devices in parallel
        logger.info("\n📡 Connecting to devices...")
        await asyncio.gather(
            jet.connect(),
            table.connect()
        )
        
        if not jet.connected or not table.connected:
            logger.error("❌ Failed to connect to one or both devices")
            return
        
        logger.info("\n" + "="*60)
        logger.info("✅ System Ready - Monitoring for events")
        logger.info("="*60)
        logger.info("Workflow: IDLE → Magnet ON → Platform Detection → Rotation")
        logger.info("Press Ctrl+C to stop\n")
        
        system.state = SystemState.JET_RUNNING
        
        # Keep the connection alive and monitor
        while True:
            await asyncio.sleep(1)
            
            # Check if devices are still connected
            if not jet.connected or not table.connected:
                logger.warning("⚠️  Connection lost to one or both devices")
                break
        
    except KeyboardInterrupt:
        logger.info("\n\n🛑 Shutting down...")
        
        # Safety: stop everything
        try:
            await table.stop_rotation()
            await table.deactivate_magnet()
        except:
            pass
            
    except Exception as e:
        logger.error(f"❌ Error in main loop: {e}", exc_info=True)
        
    finally:
        # Disconnect from both devices
        logger.info("Disconnecting devices...")
        await asyncio.gather(
            jet.disconnect(),
            table.disconnect()
        )
        logger.info("✓ Shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("\nExiting...")