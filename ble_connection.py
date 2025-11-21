#!/usr/bin/env python3
"""
BLE Central Control System for Raspberry Pi
Manages communication between Jet Arduino and Turning Table Arduino
Workflow: Idle detection -> Magnet activation -> Platform detection -> Rotation
"""

import asyncio
import struct
import logging
from bleak import BleakClient, BleakScanner

# ----------------------------
# Logging
# ----------------------------
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ----------------------------
# Device Names / UUIDs
# ----------------------------
JET_NAME = "Eurofighter"
JET_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
JET_DATA_CHAR = "19B10001-E8F2-537E-4F6C-D104768A1214"
JET_CONTROL_CHAR = "19B10003-E8F2-537E-4F6C-D104768A1214"

TABLE_NAME = "TurningTable"
TABLE_SERVICE_UUID = "19B20000-E8F2-537E-4F6C-D104768A1215"
TABLE_STATUS_CHAR = "19B20001-E8F2-537E-4F6C-D104768A1215"
TABLE_COMMAND_CHAR = "19B20002-E8F2-537E-4F6C-D104768A1215"

# ----------------------------
# Commands / Status
# ----------------------------
class JetCmd:
    RUNNING = 0x00
    IDLE = 0x01
    PLATFORM_DETECTED = 0x02

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

# ----------------------------
# System States
# ----------------------------
class SystemState:
    DISCONNECTED = "DISCONNECTED"
    JET_RUNNING = "JET_RUNNING"
    JET_IDLE = "JET_IDLE"
    WAITING_FOR_PLATFORM = "WAITING_FOR_PLATFORM"
    PLATFORM_DETECTED = "PLATFORM_DETECTED"
    ROTATING = "ROTATING"

# ----------------------------
# Jet Device
# ----------------------------
class JetDevice:
    def __init__(self, device, system_controller=None):
        self.name = JET_NAME
        self.device = device
        self.address = device.address
        self.client = None
        self.connected = False
        self.system = system_controller
        self._write_lock = asyncio.Lock()
        self.last_sensor_data = None

    def _notify_data_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_data(sender, data))

    def _notify_control_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_control(sender, data))

    async def notification_handler_data(self, sender, data):
        try:
            fmt = '<I11f'
            if len(data) < struct.calcsize(fmt):
                return
            values = struct.unpack(fmt, data)
            self.last_sensor_data = {
                'timestamp': values[0],
                'accel': {'x': values[1], 'y': values[2], 'z': values[3]},
                'quaternion': {'q0': values[4], 'q1': values[5], 'q2': values[6], 'q3': values[7]},
                'mag': {'x': values[8], 'y': values[9], 'z': values[10]},
                'temp': values[11]
            }
        except Exception:
            logger.exception(f"{self.name}: Error parsing sensor data")

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
                await self.client.start_notify(JET_DATA_CHAR, self._notify_data_callback)
                logger.info("✓ Subscribed to sensor data")
            except Exception as e:
                logger.warning(f"Could not subscribe to sensor data: {e}")
            
            try:
                await self.client.start_notify(JET_CONTROL_CHAR, self._notify_control_callback)
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

# ----------------------------
# Table Device
# ----------------------------
class TableDevice:
    def __init__(self, device, system_controller=None):
        self.name = TABLE_NAME
        self.device = device
        self.address = device.address
        self.client = None
        self.connected = False
        self.system = system_controller
        self._write_lock = asyncio.Lock()

    def _notify_status_callback(self, sender, data):
        asyncio.create_task(self.notification_handler_status(sender, data))

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
                await self.client.start_notify(TABLE_STATUS_CHAR, self._notify_status_callback)
                logger.info("✓ Subscribed to table status")
            except Exception as e:
                logger.warning(f"Could not subscribe to status: {e}")
            
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
                await self.client.write_gatt_char(TABLE_COMMAND_CHAR, bytes([command]))
                await asyncio.sleep(0.1)  # Small delay for command processing
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
# System Controller
# ----------------------------
class SystemController:
    def __init__(self, jet, table):
        self.jet = jet
        self.table = table
        self.state = SystemState.DISCONNECTED
        self.rotation_duration = 10.0
        self._state_lock = asyncio.Lock()
        self._workflow_active = False

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
            
            await asyncio.sleep(1.0)
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
            
            logger.info(f"🔄 Rotating for {self.rotation_duration} seconds...")
            await asyncio.sleep(self.rotation_duration)
            
            await self._complete_workflow()

    async def on_jet_running(self):
        async with self._state_lock:
            if self.state in [SystemState.ROTATING, SystemState.PLATFORM_DETECTED, 
                             SystemState.WAITING_FOR_PLATFORM, SystemState.JET_IDLE]:
                logger.info("✈️  Jet moved → stopping workflow")
                await self._abort_workflow()

    async def _complete_workflow(self):
        """Complete the rotation workflow cleanly"""
        await self.table.stop_rotation()
        await asyncio.sleep(0.3)
        await self.table.deactivate_magnet()
        
        logger.info("\n" + "="*60)
        logger.info("✅ WORKFLOW COMPLETE")
        logger.info("="*60 + "\n")
        
        self.state = SystemState.JET_RUNNING
        self._workflow_active = False

    async def _abort_workflow(self):
        """Abort workflow and return to safe state"""
        await self.table.stop_rotation()
        await asyncio.sleep(0.2)
        await self.table.deactivate_magnet()
        
        logger.info("⚠️  Workflow aborted - returning to normal operation")
        self.state = SystemState.JET_RUNNING
        self._workflow_active = False

# ----------------------------
# BLE Scan & Main
# ----------------------------
async def scan_for_devices(timeout=15.0):
    """Scan once and return device objects"""
    logger.info(f"🔍 Scanning for devices ({timeout}s)...")
    
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

async def main():
    logger.info("\n" + "="*60)
    logger.info("Raspberry Pi BLE Central Control System")
    logger.info("="*60 + "\n")
    
    # Scan for devices
    found = await scan_for_devices()
    
    if JET_NAME not in found:
        logger.error(f"❌ {JET_NAME} not found!")
        return
    if TABLE_NAME not in found:
        logger.error(f"❌ {TABLE_NAME} not found!")
        return

    # Create device objects
    jet = JetDevice(found[JET_NAME])
    table = TableDevice(found[TABLE_NAME])
    system = SystemController(jet, table)
    jet.system = system
    table.system = system

    try:
        # Connect to both devices
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
        logger.info("Press Ctrl+C to stop\n")

        # Monitor connections
        while True:
            await asyncio.sleep(1.0)
            if not jet.is_connected() or not table.is_connected():
                logger.warning("⚠️  Connection lost")
                break
                
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
