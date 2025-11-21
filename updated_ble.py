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
JET_RX_CHAR = "19B10004-E8F2-537E-4F6C-D104768A1214"

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
    CALIBRATE_MAG = 0x03

class TableCmd:
    ACTIVATE_MAGNET = 0x10
    DEACTIVATE_MAGNET = 0x11
    START_ROTATION = 0x12
    STOP_ROTATION = 0x13
    SET_SPEED = 0x14

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
    def __init__(self, address, system_controller=None):
        self.name = JET_NAME
        self.address = address
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
            self.client = BleakClient(self.address)
            await self.client.connect()
            self.connected = await self._is_connected()
            if self.connected:
                await self.client.start_notify(JET_DATA_CHAR, self._notify_data_callback)
                await self.client.start_notify(JET_CONTROL_CHAR, self._notify_control_callback)
        except Exception:
            self.connected = False
            logger.exception(f"{self.name}: Failed to connect")

    async def disconnect(self):
        if self.client and self.connected:
            try:
                await self.client.disconnect()
                self.connected = False
            except Exception:
                logger.exception(f"{self.name}: Failed to disconnect")

    async def _is_connected(self):
        try:
            return bool(self.client and self.client.is_connected)
        except Exception:
            return False

    async def send_command(self, command):
        if not (self.client and self.connected and await self._is_connected()):
            return False
        async with self._write_lock:
            try:
                await self.client.write_gatt_char(JET_RX_CHAR, bytes([command]))
                return True
            except Exception:
                logger.exception(f"{self.name}: Failed to send 0x{command:02X}")
                return False

# ----------------------------
# Table Device
# ----------------------------
class TableDevice:
    def __init__(self, address, system_controller=None):
        self.name = TABLE_NAME
        self.address = address
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
            self.client = BleakClient(self.address)
            await self.client.connect()
            self.connected = await self._is_connected()
            if self.connected:
                await self.client.start_notify(TABLE_STATUS_CHAR, self._notify_status_callback)
        except Exception:
            self.connected = False
            logger.exception(f"{self.name}: Failed to connect")

    async def disconnect(self):
        if self.client and self.connected:
            try:
                await self.client.disconnect()
                self.connected = False
            except Exception:
                logger.exception(f"{self.name}: Failed to disconnect")

    async def _is_connected(self):
        try:
            prop = getattr(self.client, "is_connected", None)
            if callable(prop):
                return await prop()
            return bool(prop)
        except Exception:
            return False

    async def send_command(self, command):
        if not (self.client and self.connected and await self._is_connected()):
            return False
        async with self._write_lock:
            try:
                await self.client.write_gatt_char(TABLE_COMMAND_CHAR, bytes([command]))
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

    async def on_jet_idle(self):
        if self.state != SystemState.JET_RUNNING:
            return
        logger.info("IDLE detected → activating magnet")
        self.state = SystemState.JET_IDLE
        await self.table.activate_magnet()
        await asyncio.sleep(0.5)
        await self.table.deactivate_magnet()
        self.state = SystemState.WAITING_FOR_PLATFORM
        logger.info("Waiting for platform detection")

    async def on_platform_detected(self):
        if self.state != SystemState.WAITING_FOR_PLATFORM:
            return
        logger.info("Platform detected → start rotation")
        await self.table.start_rotation()
        self.state = SystemState.ROTATING

    async def on_jet_running(self):
        if self.state == SystemState.ROTATING:
            logger.info("Jet moved → stop rotation")
            await self.table.stop_rotation()
            await self.table.deactivate_magnet()
            self.state = SystemState.JET_RUNNING
        elif self.state in [SystemState.JET_IDLE, SystemState.WAITING_FOR_PLATFORM]:
            logger.info("Jet moved before detection → reset state")
            self.state = SystemState.JET_RUNNING

# ----------------------------
# BLE Scan & Main
# ----------------------------
async def scan_for_devices(timeout=10.0):
    devices = await BleakScanner.discover(timeout=timeout)
    found = {}
    for d in devices:
        name = d.name or ""
        if name == JET_NAME:
            found[JET_NAME] = d.address
        elif name == TABLE_NAME:
            found[TABLE_NAME] = d.address
    return found

async def main():
    found = await scan_for_devices()
    if JET_NAME not in found or TABLE_NAME not in found:
        logger.error("Devices not found")
        return

    jet = JetDevice(found[JET_NAME])
    table = TableDevice(found[TABLE_NAME])
    system = SystemController(jet, table)
    jet.system = system
    table.system = system

    async def connect_with_retry(device, attempts=5, delay=1.0):
        for _ in range(attempts):
            try:
                await device.connect()
                if device.connected:
                    return True
            except Exception:
                pass
            await asyncio.sleep(delay)
        return False

    connected = await asyncio.gather(connect_with_retry(jet), connect_with_retry(table))
    if not all(connected):
        logger.error("Failed to connect to devices")
        return

    system.state = SystemState.JET_RUNNING

    try:
        while True:
            await asyncio.sleep(0.5)
            if not (await jet._is_connected()) or not (await table._is_connected()):
                logger.warning("Connection lost → stop system")
                await table.stop_rotation()
                await table.deactivate_magnet()
                system.state = SystemState.DISCONNECTED
                break
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        await table.stop_rotation()
        await table.deactivate_magnet()
    finally:
        await asyncio.gather(jet.disconnect(), table.disconnect())
        logger.info("Shutdown complete")

if __name__ == "__main__":
    asyncio.run(main())
