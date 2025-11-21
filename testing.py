#!/usr/bin/env python3
"""
Simulation harness for the BLE Central Control System.

- Reuses SystemController as-is.
- Replaces JetDevice and TableDevice with mock classes that:
  - Never touch real BLE.
  - Log commands / state instead.
- Simulates the Jet sending RUNNING / IDLE / PLATFORM_DETECTED
  notifications and checks how the SystemController reacts.
"""

import asyncio
import logging

# ----------------------------
# Copy / import enums & states
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


class SystemState:
    DISCONNECTED = "DISCONNECTED"
    JET_RUNNING = "JET_RUNNING"
    JET_IDLE = "JET_IDLE"
    WAITING_FOR_PLATFORM = "WAITING_FOR_PLATFORM"
    PLATFORM_DETECTED = "PLATFORM_DETECTED"
    ROTATING = "ROTATING"


# ----------------------------
# SystemController (unchanged)
# ----------------------------

class SystemController:
    def __init__(self, jet, table):
        self.jet = jet
        self.table = table
        self.state = SystemState.DISCONNECTED

    async def on_jet_idle(self):
        if self.state != SystemState.JET_RUNNING:
            logging.info("on_jet_idle ignored in state %s", self.state)
            return
        logging.info("IDLE detected → activating magnet")
        self.state = SystemState.JET_IDLE
        await self.table.activate_magnet()
        await asyncio.sleep(0.5)
        await self.table.deactivate_magnet()
        self.state = SystemState.WAITING_FOR_PLATFORM
        logging.info("Waiting for platform detection")

    async def on_platform_detected(self):
        if self.state != SystemState.WAITING_FOR_PLATFORM:
            logging.info("on_platform_detected ignored in state %s", self.state)
            return
        logging.info("Platform detected → start rotation")
        await self.table.start_rotation()
        self.state = SystemState.ROTATING

    async def on_jet_running(self):
        if self.state == SystemState.ROTATING:
            logging.info("Jet moved → stop rotation")
            await self.table.stop_rotation()
            await self.table.deactivate_magnet()
            self.state = SystemState.JET_RUNNING
        elif self.state in [SystemState.JET_IDLE, SystemState.WAITING_FOR_PLATFORM]:
            logging.info("Jet moved before detection → reset state")
            self.state = SystemState.JET_RUNNING
        else:
            logging.info("on_jet_running in state %s → no special action", self.state)


# ----------------------------
# Mock devices
# ----------------------------

class MockJetDevice:
    """Mimics JetDevice but without BLE."""
    def __init__(self, system_controller=None):
        self.name = "MockEurofighter"
        self.system = system_controller
        self.connected = False

    async def connect(self):
        logging.info("%s: mock connect", self.name)
        self.connected = True

    async def disconnect(self):
        logging.info("%s: mock disconnect", self.name)
        self.connected = False

    # Simulation helpers to imitate notifications:
    async def simulate_idle(self):
        logging.info("%s: simulate IDLE notification", self.name)
        if self.system:
            await self.system.on_jet_idle()

    async def simulate_running(self):
        logging.info("%s: simulate RUNNING notification", self.name)
        if self.system:
            await self.system.on_jet_running()

    async def simulate_platform_detected(self):
        logging.info("%s: simulate PLATFORM_DETECTED notification", self.name)
        if self.system:
            await self.system.on_platform_detected()


class MockTableDevice:
    """Mimics TableDevice but logs commands instead of using BLE."""
    def __init__(self, system_controller=None):
        self.name = "MockTurningTable"
        self.system = system_controller
        self.connected = False
        self.magnet_on = False
        self.rotating = False

    async def connect(self):
        logging.info("%s: mock connect", self.name)
        self.connected = True

    async def disconnect(self):
        logging.info("%s: mock disconnect", self.name)
        self.connected = False

    async def activate_magnet(self):
        logging.info("→ [MOCK] Activating magnet")
        self.magnet_on = True

    async def deactivate_magnet(self):
        logging.info("→ [MOCK] Deactivating magnet")
        self.magnet_on = False

    async def start_rotation(self):
        logging.info("→ [MOCK] Starting rotation")
        self.rotating = True

    async def stop_rotation(self):
        logging.info("→ [MOCK] Stopping rotation")
        self.rotating = False


# ----------------------------
# Simulation Scenarios
# ----------------------------

async def scenario_normal_landing(system, jet, table):
    """
    Jet running → idle over table → platform detected → rotation →
    jet runs again (leaves table).
    """
    logging.info("=== Scenario: normal landing and rotate ===")
    system.state = SystemState.JET_RUNNING
    logging.info("Initial state: %s", system.state)

    await jet.simulate_idle()              # triggers magnet pulse and WAITING_FOR_PLATFORM
    await asyncio.sleep(0.1)
    await jet.simulate_platform_detected() # triggers rotation
    await asyncio.sleep(0.1)
    await jet.simulate_running()           # jet moves, stop rotation + magnet off

    logging.info("Final system state: %s", system.state)
    logging.info("Table magnet_on=%s, rotating=%s", table.magnet_on, table.rotating)


async def scenario_abort_before_platform(system, jet, table):
    """
    Jet goes idle but then moves (RUNNING) before platform detected.
    Ensures state machine resets correctly.
    """
    logging.info("=== Scenario: abort before platform ===")
    system.state = SystemState.JET_RUNNING
    logging.info("Initial state: %s", system.state)

    await jet.simulate_idle()
    await asyncio.sleep(0.1)
    await jet.simulate_running()  # should reset to JET_RUNNING, no rotation

    logging.info("Final system state: %s", system.state)
    logging.info("Table magnet_on=%s, rotating=%s", table.magnet_on, table.rotating)


async def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )

    # Wire up mock devices and controller
    jet = MockJetDevice()
    table = MockTableDevice()
    system = SystemController(jet, table)
    jet.system = system
    table.system = system

    await asyncio.gather(jet.connect(), table.connect())
    system.state = SystemState.JET_RUNNING

    await scenario_normal_landing(system, jet, table)
    await asyncio.sleep(0.5)
    await scenario_abort_before_platform(system, jet, table)

    await asyncio.gather(jet.disconnect(), table.disconnect())
    logging.info("Simulation complete.")


if __name__ == "__main__":
    asyncio.run(main())


