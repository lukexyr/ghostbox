import asyncio
import csv
from datetime import datetime
from bleak import BleakScanner, BleakClient
import struct
import websockets
import json

class DualIMULogger:
    def __init__(self, nano_ws_url="ws://localhost:8000/ws", iphone_name="iPhoneMotion"):
        self.nano_ws_url = nano_ws_url
        self.iphone_name = iphone_name
        
        self.iphone_address = None
        
        # UUIDs für iPhone BLE
        self.IPHONE_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
        self.IPHONE_CHAR_UUID    = "87654321-4321-8765-4321-fedcba987654"

        # Datenspeicher
        self.nano_data = []
        self.iphone_data = []

        self.is_recording = False
        self.csv_filename = f"imu_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    # -------------------------------------------------------------------------
    # SCAN: nur iPhone wird über BLE gesucht
    # -------------------------------------------------------------------------
    async def scan_devices(self):
        print("Scanning for BLE devices...")
        devices = await BleakScanner.discover(timeout=10.0)

        for device in devices:
            print(f"Found: {device.name} - {device.address}")
            if self.iphone_name in str(device.name):
                self.iphone_address = device.address
                print(f"✓ iPhone found: {device.address}")

        if not self.iphone_address:
            print("iPhone wurde nicht gefunden!")
            return False

        return True

    # -------------------------------------------------------------------------
    # PARSE: BLE-Daten (10 floats = 40 bytes)
    # -------------------------------------------------------------------------
    def parse_imu_data(self, data):
        if len(data) != 40:
            return None

        values = struct.unpack('10f', data)

        return {
            "qx": values[0], "qy": values[1], "qz": values[2], "qw": values[3],
            "ax": values[4], "ay": values[5], "az": values[6],
            "mx": values[7], "my": values[8], "mz": values[9],
        }

    # -------------------------------------------------------------------------
    # CALLBACK für iPhone BLE
    # -------------------------------------------------------------------------
    def iphone_callback(self, sender, data):
        if not self.is_recording:
            return

        parsed = self.parse_imu_data(data)
        if parsed:
            timestamp = datetime.now().timestamp()
            self.iphone_data.append({
                "timestamp": timestamp,
                "source": "iphone",
                **parsed
            })
            print(f"iPhone: ax={parsed['ax']:.2f}, ay={parsed['ay']:.2f}, az={parsed['az']:.2f}")

    # -------------------------------------------------------------------------
    # WEBSOCKET: Nano-Daten vom lokalen Server abhören
    # -------------------------------------------------------------------------
    async def listen_to_server(self):
        print(f"Connecting to Nano server: {self.nano_ws_url}")

        async with websockets.connect(self.nano_ws_url) as websocket:
            print("✓ Connected to Nano WebSocket!")

            while True:
                if not self.is_recording:
                    await asyncio.sleep(0.01)
                    continue

                msg = await websocket.recv()

                try:
                    data = json.loads(msg)
                except:
                    print("⚠️ Invalid JSON from server")
                    continue

                # Erwartete Werte: qx,qy,qz,qw, ax,ay,az, mx,my,mz
                if not all(key in data for key in ["qx","qy","qz","qw","ax","ay","az","mx","my","mz"]):
                    print("⚠️ Missing IMU fields in Nano data")
                    continue

                timestamp = datetime.now().timestamp()
                self.nano_data.append({
                    "timestamp": timestamp,
                    "source": "nano",
                    **data
                })
                print(f"Nano: ax={data['ax']:.2f}, ay={data['ay']:.2f}, az={data['az']:.2f}")

    # -------------------------------------------------------------------------
    # BLE + WEBSOCKET gleichzeitig ausführen
    # -------------------------------------------------------------------------
    async def connect_and_record(self, duration=30):
        print("Connecting to iPhone BLE...")

        async with BleakClient(self.iphone_address) as iphone_client:

            print("✓ Connected to iPhone")

            await iphone_client.start_notify(self.IPHONE_CHAR_UUID, self.iphone_callback)

            # WebSocket Task starten
            ws_task = asyncio.create_task(self.listen_to_server())

            print(f"\n🔴 Recording for {duration} seconds...\n")
            self.is_recording = True

            await asyncio.sleep(duration)

            self.is_recording = False
            await iphone_client.stop_notify(self.IPHONE_CHAR_UUID)

            print("Stopping WebSocket…")
            ws_task.cancel()

            print(f"✓ Recording done!")
            print(f"iPhone samples: {len(self.iphone_data)}")
            print(f"Nano samples:   {len(self.nano_data)}")

    # -------------------------------------------------------------------------
    # CSV schreiben
    # -------------------------------------------------------------------------
    def save_to_csv(self):
        if not self.nano_data and not self.iphone_data:
            print("Keine Daten vorhanden!")
            return

        all_data = self.nano_data + self.iphone_data
        all_data.sort(key=lambda x: x["timestamp"])

        with open(self.csv_filename, "w", newline="") as csvfile:
            fieldnames = [
                "timestamp", "source",
                "qx","qy","qz","qw",
                "ax","ay","az",
                "mx","my","mz"
            ]

            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            for row in all_data:
                writer.writerow(row)

        print(f"✓ CSV gespeichert: {self.csv_filename}")
        return self.csv_filename

    # -------------------------------------------------------------------------
    # RUN
    # -------------------------------------------------------------------------
    async def run(self, recording_duration=30):
        if not await self.scan_devices():
            return None

        await self.connect_and_record(recording_duration)
        return self.save_to_csv()


# MAIN -------------------------------------------------------------------------
async def main():
    logger = DualIMULogger(
        nano_ws_url="ws://localhost:8000/ws",
        iphone_name="iPhone"
    )

    csv_file = await logger.run(recording_duration=20)

    if csv_file:
        print(f"\n✓ Data ready for ML: {csv_file}")


if __name__ == "__main__":
    asyncio.run(main())
