import asyncio
import csv
from datetime import datetime
from bleak import BleakScanner, BleakClient
import struct
import numpy as np

class DualIMULogger:
    def __init__(self, nano_name="Nano", iphone_name="iPhoneMotion"):
        self.nano_name = nano_name
        self.iphone_name = iphone_name
        self.nano_address = None
        self.iphone_address = None
        
        # UUIDs - Update these with your actual service and characteristic UUIDs
        self.NANO_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
        self.NANO_CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
        self.IPHONE_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
        self.IPHONE_CHAR_UUID =    "87654321-4321-8765-4321-fedcba987654"

        
        self.nano_data = []
        self.iphone_data = []
        self.csv_filename = f"imu_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.is_recording = False
        
    async def scan_devices(self):
        """Scan for BLE devices and identify Nano and iPhone"""
        print("Scanning for BLE devices...")
        devices = await BleakScanner.discover(timeout=10.0)
        
        for device in devices:
            print(f"Found: {device.name} - {device.address}")
            if self.nano_name in str(device.name):
                self.nano_address = device.address
                print(f"✓ Arduino Nano found: {device.address}")
            if self.iphone_name in str(device.name):
                self.iphone_address = device.address
                print(f"✓ iPhone found: {device.address}")
        
        if not self.nano_address or not self.iphone_address:
            print("\nWarning: Not all devices found!")
            print(f"Nano: {'Found' if self.nano_address else 'Not found'}")
            print(f"iPhone: {'Found' if self.iphone_address else 'Not found'}")
            return False
        return True
    
    def parse_imu_data(self, data):
        if len(data) != 40:
            return None

        # quat(x,y,z,w), accel(x,y,z), mag(x,y,z)
        values = struct.unpack('10f', data)

        return {
            "qx": values[0], "qy": values[1], "qz": values[2], "qw": values[3],
            "ax": values[4], "ay": values[5], "az": values[6],
            "mx": values[7], "my": values[8], "mz": values[9],
        }

    
    def nano_callback(self, sender, data):
        """Callback for Arduino Nano data"""
        if self.is_recording:
            parsed = self.parse_imu_data(data)
            if parsed:
                timestamp = datetime.now().timestamp()
                self.nano_data.append({
                    'timestamp': timestamp,
                    'source': 'nano',
                    **parsed
                })
                print(f"Nano: ax={parsed['ax']:.2f}, ay={parsed['ay']:.2f}, az={parsed['az']:.2f}")
    
    def iphone_callback(self, sender, data):
        """Callback for iPhone data"""
        if self.is_recording:
            parsed = self.parse_imu_data(data)
            if parsed:
                timestamp = datetime.now().timestamp()
                self.iphone_data.append({
                    'timestamp': timestamp,
                    'source': 'iphone',
                    **parsed
                })
                print(f"iPhone: ax={parsed['ax']:.2f}, ay={parsed['ay']:.2f}, az={parsed['az']:.2f}")
    
    async def connect_and_record(self, duration=60):
        """Connect to both devices and record data"""
        print(f"\nConnecting to devices...")
        
        async with BleakClient(self.nano_address) as nano_client, \
                   BleakClient(self.iphone_address) as iphone_client:
            
            print("✓ Connected to both devices")
            
            # Start notifications
            await nano_client.start_notify(self.NANO_CHAR_UUID, self.nano_callback)
            await iphone_client.start_notify(self.IPHONE_CHAR_UUID, self.iphone_callback)
            
            print(f"\n🔴 Recording for {duration} seconds...")
            print("Move both devices identically and simultaneously!\n")
            
            self.is_recording = True
            await asyncio.sleep(duration)
            self.is_recording = False
            
            # Stop notifications
            await nano_client.stop_notify(self.NANO_CHAR_UUID)
            await iphone_client.stop_notify(self.IPHONE_CHAR_UUID)
            
            print(f"\n✓ Recording complete!")
            print(f"Nano samples: {len(self.nano_data)}")
            print(f"iPhone samples: {len(self.iphone_data)}")
    
    def save_to_csv(self):
        """Save collected data to CSV file"""
        if not self.nano_data and not self.iphone_data:
            print("No data to save!")
            return
        
        # Combine and sort by timestamp
        all_data = self.nano_data + self.iphone_data
        all_data.sort(key=lambda x: x['timestamp'])
        
        # Write to CSV
        with open(self.csv_filename, 'w', newline='') as csvfile:
            fieldnames = [
                'timestamp', 'source',
                'qx', 'qy', 'qz', 'qw',
                'ax', 'ay', 'az',
                'mx', 'my', 'mz'
            ]

            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for row in all_data:
                writer.writerow(row)
        
        print(f"✓ Data saved to {self.csv_filename}")
        return self.csv_filename
    
    async def run(self, recording_duration=60):
        """Main execution flow"""
        if await self.scan_devices():
            await self.connect_and_record(recording_duration)
            return self.save_to_csv()
        else:
            print("Failed to find devices. Please check device names and try again.")
            return None


async def main():
    # Update device names to match your actual device names
    logger = DualIMULogger(
        nano_name="Arduino",  # Part of your Arduino's BLE name
        iphone_name="iPhone"   # Part of your iPhone's BLE name
    )
    
    # Record for 60 seconds (adjust as needed)
    csv_file = await logger.run(recording_duration=60)
    
    if csv_file:
        print(f"\n✓ Ready for training! CSV file: {csv_file}")


if __name__ == "__main__":
    asyncio.run(main())