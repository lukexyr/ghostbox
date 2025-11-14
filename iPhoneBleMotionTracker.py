import asyncio
import struct
from bleak import BleakClient, BleakScanner
from direct.showbase import MessengerGlobal
import csv
from datetime import datetime
messenger = MessengerGlobal.messenger

# BLE Service/Characteristic UUIDs
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHARACTERISTIC_UUID = "87654321-4321-8765-4321-fedcba987654"
CSV_FILE = "motion_data.csv"

class iPhoneBleMotionTracker:
    def __init__(self):
        self.device = None
        self.client = None
        self.quaternion = (0.0, 0.0, 0.0, 1.0)
        self.acceleration = (0.0, 0.0, 0.0)
        self.magnetometer = (0.0, 0.0, 0.0)
        self._callback = None

    with open(CSV_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "label",
            "quat_x", "quat_y", "quat_z", "quat_w",
            "acc_x", "acc_y", "acc_z",
            "mag_x", "mag_y", "mag_z"
        ])

    async def scan_devices(self, timeout=10.0):
        """Scannt nach allen verfügbaren BLE-Geräten und zeigt sie an"""
        print(f"🔍 Scanne {timeout}s nach BLE-Geräten...")
        devices = await BleakScanner.discover(timeout=timeout)
        
        print(f"\n📱 Gefundene Geräte ({len(devices)}):")
        for i, d in enumerate(devices):
            print(f"  [{i}] Name: {d.name or 'N/A':<20} | Adresse: {d.address}")
        
        return devices

    async def connect(self, timeout=15.0):
        """Verbindet mit dem iPhone Motion Tracker"""
        print(f"🔍 Suche nach 'iPhoneMotion' (Timeout: {timeout}s)...")
        
        # Methode 1: Nach Name filtern
        device = await BleakScanner.find_device_by_filter(
            lambda d, ad: d.name and "iPhoneMotion" in d.name,
            timeout=timeout
        )

        # Methode 2: Falls Name nicht funktioniert, nach Service-UUID suchen
        if not device:
            print("⚠️ Name nicht gefunden, suche nach Service-UUID...")
            device = await BleakScanner.find_device_by_filter(
                lambda d, ad: SERVICE_UUID.lower() in [str(u).lower() for u in ad.service_uuids],
                timeout=timeout
            )

        # Methode 3: Alle Geräte anzeigen zur manuellen Auswahl
        if not device:
            print("\n⚠️ Automatische Suche fehlgeschlagen!")
            devices = await self.scan_devices(timeout=5.0)
            
            if devices:
                print("\n💡 Möchtest du ein Gerät manuell auswählen?")
                print("   Starte die App neu mit device_address Parameter")
            
            raise RuntimeError("❌ Kein iPhoneMotion-Gerät gefunden!")

        self.device = device
        print(f"✅ Gefunden: {device.name} ({device.address})")
        
        # Verbindung herstellen
        print("🔗 Verbinde...")
        self.client = BleakClient(device.address, timeout=timeout)
        
        try:
            await self.client.connect()
            print(f"✅ Verbunden mit {device.address}")
            
            # Services auflisten
            print("\n📋 Verfügbare Services:")
            for service in self.client.services:
                print(f"  Service: {service.uuid}")
                for char in service.characteristics:
                    print(f"    └─ Char: {char.uuid} | Props: {char.properties}")
            
            # Notifications starten
            print(f"\n🔔 Starte Notifications für {CHARACTERISTIC_UUID}...")
            await self.client.start_notify(CHARACTERISTIC_UUID, self._notification_handler)
            print("✅ Notifications aktiv!")
            messenger.send("iphone-connected")
            
        except Exception as e:
            print(f"❌ Verbindungsfehler: {e}")
            raise

    async def connect_by_address(self, address, timeout=15.0):
        """Verbindet direkt mit einer MAC-Adresse"""
        print(f"🔗 Verbinde mit {address}...")
        
        self.client = BleakClient(address, timeout=timeout)
        await self.client.connect()
        print(f"✅ Verbunden!")
        
        await self.client.start_notify(CHARACTERISTIC_UUID, self._notification_handler)
        print("✅ Notifications gestartet")

    async def disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print("🔌 Verbindung getrennt")

    def _notification_handler(self, sender: int, data: bytearray):
        """Callback für empfangene Quaternion + Accelerometer + Magnetometer-Daten"""
        # Neues Format: 40 bytes
        # - 16 bytes: Quaternion (4x float32)
        # - 12 bytes: Acceleration (3x float32)
        # - 12 bytes: Magnetometer (3x float32)
        
        if len(data) >= 40:
            # Quaternion auspacken (bytes 0-15)
            self.quaternion = struct.unpack("<4f", data[:16])
            
            # Acceleration auspacken (bytes 16-27)
            self.acceleration = struct.unpack("<3f", data[16:28])
            
            # Magnetometer auspacken (bytes 28-39)
            self.magnetometer = struct.unpack("<3f", data[28:40])
            
            if self._callback:
                self._callback(self.quaternion, self.acceleration, self.magnetometer)
                
        elif len(data) >= 28:
            # Rückwärtskompatibilität: Quaternion + Acceleration (ohne Magnetometer)
            self.quaternion = struct.unpack("<4f", data[:16])
            self.acceleration = struct.unpack("<3f", data[16:28])
            self.magnetometer = (0.0, 0.0, 0.0)
            
            if self._callback:
                self._callback(self.quaternion, self.acceleration, self.magnetometer)
                
        elif len(data) >= 16:
            # Rückwärtskompatibilität: Nur Quaternion
            self.quaternion = struct.unpack("<4f", data[:16])
            self.acceleration = (0.0, 0.0, 0.0)
            self.magnetometer = (0.0, 0.0, 0.0)
            
            if self._callback:
                self._callback(self.quaternion, self.acceleration, self.magnetometer)
        else:
            print(f"⚠️ Ungültige Datenlänge: {len(data)} bytes (erwartet 40)")

    def set_callback(self, callback):
        """Setzt Callback-Funktion für neue Quaternions + Acceleration + Magnetometer
        
        Callback-Signatur: callback(quaternion: tuple, acceleration: tuple, magnetometer: tuple)
        - quaternion: (x, y, z, w)
        - acceleration: (x, y, z) in G (Earth's gravity)
        - magnetometer: (x, y, z) in µT (microtesla)
        """
        self._callback = callback

    async def run_forever(self):
        """Hauptloop - bleibt verbunden und empfängt Daten"""
        try:
            await self.connect()
            print("\n🎯 Warte auf Motion-Daten... (Strg+C zum Beenden)\n")
            
            while True:
                await asyncio.sleep(1)
                
        except KeyboardInterrupt:
            print("\n⏸️ Beende...")
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"❌ Fehler: {e}")
        finally:
            await self.disconnect()


# MARK: - Beispiel-Nutzung
async def main():
    tracker = iPhoneBleMotionTracker()
    
    def on_motion_data(quat, accel, mag):
        """Callback der bei jedem Update aufgerufen wird"""
        timestamp = datetime.now().isoformat(sep=' ', timespec='milliseconds')
        label = "iPhone"
        qx, qy, qz, qw = quat
        ax, ay, az = accel
        mx, my, mz = mag

        # In CSV schreiben
        with open(CSV_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, label, qx, qy, qz, qw, ax, ay, az, mx, my, mz])

        # Optional: Ausgabe auf Konsole
        print(f"{timestamp} | 📐 Quat: [{qx:+.3f}, {qy:+.3f}, {qz:+.3f}, {qw:+.3f}] | "
            f"🚀 Accel: [{ax:+.3f}, {ay:+.3f}, {az:+.3f}] G | "
            f"🧲 Mag: [{mx:+.1f}, {my:+.1f}, {mz:+.1f}] µT", end='\r')
    
    tracker.set_callback(on_motion_data)
    await tracker.run_forever()


if __name__ == "__main__":
    asyncio.run(main())