import asyncio
import csv
import json
from datetime import datetime
import websockets

class NanoLogger:
    def __init__(self, ws_url="ws://localhost:8000/ws", csv_filename=None):
        self.ws_url = ws_url
        self.csv_filename = csv_filename or f"nano_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.is_recording = False
        self.data = []

    async def listen_to_server(self):
        """Verbindet zum lokalen Server und empfängt Daten"""
        print(f"🌐 Verbinde zu Nano WebSocket: {self.ws_url}")
        try:
            async with websockets.connect(self.ws_url) as websocket:
                print("✅ Verbunden zum Nano WebSocket!")

                self.is_recording = True
                while self.is_recording:
                    msg = await websocket.recv()
                    try:
                        data = json.loads(msg)
                    except json.JSONDecodeError:
                        print("⚠️ Ungültiges JSON erhalten")
                        continue

                    # Prüfen, ob alle erwarteten Felder da sind
                    expected_keys = ["qx","qy","qz","qw","ax","ay","az","mx","my","mz"]
                    if not all(k in data for k in expected_keys):
                        print("⚠️ Fehlende IMU-Felder")
                        continue

                    # Zeitstempel hinzufügen
                    timestamp = datetime.now().timestamp()
                    self.data.append({
                        "timestamp": timestamp,
                        "source": "nano",
                        **data
                    })

                    print(f"Nano: ax={data['ax']:.3f}, ay={data['ay']:.3f}, az={data['az']:.3f}", end='\r')

        except Exception as e:
            print(f"❌ Fehler bei Verbindung: {e}")

    def save_to_csv(self):
        """Speichert die empfangenen Daten in einer CSV-Datei"""
        if not self.data:
            print("Keine Daten zum Speichern vorhanden!")
            return

        fieldnames = ["timestamp", "source", "qx","qy","qz","qw","ax","ay","az","mx","my","mz"]
        with open(self.csv_filename, "w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.data:
                writer.writerow(row)

        print(f"✅ CSV gespeichert: {self.csv_filename}")
        return self.csv_filename

# -------------------------------------------------------------------------
# Beispiel-Nutzung
# -------------------------------------------------------------------------
async def main():
    logger = NanoLogger(ws_url="ws://localhost:8000/ws")
    
    # Aufzeichnen für 30 Sekunden
    recording_task = asyncio.create_task(logger.listen_to_server())
    await asyncio.sleep(30)  # Dauer anpassen
    logger.is_recording = False
    await asyncio.sleep(1)  # kurze Verzögerung, um letzte Nachrichten zu erhalten

    logger.save_to_csv()

if __name__ == "__main__":
    asyncio.run(main())
