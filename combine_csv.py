import pandas as pd

# CSV-Dateien einlesen
df_iphone = pd.read_csv("iphone.csv")
df_nano = pd.read_csv("nano.csv")

# Zusammenfügen
df_combined = pd.concat([df_iphone, df_nano])

# Nach Timestamp sortieren
df_combined = df_combined.sort_values("timestamp").reset_index(drop=True)

# Als gemeinsame CSV speichern
df_combined.to_csv("combined.csv", index=False)

print("✅ Zusammengeführte CSV erstellt: combined.csv")
