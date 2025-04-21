Pentru placa legata la senzorii de vibratie:
Descarca fisierul ``LastResort.bin`` intr-un folder nou. Deschide linia de comanda in locatia acelui folder.

Ruleaza apoi comanda asta, trebue doar sa schimbi portul COM 
v2(care merge):
```
python -m esptool --chip esp32 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 2MB --flash_freq 40m 0x1000 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x10000 build\newHomeSpanTrial.bin
``
v1(care nu merge):
```esptool.py --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 LastResort.bin```

Deschide apoi arduino IDE si configureaza-l dupa bunul plac.
