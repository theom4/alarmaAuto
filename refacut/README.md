Pentru placa legata la senzorii de vibratie:
Descarca fisierul ``LastResort.bin`` intr-un folder nou. Deschide linia de comanda in locatia acelui folder.

Ruleaza apoi comanda asta, trebue doar sa schimbi portul COM 

```esptool.py --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 LastResort.bin```

Deschide apoi arduino IDE si configureaza-l dupa bunul plac.
