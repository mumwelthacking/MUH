# SM-UART-04L (legacy)
Supplier Doku: https://www.mouser.com/pdfDocs/SM-UART-04LApplicationNote.pdf  
Nur per UART ansprechbar --> **wir nehmen den SPS30 weiter unten mit I2C**

## Pi-Stuff
- Raspbian geflashed
- ssh per usb
- ssh per wifi
- passwort geändert, authorized key geadded

feinstaubsensor läuft mit uart auf 3,3V. das passt mit dem pi, 5V wär gefährlich

- pi hat zwei uart-controller:
  - mini-uart ist an prozessorspeed gekoppelt
  - PL011 UART ist unabhängig, kümmert sich aber ums bluetooth
    - --> bluetooth ist uns egal, PL011 ist energiesparender und weniger nervenaufreibend
  - https://www.programmersought.com/article/93804026224/
 
- löten von pi und connector
- python script für serial read (serial_read.py)
  - wie decode ich den readout?
  - bits 5-16 sind interessant. rest wegwerfen und nach int konvertieren
    - zwei aufeinanderfolgende bytes ergeben je einen wert in µg/m³. wert1 * 256 + wert2
- ergebnis plotten :)
- was ist der unteschied zwischen werten 1-6 (standard smoke) und 7-12 (environment)?
