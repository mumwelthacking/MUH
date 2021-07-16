#Anleitung nach https://www.raspberrypi-spy.co.uk/2016/07/using-bme280-i2c-temperature-pressure-sensor-in-python/ </br >
#Datasheet BME280 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
#1.0 Das Beispielskript (siehe https://www.raspberrypi-spy.co.uk/2016/07/using-bme280-i2c-temperature-pressure-sensor-in-python/ ) läuft auf python2 deshalb muss noch Software nachinstalliert werden

```
sudo apt-get install python-pip
```
```
sudo python -m pip install smbus
```
#2.0 Optional: Um die I2C Adresse des Sensors zu überprüfen muss das Paket i2c-tools installiert werden
```
sudo apt-get i2c-tools
```
#2.1 die Device i2C Adresse auslesen (aktiviertes i2C vorrausgesetzt, siehe Konfiguration BH1750)
```
i2cdetect -y 1
```
#hier sollte nun 77 als I2C Adresse ausgegeben werden 

#2.2 