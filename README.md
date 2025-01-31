# bme280_espidf
integrating bme280 with esp32 without using coines library


for use with bme280 in esp idf, current issues:

the drivers packaged here (bme280.c, bme280.h, bme280\_defs.h) are NOT COMPATIBLE WITH THE OFFICIAL DRIVERS FROM BOSCH due to a change in my read/write function in order to get them working. if you're looking for the official drivers, see: <https://github.com/boschsensortec/BME280_SensorAPI/tree/master>. 

working on fix for compatibility issue currently
