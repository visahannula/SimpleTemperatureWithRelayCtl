# Basic temperature monitoring and management with Arduino

This code reads temperature from a temperature sensor and
then control a relay (assumed a heater).

# Serial commands:
## Get temperature
 - Command T
 - Returns all the connected sensors temperatures

## Set temperature
 - Command S<sensor bus num>[H|L] <float temp>
 - Allows to set high temp with S (ex. S1H 22.1)
 - Set low temp (ex. S1L 20.5)


This is made with Arduino Uno board, DS18B20 Temperature sensors and
a relay board.

2020 Visa Hannula (visa at viha.fi).

USE AT YOUR OWN RISK, NO WARRANTY.
