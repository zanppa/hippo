# hippo
Convert HIPPO GPS messages to NMEA

This repository contains code to convert (a subset of) HIPPO GPS messages to NMEA.

HIPPO protocol is used at least by some GPS modules by [Trimble](https://www.trimble.com/), I have used [Lassen DR+GPS](http://trl.trimble.com/docushare/dsweb/Get/Document-362863/DR_GPS_Manual_58059-00_RevA.pdf). The HIPPO protocol is described starting from page 44 of that document.

The basic version of the converter works with Raspberry PI and uses the pigpio library for reading the GPS with software UART. The messages are converted to NMEA and output to standard output. The idea is that the output can be piped to a virtual serial port and then used for example with gspd.

The basic usage is shown in run.sh:
```
python -u hippo_pigpio.py | socat -d -d stdin pty,raw,echo=0
```
socat creates an virtual serial port. 
