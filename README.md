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

This was supposed to read standard input for (binary) HIPPO messages, but I never got that to work, probably python/something else mangles the bytes...

Following HIPPO messages are parsed and converted to following messages:

| HIPPO message | HIPPO description | NMEA message | NMEA description |
| --------------|-------------------|--------------|------------------|
| 0x30 0x02 | Fast fix data | RMC<br />VTG | Recommended minimum (position, velocity, time) <br />Track made good and ground speed |
| 0x31 0x01 | GPS fix | GSA<br />GGA| GPS DOP and Active satellites<br />GPS Fix data
| 0x32 0x01 | UTC time and constellation | ZDA<br />GSA | Date and time<br />GPS DOP and Active satellites
| 0x32 0x03 | UTC time | ZDA | Date and time |
| 0x33 0x01 | GPS channel measurement short status | GSV | GPS satellites in view |

The message mapping is not 1:1 which is why some HIPPO messages result in two different types of messages, or only part of the NMEA message is updated by one HIPPO message.
