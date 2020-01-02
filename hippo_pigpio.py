#!/usr/bin/env python

# hippo_pigpio.py
# Reads HIPPO GPS protocol (with bit-banged serial port) and converts it to NMEA which is
# output to standard output
# Lauri Peltonen, 2019

# Original serial port code from https://raspberrypi.stackexchange.com/questions/27488/pigpio-library-example-for-bit-banging-a-uart
# Original license:
# bb_serial.py
# 2014-12-23
# Public Domain

import time
from struct import unpack
import pigpio
from math import ceil
import sys

do_debug = 0

# Raspberry PI pin to use for reading, same as the programming interface MISO
RX = 15

# Baud rate etc
baud = 38400
bits = 8


def convertToW(tow):
   """Convert time of week to hours, minutes, seconds, milliseconds"""
   msecs = tow % 1000
   tow = int(tow/1000)
   secs = tow % 60
   tow = int(tow/60)
   mins = tow % 60
   tow = int(tow/60)
   hours = tow % 24
   return (hours, mins, secs, msecs)

def checksum_nmea(data):
   """Calculate the NMEA checksum from the string, including the
   leading $ and last comma but without the trailing *"""
   data = data[1:]	# Strip the leading $
   sum = 0
   for byte in data:
      sum ^= ord(byte)
   return sum

def decimalToMinutes(coord):
   """Convert decimal representation to degrees, minutes and return as abs value string"""
   coord = abs(coord)
   degrees = int(coord)
   decimal = coord - degrees
   minutes = decimal * 60.0
   return '{:02d}{:02.4f}'.format(degrees, minutes)


def make_nmea_gga(data, gpstime):
   """Convert the position struct into NMEA GGA (GPS Fix data) string"""
   output = '$GPGGA,'

   # TODO: Might be a bad idea to use the time-of-week since there seems to be a difference
   if 'hour' in gpstime:
      output += '{:02d}{:02d}{:02d},'.format(gpstime['hour'], gpstime['minute'], gpstime['second'])
   else:
      (hours, mins, secs, msecs) = convertToW(data['time_of_week'])
      output += '{:02d}{:02d}{:02d}.{:02d},'.format(hours, mins, secs, msecs)

   #(hours, mins, secs, msecs) = convertToW(data['time_of_week'])
   #output += '{:02d}{:02d}{:02d}.{:02d},'.format(hours, mins, secs, msecs)
#   output += '{:02d}{:02d}{:02d},'.format(hours, mins, secs)

   if data['position_valid']:
      # TODO: Fix N and S and maybe also the degree values accordingly?
      output += '{},{},{},{},'.format(decimalToMinutes(data['lat']), 'N' if data['lat']>=0 else 'S', decimalToMinutes(data['lon']), 'E' if data['lon']>=0 else 'W')
   else:
      output += ',,,,'

   output += '1,'  # Quality indicator, 0=No GPS, 1=GPS, 2=DGPS

   if 'tracked_count' in position:
     output += '{},'.format(data['tracked_count']) # Number of satellites tracked
   else:
      output += ','

   if 'hdop' in data:
      output += '{:.2f},'.format(data['hdop'])
   else:
      output += ','

   if data['altitude_valid']:
      output += '{:.1f},M,'.format(data['alt'])
   else:
      output += ',M,'

   output += ',M,'	# TODO: Geoidial separation in meters, dunno how to get this from HIPPO?
   output += ','       # Age of differential GPS data and station ID, not in use

   output += '*{:02X}'.format(checksum_nmea(output))

   return output


def make_nmea_rmc(data, gpstime):
   """Convert the position struct into NMEA RMC (Recom. minimum spec. data) string"""
   output = '$GPRMC,'

   # TODO: Might be a bad idea to use the time-of-week since there seems to be a difference
   if 'hour' in gpstime:
      output += '{:02d}{:02d}{:02d},'.format(gpstime['hour'], gpstime['minute'], gpstime['second'])
   else:
      (hours, mins, secs, msecs) = convertToW(data['time_of_week'])
      output += '{:02d}{:02d}{:02d}.{:02d},'.format(hours, mins, secs, msecs)

   output += 'A,'  # Always A=Active, could also be V=void/warning

   if data['position_valid']:
      output += '{},{},{},{},'.format(decimalToMinutes(data['lat']), 'N' if data['lat']>=0 else 'S', decimalToMinutes(data['lon']), 'E' if data['lon']>=0 else 'W')
   else:
      output += ',,,,'

   if data['speed_valid']:
      output += '{:.2f},'.format(data['speed'] * 0.01943844) # cm/s to knots
   else:
      output += ','

   if 'heading' in position:
      output += '{},'.format(position['heading'])
   else:
      output += ','  # TODO: Track made good in degrees true, maybe heading?

   if 'year' in gpstime:
      output += '{:02d}{:02d}{:02d},'.format(gpstime['day'], gpstime['month'], gpstime['year']%100)
   else:
      output += ','

   output += ',,'  # TODO: Magnetic variation in degrees
   output += 'A'  # Position mode, A=Autonomous, D=Differential, E=Estimated, M=Manual, S=simulation, N=Not valid

   output += '*{:02X}'.format(checksum_nmea(output))

   return output


def make_nmea_gsv(position, sats, msg_count, msg_n):
   """Make GSV (GPS Satellites in view) report, given visible satellites and message number"""
   msg = '$GPGSV,'
   msg += '{},{},'.format(msg_count, msg_n+1)  # Total messages, current message
   n_sats = len(sats)
   msg += '{},'.format(n_sats)  # Satellites in view

   first = msg_n*4
   for i in range(4):
      if first+i >= n_sats or position['channel'][sats[first+i]] is None:
         msg += ',,,,'
         continue

      ch_no = sats[first+i]

      msg += '{:d},'.format(position['channel'][ch_no]['prn'])
      msg += '{:d},{:d},'.format(position['channel'][ch_no]['elevation'], position['channel'][ch_no]['azimuth'])

      if position['channel'][ch_no]['tracking']:
         msg += '{:d},'.format(position['channel'][ch_no]['snr'])
      else:
         msg += ','

   msg = msg[:-1] # Remove trailing ,

   msg += '*{:02X}'.format(checksum_nmea(msg))
   return msg


def make_nmea_gsa(position):
   """Make GSA (GPS DOP and Active Satellites) message"""
   msg = '$GPGSA,A,' # Always in autmatic mode, M would be manual

   if not 'fix_source' in position or position['fix_source'] < 13:
      # Assume no fix if old fix or internal position or whatever
      msg += '3,'  # 1=Fix not available, 2 = 2D, 3 = 3D
   elif position['fix_source'] < 17:
      msg += '3,'
   else:
      msg += '3,'

   count = 0
   for i in range(12):
      if position['channel'][i] is not None and position['channel'][i]['tracking']:
         msg += '{},'.format(position['channel'][i]['prn'])
         count +=1
   for i in range(count,12):
      msg += ','

   msg += '{},{},{}'.format(position['pdop'], position['hdop'], position['vdop'])

   msg += '*{:02X}'.format(checksum_nmea(msg))
   return msg

def make_nmea_vtg(position):
   """Make VTG (Track made good and ground speed) message"""
   msg = '$GPVTG,'

   if 'heading' in position:
      msg += '{},,'.format(position['heading'])  # TODO Only degrees true at the moment
   else:
      msg += ',,'

   msg += '{},{},'.format(position['speed']*0.01943844, position['speed']*0.036)  # cm/s to knots and km/h
   msg += 'A' # Mode indicator, currently only automatic

   msg += '*{:02X}'.format(checksum_nmea(msg))
   return msg


def make_nmea_zda(gpstime):
   """Make ZDA (Date & time) message"""
   msg = '$GPZDA,'

   if 'hour' in gpstime:
      msg += '{:02d}{:02d}{:02d},'.format(gpstime['hour'], gpstime['minute'], gpstime['second'])
      msg += '{:02d},{:02d},{:04d},'.format(gpstime['day'], gpstime['month'], gpstime['year'])
   else:
      msg += ',,,,'

   msg += ','  # Local zone description (hours) and minutes

   msg += '*{:02X}'.format(checksum_nmea(msg))
   return msg



def check_checksum_hippo(data):
   """Check that HIPPO checksum is correct"""
   sum = 0
   for byte in data:
      sum += byte
   if (sum % 256) == 0:
      return (True, sum)
   else:
      return (False, sum)


# This struct always contains the latest values
position = { \
   'position_valid' : False, \
   'altitude_valid' : False, \
   'heading_valid' : False, \
   'speed_valid' : False, \
   'age' : 0, # Seconds, 254 = more than 253s, 255 = GPS N/A \
   'time_of_week' : 0,  # Milliseconds \
   'lat' : 0.0,  # [-0.5 ... 0.5] sc \
   'lon' : 0.0,  # [-1 ... 1) sc \
   'alt' : 0.0,  # Altitude meters above MSL (-400 .... 10000 m) \
   'heading' : 0,  # [0 ... 2) sc \
   'speed' : 0,  # cm/s [0 ... 655,34] m/s \
   'hdop' : 255.0,  # 255 seems to be maximum \
   'vdop' : 255.0, \
   'pdop' : 255.0, \
   'SV': 0, \
   'channel' : [None]*12 # 12 channels max
   }

# This dictionary holds the GPS reported time
gps_time = {}


# Dictionary holding the expected HIPPO message lengths
# Only data bytes are counted (no header, checksum or end of mesasge indicator)
# These are provided to ease parsing (detecting correct start of message
# TODO: Only some message lengths are provided
msg_len = { \
   0x1201 : 4, \
   0x1203 : 24, \
   0x1204 : 16, \
   0x1401 : 9, \
   0x1402 : 31, \
   0x1601 : 3, \
   0x1602 : 9, \
   0x2201 : 31, \
   0x2202 : 5, \
   0x2601 : 16, \
   0x2602 : 16, \
   0x3002 : 46, \
   0x3101 : 28, \
   0x3201 : 18, \
   0x3203 : 15, \
   0x3301 : 7, # Including index byte \
   0x3603 : 9, \
   0x3604 : 9, \
   0x3605 : 2, \
   0x3607 : 10, # + index \
   0x3608 : 9, \
   0x3F01 : 13, \
   0x3003 : 101, \
   0x2812 : 26, \
   0x2813 : 50, \
   0x2814 : 26, \
   0x2816 : 74, \
   0x2901 : 11, \
   0x2902 : 21, \
   0x2903 : 5, \
   0x2904 : 7, \
   0x2905 : 5, \
   0x2907 : 25, \
   0x2908 : 9 \
   }


pi = pigpio.pi()

# fatal exceptions off (so that closing an unopened gpio doesn't error)
pigpio.exceptions = False
pi.bb_serial_read_close(RX)

# fatal exceptions on
pigpio.exceptions = True

# open a gpio to bit bang read the echoed data
pi.bb_serial_read_open(RX, baud, bits)

data = []
bytes_read = 0
expected_length = 0
in_message = False
parse_now = False
unstuff_next = False
last_sat_time = 0

while 1:
   (raw_count, raw_data) = pi.bb_serial_read(RX)

   if do_debug > 2 and raw_count > 0:
      # Print received data in HEX
      sys.stderr.write('{}\n'.format(' '.join(format(x, '02x') for x in raw_data)))

   for i in range(raw_count):
      databyte = raw_data[i]

      # Messages start with 0x81 and end with 0x82
      # 0x81 never appears inside a message!
      if not in_message and databyte == 0x81:
         in_message = True
         data = []
         bytes_read = 0
         expected_length = 0
         unstuff_next = False

      if in_message:
         if databyte == 0x80: # Next byte must be unstuffed
            unstuff_next = True
            continue

         # Unstuff this byte
         if unstuff_next:
            databyte = databyte | 0x80
            unstuff_next = False

         data.append(databyte)
         bytes_read += 1

         if bytes_read == 3:  # Parser codes received
            packet = (data[1] << 8) + data[2]
            if not packet in msg_len:
               if do_debug:
                  sys.stderr.write('Packet length not known: 0x{:X}\n'.format(packet))
               expected_length = 0 # Read until next 0x82 and try again
            else:
               expected_length = msg_len[packet] + 5 # 3 bytes header, checksum, end-of-message byte

         if databyte == 0x82:
            if bytes_read == expected_length or expected_length == 0:
               # Correct length, we're in sync! (or unknown message...)

               # Check checksum
               (csum_pass, sum) = check_checksum_hippo(data)
               if not csum_pass:
                  if do_debug:
                     sys.stderr.write('Packet checksum failed, sum is {}\n'.format(sum))
                     sys.stderr.write('Data was: {}\n'.format(' '.join(format(x, '02x') for x in data)))

               in_message = False
               parse_now = True
#            print "End of message"

         elif bytes_read == expected_length:
            # Reached the expected end, but not end byte
            # Out of sync or data corruption?
            # Scrap this data and start over from next 0x81
            if do_debug:
               sys.stderr.write('No 0x82 when packet should end!\n')
               sys.stderr.write('Data was: {}\n'.format(' '.join(format(x, '02x') for x in data)))
            in_message = False
            parse_now = False
            data = []
            bytes_read = 0
            expected_length = 0


      if parse_now:
         parse_now = False

         count = len(data)

         # Parse message
         if data[1] == 0x30 and data[2] == 0x02:  # Fast fix data with raw DR

            status = data[0+3]
            gps_age = data[2+3] # Seconds

            time_of_week = data[3+3] + (data[4+3] << 8) + (data[5+3] << 16) + (data[6+3] << 24)

            lat = (data[7+3]) + (data[8+3] << 8) + (data[9+3] << 16) + (data[10+3] << 24)
            lon = (data[11+3]) + (data[12+3] << 8) + (data[13+3] << 16) + (data[14+3] << 24)
            alt = (data[15+3]) + (data[16+3] << 8)

            lat /= 11930464.7111   # *= 2^-31 * 180.0
            lon /= 11930464.7111   # --"--

            # Parse status word
            position['position_valid'] = True if status & 0x01 else False
            position['altitude_valid'] = True if status & 0x02 else False
            position['heading_valid'] = True if status & 0x04 else False
            position['speed_valid'] = True if status & 0x08 else False

            position['age'] = gps_age
            position['time_of_week'] = time_of_week
            position['lat'] = lat
            position['lon'] = lon
            position['alt'] = alt

            print make_nmea_rmc(position, gps_time).strip()
            #print make_nmea_gga(position, gps_time).strip()  # TODO: Something fishy with this one (or GGA in general)
            print make_nmea_vtg(position).strip()


         elif data[1] == 0x32 and data[2] == 0x01: # UTC time and constellation summary

            gps_time['year'] = data[0+3] + (data[1+3] << 8)
            gps_time['month'] = data[2+3]
            gps_time['day'] = data[3+3]
            gps_time['hour'] = data[4+3]
            gps_time['minute'] = data[5+3]
            gps_time['second'] = data[6+3]
            gps_time['offset'] = data[7+3]

            pdop = data[8+3] + (data[9+3] << 8)
            hdop = data[10+3] + (data[11+3] << 8)
            vdop = data[12+3] + (data[13+3] << 8)

            pdop *= 0.00390625 # 2e-8
            hdop *= 0.00390625
            vdop *= 0.00390625

            position['SV'] = data[17+3] & 0x0F  # Visible satellites

            position['pdop'] = pdop
            position['hdop'] = hdop
            position['vdop'] = vdop

            print make_nmea_zda(gps_time).strip()
            print make_nmea_gsa(position).strip()


         elif data[1] == 0x32 and data[2] == 0x03: # UTC time

            time_of_week = data[1+3] + (data[2+3] << 8) + (data[3+3] << 16) + (data[4+3] << 24)

            gps_time['time_of_week'] = time_of_week
            position['time_of_week'] = time_of_week
            gps_time['week'] = data[5+3] + (data[6+3]<<8)
            gps_time['year'] = data[8+3] + (data[9+3] << 8)
            gps_time['month'] = data[10+3]
            gps_time['day'] = data[11+3]
            gps_time['hour'] = data[12+3]
            gps_time['minute'] = data[13+3]
            gps_time['second'] = data[14+3]
            gps_time['offset'] = data[7+3]

            print make_nmea_zda(gps_time).strip()


         elif data[1] == 0x31 and data[2] == 0x01: # GPS Fix message
            position['fix_source'] = data[4+3] & 0x3F
            position['altitude_hold'] = (data[4+3] & 0xC0) >> 6 # 0=full pos 3D fix, 1= altitude hold 2D fix

            lat = (data[6+3]) + (data[7+3] << 8) + (data[8+3] << 16) + (data[9+3] << 24)
            lon = (data[10+3]) + (data[11+3] << 8) + (data[12+3] << 16) + (data[13+3] << 24)
            alt = (data[14+3]) + (data[15+3] << 8)

            lat /= 11930464.7111   # *= 2^-31 * 180.0
            lon /= 11930464.7111   # --"--

            heading = data[16+3] + (data[17+3]<<8)
            heading /= 32768.0  # *= 2^-15 sc [0, 2)

            speed = data[18+3] + (data[19+3]<<8)

            status = data[5+3]

            # Parse status word
            position['position_valid'] = True if status & 0x01 else False
            position['altitude_valid'] = True if status & 0x02 else False
            position['heading_valid'] = True if status & 0x04 else False
            position['speed_valid'] = True if status & 0x08 else False

            position['lat'] = lat
            position['lon'] = lon
            position['alt'] = alt
            position['heading'] = heading
            position['speed'] = speed

            print make_nmea_gsa(position).strip()
            print make_nmea_gga(position, gps_time).strip()


         elif data[1] == 0x33 and data[2] == 0x01: # GPS channel meas. short status
            # These contain data that can be used for GSV messages in NMEA
            index = data[3]
            if index > 11:
               index = 11  # TODO: Handle properly...

            if position['channel'][index] is None:
               position['channel'][index] = {}

            position['channel'][index]['prn'] = data[0+4]
            position['channel'][index]['visible'] = True if (data[1+4] & 0x01) else False
            position['channel'][index]['has_tracked'] = True if (data[1+4] & 0x04) else False
            position['channel'][index]['tracking'] = True if (data[1+4] & 0x10) else False
            position['channel'][index]['meets_mask'] = True if (data[1+4] & 0x40) else False
            position['channel'][index]['snr'] = data[2+4]
            position['channel'][index]['azimuth'] = data[3+4]
            position['channel'][index]['elevation'] = data[4+4]
            position['channel'][index]['almanac'] = data[5+4] & 0x03  # 0 = None, 1 = Old, 2 = Current
            position['channel'][index]['ephemeris'] = (data[5+4] & 0x0C) >> 2  # 0 = None, 1 = Old, 2 = Decoded, 3 = Verified

            # Re-send all when one has changed
            # TODO: Maybe this could be more intelligent? Send only changed ones? Something?
            sats = []
            tracking = 0
            for i in range(12): # All 12 channels
               if position['channel'][i] is not None:
                  if position['channel'][i]['visible']:
                     sats.append(i)
                  if position['channel'][i]['tracking']:
                     tracking += 1

            position['tracked_count'] = tracking

            sat_time = time.time()
            if sat_time > last_sat_time + 5.0:
               last_sat_time = sat_time

               msg_count = int(ceil(len(sats) / 4.0))
               for i in range(msg_count):
                  print make_nmea_gsv(position, sats, msg_count, i).strip()


         else:
           if do_debug:
               sys.stderr.write('Unhandled message: 0x{:02X} {:02X}\n'.format(data[1], data[2]))

               if do_debug > 1:
                  sys.stderr.write('{}\n'.format(' '.join(format(x, '02x') for x in data)))


pi.bb_serial_read_close(RX)
pi.stop()
