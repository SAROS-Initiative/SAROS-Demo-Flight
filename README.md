# SAROS-Demo-Flight
Software for payload demonstration flights.

## LED Code Guide:
O-off 
S-static 
F-flashing

Order: BLUE/GREEN/RED

-1: F/F/F -GPS Locked
0:  F/O/O
1:  O/F/O
2:  O/O/F -SD Init Failure (SD Card not found)
3:  S/F/O -SD File Limit Reached (25 data_out files)
4:  S/O/F -BME Startup Failure
5:  F/S/O
6:  O/S/F -GPS Startup Failure
7:  F/O/S -ADS Startup Failure
8:  O/F/S -SHTX Startup Failure
9:  F/F/O
10:O/F/F -BNO Startup Failure
11:F/O/F  -SD No ID found
20: Rolling colors across all LEDs - Setup Complete (After attempting GPS Lock for ~4.5 minutes)

Constant Blinking Red- Trying to lock GPS (This will go for ~4.5 minutes)

