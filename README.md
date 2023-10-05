# SAROS-Demo-Flight
Software for payload demonstration flights.

## LED Code Guide:
O-off  \n
S-static  \n
F-flashing \n
 \n
Order: BLUE/GREEN/RED \n
 \n
-1: F/F/F -GPS Locked \n
0:  F/O/O \n
1:  O/F/O \n
2:  O/O/F -SD Init Failure (SD Card not found) \n
3:  S/F/O -SD File Limit Reached (25 data_out files) \n
4:  S/O/F -BME Startup Failure \n
5:  F/S/O \n
6:  O/S/F -GPS Startup Failure \n
7:  F/O/S -ADS Startup Failure \n
8:  O/F/S -SHTX Startup Failure \n
9:  F/F/O \n
10:O/F/F -BNO Startup Failure \n
11:F/O/F  -SD No ID found \n
20: Rolling colors across all LEDs - Setup Complete (After attempting GPS Lock for ~4.5 minutes) \n
 \n
Constant Blinking Red- Trying to lock GPS (This will go for ~4.5 minutes) \n

