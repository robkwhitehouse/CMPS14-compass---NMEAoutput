# CMPS14-compass---NMEAoutput
Outputs a tilt compensated compass heading over WiFi in NMEA format

Uses an ESP32 and a CMPS 14 breakout. Also supports an OLED display but this is somewhat redundant

It provides a WiFi access point. This AP has two Telnet servers;
port 23 - This has a stream of current compass headings in NMEA "HDM" message format - 5 per second
port 1023 - This supports a simple configuration and calibration menu
