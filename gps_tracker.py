# GPS Module – Position Tracking & Navigation
# 🔌 Wiring
# GPS Pin	 Raspberry Pi Pin
# VCC (5V)	 5V (Pin 4)
# GND	     GND (Pin 6)
# TX	     GPIO 14 (UART0 TX)
# RX	     GPIO 15 (UART0 RX)
import pynmea2
import serial

port = "/dev/serial0"
ser = serial.Serial(port, baudrate=9600, timeout=1)

def read_gps():
    while True:
        line = ser.readline().decode("utf-8", errors="ignore")
        if line.startswith("$GPGGA"):
            msg = pynmea2.parse(line)
            print(f"Latitude: {msg.latitude}, Longitude: {msg.longitude}")

try:
    read_gps()
except KeyboardInterrupt:
    ser.close()
