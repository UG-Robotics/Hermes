import serial
import time

PORT = "/dev/ttyAMA0"
BAUD = 115200

ser = serial.Serial(
    PORT,
    BAUD,
    timeout=1
)

print(f"Connected to {PORT}")

time.sleep(2)  # allow ESP32 reset after serial connection

packet = "CMD,40,0,FORWARD,1\n"

while True:
    ser.write(packet.encode("utf-8"))
    print("Sent:", packet.strip())

    time.sleep(1)