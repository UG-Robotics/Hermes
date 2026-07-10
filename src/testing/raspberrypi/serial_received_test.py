import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Connected to {PORT}")

while True:
    line = ser.readline().decode(
        "utf-8",
        errors="replace"
    ).strip()

    if line:
        print(line)