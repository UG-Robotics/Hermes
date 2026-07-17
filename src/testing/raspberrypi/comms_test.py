
import serial
import time


PORT = "/dev/serial0"
BAUD = 115200


ser = serial.Serial(
    PORT,
    BAUD,
    timeout=1
)

time.sleep(2)


def send_command(msg):

    print("TX:", msg)

    ser.write((msg + "\n").encode())

    start = time.time()

    while time.time() - start < 2:

        if ser.in_waiting:

            reply = ser.readline().decode().strip()

            print("RX:", reply)


            if reply.startswith("ACK"):
                return True

            if reply == "PONG":
                return True


    print("NO RESPONSE")
    return False



while True:

    cmd = input("> ")

    if cmd == "ping":
        send_command("PING")

    else:
        send_command("CMD," + cmd)

# import serial
# import time

# PORT = "/dev/ttyAMA0" 
# BAUD = 115200

# ser = serial.Serial(PORT, BAUD, timeout=1)
# time.sleep(2)  # let the port settle

# print(f"Sending on {PORT}...")

# count = 0
# try:
#     while True:
#         msg = f"Hello from Pi #{count}"
#         ser.write((msg + "\n").encode("utf-8"))
#         print("TX:", msg)
#         count += 1
#         time.sleep(0.5)
# except KeyboardInterrupt:
#     print("\nStopped")
# finally:
#     ser.close()

# import serial
# import time

# PORT = "/dev/ttyAMA0"  
# BAUD = 115200

# ser = serial.Serial(PORT, BAUD, timeout=1)
# time.sleep(2)  # let the port settle, ESP32 may reset on connect

# print(f"Listening on {PORT}...")

# try:
#     while True:
#         line = ser.readline().decode("utf-8", errors="replace").strip()
#         if line:
#             print("RX:", line)
# except KeyboardInterrupt:
#     print("\nStopped")
# finally:
#     ser.close()