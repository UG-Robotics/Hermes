import argparse
import pathlib
import sys
import time
import logging

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[0]))

from testing.test_transitions import run_simulation
from config.robot_config import SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT
from hardware.buttons import KeyboardOverrideListener
from communication.protocol import serialize_packet, get_emergency_packet

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("MainController")

def run_realtime_loop():
    """Main control loop coordinating inputs, overrides, and serial output."""
    logger.info("Initializing Real-time Controller Loop...")
    
    listener = KeyboardOverrideListener()
    listener.start()
    
    ser = None
    try:
        import serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        logger.info(f"Connected to Arduino on {SERIAL_PORT}")
    except (ImportError, Exception) as e:
        logger.warning(f"Running in EMULATION mode. Serial port unavailable: {e}")

    loop_interval = 1.0 / 20.0  # 20 Hz
    logger.info("Manual controls active: 'w' (FORWARD), 's' (BACKWARD), 'm' (TOGGLE MANUAL)")

    try:
        while True:
            start_time = time.time()
            manual_active = listener.is_manual_mode_active()
            
            if manual_active:
                speed, steer, action = listener.get_manual_target()
                mode_flag = 1
            else:
                # Autonomous mode (placeholder: vehicle remains stopped)
                # Will replace this block with autonomous navigation / sensor loop when ready
                speed, steer, action = 0, 0, "STOP"
                mode_flag = 0
            
            packet = serialize_packet(speed, steer, action, mode_flag)
            
            if ser and ser.is_open:
                try:
                    ser.write(packet.encode('utf-8'))
                    ser.flush()
                except Exception as serial_err:
                    logger.error(f"Serial write error: {serial_err}")
            else:
                print(f"[EMU TX] {packet.strip()}")

            elapsed = time.time() - start_time
            time.sleep(max(0.0, loop_interval - elapsed))

    except KeyboardInterrupt:
        logger.info("Shutdown requested.")
    except Exception as run_err:
        logger.error(f"Runtime error: {run_err}", exc_info=True)
    finally:
        logger.warning("Triggering safety shutdown.")
        try:
            emergency_packet = get_emergency_packet(mode=1)
            if ser and ser.is_open:
                ser.write(emergency_packet.encode('utf-8'))
                ser.flush()
                ser.close()
            else:
                print(f"[EMU EMERGENCY TX] {emergency_packet.strip()}")
        except Exception as fail_safe_err:
            logger.critical(f"Fail-safe error: {fail_safe_err}")

        listener.stop()
        logger.info("Shutdown completed.")

def main():
    parser = argparse.ArgumentParser(description='Robot entrypoint')
    parser.add_argument('--mode', choices=['simulate', 'test', 'run'], default='run')
    args = parser.parse_args()

    if args.mode in ('simulate', 'test'):
        run_simulation()
    else:
        run_realtime_loop()

if __name__ == '__main__':
    main()
