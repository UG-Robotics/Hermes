import argparse
import pathlib
import sys
import time
from utils.logger import get_logger

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[0]))

from testing.test_transitions import run_simulation
from config.robot_config import SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT
from hardware.buttons import KeyboardOverrideListener
from communication.protocol import serialize_command, get_emergency_packet
from communication.serial_link import SerialLink
from communication.packet_parser import parse_incoming

from control.drive_command import drive_command
from state_machine.manager import StateMachine


logger = get_logger("MainController")

def run_realtime_loop():
    """Main control loop coordinating inputs, overrides, and serial output."""        
    logger.info(
        "\n\n==============================================="
        "\nHermes Autonomous Vehicle Software Starting..."
        "\n==============================================="
    )
    logger.info("Initializing Real-time Controller Loop...")
    
    listener = None

    try:
        listener = KeyboardOverrideListener()
        listener.start()

    except Exception as e:
        logger.warning(
            f"Keyboard listener unavailable: {e}"
        )

    link = SerialLink(SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT)
    link.connect()

    loop_interval = 1.0 / 20.0  # 20 Hz
    logger.info("Manual controls active: 'w' (FORWARD), 's' (BACKWARD), 'm' (TOGGLE MANUAL)")

    try:
        state_machine = StateMachine()
        
        while True:
            start_time = time.time()
            manual_active = (
                listener.is_manual_mode_active()
                if listener
                else False
            )
            
            if manual_active:
                speed, steer, action = listener.get_manual_target()
                mode_flag = 1
            
            else:
                speed, steer, action = drive_command(
                    state_machine.current_state,
                    state_machine.context  # see Step 3 below
                )
                mode_flag = 0
            
            packet = serialize_command(speed, steer, action, mode_flag)
            link.send(packet)

            incoming = link.read_line()
            if incoming:
                parse_incoming(incoming)

            elapsed = time.time() - start_time
            time.sleep(max(0.0, loop_interval - elapsed))

    except KeyboardInterrupt:
        logger.info("Shutdown requested.")
    except Exception as run_err:
        logger.error(f"Runtime error: {run_err}", exc_info=True)
    finally:
        logger.warning("Triggering safety shutdown.")
        emergency_packet = get_emergency_packet(mode=1)
        link.send_emergency(emergency_packet)

        if listener:
            listener.stop()
        logger.info("Shutdown completed.")

def main():
    parser = argparse.ArgumentParser(description='Robot entrypoint')
    parser.add_argument('--mode', choices=['simulate', 'test', 'run'], default='run')
    args = parser.parse_args()

    if args.mode == "simulate":
        run_simulation()

    elif args.mode == "test":
        import unittest
        loader = unittest.TestLoader()
        suite = loader.discover(
            start_dir=str(pathlib.Path(__file__).resolve().parent / "testing"),
            pattern="test_*.py"
        )
        runner = unittest.TextTestRunner(verbosity=2)
        runner.run(suite)
        
        
    else:
        run_realtime_loop()

if __name__ == '__main__':
    main()