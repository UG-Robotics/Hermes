"""
Hermes entry point.

Modes:
    run        real hardware, 20 Hz control loop (default).
    sim        same loop but against simulated hardware (virtual ESP32 +
               synthetic camera) — drive and inject events from the keyboard
               with no robot attached.
    dashboard  launch the web monitoring UI (simulated by default; add
               --real to drive real hardware from the browser).
    scenario   run a scripted event scenario headless and print the log.
    simulate   the original state-machine mock walk-through (unit-test style).
    test       run the unittest suite.

Hardware substitution is controlled by --real / --sim (or the HERMES_SIM env
var). Everything is logged through the TelemetryHub, so logs work identically
whether the hardware is real or simulated.
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[0]))

from utils.logger import get_logger
from config.robot_config import SIMULATION

logger = get_logger("MainController")


def main():
    parser = argparse.ArgumentParser(description="Hermes robot entrypoint")
    parser.add_argument(
        "--mode",
        choices=["run", "sim", "dashboard", "scenario", "simulate", "test"],
        default="run",
    )
    parser.add_argument("--real", action="store_true", help="force real hardware")
    parser.add_argument("--sim", action="store_true", help="force simulated hardware")
    parser.add_argument("--host", default="0.0.0.0", help="dashboard bind host")
    parser.add_argument("--port", type=int, default=5000, help="dashboard port")
    parser.add_argument("--scenario", default="full_match", help="scenario name (scenario mode)")
    parser.add_argument("--no-keyboard", action="store_true", help="disable keyboard listener")
    parser.add_argument("--auto-start", action="store_true", help="inject START_BUTTON_PRESSED automatically once ready")
    args = parser.parse_args()

    # Resolve whether hardware is simulated. Explicit flags win; then the mode's
    # sensible default; then the config/env value.
    if args.real:
        simulated = False
    elif args.sim:
        simulated = True
    elif args.mode in ("sim", "dashboard", "scenario", "simulate"):
        simulated = True
    else:
        simulated = SIMULATION

    if args.mode == "simulate":
        # Original lightweight state-machine walkthrough (no hardware at all).
        from testing.test_transitions import run_simulation
        run_simulation()

    elif args.mode == "test":
        import unittest
        loader = unittest.TestLoader()
        suite = loader.discover(
            start_dir=str(pathlib.Path(__file__).resolve().parent / "testing"),
            pattern="test_*.py",
        )
        unittest.TextTestRunner(verbosity=2).run(suite)

    elif args.mode == "scenario":
        from monitoring.scenario import run_scenario_headless
        logs = run_scenario_headless(args.scenario)
        print(f"\n--- scenario '{args.scenario}' captured {len(logs)} log records ---")

    elif args.mode == "dashboard":
        from runtime import Runtime
        from monitoring.dashboard import run_dashboard
        runtime = Runtime(simulated=simulated, use_keyboard=not args.no_keyboard, use_camera=True,
                          auto_start=args.auto_start)
        run_dashboard(runtime, host=args.host, port=args.port)

    else:  # run / sim
        from runtime import Runtime
        runtime = Runtime(simulated=simulated, use_keyboard=not args.no_keyboard, use_camera=True,
                          auto_start=args.auto_start)
        runtime.run_forever()


if __name__ == "__main__":
    main()
