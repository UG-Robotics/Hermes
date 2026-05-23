try:
    from state_machine.manager import StateMachine
    from testing.mock_events import generate_mock_events
except ModuleNotFoundError:
    # Allow running this file directly by adding src to sys.path
    import sys
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from state_machine.manager import StateMachine
    from testing.mock_events import generate_mock_events

import time


def run_simulation():

    robot = StateMachine()

    events = generate_mock_events()

    for event in events:

        print("\n----------------------------------")

        robot.handle_event(event)

        time.sleep(1)


if __name__ == "__main__":
    run_simulation()
