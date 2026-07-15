"""
Scenario runner — "string together events and watch what the bot does".

A scenario is a JSON file listing timed state-machine events, e.g. press start,
then a red pillar appears, then a lap marker, etc. Running one injects those
events into a Runtime at the scripted times, so you can reproduce a whole match
sequence deterministically and read back exactly how the state machine, planner
and (simulated) hardware responded — all in the logs and on the dashboard.

Scenario file format:
    {
      "name": "obstacle_lap",
      "description": "Start, dodge a red then green pillar, complete a lap.",
      "events": [
        {"at": 0.5, "event": "START_BUTTON_PRESSED"},
        {"at": 2.0, "event": "PILLAR_DETECTED_RED"},
        {"at": 3.0, "event": "OBSTACLE_CLEARED"},
        {"at": 5.0, "event": "LAP_MARKER_DETECTED"}
      ]
    }

`at` is seconds from scenario start.
"""

from __future__ import annotations

import json
import pathlib
import threading
import time
from typing import Callable, List, Optional

from utils.logger import get_logger

logger = get_logger("Scenario")

SCENARIO_DIR = pathlib.Path(__file__).resolve().parent / "scenarios"


def list_scenarios() -> List[dict]:
    """Return metadata for every scenario file shipped in scenarios/."""
    out = []
    if not SCENARIO_DIR.exists():
        return out
    for path in sorted(SCENARIO_DIR.glob("*.json")):
        try:
            data = json.loads(path.read_text())
            out.append({
                "file": path.name,
                "name": data.get("name", path.stem),
                "description": data.get("description", ""),
                "event_count": len(data.get("events", [])),
            })
        except Exception as e:
            logger.warning(f"Skipping unreadable scenario {path.name}: {e}")
    return out


def load_scenario(name_or_file: str) -> dict:
    """Load a scenario by file name ('foo.json') or bare name ('foo')."""
    candidate = SCENARIO_DIR / name_or_file
    if not candidate.exists() and not name_or_file.endswith(".json"):
        candidate = SCENARIO_DIR / f"{name_or_file}.json"
    if not candidate.exists():
        raise FileNotFoundError(f"Scenario not found: {name_or_file}")
    return json.loads(candidate.read_text())


class ScenarioPlayer:
    """Plays a scenario's events into a Runtime on a background timer thread."""

    def __init__(self, runtime, scenario: dict):
        self.runtime = runtime
        self.scenario = scenario
        self._thread: Optional[threading.Thread] = None
        self._cancel = threading.Event()

    def start(self, on_done: Optional[Callable[[], None]] = None) -> None:
        name = self.scenario.get("name", "scenario")
        events = sorted(self.scenario.get("events", []), key=lambda e: e.get("at", 0))
        logger.info(f"SCENARIO START: '{name}' ({len(events)} events)")

        def _run():
            t0 = time.time()
            for item in events:
                if self._cancel.is_set():
                    logger.warning("Scenario cancelled.")
                    break
                target = t0 + float(item.get("at", 0))
                # Sleep in small slices so cancel is responsive.
                while time.time() < target and not self._cancel.is_set():
                    time.sleep(0.02)
                if self._cancel.is_set():
                    break
                self.runtime.inject_event(item["event"], source="scenario")
            logger.info(f"SCENARIO COMPLETE: '{name}'")
            if on_done:
                on_done()

        self._thread = threading.Thread(target=_run, daemon=True, name="ScenarioPlayer")
        self._thread.start()

    def cancel(self) -> None:
        self._cancel.set()


def run_scenario_headless(name_or_file: str, settle: float = 2.0) -> List[dict]:
    """Run a scenario against a fresh simulated Runtime and return the log records.

    Handy for CI / quick checks: no dashboard, no keyboard, just "did the state
    machine do the right thing?". Returns the log records captured during the run.
    """
    from runtime import Runtime  # local import to avoid a cycle at module load

    scenario = load_scenario(name_or_file)
    runtime = Runtime(simulated=True, use_keyboard=False, use_camera=False)
    runtime.start_background()

    player = ScenarioPlayer(runtime, scenario)
    done = threading.Event()
    player.start(on_done=done.set)

    # Wait for the scripted events, plus a little settle time for effects.
    horizon = max((e.get("at", 0) for e in scenario.get("events", [])), default=0)
    done.wait(timeout=horizon + settle + 5)
    time.sleep(settle)

    runtime.stop()
    runtime.shutdown()
    return runtime._hub.recent_logs()
