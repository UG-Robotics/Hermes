"""
Centralised logging for the whole robot.

Every module obtains its logger via :func:`get_logger`. That single factory
wires up three sinks so a log line written anywhere ends up everywhere it is
useful:

    1. the console   — for live terminal use,
    2. a rotating file in ``logs/hermes.log`` — for post-run analysis, and
    3. the TelemetryHub — so the monitoring dashboard shows the same stream.

Because the hub bridge is attached at the root, ANY existing ``logger.info(...)``
call in the codebase automatically appears on the dashboard with no extra work.
That is what lets the logging "work all the way even if the hardware isn't":
state transitions, drive commands, serial packets and errors are all ordinary
log calls, and they all flow to the same place.
"""

from __future__ import annotations

import logging
import logging.handlers
import os
import pathlib

from utils.telemetry_hub import get_hub, CH_LOG

# logs/ lives at the repo's src/ root: src/utils/logger.py -> parents[1] == src/
_LOG_DIR = pathlib.Path(__file__).resolve().parents[1] / "logs"
_LOG_FILE = _LOG_DIR / "hermes.log"

_LEVEL = getattr(logging, os.environ.get("HERMES_LOG_LEVEL", "INFO").upper(), logging.INFO)
_FORMAT = "[%(levelname)s] %(name)s: %(message)s"
_FILE_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"

_configured = False


class _HubHandler(logging.Handler):
    """A logging handler that forwards every record to the TelemetryHub.

    This is the bridge that makes all logging visible on the dashboard. It is
    intentionally defensive: publishing to the hub must never raise back into
    the logging machinery, or a single bad record could take down the loop.
    """

    def __init__(self):
        super().__init__()
        self._hub = get_hub()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._hub.publish(
                CH_LOG,
                {
                    "level": record.levelname,
                    "module": record.name,
                    "message": record.getMessage(),
                },
            )
        except Exception:  # pragma: no cover - logging must never crash callers
            pass


def _configure_root() -> None:
    """Attach console, rotating-file and hub handlers to the root logger once."""
    global _configured
    if _configured:
        return

    root = logging.getLogger("hermes")
    root.setLevel(_LEVEL)
    root.propagate = False

    console = logging.StreamHandler()
    console.setFormatter(logging.Formatter(_FORMAT))
    root.addHandler(console)

    try:
        _LOG_DIR.mkdir(parents=True, exist_ok=True)
        file_handler = logging.handlers.RotatingFileHandler(
            _LOG_FILE, maxBytes=2_000_000, backupCount=5, encoding="utf-8"
        )
        file_handler.setFormatter(logging.Formatter(_FILE_FORMAT))
        root.addHandler(file_handler)
    except Exception as exc:  # e.g. read-only filesystem — degrade, don't crash
        root.warning("File logging unavailable: %s", exc)

    root.addHandler(_HubHandler())
    _configured = True


def get_logger(name: str) -> logging.Logger:
    """Return a configured logger.

    All loggers are children of the ``hermes`` root so they share its handlers
    (console + file + hub) exactly once, avoiding duplicate lines.
    """
    _configure_root()
    # Namespace every logger under "hermes" so the root handlers apply and we
    # never double-handle. "hermes.state_machine.manager", etc.
    child = name if name.startswith("hermes") else f"hermes.{name}"
    return logging.getLogger(child)
