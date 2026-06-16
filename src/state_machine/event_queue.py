import heapq
import threading
import time

try:
    from state_machine.events import Event, EventType, make_event
    from state_machine.priorities import Priority
    from utils.logger import get_logger
except ModuleNotFoundError:
    import sys
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from state_machine.events import Event, EventType, make_event
    from state_machine.priorities import Priority
    from utils.logger import get_logger

logger = get_logger(__name__)


class EventQueue:
    """
    Thread-safe priority queue for robot events.

    Events are processed in priority order (CRITICAL first, LOW last).
    Within the same priority level, events are processed in arrival order (FIFO).

    Usage:
        queue = EventQueue()
        queue.push(make_event(EventType.PILLAR_DETECTED_RED))
        queue.push(make_event(EventType.LAP_MARKER_DETECTED))

        while not queue.is_empty():
            event = queue.pop()
            state_machine.handle_event(event)
    """

    def __init__(self):
        self._heap = []
        self._lock = threading.Lock()
        self._counter = 0  # tiebreaker to preserve FIFO within same priority

    def push(self, event: Event) -> None:
        """
        Add an event to the queue.
        Higher priority events will be popped first.
        """
        with self._lock:
            # heapq is a min-heap, so negate priority to get max-heap behaviour
            priority_key = -event.priority.value
            entry = (priority_key, self._counter, event)
            heapq.heappush(self._heap, entry)
            self._counter += 1
            logger.debug(
                f"QUEUED: {event.type.name} "
                f"[{event.priority.name}] "
                f"(queue size: {len(self._heap)})"
            )

    def pop(self) -> Event:
        """
        Remove and return the highest priority event.
        Raises IndexError if the queue is empty.
        """
        with self._lock:
            if not self._heap:
                raise IndexError("pop from empty EventQueue")
            _, _, event = heapq.heappop(self._heap)
            logger.debug(
                f"DISPATCHING: {event.type.name} "
                f"[{event.priority.name}] "
                f"(queue size: {len(self._heap)})"
            )
            return event

    def peek(self) -> Event:
        """
        Return the highest priority event without removing it.
        Raises IndexError if the queue is empty.
        """
        with self._lock:
            if not self._heap:
                raise IndexError("peek on empty EventQueue")
            _, _, event = self._heap[0]
            return event

    def is_empty(self) -> bool:
        with self._lock:
            return len(self._heap) == 0

    def size(self) -> int:
        with self._lock:
            return len(self._heap)

    def clear(self) -> None:
        """Flush all pending events. Use on reset or ERROR entry."""
        with self._lock:
            self._heap.clear()
            self._counter = 0
            logger.warning("EventQueue cleared.")

    def drain(self) -> list[Event]:
        """
        Pop all events in priority order and return them as a list.
        Useful for processing an entire tick's worth of events at once.
        """
        events = []
        while not self.is_empty():
            events.append(self.pop())
        return events