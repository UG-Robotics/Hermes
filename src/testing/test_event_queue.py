import unittest
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from state_machine.event_queue import EventQueue
from state_machine.events import make_event, EventType
from state_machine.priorities import Priority


class TestEventQueue(unittest.TestCase):

    def test_higher_priority_popped_first(self):
        q = EventQueue()
        q.push(make_event(EventType.LAP_MARKER_DETECTED))   # MEDIUM
        q.push(make_event(EventType.PILLAR_DETECTED_RED))   # HIGH
        first = q.pop()
        self.assertEqual(first.type, EventType.PILLAR_DETECTED_RED)

    def test_critical_always_first(self):
        q = EventQueue()
        q.push(make_event(EventType.LAP_MARKER_DETECTED))   # MEDIUM
        q.push(make_event(EventType.PILLAR_DETECTED_RED))   # HIGH
        q.push(make_event(EventType.FAILURE_DETECTED))      # CRITICAL
        first = q.pop()
        self.assertEqual(first.priority, Priority.CRITICAL)

    def test_fifo_within_same_priority(self):
        q = EventQueue()
        q.push(make_event(EventType.PILLAR_DETECTED_RED))   # HIGH, arrives first
        q.push(make_event(EventType.OBSTACLE_CLEARED))      # HIGH, arrives second
        first = q.pop()
        self.assertEqual(first.type, EventType.PILLAR_DETECTED_RED)

    def test_is_empty(self):
        q = EventQueue()
        self.assertTrue(q.is_empty())
        q.push(make_event(EventType.START_BUTTON_PRESSED))
        self.assertFalse(q.is_empty())

    def test_pop_empty_raises(self):
        q = EventQueue()
        with self.assertRaises(IndexError):
            q.pop()

    def test_peek_does_not_remove(self):
        q = EventQueue()
        q.push(make_event(EventType.PILLAR_DETECTED_RED))
        q.peek()
        self.assertEqual(q.size(), 1)

    def test_clear_flushes_queue(self):
        q = EventQueue()
        q.push(make_event(EventType.PILLAR_DETECTED_RED))
        q.push(make_event(EventType.LAP_MARKER_DETECTED))
        q.clear()
        self.assertTrue(q.is_empty())

    def test_drain_returns_priority_ordered_list(self):
        q = EventQueue()
        q.push(make_event(EventType.LAP_MARKER_DETECTED))   # MEDIUM
        q.push(make_event(EventType.FAILURE_DETECTED))      # CRITICAL
        q.push(make_event(EventType.PILLAR_DETECTED_GREEN)) # HIGH
        events = q.drain()
        self.assertEqual(events[0].priority, Priority.CRITICAL)
        self.assertEqual(events[1].priority, Priority.HIGH)
        self.assertEqual(events[2].priority, Priority.MEDIUM)
        self.assertTrue(q.is_empty())

    def test_size(self):
        q = EventQueue()
        self.assertEqual(q.size(), 0)
        q.push(make_event(EventType.START_BUTTON_PRESSED))
        q.push(make_event(EventType.LAP_MARKER_DETECTED))
        self.assertEqual(q.size(), 2)


if __name__ == "__main__":
    unittest.main()