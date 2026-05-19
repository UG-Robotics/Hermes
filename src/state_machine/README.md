# State Machine

This directory contains the core state machine primitives and logic used to
drive robot behavior based on events and priorities.

## Files

1. README.md: Overview of the state machine module and file responsibilities.
2. events.py: Defines the EventType enum and the Event dataclass used to carry
   event metadata and timestamps.
3. priorities.py: Defines Priority as an IntEnum to order events by urgency.
4. states.py: Defines the State enum used by the state machine.
5. transitions.py: Implements TransitionManager with the transition table and
   logging for valid/invalid transitions.
6. manager.py: Implements StateMachine, applies priority handling, and updates
   current state via TransitionManager.
